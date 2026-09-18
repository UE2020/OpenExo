from typing import List
import time
import logging
import traceback

try:
    from PySide6 import QtCore
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e


class RtBridge(QtCore.QObject):
    """
    Self-contained real-time parser for the Qt app.
    Parses the same ASCII protocol used by firmware/Tk GUI:
    - handshake: literal "handshake"
    - parameter names: plain lines until "END"
    - controllers: "!<controller>" then "!!<param>" … and "!END"
    - rt data: frames containing 'c' with numeric payload per the existing scheme
    """

    handshakeReceived = QtCore.Signal(str)
    parameterNamesReceived = QtCore.Signal(list)
    controllersReceived = QtCore.Signal(list, list)
    controllerMatrixReceived = QtCore.Signal(list)
    controllerValuesReceived = QtCore.Signal(list)
    paramUpdateAckReceived = QtCore.Signal(dict)
    rtDataUpdated = QtCore.Signal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Setup logger
        self.logger = logging.getLogger("OpenExo.RtBridge")
        self.logger.info("Initializing RtBridge...")
        
        # Name/controller state
        self._handshake = False
        self._collecting_names = True
        self._names: List[str] = []
        self._controllers: List[str] = []
        self._controller_params: List[List[str]] = []
        self._temp_params: List[str] = []
        self._controllers_done = False
        self._controller_matrix: List[List[str]] = []
        self._rows_68: List[List[str]] = []
        self._rows_36: List[List[str]] = []
        self._rows_38: List[List[str]] = []

        # Stream parse state (port of minimal logic)
        self._event_count_regex = QtCore.QRegularExpression("[0-9]+")
        self._start_transmission = False
        self._command = None
        self._num_count = 0
        self._buffer: List[str] = []
        self._payload: List[float] = []
        self._data_length = 0

        # Byte-level reassembly for live protocol frames. BLE notifications can
        # split a frame across callbacks or coalesce several frames into one, so
        # frames must be parsed as a stream rather than per callback.
        self._rx_buffer: bytes = b""

        # Handshake payload reassembly state
        self._collecting_handshake_payload = False
        
        # Data rate monitoring (verbose console stats; use logging level DEBUG if enabled)
        self.DEBUG_DATA_RATE = False
        self._data_packet_count = 0
        self._bytes_received = 0
        self._data_rate_timer = QtCore.QTimer()
        self._data_rate_timer.timeout.connect(self._print_data_rate)
        self._data_rate_timer.start(1000)  # Print every 1 second
        self._last_packet_time = None
        self._packet_intervals = []  # Track time between packets for jitter analysis
        self._expected_hz = 100  # Expected data rate (will auto-detect)
        self._total_packets_received = 0
        self._total_time_elapsed = 0.0
        self._monitoring_start_time = None
        self._dropped_packet_count = 0
        
        # Additional BLE metrics
        self._ble_chunk_count = 0  # Number of BLE chunks received (not packets)
        self._ble_chunk_sizes = []  # Track BLE chunk sizes
        self._consecutive_drops = 0  # Track consecutive packet drops
        self._max_consecutive_drops = 0  # Worst consecutive drop streak
        self._stall_count = 0  # Number of times data stopped flowing >100ms
        self._last_stall_time = None
        
        self._handshake_payload_buf: str = ""
        self._max_handshake_payload_chars = 25002
        self._max_protocol_values = 32
        self._max_numeric_token_chars = 16
        self._max_metadata_items = 512
        self._max_frame_buffer = 512
        # ArduinoBLE truncates notifications to MTU-3 bytes, which drops the
        # final delimiter from a 21-byte ACK. If a partial frame does not
        # complete within this window, accept its last token as terminated.
        self._frame_flush_ms = 100
        self._frame_flush_timer = QtCore.QTimer()
        self._frame_flush_timer.setSingleShot(True)
        self._frame_flush_timer.timeout.connect(self._flush_partial_frame)

    @QtCore.Slot(bytes)
    def feed_bytes(self, data: bytes):
        # Track bytes received
        self._bytes_received += len(data)
        
        # Track BLE chunk metrics
        self._ble_chunk_count += 1
        chunk_size = len(data)
        self._ble_chunk_sizes.append(chunk_size)
        
        # Detect stalls (>100ms between any data)
        current_time = time.monotonic()
        if self._last_stall_time is not None:
            stall_interval = (current_time - self._last_stall_time) * 1000
            if stall_interval > 100:
                self._stall_count += 1
        self._last_stall_time = current_time

        # Fresh bytes invalidate any pending idle-flush of a partial frame.
        self._frame_flush_timer.stop()

        self.logger.debug(
            "RX callback: %d byte(s)%s: %r",
            chunk_size,
            f", {len(self._rx_buffer)} buffered" if self._rx_buffer else "",
            bytes(data[:64]),
        )

        if self._is_metadata_phase():
            self._feed_metadata_bytes(data)
            return

        self._rx_buffer += data
        self._drain_frames()
        if self._rx_buffer:
            self._frame_flush_timer.start(self._frame_flush_ms)

    def _is_metadata_phase(self) -> bool:
        """True while notifications carry handshake/metadata text, not live frames."""
        return (
            self._collecting_handshake_payload
            or self._collecting_names
            or (self._handshake and not self._controllers_done)
        )

    def _feed_metadata_bytes(self, data: bytes):
        try:
            s = data.decode("utf-8")
        except Exception as e:
            self.logger.error(f"Failed to decode received data: {e}")
            self.logger.debug(traceback.format_exc())
            return

        # Handshake
        if s == "READY":
            self._handshake = True
            # Begin collecting the initial long handshake payload split across notifications
            self._collecting_handshake_payload = True
            self._handshake_payload_buf = ""
            return

        # If we're collecting the extended handshake payload, accumulate until newline
        if self._collecting_handshake_payload:
            self._handshake_payload_buf += s
            if (
                len(self._handshake_payload_buf)
                > self._max_handshake_payload_chars
            ):
                self.logger.error(
                    "Discarding oversized handshake payload (%s chars)",
                    len(self._handshake_payload_buf),
                )
                self.reset_for_new_ble_session()
                return
            if "\n" in self._handshake_payload_buf:
                line, _, _ = self._handshake_payload_buf.partition("\n")
                # Split by commas and drop empty entries
                tokens = [tok.strip() for tok in line.split(",") if tok.strip()]
                
                payload_line = line.replace('|', '\n')
                joined_tokens = ", ".join(tokens)
                payload_str = "READY" if not joined_tokens else f"READY, {joined_tokens}"
                self.handshakeReceived.emit(payload_str)

                # Parse controllers and parameter headers from the payload blob
                rows = [row.strip() for row in payload_line.split("\n") if row.strip()]
                controller_rows = []
                value_rows = []
                controller_values = {}
                param_names = []
                self._rows_68 = []
                self._rows_36 = []
                self._rows_38 = []
                current_joint = None
                current_rows: List[str] = []
                
                for row in rows:
                    parts_raw = row.split(",")
                    parts = [part.strip() for part in parts_raw if part.strip()]
                    if not parts:
                        continue
                    prefix = parts[0].lower()
                    if prefix == 'f':
                        # Legacy fetch command header; keep data if the row has content.
                        if len(parts) > 1:
                            parts = parts[1:]
                        else:
                            continue
                    if prefix == 't':
                        param_names = [p.strip() for p in parts[1:] if p.strip()]
                        continue
                    if prefix == 'v':
                        value_rows.append(parts)
                        # Expected format: v,<joint_id>,<controller_id>,<v1>,<v2>,...
                        if len(parts) >= 3:
                            key = (parts[1], parts[2])
                            controller_values[key] = parts[3:]
                        continue
                    if prefix.startswith('?'):
                        # End-of-handshake sentinel
                        continue
                    
                    # Each row format: [JointName, JointID, ControllerName, ParamCount, ...params]
                    # Example: ['Ankle(L)', '68', 'zeroTorqu', '2', 'use_pid', 'p_gain', 'i_gain', 'd_gain']
                    controller_rows.append(parts)
                    
                    # Group by joint name for display blocks
                    if len(parts) >= 2:
                        joint_name = parts[0]
                        joint_id = parts[1]
                        
                        if current_joint is None:
                            current_joint = joint_name
                        elif current_joint != joint_name:
                            formatted_block = "\n".join(current_rows)
                            if joint_id == '68':
                                self._rows_68.append(formatted_block)
                            elif joint_id == '36':
                                self._rows_36.append(formatted_block)
                            elif joint_id == '38':
                                self._rows_38.append(formatted_block)
                            current_rows = []
                            current_joint = joint_name
                        
                        row_string = ",".join(parts)
                        current_rows.append(row_string)

                if current_rows and current_joint:
                    # Flush last group - need to determine ID from the last row
                    if controller_rows:
                        last_id = controller_rows[-1][1] if len(controller_rows[-1]) > 1 else None
                        formatted_block = "\n".join(current_rows)
                        if last_id == '68':
                            self._rows_68.append(formatted_block)
                        elif last_id == '36':
                            self._rows_36.append(formatted_block)
                        elif last_id == '38':
                            self._rows_38.append(formatted_block)

                if param_names:
                    self._param_names = list(param_names)
                    self.parameterNamesReceived.emit(list(param_names))

                if controller_rows:
                    # Build matrix: [JointName, JointID, ControllerName, ControllerID, Param1, Param2, ...]
                    self._controller_matrix = []
                    for row in controller_rows:
                        if len(row) >= 3:
                            # row[0] = joint name (e.g., "Ankle(L)")
                            # row[1] = joint ID (e.g., "68")
                            # row[2] = controller name (e.g., "pjmc_plus")
                            # row[3] = controller ID (e.g., "11")
                            # row[4:] = parameter names
                            joint_name = row[0]
                            joint_id = row[1]
                            controller_name = row[2]
                            controller_id = row[3] if len(row) > 3 else "0"
                            params = row[4:] if len(row) > 4 else []
                            
                            # Create display row: [Joint(ID), JointID, ControllerName, ControllerID, Param1, Param2, ...]
                            display_row = [f"{joint_name} ({joint_id})", joint_id, controller_name, controller_id] + params
                            self._controller_matrix.append(display_row)
                    
                    if self._controller_matrix:
                        self.controllerMatrixReceived.emit(list(self._controller_matrix))

                # Always emit (possibly empty) so MainWindow can replace stale DB and pad from matrix.
                flat_values: List[list] = []
                for (joint_id, controller_id), values in controller_values.items():
                    flat_values.append([joint_id, controller_id] + list(values))
                self.controllerValuesReceived.emit(flat_values)

                # Done collecting extended handshake
                self._collecting_handshake_payload = False
                self._handshake_payload_buf = ""
                # Treat the handshake payload as the complete parameter preamble
                self._collecting_names = False
                self._names.clear()
                self._controllers_done = True
            return

        # Parameter names first, plain strings until END
        # Accept all lines (including those containing the letter 'c'); only exclude controller-prefixed lines
        if self._handshake and self._collecting_names and not s.startswith("!"):
            if s == "END":
                self._collecting_names = False
                if self._names:
                    self.parameterNamesReceived.emit(list(self._names))
            else:
                if (
                    len(self._names) < self._max_metadata_items
                    and len(s) <= 256
                ):
                    self._names.append(s)
                else:
                    self.logger.warning("Discarding oversized parameter metadata")
            return

        # Controllers and their parameters using ! protocol
        if self._handshake and s.startswith("!"):
            # strip leading '!'
            payload = s[1:]
            if payload == 'END':
                # Close out the last controller params if any
                if self._temp_params:
                    self._controller_params.append(self._temp_params)
                    self._temp_params = []
                # Build 2D controller-parameter matrix: [ [controller, param1, param2, ...], ... ]
                self._controller_matrix = []
                for i, ctrl in enumerate(self._controllers):
                    params = self._controller_params[i] if i < len(self._controller_params) else []
                    row = [ctrl] + list(params)
                    self._controller_matrix.append(row)
                if self._controllers:
                    self.controllersReceived.emit(list(self._controllers), list(self._controller_params))
                    self.controllerMatrixReceived.emit(list(self._controller_matrix))
                self._controllers_done = True
                return
            # parameter vs controller
            if payload.startswith("!"):
                # parameter name
                if len(self._temp_params) < self._max_metadata_items:
                    self._temp_params.append(payload[1:257])
            else:
                # new controller begins
                if self._temp_params:
                    self._controller_params.append(self._temp_params)
                    self._temp_params = []
                if len(self._controllers) < self._max_metadata_items:
                    self._controllers.append(payload[:256])
            return

        # Real-time data frames
        if 'c' in s:
            parts = s.split('c')
            if len(parts) < 2:
                return
            event_info = parts[0]
            event_data = parts[1]
            if len(event_info) < 2:
                return
            command = event_info[1]
            # Extract count from event_info using regex
            m = self._event_count_regex.match(event_info)
            if not m.hasMatch():
                return
            try:
                self._data_length = int(m.captured(0))
            except Exception as e:
                self.logger.error(f"Failed to parse data length: {e}, captured: {m.captured(0) if m else 'None'}")
                self.logger.debug(traceback.format_exc())
                return

            if not 0 <= self._data_length <= self._max_protocol_values:
                self.logger.warning(
                    "Discarding frame with invalid value count: %s",
                    self._data_length,
                )
                self._reset_stream()
                return

            if command == 'a':
                self._handle_param_update_ack(event_data, self._data_length)
                return

            event_without_count = f"{event_info[0]}{event_info[1]}{event_data}"
            # Parse stream similar to original logic
            for ch in event_without_count:
                if ch == 'S' and not self._start_transmission:
                    self._start_transmission = True
                    continue
                elif self._start_transmission:
                    if not self._command:
                        self._command = ch
                    elif ch == 'n':
                        self._num_count += 1
                        if self._num_count > self._data_length:
                            self._reset_stream()
                            return
                        token = ''.join(self._buffer)
                        try:
                            val = float(token) / 100.0
                        except Exception as e:
                            self.logger.error(f"Failed to parse float value from token '{token}': {e}")
                            self.logger.debug(traceback.format_exc())
                            val = None
                        self._buffer.clear()
                        if val is not None:
                            self._payload.append(val)
                        if self._num_count == self._data_length:
                            # Drop spurious single-value frames (e.g., fragmented BLE chunks)
                            if self._data_length <= 1:
                                self._reset_stream()
                                return
                            # Emit payload; pad/crop to 16 entries for safety
                            self._publish_rt_data(list(self._payload))
                            
                            # reset state
                            self._reset_stream()
                        else:
                            continue
                    else:
                        if self._data_length != 0:
                            if len(self._buffer) >= self._max_numeric_token_chars:
                                self.logger.warning(
                                    "Discarding oversized numeric token"
                                )
                                self._reset_stream()
                                return
                            self._buffer.append(ch)
                        else:
                            return
                else:
                    return

    def _reset_stream(self):
        self._start_transmission = False
        self._command = None
        self._data_length = 0
        self._num_count = 0
        self._payload.clear()
        self._buffer.clear()

    def _handle_param_update_ack(self, event_data: str, data_length: int):
        try:
            values = []
            for token in event_data.split('n'):
                if not token:
                    continue
                values.append(float(token) / 100.0)
                if len(values) == data_length:
                    break

            if len(values) < 5:
                self.logger.warning("Ignoring short parameter update ack: %s", event_data)
                return

            ack = {
                "joint_id": int(round(values[0])),
                "controller_id": int(round(values[1])),
                "param_index": int(round(values[2])),
                "accepted": bool(int(round(values[3]))),
                "reason": int(round(values[4])),
            }
            self.paramUpdateAckReceived.emit(ack)
        except Exception as e:
            self.logger.error(f"Failed to parse parameter update ack: {e}")
            self.logger.debug(traceback.format_exc())

    def _publish_rt_data(self, values: List[float]):
        """Pad/crop a completed real-time frame, emit it, and update rate stats."""
        values = list(values)
        if len(values) < 16:
            values.extend([0.0] * (16 - len(values)))
        elif len(values) > 16:
            values = values[:16]
        self.rtDataUpdated.emit(values)

        # Track data rate and timing
        self._data_packet_count += 1
        self._total_packets_received += 1
        current_time = time.monotonic()

        if self._monitoring_start_time is None:
            self._monitoring_start_time = current_time

        if self._last_packet_time is not None:
            interval = (current_time - self._last_packet_time) * 1000  # Convert to ms
            self._packet_intervals.append(interval)

            # Detect dropped packets (interval > 2.5x expected). More
            # conservative threshold to avoid false positives from jitter.
            expected_interval = 1000.0 / self._expected_hz if self._expected_hz > 0 else 14.3
            if interval > expected_interval * 2.5:
                dropped = int(round(interval / expected_interval)) - 1
                self._dropped_packet_count += max(0, dropped)
                self._consecutive_drops += dropped
                if self._consecutive_drops > self._max_consecutive_drops:
                    self._max_consecutive_drops = self._consecutive_drops
            else:
                self._consecutive_drops = 0

        self._last_packet_time = current_time

    def _drain_frames(self):
        """Extract every complete protocol frame from the reassembly buffer."""
        while self._rx_buffer:
            buf = self._rx_buffer
            parsed = self._parse_frame(buf)
            if parsed is None:
                if len(buf) > self._max_frame_buffer:
                    self.logger.warning(
                        "Dropping oversized frame buffer (%d bytes)", len(buf)
                    )
                    self._rx_buffer = b""
                return
            if parsed is False:
                self.logger.warning(
                    "Discarding malformed frame bytes: %r", bytes(buf[:32])
                )
                next_start = buf.find(b"S", 1)
                self._rx_buffer = buf[next_start:] if next_start != -1 else b""
                continue
            end, command, count, tokens = parsed
            self._rx_buffer = buf[end:]
            self._process_frame(command, count, tokens)

    def _parse_frame(self, buf: bytes, allow_unterminated_tail: bool = False):
        """Parse one ``S<command><count>c<v>n...`` frame.

        Returns ``(end, command, count, tokens)`` on success, ``None`` when the
        buffer holds only an incomplete prefix, or ``False`` when the bytes
        cannot begin a frame and the stream must resynchronise. When
        ``allow_unterminated_tail`` is set, a final token without its trailing
        delimiter is accepted (BLE truncates oversized notifications).
        """
        if not buf or buf[0:1] != b"S":
            return False
        if len(buf) < 2:
            return None
        command = buf[1:2]
        index = 2
        digits_start = index
        while index < len(buf) and 0x30 <= buf[index] <= 0x39:
            index += 1
        if index == digits_start:
            return None if index >= len(buf) else False
        if index >= len(buf):
            return None
        if buf[index:index + 1] != b"c":
            return False
        try:
            count = int(buf[digits_start:index])
        except ValueError:
            return False
        if count < 0 or count > self._max_protocol_values:
            return False
        index += 1
        tokens: List[bytes] = []
        for _ in range(count):
            delimiter = buf.find(b"n", index)
            if delimiter == -1:
                if (
                    allow_unterminated_tail
                    and len(tokens) == count - 1
                    and index < len(buf)
                ):
                    tokens.append(buf[index:])
                    return len(buf), command, count, tokens
                return None
            tokens.append(buf[index:delimiter])
            index = delimiter + 1
        return index, command, count, tokens

    def _process_frame(self, command: bytes, count: int, tokens: List[bytes]):
        """Route a fully reassembled frame to the ACK or real-time handler."""
        self.logger.debug(
            "Parsed frame: command=%r count=%d payload=%r",
            command.decode("ascii", errors="replace"),
            count,
            [token.decode("ascii", errors="replace") for token in tokens],
        )
        if command == b"a":
            event_data = b"n".join(tokens).decode("ascii", errors="ignore")
            self._handle_param_update_ack(event_data, count)
            return

        values: List[float] = []
        for token in tokens:
            try:
                values.append(float(token) / 100.0)
            except Exception:
                self.logger.warning("Skipping non-numeric frame token: %r", token)
        # Single-value frames are BLE fragmentation artefacts, not real packets.
        if len(values) <= 1:
            return
        self._publish_rt_data(values)

    def _flush_partial_frame(self):
        """Accept a trailing frame whose final delimiter was dropped by BLE.

        A 21-byte ACK is truncated to the 20-byte minimum ATT payload, so the
        final ``n`` never arrives. If no bytes complete the frame within the
        flush window, treat the last token as terminated.
        """
        buf = self._rx_buffer
        if not buf:
            return
        parsed = self._parse_frame(buf, allow_unterminated_tail=True)
        if isinstance(parsed, tuple):
            end, command, count, tokens = parsed
            self._rx_buffer = buf[end:]
            self._process_frame(command, count, tokens)
            self._drain_frames()
            if self._rx_buffer:
                self._frame_flush_timer.start(self._frame_flush_ms)
            return
        self.logger.warning(
            "Dropping incomplete frame after flush timeout: %r", bytes(buf[:32])
        )
        self._rx_buffer = b""

    def reset_for_new_ble_session(self):
        """Drop handshake/name/controller parse state before data from the next link arrives."""
        self._handshake = False
        self._collecting_names = True
        self._names.clear()
        self._controllers.clear()
        self._controller_params.clear()
        self._temp_params.clear()
        self._controllers_done = False
        self._controller_matrix.clear()
        self._rows_68.clear()
        self._rows_36.clear()
        self._rows_38.clear()
        self._collecting_handshake_payload = False
        self._handshake_payload_buf = ""
        self._rx_buffer = b""
        self._reset_stream()
    
    def print_trial_summary(self):
        """Print comprehensive trial summary with all statistics."""
        if not self.DEBUG_DATA_RATE or self._total_packets_received == 0:
            return
        
        if self._monitoring_start_time is not None:
            total_time = time.monotonic() - self._monitoring_start_time
        else:
            total_time = self._total_time_elapsed
        
        if total_time == 0:
            return
        
        overall_hz = self._total_packets_received / total_time
        total_bytes = self._total_packets_received * (self._bytes_received / max(1, self._data_packet_count))
        total_kb = total_bytes / 1024.0
        total_mb = total_kb / 1024.0
        
        # Calculate expected packets and loss
        expected_total = self._expected_hz * total_time
        total_lost = max(0, expected_total - self._total_packets_received)
        loss_pct = (total_lost / expected_total * 100) if expected_total > 0 else 0
        
        self.logger.debug("\n" + "="*60)
        self.logger.debug("           TRIAL DATA COLLECTION SUMMARY")
        self.logger.debug("="*60)
        self.logger.debug(f"  Duration: {total_time:.1f} seconds ({total_time/60:.1f} minutes)")
        self.logger.debug(f"  Total packets: {self._total_packets_received}")
        self.logger.debug(f"  Average rate: {overall_hz:.1f} Hz")
        self.logger.debug(f"  Expected rate: {self._expected_hz} Hz")
        self.logger.debug(f"  Total data: {total_bytes:.0f} bytes ({total_kb:.2f} KB / {total_mb:.2f} MB)")
        self.logger.debug(f"  Packet loss: {loss_pct:.2f}% (~{int(total_lost)} packets)")
        
        if self._max_consecutive_drops > 0:
            self.logger.debug(f"  Worst drop streak: {self._max_consecutive_drops} consecutive packets")
        
        # BLE reliability indicators
        if self._max_consecutive_drops == 0 and loss_pct < 1:
            reliability = "EXCELLENT - No significant interruptions"
        elif self._max_consecutive_drops < 5 and loss_pct < 5:
            reliability = "GOOD - Minor interruptions"
        elif self._max_consecutive_drops < 10 and loss_pct < 10:
            reliability = "FAIR - Noticeable interruptions"
        else:
            reliability = "POOR - Frequent interruptions"
        
        if overall_hz >= self._expected_hz * 0.95:
            quality = "EXCELLENT - Minimal packet loss"
        elif overall_hz >= self._expected_hz * 0.90:
            quality = "GOOD - Acceptable performance"
        elif overall_hz >= self._expected_hz * 0.80:
            quality = "FAIR - Some packet loss detected"
        else:
            quality = "POOR - Significant packet loss"
        
        self.logger.debug(f"  Data quality: {quality}")
        self.logger.debug(f"  BLE reliability: {reliability}")
        self.logger.debug("="*60 + "\n")
    
    def reset_monitoring(self):
        """Reset data rate monitoring statistics (call when starting new trial)."""
        self._data_packet_count = 0
        self._bytes_received = 0
        self._packet_intervals.clear()
        self._total_packets_received = 0
        self._total_time_elapsed = 0.0
        self._monitoring_start_time = None
        self._last_packet_time = None
        self._dropped_packet_count = 0
        self._ble_chunk_count = 0
        self._ble_chunk_sizes.clear()
        self._consecutive_drops = 0
        self._max_consecutive_drops = 0
        self._stall_count = 0
        self._last_stall_time = None

    @QtCore.Slot()
    def _print_data_rate(self):
        """Print comprehensive data collection statistics every second."""
        # The reset runs on every tick, including the seconds that produced no
        # completed packet, so the diagnostic lists hold a constant amount of
        # memory whether or not verbose reporting is enabled.
        try:
            if not self.DEBUG_DATA_RATE:
                return
            self._report_data_rate()
        finally:
            self._reset_per_second_counters()

    def _reset_per_second_counters(self):
        """Drop the per-second diagnostic samples collected since the last tick."""
        self._data_packet_count = 0
        self._bytes_received = 0
        self._packet_intervals.clear()
        self._dropped_packet_count = 0
        self._ble_chunk_count = 0
        self._ble_chunk_sizes.clear()
        self._stall_count = 0

    def _report_data_rate(self):
        """Log one second of collection statistics; the caller resets the counters."""
        if self._data_packet_count > 0:
            hz = self._data_packet_count
            bytes_per_sec = self._bytes_received
            kb_per_sec = bytes_per_sec / 1024.0
            
            # Calculate average packet size
            avg_packet_size = bytes_per_sec / self._data_packet_count if self._data_packet_count > 0 else 0
            
            # Update expected Hz based on observed rate (after first few seconds)
            if self._total_packets_received > 100 and hz > 10:
                self._expected_hz = hz
            
            # Calculate overall statistics
            if self._monitoring_start_time is not None:
                self._total_time_elapsed = (
                    time.monotonic() - self._monitoring_start_time
                )
                overall_hz = self._total_packets_received / self._total_time_elapsed if self._total_time_elapsed > 0 else 0
            else:
                overall_hz = 0
            
            # Calculate packet loss percentage (two methods)
            # Method 1: Based on timing gap detection
            gap_detected_drops = self._dropped_packet_count
            gap_loss_pct = 0.0
            if (self._data_packet_count + gap_detected_drops) > 0:
                gap_loss_pct = (gap_detected_drops / (self._data_packet_count + gap_detected_drops)) * 100
            
            # Method 2: Based on expected Hz vs actual count
            expected_packets_hz = self._expected_hz  # Expected in this 1-second window
            hz_detected_drops = max(0, expected_packets_hz - self._data_packet_count)
            hz_loss_pct = 0.0
            if expected_packets_hz > 0:
                hz_loss_pct = (hz_detected_drops / expected_packets_hz) * 100
            
            # Use Hz-based method as primary (more reliable)
            packet_loss_pct = hz_loss_pct
            actual_drops = hz_detected_drops
            
            # Calculate timing statistics
            if len(self._packet_intervals) > 0:
                avg_interval = sum(self._packet_intervals) / len(self._packet_intervals)
                min_interval = min(self._packet_intervals)
                max_interval = max(self._packet_intervals)
                jitter = max_interval - min_interval
                
                # Connection quality indicator
                quality = "EXCELLENT"
                if jitter > 50 or packet_loss_pct > 10:
                    quality = "POOR"
                elif jitter > 20 or packet_loss_pct > 5:
                    quality = "FAIR"
                elif jitter > 10 or packet_loss_pct > 2:
                    quality = "GOOD"
                
                # Calculate BLE chunk statistics
                avg_chunk_size = sum(self._ble_chunk_sizes) / len(self._ble_chunk_sizes) if self._ble_chunk_sizes else 0
                min_chunk = min(self._ble_chunk_sizes) if self._ble_chunk_sizes else 0
                max_chunk = max(self._ble_chunk_sizes) if self._ble_chunk_sizes else 0
                chunks_per_packet = self._ble_chunk_count / self._data_packet_count if self._data_packet_count > 0 else 0
                
                self.logger.debug("[RtBridge] ===== Data Rate Stats =====")
                self.logger.debug(f"  Rate: {hz} Hz (expected: {self._expected_hz} Hz)")
                self.logger.debug(f"  Throughput: {bytes_per_sec} bytes/sec ({kb_per_sec:.2f} KB/s)")
                self.logger.debug(f"  Avg packet: {avg_packet_size:.1f} bytes")
                self.logger.debug(f"  Timing: avg={avg_interval:.1f}ms, min={min_interval:.1f}ms, max={max_interval:.1f}ms")
                self.logger.debug(f"  Jitter: {jitter:.1f}ms")
                self.logger.debug(f"  Packet loss: {packet_loss_pct:.1f}% (~{int(actual_drops)} packets this second)")
                if gap_detected_drops > 0:
                    self.logger.debug(f"  Gap-detected drops: {gap_detected_drops} (from timing analysis)")
                if self._max_consecutive_drops > 0:
                    self.logger.debug(f"  Max consecutive drops: {self._max_consecutive_drops} packets")
                self.logger.debug(f"  BLE chunks: {self._ble_chunk_count} (avg {chunks_per_packet:.1f} per packet)")
                self.logger.debug(f"  BLE chunk size: avg={avg_chunk_size:.1f}B, min={min_chunk}B, max={max_chunk}B")
                if self._stall_count > 0:
                    self.logger.debug(f"  Data stalls: {self._stall_count} (>100ms gaps)")
                self.logger.debug(f"  Overall: {self._total_packets_received} packets in {self._total_time_elapsed:.1f}s (avg {overall_hz:.1f} Hz)")
                self.logger.debug(f"  Quality: {quality}")
                self.logger.debug("================================")
            else:
                self.logger.debug(
                    f"[RtBridge] Data rate: {hz} Hz | {bytes_per_sec} bytes/sec ({kb_per_sec:.2f} KB/s) | "
                    f"Avg packet: {avg_packet_size:.1f} bytes"
                )
            
            # The per-second counters are reset by the caller's finally block.
