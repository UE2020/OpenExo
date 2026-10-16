#ifndef ANKLE_PERTURBATION_SUPPORT_H
#define ANKLE_PERTURBATION_SUPPORT_H
#include "ExoData.h"

namespace ankle_perturbation
{
    inline bool calibrating(const SideData& side)
    {
        return side.do_calibration_toe_fsr || side.do_calibration_heel_fsr ||
            side.do_calibration_refinement_toe_fsr || side.do_calibration_refinement_heel_fsr ||
            side.reset_fsr_calibration || side.ankle.calibrate_torque_sensor ||
            side.ankle.calibrate_angle_sensor || side.ankle.motor.do_zero;
    }

    inline uint8_t rejection_reason(ExoData* data, uint8_t selected_side)
    {
        if (selected_side != 1 && selected_side != 2) return malformed_request;
        const uint16_t status = data->get_status();
        // Actual calibration flags are authoritative when legacy status latches.
        const bool running = status == status_defs::messages::trial_on ||
            status == status_defs::messages::fsr_calibration || status == status_defs::messages::fsr_refinement;
        if (!running) return trial_not_running;
        if (data->estop) return estop_active;
        if (data->user_paused) return user_is_paused;
        if (data->error_code != 0) return device_error;
        if (calibrating(data->left_side) || calibrating(data->right_side)) return calibration_in_progress;
        const SideData& side = selected_side == 1 ? data->left_side : data->right_side;
        const JointData& ankle = side.ankle;
        const uint8_t controller = ankle.controller.controller;
        if (controller != (uint8_t)config_defs::ankle_controllers::pjmc &&
            controller != (uint8_t)config_defs::ankle_controllers::pjmc_plus) return unsupported_controller;
        if (!side.is_used || !ankle.is_used) return ankle_unused;
        if (!ankle.motor.enabled) return motor_disabled;
        if (ankle.motor.timeout_count >= ankle.motor.timeout_count_max) return motor_timeout;
        if (!isfinite(ankle.torque_reading) || !isfinite(ankle.torque_offset_reading) ||
            !isfinite(side.toe_fsr) || !isfinite(side.heel_fsr)) return invalid_sensor;
        if (ankle.torque_offset_reading == 0) return torque_uncalibrated;
        return no_rejection;
    }

    inline bool allowed(ExoData* data, uint8_t selected_side)
    {
        return rejection_reason(data, selected_side) == no_rejection;
    }

    inline float offset(ExoData* data, JointData* joint)
    {
        if (joint == &data->left_side.ankle) return data->perturbation.offset(1);
        if (joint == &data->right_side.ankle) return data->perturbation.offset(2);
        return 0; // PJMC can also control other joints: never perturb those.
    }
}
#endif
