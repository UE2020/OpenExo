#include "Time_Helper.h"
#include "Logger.h"
#include <Arduino.h>

/* Public */
Time_Helper::Time_Helper(bool use_micros)
{
    _k_use_micros = use_micros;
}

Time_Helper* Time_Helper::get_instance()
{
    static Time_Helper instance;
    return &instance;
}

float Time_Helper::peek(float context)
{
    ticker_t* ticker = _ticker_from_context(context);

    if (ticker->k_index < 0 || !ticker->initialized) {
        return 0;
    }

    const uint32_t new_time = _k_use_micros ? micros() : millis();
    return static_cast<float>(new_time - ticker->old_time);
}

float Time_Helper::tick(float context)
{
    const uint32_t new_time = _k_use_micros ? micros() : millis();
    ticker_t* ticker = _ticker_from_context(context);

    if (ticker->k_index < 0) {
        return 0;
    }

    if (!ticker->initialized) {
        ticker->old_time = new_time;
        ticker->initialized = true;
        return 0;
    }

    const uint32_t elapsed = new_time - ticker->old_time;
    ticker->old_time = new_time;
    return static_cast<float>(elapsed);
}

float Time_Helper::generate_new_context()
{
    const float found = static_cast<float>(_next_context++);

    ticker_t new_ticker;
    new_ticker.context = found;
    new_ticker.k_index = static_cast<int>(tickers.size());
    tickers.push_back(new_ticker);

    return found;
}

void Time_Helper::destroy_context(float context)
{
    ticker_t* ticker_to_destroy = _ticker_from_context(context);
    if (ticker_to_destroy->k_index < 0) {
        return;
    }

    const int index = ticker_to_destroy->k_index;
    tickers.erase(tickers.begin() + index);
    for (int i = index; i < static_cast<int>(tickers.size()); i++) {
        tickers[i].k_index = i;
    }
}

/* Private */
ticker_t* Time_Helper::_ticker_from_context(float context)
{
    static ticker_t err_ticker = {
        .context = 0,
        .old_time = 0,
        .k_index = -1,
        .initialized = false
    };
    for (size_t i = 0; i < tickers.size(); i++) {
        if (context == tickers[i].context) {
            return &tickers[i];
        }
    }
    return &err_ticker;
}
