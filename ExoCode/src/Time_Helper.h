#ifndef TIME_HELPER_H
#define TIME_HELPER_H

#include <stdint.h>
#include <vector>

/* Class to help track the time of code execution. The class uses a singleton design pattern. 
 * 
 Example usage: 
 *      Time_Helper* my_time_helper_singleton = get_instance();
 *      static const float my_context = my_time_helper_singleton->generate_new_context();
 *      static my_delta_time;
 *      my_delta_time = my_time_helper_singleton->tick(my_context);
 *
 * The above code gets a reference to the singleton and uses it to generate a persisent context (see below). 
 * 'my_delta_time' will be filled with the time between the calls to tick in millis (first call will return 0).
 * 
 * When you are done with a context, clean it up using 'destroy_context'. If you would like to use microseconds,
 * the default value of 'use_micros' in the constructor should be true. 
 * 
 * If 'tick()' is continuosly returning 0, you are passing an invalid context. 
 *
 */

typedef struct {
    float context;
    uint32_t old_time = 0;
    int k_index = -1;
    bool initialized = false;
} ticker_t;

class Time_Helper
{
    public:
        Time_Helper(bool use_micros=true);
        static Time_Helper* get_instance();

        float peek(float context);
        float tick(float context);

        float generate_new_context();
        void destroy_context(float context);
    
    private:
        ticker_t* _ticker_from_context(float context);

        std::vector<ticker_t> tickers;
        uint32_t _next_context = 1;

        bool _k_use_micros;
};

#endif
