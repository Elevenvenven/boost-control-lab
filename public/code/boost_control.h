#ifndef BOOST_CONTROL_H
#define BOOST_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BOOST_STATE_OFF = 0,
    BOOST_STATE_SOFT_START,
    BOOST_STATE_RUNNING,
    BOOST_STATE_FAULT
} BoostState;

typedef enum
{
    BOOST_FAULT_NONE = 0,
    BOOST_FAULT_CONFIG = (1U << 0),
    BOOST_FAULT_BAD_SAMPLE = (1U << 1),
    BOOST_FAULT_INPUT_UVLO = (1U << 2),
    BOOST_FAULT_OUTPUT_OVP = (1U << 3),
    BOOST_FAULT_OVERCURRENT = (1U << 4),
    BOOST_FAULT_OVERTEMPERATURE = (1U << 5),
    BOOST_FAULT_EXTERNAL = (1U << 6)
} BoostFault;

typedef struct
{
    float control_period_s;
    float target_output_v;
    float input_uvlo_v;
    float output_ovp_v;
    float current_limit_a;
    float temperature_limit_c;
    float minimum_duty;
    float maximum_duty;
    float duty_slew_rate_per_s;
    float soft_start_time_s;
    float proportional_gain;
    float integral_gain_per_s;
} BoostConfig;

typedef struct
{
    float input_v;
    float output_v;
    float inductor_current_a;
    float temperature_c;
} BoostMeasurements;

typedef struct
{
    BoostState state;
    uint32_t faults;
    float duty;
    float reference_v;
    bool pwm_enabled;
} BoostOutput;

typedef struct
{
    BoostConfig config;
    BoostState state;
    uint32_t faults;
    float duty;
    float reference_v;
    float integrator;
} BoostController;

BoostConfig Boost_ConfigSafeDefaults(void);
bool Boost_ConfigIsValid(const BoostConfig *config);

void Boost_Init(BoostController *controller, const BoostConfig *config);
bool Boost_Start(BoostController *controller);
void Boost_Stop(BoostController *controller);
void Boost_Trip(BoostController *controller, BoostFault fault);
bool Boost_ClearFaults(BoostController *controller);

BoostOutput Boost_Step(BoostController *controller,
                       const BoostMeasurements *measurements);
BoostOutput Boost_GetOutput(const BoostController *controller);

#endif
