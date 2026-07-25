#include "boost_control.h"

#include <math.h>
#include <stddef.h>

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float move_towards(float current, float target, float maximum_delta)
{
    if (target > current + maximum_delta)
    {
        return current + maximum_delta;
    }
    if (target < current - maximum_delta)
    {
        return current - maximum_delta;
    }
    return target;
}

static bool measurements_are_valid(const BoostMeasurements *measurements)
{
    return measurements != NULL &&
           isfinite(measurements->input_v) &&
           isfinite(measurements->output_v) &&
           isfinite(measurements->inductor_current_a) &&
           isfinite(measurements->temperature_c) &&
           measurements->input_v >= 0.0f &&
           measurements->output_v >= 0.0f;
}

static uint32_t detect_faults(const BoostController *controller,
                              const BoostMeasurements *measurements)
{
    uint32_t faults = BOOST_FAULT_NONE;

    if (!measurements_are_valid(measurements))
    {
        return BOOST_FAULT_BAD_SAMPLE;
    }

    if (measurements->input_v < controller->config.input_uvlo_v)
    {
        faults |= BOOST_FAULT_INPUT_UVLO;
    }
    if (measurements->output_v > controller->config.output_ovp_v)
    {
        faults |= BOOST_FAULT_OUTPUT_OVP;
    }
    if (fabsf(measurements->inductor_current_a) >
        controller->config.current_limit_a)
    {
        faults |= BOOST_FAULT_OVERCURRENT;
    }
    if (measurements->temperature_c >
        controller->config.temperature_limit_c)
    {
        faults |= BOOST_FAULT_OVERTEMPERATURE;
    }

    return faults;
}

static void latch_faults(BoostController *controller, uint32_t faults)
{
    controller->faults |= faults;
    controller->state = BOOST_STATE_FAULT;
    controller->duty = 0.0f;
    controller->integrator = 0.0f;
}

BoostConfig Boost_ConfigSafeDefaults(void)
{
    const BoostConfig config = {
        .control_period_s = 0.0f,
        .target_output_v = 0.0f,
        .input_uvlo_v = 0.0f,
        .output_ovp_v = 0.0f,
        .current_limit_a = 0.0f,
        .temperature_limit_c = 0.0f,
        .minimum_duty = 0.0f,
        .maximum_duty = 0.0f,
        .duty_slew_rate_per_s = 0.0f,
        .soft_start_time_s = 0.0f,
        .proportional_gain = 0.0f,
        .integral_gain_per_s = 0.0f,
    };
    return config;
}

bool Boost_ConfigIsValid(const BoostConfig *config)
{
    return config != NULL &&
           isfinite(config->control_period_s) &&
           isfinite(config->target_output_v) &&
           isfinite(config->input_uvlo_v) &&
           isfinite(config->output_ovp_v) &&
           isfinite(config->current_limit_a) &&
           isfinite(config->temperature_limit_c) &&
           isfinite(config->minimum_duty) &&
           isfinite(config->maximum_duty) &&
           isfinite(config->duty_slew_rate_per_s) &&
           isfinite(config->soft_start_time_s) &&
           isfinite(config->proportional_gain) &&
           isfinite(config->integral_gain_per_s) &&
           config->control_period_s > 0.0f &&
           config->control_period_s <= 0.01f &&
           config->target_output_v > config->input_uvlo_v &&
           config->input_uvlo_v > 0.0f &&
           config->output_ovp_v > config->target_output_v &&
           config->current_limit_a > 0.0f &&
           config->temperature_limit_c > 0.0f &&
           config->minimum_duty >= 0.0f &&
           config->maximum_duty > config->minimum_duty &&
           config->maximum_duty < 0.95f &&
           config->duty_slew_rate_per_s > 0.0f &&
           config->soft_start_time_s > 0.0f &&
           config->proportional_gain >= 0.0f &&
           config->integral_gain_per_s >= 0.0f;
}

void Boost_Init(BoostController *controller, const BoostConfig *config)
{
    if (controller == NULL)
    {
        return;
    }

    controller->config = (config != NULL) ? *config : Boost_ConfigSafeDefaults();
    controller->state = BOOST_STATE_OFF;
    controller->faults = BOOST_FAULT_NONE;
    controller->duty = 0.0f;
    controller->reference_v = 0.0f;
    controller->integrator = 0.0f;

    if (!Boost_ConfigIsValid(&controller->config))
    {
        latch_faults(controller, BOOST_FAULT_CONFIG);
    }
}

bool Boost_Start(BoostController *controller)
{
    if (controller == NULL ||
        controller->faults != BOOST_FAULT_NONE ||
        !Boost_ConfigIsValid(&controller->config))
    {
        return false;
    }

    controller->state = BOOST_STATE_SOFT_START;
    controller->duty = 0.0f;
    controller->reference_v = 0.0f;
    controller->integrator = 0.0f;
    return true;
}

void Boost_Stop(BoostController *controller)
{
    if (controller == NULL)
    {
        return;
    }

    controller->state = BOOST_STATE_OFF;
    controller->duty = 0.0f;
    controller->reference_v = 0.0f;
    controller->integrator = 0.0f;
}

void Boost_Trip(BoostController *controller, BoostFault fault)
{
    if (controller == NULL)
    {
        return;
    }

    latch_faults(controller, (fault == BOOST_FAULT_NONE) ?
                                      BOOST_FAULT_EXTERNAL :
                                      (uint32_t)fault);
}

bool Boost_ClearFaults(BoostController *controller)
{
    if (controller == NULL || controller->state != BOOST_STATE_FAULT)
    {
        return false;
    }

    controller->faults = BOOST_FAULT_NONE;
    Boost_Stop(controller);
    return true;
}

BoostOutput Boost_Step(BoostController *controller,
                       const BoostMeasurements *measurements)
{
    if (controller == NULL)
    {
        const BoostOutput invalid_output = {
            .state = BOOST_STATE_FAULT,
            .faults = BOOST_FAULT_CONFIG,
            .duty = 0.0f,
            .reference_v = 0.0f,
            .pwm_enabled = false,
        };
        return invalid_output;
    }

    if (controller->state == BOOST_STATE_OFF ||
        controller->state == BOOST_STATE_FAULT)
    {
        return Boost_GetOutput(controller);
    }

    const uint32_t new_faults = detect_faults(controller, measurements);
    if (new_faults != BOOST_FAULT_NONE)
    {
        latch_faults(controller, new_faults);
        return Boost_GetOutput(controller);
    }

    const float period = controller->config.control_period_s;

    if (controller->state == BOOST_STATE_SOFT_START)
    {
        const float reference_step =
            controller->config.target_output_v * period /
            controller->config.soft_start_time_s;
        controller->reference_v =
            clampf(controller->reference_v + reference_step,
                   0.0f,
                   controller->config.target_output_v);

        if (controller->reference_v >= controller->config.target_output_v)
        {
            controller->state = BOOST_STATE_RUNNING;
        }
    }
    else
    {
        controller->reference_v = controller->config.target_output_v;
    }

    float feed_forward = 0.0f;
    if (controller->reference_v > measurements->input_v &&
        controller->reference_v > 0.0f)
    {
        feed_forward =
            1.0f - measurements->input_v / controller->reference_v;
    }

    const float error = controller->reference_v - measurements->output_v;
    const float proportional =
        controller->config.proportional_gain * error;
    const float unsaturated =
        feed_forward + proportional + controller->integrator;
    const float saturated =
        clampf(unsaturated,
               controller->config.minimum_duty,
               controller->config.maximum_duty);

    const bool can_integrate =
        unsaturated == saturated ||
        (unsaturated > saturated && error < 0.0f) ||
        (unsaturated < saturated && error > 0.0f);

    if (can_integrate)
    {
        controller->integrator +=
            controller->config.integral_gain_per_s * error * period;
        controller->integrator =
            clampf(controller->integrator,
                   -controller->config.maximum_duty,
                   controller->config.maximum_duty);
    }

    const float duty_step =
        controller->config.duty_slew_rate_per_s * period;
    controller->duty =
        move_towards(controller->duty, saturated, duty_step);

    return Boost_GetOutput(controller);
}

BoostOutput Boost_GetOutput(const BoostController *controller)
{
    if (controller == NULL)
    {
        const BoostOutput invalid_output = {
            .state = BOOST_STATE_FAULT,
            .faults = BOOST_FAULT_CONFIG,
            .duty = 0.0f,
            .reference_v = 0.0f,
            .pwm_enabled = false,
        };
        return invalid_output;
    }

    const bool active =
        (controller->state == BOOST_STATE_SOFT_START ||
         controller->state == BOOST_STATE_RUNNING) &&
        controller->faults == BOOST_FAULT_NONE;

    const BoostOutput output = {
        .state = controller->state,
        .faults = controller->faults,
        .duty = active ? controller->duty : 0.0f,
        .reference_v = controller->reference_v,
        .pwm_enabled = active,
    };
    return output;
}
