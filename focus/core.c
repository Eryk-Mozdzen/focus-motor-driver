#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "focus/api.h"
#include "focus/biquad.h"
#include "focus/config.h"
#include "focus/debug.h"
#include "focus/fsm.h"
#include "focus/math.h"
#include "focus/pid.h"
#include "focus/port.h"
#include "focus/smo.h"

#define FOCUS_API_STATE_NONE  0
#define FOCUS_API_STATE_PANIC 1

#define FOCUS_FSM_STATES_NUM      32
#define FOCUS_FSM_TRANSITIONS_NUM 64

#define FOCUS_CURRENT_CALIBRATED(measurement, core, phase)                                         \
    ((core)->calibration.data.current.scale[(phase)] *                                             \
     ((measurement) - (core)->calibration.data.current.offset[(phase)]))

#define FOCUS_MECHANICAL_TO_ELECTRICAL(mech)                                                       \
    (focus_math_angle_wrap(FOCUS_CONFIG_MOTOR_POLE_PAIRS_NUM * (mech)))

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
#define FOCUS_ENCODER_TO_MECHANICAL(count)                                                         \
    (focus_math_angle_wrap((((uint32_t)(count)) % FOCUS_CONFIG_ENCODER_CPR) <=                     \
                           (FOCUS_CONFIG_ENCODER_CPR / 2))                                         \
         ? ((FOCUS_2PI / FOCUS_CONFIG_ENCODER_CPR) *                                               \
            (((uint32_t)(count)) % FOCUS_CONFIG_ENCODER_CPR))                                      \
         : (((FOCUS_2PI / FOCUS_CONFIG_ENCODER_CPR) *                                              \
             (((uint32_t)(count)) % FOCUS_CONFIG_ENCODER_CPR)) -                                   \
            FOCUS_2PI))

#define FOCUS_ENCODER_TO_ELECTRICAL(count)                                                         \
    (FOCUS_MECHANICAL_TO_ELECTRICAL(FOCUS_ENCODER_TO_MECHANICAL(count)))

#define FOCUS_MECHANICAL_TO_ENCODER(theta)                                                         \
    (((uint32_t)(((focus_math_angle_wrap(theta)) >= 0.f)                                           \
                     ? ((FOCUS_CONFIG_ENCODER_CPR / FOCUS_2PI) * focus_math_angle_wrap(theta))     \
                     : (((FOCUS_CONFIG_ENCODER_CPR / FOCUS_2PI) * focus_math_angle_wrap(theta)) +  \
                        FOCUS_CONFIG_ENCODER_CPR))) %                                              \
     FOCUS_CONFIG_ENCODER_CPR)

#ifdef FOCUS_CONFIG_ENCODER_TYPE_ABI
#define FOCUS_ENCODER_ALIGNED(count, core)                                                         \
    (((((uint32_t)(count)) + (2 * FOCUS_CONFIG_ENCODER_CPR)) -                                     \
      (core)->calibration.data.encoder.align_offset - (core)->encoder.index_offset) %              \
     FOCUS_CONFIG_ENCODER_CPR)
#else
#define FOCUS_ENCODER_ALIGNED(count, core)                                                         \
    (((((uint32_t)(count)) + FOCUS_CONFIG_ENCODER_CPR) -                                           \
      (core)->calibration.data.encoder.align_offset) %                                             \
     FOCUS_CONFIG_ENCODER_CPR)
#endif

#ifdef FOCUS_CONFIG_ENCODER_ECCENTRICITY_ENABLE
#ifdef FOCUS_CONFIG_ENCODER_TYPE_AB
#define FOCUS_CONFIG_ENCODER_ECCENTRICITY_LOOKUP(core) ((core)->encoder.eccentricity_lookup)
#else
#define FOCUS_CONFIG_ENCODER_ECCENTRICITY_LOOKUP(core)                                             \
    ((core)->calibration.data.encoder.eccentricity_lookup)
#endif

#define FOCUS_ENCODER_CALIBRATED(count, core)                                                      \
    (((FOCUS_ENCODER_ALIGNED((count), (core)) + FOCUS_CONFIG_ENCODER_CPR) -                        \
      FOCUS_CONFIG_ENCODER_ECCENTRICITY_LOOKUP((core))[FOCUS_ENCODER_ALIGNED((count), (core))]) %  \
     FOCUS_CONFIG_ENCODER_CPR)
#else
#define FOCUS_ENCODER_CALIBRATED(count, core) FOCUS_ENCODER_ALIGNED((count), (core))
#endif
#endif

typedef struct {
    uint32_t index;
    void *user;

    focus_pid_t pid_d;
    focus_pid_t pid_q;

    float iq_setpoint;
#ifdef FOCUS_CONFIG_ENCODER_ENABLE
    volatile float position;
#endif
    volatile float velocity;

    float current_state_enter_time;
    volatile focus_port_sample_t sample;

    focus_api_state_t state_requested;
    focus_api_state_t state_current;
    focus_api_state_ended_t state_ended_callback;
    focus_fsm_t fsm;
    focus_fsm_state_t fsm_states[FOCUS_FSM_STATES_NUM];
    focus_fsm_transition_t fsm_transitions[FOCUS_FSM_TRANSITIONS_NUM];

    focus_biquad_t i_dq_filter[2];

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
    struct {
        volatile float position_prev;
#ifdef FOCUS_CONFIG_ENCODER_TYPE_AB
        int16_t eccentricity_lookup[FOCUS_CONFIG_ENCODER_CPR];
#endif
#ifdef FOCUS_CONFIG_ENCODER_TYPE_ABI
        uint32_t index_offset;
#endif
        focus_biquad_t velocity_filter;
    } encoder;
#endif

#ifdef FOCUS_CONFIG_SENSORLESS_ENABLE
    struct {
        volatile float ramp_open_loop;
        focus_smo_t smo;
    } sensorless;
#endif

    struct {
        union {
            struct {
                volatile uint32_t num;
                volatile uint32_t state;
                volatile float time;
                volatile float buffer_u[FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES];
                volatile float buffer_v[FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES];
                volatile float buffer_w[FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES];
            } current;

            struct {
                volatile uint32_t num;
                volatile float time;
                volatile float buffer[FOCUS_CONFIG_MOTOR_CALIBRATION_SAMPLES];
            } motor;

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
            struct {
                volatile float open_loop;
                volatile uint32_t lut_prev;
#ifdef FOCUS_CONFIG_ENCODER_TYPE_ABI
                volatile uint32_t index_offset;
                volatile bool index_occurred;
#endif
            } encoder;
#endif
        } context;

        volatile focus_api_calibration_t data;
    } calibration;
} focus_core_t;

static bool requested_idle(const void *user) {
    const focus_core_t *core = user;
    return (core->state_requested == FOCUS_API_STATE_IDLE);
}

static bool requested_calibrate_current(const void *user) {
    const focus_core_t *core = user;
    return (core->state_requested == FOCUS_API_STATE_CALIBRATE_CURRENT);
}

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
#ifndef FOCUS_CONFIG_ENCODER_TYPE_AB
static bool requested_calibrate_encoder(const void *user) {
    const focus_core_t *core = user;
    return (core->state_requested == FOCUS_API_STATE_CALIBRATE_ENCODER);
}
#endif
#endif

static bool requested_calibrate_motor(const void *user) {
    const focus_core_t *core = user;
    return (core->state_requested == FOCUS_API_STATE_CALIBRATE_MOTOR);
}

static bool requested_running(const void *user) {
    const focus_core_t *core = user;
    return (core->state_requested == FOCUS_API_STATE_RUNNING);
}

static bool core_panicked(const void *user) {
    const focus_core_t *core = user;
    return (core->state_requested == FOCUS_API_STATE_PANIC);
}

static void idle_enter(void *user) {
    focus_core_t *core = user;

    focus_port_shutdown(core->index, core->user);

    if((core->state_ended_callback != NULL) && (core->state_current != FOCUS_API_STATE_IDLE)) {
        core->state_ended_callback(core->index, core->state_current, core->user);
    }

    core->state_current = FOCUS_API_STATE_IDLE;
}

static void idle_exit(void *user) {
    focus_core_t *core = user;

    focus_port_start(core->index, core->user);
}

static void calibration_current_offset_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_CALIBRATE_CURRENT;

    core->calibration.context.current.time = 0;
    core->calibration.context.current.num = 0;
}

static void calibration_current_offset_execute(void *user) {
    focus_core_t *core = user;

    if((core->calibration.context.current.time >=
        (FOCUS_CONFIG_SAMPLING_FREQUENCY / FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES)) &&
       (core->calibration.context.current.num < FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES)) {
        core->calibration.context.current.buffer_u[core->calibration.context.current.num] =
            core->sample.current_u;
        core->calibration.context.current.buffer_v[core->calibration.context.current.num] =
            core->sample.current_v;
        core->calibration.context.current.buffer_w[core->calibration.context.current.num] =
            core->sample.current_w;
        core->calibration.context.current.num++;
        core->calibration.context.current.time = 0;
    }

    core->calibration.context.current.time++;

    const focus_port_control_t control = {
        .duty_cycle_u = 0.5f,
        .duty_cycle_v = 0.5f,
        .duty_cycle_w = 0.5f,
    };
    focus_port_control(core->index, &control, core->user);

    FOCUS_DEBUG_BUFFER_APPEND(core->sample.voltage_vbus, core->sample.current_u,
                              core->sample.current_v, core->sample.current_w, 0, 0, 0, 0, 0, 0, 0,
                              control.duty_cycle_u, control.duty_cycle_v, control.duty_cycle_w);
}

static void calibration_current_offset_exit(void *user) {
    focus_core_t *core = user;

    float u_bias = 0;
    float v_bias = 0;
    float w_bias = 0;
    for(uint32_t i = 0; i < FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES; i++) {
        u_bias += core->calibration.context.current.buffer_u[i];
        v_bias += core->calibration.context.current.buffer_v[i];
        w_bias += core->calibration.context.current.buffer_w[i];
    }
    u_bias /= FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES;
    v_bias /= FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES;
    w_bias /= FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES;

    core->calibration.data.current.offset[0] = u_bias;
    core->calibration.data.current.offset[1] = v_bias;
    core->calibration.data.current.offset[2] = w_bias;

    focus_api_calibration_update(core->index);
}

static bool calibration_current_offset_ended(const void *user) {
    const focus_core_t *core = user;
    return (core->calibration.context.current.num >= FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES);
}

static void calibration_current_scale_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_CALIBRATE_CURRENT;

    core->calibration.context.current.time = 0;
    core->calibration.context.current.num = 0;
    core->calibration.context.current.state = 0;
}

static void calibration_current_scale_execute(void *user) {
    focus_core_t *core = user;

    const float period =
        FOCUS_CONFIG_CURRENT_CALIBRATION_TIME / FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES;
    const float inject = FOCUS_CONFIG_CURRENT_CALIBRATION_VOLTAGE / core->sample.voltage_vbus;

    focus_port_control_t control = {
        .duty_cycle_u = 0.5f,
        .duty_cycle_v = 0.5f,
        .duty_cycle_w = 0.5f,
    };

    switch(core->calibration.context.current.state) {
        case 0: {
            control.duty_cycle_u += inject;

            if(((core->calibration.context.current.num * period) <
                core->calibration.context.current.time) &&
               (core->calibration.context.current.num < FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES)) {
                core->calibration.context.current.buffer_u[core->calibration.context.current.num] =
                    core->sample.current_u;
                core->calibration.context.current.num++;
            }

            if(core->calibration.context.current.num >= FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES) {
                core->calibration.context.current.state = 1;
                core->calibration.context.current.num = 0;
                core->calibration.context.current.time = 0;
            }
        } break;
        case 1: {
            control.duty_cycle_v += inject;

            if(((core->calibration.context.current.num * period) <
                core->calibration.context.current.time) &&
               (core->calibration.context.current.num < FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES)) {
                core->calibration.context.current.buffer_v[core->calibration.context.current.num] =
                    core->sample.current_v;
                core->calibration.context.current.num++;
            }

            if(core->calibration.context.current.num >= FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES) {
                core->calibration.context.current.state = 2;
                core->calibration.context.current.num = 0;
                core->calibration.context.current.time = 0;
            }
        } break;
        case 2: {
            control.duty_cycle_w += inject;

            if(((core->calibration.context.current.num * period) <
                core->calibration.context.current.time) &&
               (core->calibration.context.current.num < FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES)) {
                core->calibration.context.current.buffer_w[core->calibration.context.current.num] =
                    core->sample.current_w;
                core->calibration.context.current.num++;
            }
        } break;
    }

    focus_port_control(core->index, &control, core->user);

    core->calibration.context.current.time += FOCUS_CONFIG_SAMPLING_PERIOD;

    FOCUS_DEBUG_BUFFER_APPEND(core->sample.voltage_vbus, core->sample.current_u,
                              core->sample.current_v, core->sample.current_w, 0, 0, 0, 0, 0, 0, 0,
                              control.duty_cycle_u, control.duty_cycle_v, control.duty_cycle_w);
}

static void calibration_current_scale_exit(void *user) {
    focus_core_t *core = user;

    float u_amplitude = 0.f;
    float v_amplitude = 0.f;
    float w_amplitude = 0.f;
    for(uint32_t i = 0; i < FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES; i++) {
        u_amplitude += core->calibration.context.current.buffer_u[i];
        v_amplitude += core->calibration.context.current.buffer_v[i];
        w_amplitude += core->calibration.context.current.buffer_w[i];
    }
    u_amplitude /= FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES;
    v_amplitude /= FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES;
    w_amplitude /= FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES;

    const float mean_amplitude = (u_amplitude + v_amplitude + w_amplitude) / 3.f;

    core->calibration.data.current.scale[0] = mean_amplitude / u_amplitude;
    core->calibration.data.current.scale[1] = mean_amplitude / v_amplitude;
    core->calibration.data.current.scale[2] = mean_amplitude / w_amplitude;

    focus_api_calibration_update(core->index);
}

static bool calibration_current_scale_ended(const void *user) {
    const focus_core_t *core = user;
    return ((core->calibration.context.current.state == 2) &&
            (core->calibration.context.current.num >= FOCUS_CONFIG_CURRENT_CALIBRATION_SAMPLES));
}

static void calibration_motor_resistance_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_CALIBRATE_MOTOR;
    core->calibration.context.motor.num = 0;
    memset((float *)core->calibration.context.motor.buffer, 0,
           sizeof(core->calibration.context.motor.buffer));
}

static void calibration_motor_resistance_execute(void *user) {
    focus_core_t *core = user;

    const float i_uvw[3] = {
        FOCUS_CURRENT_CALIBRATED(core->sample.current_u, core, 0),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_v, core, 1),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_w, core, 2),
    };

    float i_ab[2];
    focus_math_clark_transform(i_uvw, i_ab);
    float i_dq[2];
    focus_math_park_transform(i_ab, 0, i_dq);

    if(core->calibration.context.motor.num < FOCUS_CONFIG_MOTOR_CALIBRATION_SAMPLES) {
        core->calibration.context.motor.buffer[core->calibration.context.motor.num] = i_dq[0];
        core->calibration.context.motor.num++;
    }

    const float u_dq[2] = {
        FOCUS_CONFIG_MOTOR_CALIBRATION_RESISTANCE_VOLTAGE,
        0,
    };
    float u_dq_clamped[2];
    focus_math_clamp_vector(u_dq, core->sample.voltage_vbus / FOCUS_SQRT3, u_dq_clamped);
    float u_ab[2];
    focus_math_inverse_park_transform(u_dq_clamped, 0, u_ab);
    float duty_cycle_uvw[3];
    focus_math_svpwm(u_ab, core->sample.voltage_vbus, duty_cycle_uvw);

    const focus_port_control_t control = {
        .duty_cycle_u = duty_cycle_uvw[0],
        .duty_cycle_v = duty_cycle_uvw[1],
        .duty_cycle_w = duty_cycle_uvw[2],
    };
    focus_port_control(core->index, &control, core->user);

    if(core->calibration.context.motor.num == 1) {
        _focus_debug_buffer_index = 0;
    }

    FOCUS_DEBUG_BUFFER_APPEND(core->sample.voltage_vbus, i_uvw[0], i_uvw[1], i_uvw[2], i_dq[0],
                              i_dq[1], 0, u_dq[0], u_dq[1], 0, 0, control.duty_cycle_u,
                              control.duty_cycle_v, control.duty_cycle_w);
}

static void calibration_motor_resistance_exit(void *user) {
    focus_core_t *core = user;

    const float ud = FOCUS_CONFIG_MOTOR_CALIBRATION_RESISTANCE_VOLTAGE;

    float id = 0;
    for(uint32_t i = 0; i < core->calibration.context.motor.num; i++) {
        id += core->calibration.context.motor.buffer[i];
    }
    id /= core->calibration.context.motor.num;

    core->calibration.data.motor.rs = ud / id;

    focus_api_calibration_update(core->index);
}

static bool calibration_motor_resistance_ended(const void *user) {
    const focus_core_t *core = user;
    return (core->calibration.context.motor.num >= FOCUS_CONFIG_MOTOR_CALIBRATION_SAMPLES);
}

static void calibration_motor_inductance_d_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_CALIBRATE_MOTOR;

    core->calibration.context.motor.num = 0;
    core->calibration.context.motor.time = 0;
}

static void calibration_motor_inductance_d_execute(void *user) {
    focus_core_t *core = user;

    const float i_uvw[3] = {
        FOCUS_CURRENT_CALIBRATED(core->sample.current_u, core, 0),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_v, core, 1),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_w, core, 2),
    };

    float i_ab[2];
    focus_math_clark_transform(i_uvw, i_ab);
    float i_dq[2];
    focus_math_park_transform(i_ab, 0, i_dq);

    if(core->calibration.context.motor.num < FOCUS_CONFIG_MOTOR_CALIBRATION_SAMPLES) {
        core->calibration.context.motor.buffer[core->calibration.context.motor.num] = i_dq[0];
        core->calibration.context.motor.num++;
    }

    const float ud_amplitude = FOCUS_CONFIG_MOTOR_CALIBRATION_INDUCTANCE_VOLTAGE;
    const float w = FOCUS_2PI * FOCUS_CONFIG_MOTOR_CALIBRATION_INDUCTANCE_FREQUENCY;

    const float u_dq[2] = {
        ud_amplitude * sinf(w * core->calibration.context.motor.time),
        0,
    };
    float u_dq_clamped[2];
    focus_math_clamp_vector(u_dq, core->sample.voltage_vbus / FOCUS_SQRT3, u_dq_clamped);
    float u_ab[2];
    focus_math_inverse_park_transform(u_dq_clamped, 0, u_ab);
    float duty_cycle_uvw[3];
    focus_math_svpwm(u_ab, core->sample.voltage_vbus, duty_cycle_uvw);

    const focus_port_control_t control = {
        .duty_cycle_u = duty_cycle_uvw[0],
        .duty_cycle_v = duty_cycle_uvw[1],
        .duty_cycle_w = duty_cycle_uvw[2],
    };
    focus_port_control(core->index, &control, core->user);

    if(core->calibration.context.motor.num == 1) {
        _focus_debug_buffer_index = 0;
    }

    FOCUS_DEBUG_BUFFER_APPEND(core->sample.voltage_vbus, i_uvw[0], i_uvw[1], i_uvw[2], i_dq[0],
                              i_dq[1], 0, u_dq[0], u_dq[1], 0, 0, control.duty_cycle_u,
                              control.duty_cycle_v, control.duty_cycle_w);

    core->calibration.context.motor.time += FOCUS_CONFIG_SAMPLING_PERIOD;
}

static void calibration_motor_inductance_d_exit(void *user) {
    focus_core_t *core = user;

    const float ud_amplitude = FOCUS_CONFIG_MOTOR_CALIBRATION_INDUCTANCE_VOLTAGE;
    const float w = FOCUS_2PI * FOCUS_CONFIG_MOTOR_CALIBRATION_INDUCTANCE_FREQUENCY;

    float id_amplitude;
    float id_phase;
    focus_math_dft((float *)core->calibration.context.motor.buffer,
                   FOCUS_CONFIG_MOTOR_CALIBRATION_SAMPLES, FOCUS_CONFIG_SAMPLING_PERIOD,
                   FOCUS_CONFIG_MOTOR_CALIBRATION_INDUCTANCE_FREQUENCY, &id_amplitude, &id_phase,
                   NULL);

    const float z = ud_amplitude / id_amplitude;

    core->calibration.data.motor.ld = z * sinf(fabs(id_phase)) / w;

    focus_api_calibration_update(core->index);
}

static bool calibration_motor_inductance_d_ended(const void *user) {
    const focus_core_t *core = user;
    return (core->calibration.context.motor.num >= FOCUS_CONFIG_MOTOR_CALIBRATION_SAMPLES);
}

static void calibration_motor_inductance_q_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_CALIBRATE_MOTOR;

    core->calibration.context.motor.num = 0;
    core->calibration.context.motor.time = 0;
}

static void calibration_motor_inductance_q_execute(void *user) {
    focus_core_t *core = user;

    const float i_uvw[3] = {
        FOCUS_CURRENT_CALIBRATED(core->sample.current_u, core, 0),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_v, core, 1),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_w, core, 2),
    };

    float i_ab[2];
    focus_math_clark_transform(i_uvw, i_ab);
    float i_dq[2];
    focus_math_park_transform(i_ab, 0, i_dq);

    if(core->calibration.context.motor.num < FOCUS_CONFIG_MOTOR_CALIBRATION_SAMPLES) {
        core->calibration.context.motor.buffer[core->calibration.context.motor.num] = i_dq[1];
        core->calibration.context.motor.num++;
    }

    const float uq_amplitude = FOCUS_CONFIG_MOTOR_CALIBRATION_INDUCTANCE_VOLTAGE;
    const float w = FOCUS_2PI * FOCUS_CONFIG_MOTOR_CALIBRATION_INDUCTANCE_FREQUENCY;

    const float u_dq[2] = {
        0,
        uq_amplitude * sinf(w * core->calibration.context.motor.time),
    };
    float u_dq_clamped[2];
    focus_math_clamp_vector(u_dq, core->sample.voltage_vbus / FOCUS_SQRT3, u_dq_clamped);
    float u_ab[2];
    focus_math_inverse_park_transform(u_dq_clamped, 0, u_ab);
    float duty_cycle_uvw[3];
    focus_math_svpwm(u_ab, core->sample.voltage_vbus, duty_cycle_uvw);

    const focus_port_control_t control = {
        .duty_cycle_u = duty_cycle_uvw[0],
        .duty_cycle_v = duty_cycle_uvw[1],
        .duty_cycle_w = duty_cycle_uvw[2],
    };
    focus_port_control(core->index, &control, core->user);

    if(core->calibration.context.motor.num == 1) {
        _focus_debug_buffer_index = 0;
    }

    FOCUS_DEBUG_BUFFER_APPEND(core->sample.voltage_vbus, i_uvw[0], i_uvw[1], i_uvw[2], i_dq[0],
                              i_dq[1], 0, u_dq[0], u_dq[1], 0, 0, control.duty_cycle_u,
                              control.duty_cycle_v, control.duty_cycle_w);

    core->calibration.context.motor.time += FOCUS_CONFIG_SAMPLING_PERIOD;
}

static void calibration_motor_inductance_q_exit(void *user) {
    focus_core_t *core = user;

    const float uq_amplitude = FOCUS_CONFIG_MOTOR_CALIBRATION_INDUCTANCE_VOLTAGE;
    const float w = FOCUS_2PI * FOCUS_CONFIG_MOTOR_CALIBRATION_INDUCTANCE_FREQUENCY;

    float iq_amplitude;
    float iq_phase;
    focus_math_dft((float *)core->calibration.context.motor.buffer,
                   FOCUS_CONFIG_MOTOR_CALIBRATION_SAMPLES, FOCUS_CONFIG_SAMPLING_PERIOD,
                   FOCUS_CONFIG_MOTOR_CALIBRATION_INDUCTANCE_FREQUENCY, &iq_amplitude, &iq_phase,
                   NULL);

    const float z = uq_amplitude / iq_amplitude;

    core->calibration.data.motor.lq = z * sinf(fabs(iq_phase)) / w;

    focus_api_calibration_update(core->index);
}

static bool calibration_motor_inductance_q_ended(const void *user) {
    const focus_core_t *core = user;
    return (core->calibration.context.motor.num >= FOCUS_CONFIG_MOTOR_CALIBRATION_SAMPLES);
}

#ifdef FOCUS_CONFIG_MOTOR_CALIBRATION_KV_ENABLE
static void calibration_motor_kv_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_CALIBRATE_MOTOR;
    core->calibration.context.motor.time = 0;
    core->calibration.context.motor.num = 0;

    focus_pid_start(&core->pid_d);
    focus_pid_start(&core->pid_q);

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
    core->velocity = 0;
    core->encoder.position_prev =
        FOCUS_ENCODER_TO_MECHANICAL(FOCUS_ENCODER_CALIBRATED(core->sample.encoder_count, core));

    focus_biquad_design_lowpass(&core->encoder.velocity_filter,
                                FOCUS_CONFIG_ENCODER_VELOCITY_BANDWIDTH,
                                FOCUS_CONFIG_SAMPLING_FREQUENCY);
    focus_biquad_start(&core->encoder.velocity_filter);
#endif
}

static void calibration_motor_kv_execute(void *user) {
    focus_core_t *core = user;

    core->calibration.context.motor.time += FOCUS_CONFIG_SAMPLING_PERIOD;

    const float i_uvw[3] = {
        FOCUS_CURRENT_CALIBRATED(core->sample.current_u, core, 0),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_v, core, 1),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_w, core, 2),
    };

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
    const float theta_e =
        FOCUS_ENCODER_TO_ELECTRICAL(FOCUS_ENCODER_CALIBRATED(core->sample.encoder_count, core));
#endif

#ifdef FOCUS_CONFIG_SENSORLESS_ENABLE
    const float theta_e = FOCUS_SMO_GET_ELECTRICAL_POSITION(&core->sensorless.smo);
#endif

    float i_ab[2];
    focus_math_clark_transform(i_uvw, i_ab);
    float i_dq[2];
    focus_math_park_transform(i_ab, theta_e, i_dq);

    const float u_dq[2] = {
        focus_pid_calculate(&core->pid_d, 0.f, i_dq[0], FOCUS_CONFIG_SAMPLING_PERIOD),
        focus_pid_calculate(&core->pid_q, FOCUS_CONFIG_MOTOR_CALIBRATION_KV_CURRENT, i_dq[1],
                            FOCUS_CONFIG_SAMPLING_PERIOD),
    };

    const float u_dq_length = sqrtf((u_dq[0] * u_dq[0]) + (u_dq[1] * u_dq[1]));
    const float u_dq_length_max = core->sample.voltage_vbus / FOCUS_SQRT3;

    if(u_dq_length > u_dq_length_max) {
        const float u_dq_length_overflow = u_dq_length - u_dq_length_max;
        focus_pid_antiwindup(&core->pid_d, u_dq_length_overflow, FOCUS_CONFIG_SAMPLING_PERIOD);
        focus_pid_antiwindup(&core->pid_q, u_dq_length_overflow, FOCUS_CONFIG_SAMPLING_PERIOD);
    }

    float u_dq_clamped[2];
    focus_math_clamp_vector(u_dq, u_dq_length_max, u_dq_clamped);
    float u_ab[2];
    focus_math_inverse_park_transform(u_dq_clamped, theta_e, u_ab);
    float duty_cycle_uvw[3];
    focus_math_svpwm(u_ab, core->sample.voltage_vbus, duty_cycle_uvw);

    const focus_port_control_t control = {
        .duty_cycle_u = duty_cycle_uvw[0],
        .duty_cycle_v = duty_cycle_uvw[1],
        .duty_cycle_w = duty_cycle_uvw[2],
    };
    focus_port_control(core->index, &control, core->user);

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
    const float position_curr =
        FOCUS_ENCODER_TO_MECHANICAL(FOCUS_ENCODER_CALIBRATED(core->sample.encoder_count, core));
    const float velocity_curr = focus_math_angle_sub(position_curr, core->encoder.position_prev) /
                                FOCUS_CONFIG_SAMPLING_PERIOD;

    core->position = position_curr;
    core->velocity = focus_biquad_update(&core->encoder.velocity_filter, velocity_curr);

    core->encoder.position_prev = position_curr;

    FOCUS_DEBUG_BUFFER_APPEND(core->sample.voltage_vbus, i_uvw[0], i_uvw[1], i_uvw[2], i_dq[0],
                              i_dq[1], FOCUS_CONFIG_MOTOR_CALIBRATION_KV_CURRENT, u_dq[0], u_dq[1],
                              theta_e, core->position, control.duty_cycle_u, control.duty_cycle_v,
                              control.duty_cycle_w);
#endif

#ifdef FOCUS_CONFIG_SENSORLESS_ENABLE
    focus_smo_update(&core->sensorless.smo, u_ab, i_ab);

    core->velocity = FOCUS_SMO_GET_ELECTRICAL_VELOCITY(&core->sensorless.smo) /
                     FOCUS_CONFIG_MOTOR_POLE_PAIRS_NUM;

    FOCUS_DEBUG_BUFFER_APPEND(core->sample.voltage_vbus, i_uvw[0], i_uvw[1], i_uvw[2], i_dq[0],
                              i_dq[1], FOCUS_CONFIG_MOTOR_CALIBRATION_KV_CURRENT, u_dq[0], u_dq[1],
                              theta_e, 0, control.duty_cycle_u, control.duty_cycle_v,
                              control.duty_cycle_w);
#endif

    if((core->calibration.context.motor.time > FOCUS_CONFIG_MOTOR_CALIBRATION_KV_SETTLE) &&
       (core->calibration.context.motor.num < FOCUS_CONFIG_MOTOR_CALIBRATION_SAMPLES)) {
        const float u_dq_len =
            sqrtf((u_dq_clamped[0] * u_dq_clamped[0]) + (u_dq_clamped[1] * u_dq_clamped[1]));

        const float kv = core->velocity / u_dq_len;

        core->calibration.context.motor.buffer[core->calibration.context.motor.num] = kv;
        core->calibration.context.motor.num++;
    }
}

static void calibration_motor_kv_exit(void *user) {
    focus_core_t *core = user;

    float sum = 0;
    for(uint32_t i = 0; i < core->calibration.context.motor.num; i++) {
        sum += core->calibration.context.motor.buffer[i];
    }

    core->calibration.data.motor.kv = sum / core->calibration.context.motor.num;

    focus_api_calibration_update(core->index);
}

static bool calibration_motor_kv_ended(const void *user) {
    const focus_core_t *core = user;
    return (core->calibration.context.motor.num >= FOCUS_CONFIG_MOTOR_CALIBRATION_SAMPLES);
}
#endif

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
#ifdef FOCUS_CONFIG_ENCODER_TYPE_ABI
static void encoder_index_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_CALIBRATE_ENCODER;
    core->calibration.context.encoder.open_loop = 0;
    core->calibration.context.encoder.index_offset = 0;
    core->calibration.context.encoder.index_occurred = false;
}

static void encoder_index_execute(void *user) {
    focus_core_t *core = user;

    core->calibration.context.encoder.open_loop +=
        (FOCUS_CONFIG_SAMPLING_PERIOD * FOCUS_CONFIG_ENCODER_INDEX_SEARCH_VELOCITY);
    core->calibration.context.encoder.open_loop =
        focus_math_angle_wrap(core->calibration.context.encoder.open_loop);

    const float theta = FOCUS_MECHANICAL_TO_ELECTRICAL(core->calibration.context.encoder.open_loop);

    const float u_dq[2] = {
        FOCUS_CONFIG_ENCODER_INDEX_SEARCH_VOLTAGE,
        0,
    };
    float u_dq_clamped[2];
    focus_math_clamp_vector(u_dq, core->sample.voltage_vbus / FOCUS_SQRT3, u_dq_clamped);
    float u_ab[2];
    focus_math_inverse_park_transform(u_dq_clamped, theta, u_ab);
    float duty_cycle_uvw[3];
    focus_math_svpwm(u_ab, core->sample.voltage_vbus, duty_cycle_uvw);

    const focus_port_control_t control = {
        .duty_cycle_u = duty_cycle_uvw[0],
        .duty_cycle_v = duty_cycle_uvw[1],
        .duty_cycle_w = duty_cycle_uvw[2],
    };
    focus_port_control(core->index, &control, core->user);

    const float now = focus_port_timebase(core->user);
    if(((now - core->current_state_enter_time) <
        (0.1f * (FOCUS_2PI / FOCUS_CONFIG_ENCODER_INDEX_SEARCH_VELOCITY)))) {
        core->calibration.context.encoder.index_occurred = false;
    }

    core->position =
        FOCUS_ENCODER_TO_MECHANICAL(FOCUS_ENCODER_CALIBRATED(core->sample.encoder_count, core));
}

static void encoder_index_exit(void *user) {
    focus_core_t *core = user;

    core->encoder.index_offset = core->calibration.context.encoder.index_offset;
}

static bool encoder_index_ended(const void *user) {
    const focus_core_t *core = user;
    const float now = focus_port_timebase(core->user);
    return (core->calibration.context.encoder.index_occurred &&
            ((now - core->current_state_enter_time) >
             (0.1f * (FOCUS_2PI / FOCUS_CONFIG_ENCODER_INDEX_SEARCH_VELOCITY))));
}

static bool encoder_index_timeout(const void *user) {
    const focus_core_t *core = user;
    const float now = focus_port_timebase(core->user);
    return ((now - core->current_state_enter_time) >
            (1.2f * (FOCUS_2PI / FOCUS_CONFIG_ENCODER_INDEX_SEARCH_VELOCITY)));
}
#endif

static void encoder_align_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_CALIBRATE_ENCODER;
    core->calibration.context.encoder.open_loop = 0;
}

static void encoder_align_execute(void *user) {
    focus_core_t *core = user;

    const float u_dq[2] = {
        FOCUS_CONFIG_ENCODER_ALIGN_VOLTAGE,
        0,
    };
    float u_dq_clamped[2];
    focus_math_clamp_vector(u_dq, core->sample.voltage_vbus / FOCUS_SQRT3, u_dq_clamped);
    float u_ab[2];
    focus_math_inverse_park_transform(u_dq_clamped, 0, u_ab);
    float duty_cycle_uvw[3];
    focus_math_svpwm(u_ab, core->sample.voltage_vbus, duty_cycle_uvw);

    const focus_port_control_t control = {
        .duty_cycle_u = duty_cycle_uvw[0],
        .duty_cycle_v = duty_cycle_uvw[1],
        .duty_cycle_w = duty_cycle_uvw[2],
    };
    focus_port_control(core->index, &control, core->user);

    core->calibration.data.encoder.align_offset = core->sample.encoder_count;

    core->position =
        FOCUS_ENCODER_TO_MECHANICAL(FOCUS_ENCODER_CALIBRATED(core->sample.encoder_count, core));
}

static bool encoder_align_ended(const void *user) {
    const focus_core_t *core = user;
    const float now = focus_port_timebase(core->user);
    return ((now - core->current_state_enter_time) > FOCUS_CONFIG_ENCODER_ALIGN_TIME);
}

#ifdef FOCUS_CONFIG_ENCODER_ECCENTRICITY_ENABLE
static void encoder_eccentricity_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_CALIBRATE_ENCODER;
    core->calibration.context.encoder.open_loop = 0;
    core->calibration.context.encoder.lut_prev = 0;
    memset((int16_t *)FOCUS_CONFIG_ENCODER_ECCENTRICITY_LOOKUP(core), 0,
           sizeof(FOCUS_CONFIG_ENCODER_ECCENTRICITY_LOOKUP(core)));
}

static void encoder_eccentricity_execute(void *user) {
    focus_core_t *core = user;

    core->calibration.context.encoder.open_loop +=
        (FOCUS_CONFIG_SAMPLING_PERIOD * FOCUS_CONFIG_ENCODER_ECCENTRICITY_VELOCITY);
    core->calibration.context.encoder.open_loop =
        focus_math_angle_wrap(core->calibration.context.encoder.open_loop);

    const uint32_t count = FOCUS_MECHANICAL_TO_ENCODER(core->calibration.context.encoder.open_loop);
    const float theta = FOCUS_MECHANICAL_TO_ELECTRICAL(core->calibration.context.encoder.open_loop);

    const float u_dq[2] = {
        FOCUS_CONFIG_ENCODER_ECCENTRICITY_VOLTAGE,
        0,
    };
    float u_dq_clamped[2];
    focus_math_clamp_vector(u_dq, core->sample.voltage_vbus / FOCUS_SQRT3, u_dq_clamped);
    float u_ab[2];
    focus_math_inverse_park_transform(u_dq_clamped, theta, u_ab);
    float duty_cycle_uvw[3];
    focus_math_svpwm(u_ab, core->sample.voltage_vbus, duty_cycle_uvw);

    const focus_port_control_t control = {
        .duty_cycle_u = duty_cycle_uvw[0],
        .duty_cycle_v = duty_cycle_uvw[1],
        .duty_cycle_w = duty_cycle_uvw[2],
    };
    focus_port_control(core->index, &control, core->user);

    const uint32_t enc_prev = core->calibration.context.encoder.lut_prev;
    const uint32_t enc_curr = FOCUS_ENCODER_ALIGNED(core->sample.encoder_count, core);

    const int32_t diff_prev = FOCUS_CONFIG_ENCODER_ECCENTRICITY_LOOKUP(core)[enc_prev];
    const int32_t diff_curr = ((int32_t)enc_curr) - ((int32_t)count);

    const uint32_t enc_curr_not_wrapped =
        (enc_curr < enc_prev) ? enc_curr + FOCUS_CONFIG_ENCODER_CPR : enc_curr;

    for(uint32_t i = enc_prev; i <= enc_curr_not_wrapped; i++) {
        FOCUS_CONFIG_ENCODER_ECCENTRICITY_LOOKUP(core)
        [i % FOCUS_CONFIG_ENCODER_CPR] =
            focus_math_lerp(enc_prev, diff_prev, enc_curr_not_wrapped, diff_curr, i);
    }

    core->calibration.context.encoder.lut_prev = enc_curr;

#ifdef FOCUS_CONFIG_ENCODER_TYPE_ABI
    const float now = focus_port_timebase(core->user);
    if(((now - core->current_state_enter_time) <
        (0.5f * (FOCUS_2PI / FOCUS_CONFIG_ENCODER_INDEX_SEARCH_VELOCITY)))) {
        core->calibration.context.encoder.index_occurred = false;
    }
#endif

    core->position =
        FOCUS_ENCODER_TO_MECHANICAL(FOCUS_ENCODER_CALIBRATED(core->sample.encoder_count, core));
}

static bool encoder_eccentricity_ended(const void *user) {
    const focus_core_t *core = user;
    const float now = focus_port_timebase(core->user);
    return (((now - core->current_state_enter_time) >
             (1.2f * (FOCUS_2PI / FOCUS_CONFIG_ENCODER_ECCENTRICITY_VELOCITY))));
}
#endif
#endif

#ifdef FOCUS_CONFIG_SENSORLESS_ENABLE
static void running_sensorless_align_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_RUNNING;
}

static void running_sensorless_align_execute(void *user) {
    focus_core_t *core = user;

    const float u_dq[2] = {
        FOCUS_CONFIG_SENSORLESS_ALIGN_VOLTAGE,
        0,
    };
    float u_dq_clamped[2];
    focus_math_clamp_vector(u_dq, core->sample.voltage_vbus / FOCUS_SQRT3, u_dq_clamped);
    float u_ab[2];
    focus_math_inverse_park_transform(u_dq_clamped, 0, u_ab);
    float duty_cycle_uvw[3];
    focus_math_svpwm(u_ab, core->sample.voltage_vbus, duty_cycle_uvw);

    const focus_port_control_t control = {
        .duty_cycle_u = duty_cycle_uvw[0],
        .duty_cycle_v = duty_cycle_uvw[1],
        .duty_cycle_w = duty_cycle_uvw[2],
    };
    focus_port_control(core->index, &control, core->user);
}

static bool running_sensorless_align_ended(const void *user) {
    const focus_core_t *core = user;
    const float now = focus_port_timebase(core->user);
    return ((now - core->current_state_enter_time) > FOCUS_CONFIG_SENSORLESS_ALIGN_TIME);
}

static void running_sensorless_ramp_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_RUNNING;
    core->sensorless.ramp_open_loop = 0;

    focus_smo_init(&core->sensorless.smo, core->calibration.data.motor.rs,
                   core->calibration.data.motor.ld, core->calibration.data.motor.lq);
}

static void running_sensorless_ramp_execute(void *user) {
    focus_core_t *core = user;

    const float i_uvw[3] = {
        FOCUS_CURRENT_CALIBRATED(core->sample.current_u, core, 0),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_v, core, 1),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_w, core, 2),
    };

    float i_ab[2];
    focus_math_clark_transform(i_uvw, i_ab);

    const float theta_e = FOCUS_MECHANICAL_TO_ELECTRICAL(core->sensorless.ramp_open_loop);

    const float u_dq[2] = {
        FOCUS_CONFIG_SENSORLESS_RAMP_VOLTAGE,
        0,
    };
    float u_dq_clamped[2];
    focus_math_clamp_vector(u_dq, core->sample.voltage_vbus / FOCUS_SQRT3, u_dq_clamped);
    float u_ab[2];
    focus_math_inverse_park_transform(u_dq_clamped, theta_e, u_ab);
    float duty_cycle_uvw[3];
    focus_math_svpwm(u_ab, core->sample.voltage_vbus, duty_cycle_uvw);

    const focus_port_control_t control = {
        .duty_cycle_u = duty_cycle_uvw[0],
        .duty_cycle_v = duty_cycle_uvw[1],
        .duty_cycle_w = duty_cycle_uvw[2],
    };
    focus_port_control(core->index, &control, core->user);

    const float now = focus_port_timebase(core->user);
    const float elapsed = now - core->current_state_enter_time;
    const float velocity = focus_math_sign(core->iq_setpoint) *
                           FOCUS_CONFIG_SENSORLESS_RAMP_VELOCITY *
                           (1.f - expf(-FOCUS_CONFIG_SENSORLESS_RAMP_LAMBDA * elapsed));

    core->sensorless.ramp_open_loop += (FOCUS_2PI * velocity * FOCUS_CONFIG_SAMPLING_PERIOD);
    core->sensorless.ramp_open_loop = focus_math_angle_wrap(core->sensorless.ramp_open_loop);

    focus_smo_update(&core->sensorless.smo, u_ab, i_ab);

    core->velocity = FOCUS_SMO_GET_ELECTRICAL_VELOCITY(&core->sensorless.smo) /
                     FOCUS_CONFIG_MOTOR_POLE_PAIRS_NUM;
}

static bool running_sensorless_ramp_ended(const void *user) {
    const focus_core_t *core = user;
    const float now = focus_port_timebase(core->user);
    return ((now - core->current_state_enter_time) > FOCUS_CONFIG_SENSORLESS_RAMP_TIME);
}
#endif

static void running_enter(void *user) {
    focus_core_t *core = user;
    core->current_state_enter_time = focus_port_timebase(core->user);
    core->state_current = FOCUS_API_STATE_RUNNING;
    core->iq_setpoint = 0;

    focus_pid_start(&core->pid_d);
    focus_pid_start(&core->pid_q);

    focus_biquad_design_lowpass(&core->i_dq_filter[0], FOCUS_CONFIG_FOC_FF_BANDWIDTH,
                                FOCUS_CONFIG_SAMPLING_FREQUENCY);
    focus_biquad_design_lowpass(&core->i_dq_filter[1], FOCUS_CONFIG_FOC_FF_BANDWIDTH,
                                FOCUS_CONFIG_SAMPLING_FREQUENCY);

    focus_biquad_start(&core->i_dq_filter[0]);
    focus_biquad_start(&core->i_dq_filter[1]);

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
    core->velocity = 0;
    core->encoder.position_prev =
        FOCUS_ENCODER_TO_MECHANICAL(FOCUS_ENCODER_CALIBRATED(core->sample.encoder_count, core));

    focus_biquad_design_lowpass(&core->encoder.velocity_filter,
                                FOCUS_CONFIG_ENCODER_VELOCITY_BANDWIDTH,
                                FOCUS_CONFIG_SAMPLING_FREQUENCY);
    focus_biquad_start(&core->encoder.velocity_filter);
#endif
}

static void running_execute(void *user) {
    focus_core_t *core = user;

    const float i_uvw[3] = {
        FOCUS_CURRENT_CALIBRATED(core->sample.current_u, core, 0),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_v, core, 1),
        FOCUS_CURRENT_CALIBRATED(core->sample.current_w, core, 2),
    };

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
    const float theta_e =
        FOCUS_ENCODER_TO_ELECTRICAL(FOCUS_ENCODER_CALIBRATED(core->sample.encoder_count, core));
#endif

#ifdef FOCUS_CONFIG_SENSORLESS_ENABLE
    const float theta_e = FOCUS_SMO_GET_ELECTRICAL_POSITION(&core->sensorless.smo);
#endif

    const float omega_e = core->velocity * FOCUS_CONFIG_MOTOR_POLE_PAIRS_NUM;
    const float omega_m = core->velocity;

#ifdef FOCUS_CONFIG_MOTOR_CALIBRATION_KV_ENABLE
    const float ke = 1.f / (FOCUS_SQRT3 * cores[motor].calibration.data.motor.kv);
#else
    const float ke = 30.f / (FOCUS_PI * FOCUS_SQRT3 * FOCUS_CONFIG_MOTOR_KV);
#endif

    float i_ab[2];
    focus_math_clark_transform(i_uvw, i_ab);
    float i_dq[2];
    focus_math_park_transform(i_ab, theta_e, i_dq);

    const float i_dq_filtered[2] = {
        focus_biquad_update(&core->i_dq_filter[0], i_dq[0]),
        focus_biquad_update(&core->i_dq_filter[1], i_dq[1]),
    };

    const float u_dq_ff[2] = {
        -(omega_e * core->calibration.data.motor.lq * i_dq_filtered[1]),
        +(omega_e * core->calibration.data.motor.ld * i_dq_filtered[0]) +
            (core->calibration.data.motor.rs * i_dq_filtered[1]) + (ke * omega_m),
    };

    const float u_dq_desired[2] = {
        focus_pid_calculate(&core->pid_d, 0.f, i_dq[0], FOCUS_CONFIG_SAMPLING_PERIOD),
        focus_pid_calculate(&core->pid_q, core->iq_setpoint, i_dq[1], FOCUS_CONFIG_SAMPLING_PERIOD),
    };

    const float u_dq[2] = {
        u_dq_ff[0] + u_dq_desired[0],
        u_dq_ff[1] + u_dq_desired[1],
    };

    const float u_dq_length = sqrtf((u_dq[0] * u_dq[0]) + (u_dq[1] * u_dq[1]));
    const float u_dq_length_max = core->sample.voltage_vbus / FOCUS_SQRT3;

    if(u_dq_length > u_dq_length_max) {
        const float u_dq_length_overflow = u_dq_length - u_dq_length_max;
        focus_pid_antiwindup(&core->pid_d, u_dq_length_overflow, FOCUS_CONFIG_SAMPLING_PERIOD);
        focus_pid_antiwindup(&core->pid_q, u_dq_length_overflow, FOCUS_CONFIG_SAMPLING_PERIOD);
    }

    float u_dq_clamped[2];
    focus_math_clamp_vector(u_dq, u_dq_length_max, u_dq_clamped);
    float u_ab[2];
    focus_math_inverse_park_transform(u_dq_clamped, theta_e, u_ab);
    float duty_cycle_uvw[3];
    focus_math_svpwm(u_ab, core->sample.voltage_vbus, duty_cycle_uvw);

    const focus_port_control_t control = {
        .duty_cycle_u = duty_cycle_uvw[0],
        .duty_cycle_v = duty_cycle_uvw[1],
        .duty_cycle_w = duty_cycle_uvw[2],
    };
    focus_port_control(core->index, &control, core->user);

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
    const float position_curr =
        FOCUS_ENCODER_TO_MECHANICAL(FOCUS_ENCODER_CALIBRATED(core->sample.encoder_count, core));
    const float velocity_curr = focus_math_angle_sub(position_curr, core->encoder.position_prev) /
                                FOCUS_CONFIG_SAMPLING_PERIOD;

    core->position = position_curr;
    core->velocity = focus_biquad_update(&core->encoder.velocity_filter, velocity_curr);

    core->encoder.position_prev = position_curr;

    FOCUS_DEBUG_BUFFER_APPEND(core->sample.voltage_vbus, i_uvw[0], i_uvw[1], i_uvw[2], i_dq[0],
                              i_dq[1], core->iq_setpoint, u_dq[0], u_dq[1], theta_e, core->position,
                              control.duty_cycle_u, control.duty_cycle_v, control.duty_cycle_w);
#endif

#ifdef FOCUS_CONFIG_SENSORLESS_ENABLE
    focus_smo_update(&core->sensorless.smo, u_ab, i_ab);

    core->velocity = FOCUS_SMO_GET_ELECTRICAL_VELOCITY(&core->sensorless.smo) /
                     FOCUS_CONFIG_MOTOR_POLE_PAIRS_NUM;

    FOCUS_DEBUG_BUFFER_APPEND(core->sample.voltage_vbus, i_uvw[0], i_uvw[1], i_uvw[2], i_dq[0],
                              i_dq[1], core->iq_setpoint, u_dq[0], u_dq[1], theta_e, 0,
                              control.duty_cycle_u, control.duty_cycle_v, control.duty_cycle_w);
#endif
}

#ifdef FOCUS_CONFIG_SENSORLESS_ENABLE
static bool running_low_velocity(const void *user) {
    const focus_core_t *core = user;
    const float now = focus_port_timebase(core->user);
    const float omega_e = FOCUS_SMO_GET_ELECTRICAL_VELOCITY(&core->sensorless.smo);
    const float omega_m = omega_e / FOCUS_CONFIG_MOTOR_POLE_PAIRS_NUM;
    return ((fabs(omega_m) < FOCUS_CONFIG_SENSORLESS_VELOCITY_MINIMAL) &&
            ((now - core->current_state_enter_time) > FOCUS_CONFIG_SENSORLESS_VELOCITY_TIME));
}
#endif

static focus_core_t cores[FOCUS_CONFIG_MOTORS_NUM] = {0};

void focus_api_init(void *user) {
    for(uint32_t i = 0; i < FOCUS_CONFIG_MOTORS_NUM; i++) {
        cores[i].index = i;
        cores[i].user = user;

        cores[i].velocity = 0.f;

        cores[i].state_requested = FOCUS_API_STATE_NONE;
        cores[i].state_ended_callback = NULL;

        focus_fsm_init(&cores[i].fsm, cores[i].fsm_states, FOCUS_FSM_STATES_NUM,
                       cores[i].fsm_transitions, FOCUS_FSM_TRANSITIONS_NUM, &cores[i]);

        const focus_fsm_state_t *idle =
            focus_fsm_add_state(&cores[i].fsm, idle_enter, NULL, idle_exit);
        const focus_fsm_state_t *calibration_current_offset = focus_fsm_add_state(
            &cores[i].fsm, calibration_current_offset_enter, calibration_current_offset_execute,
            calibration_current_offset_exit);
        const focus_fsm_state_t *calibration_current_scale =
            focus_fsm_add_state(&cores[i].fsm, calibration_current_scale_enter,
                                calibration_current_scale_execute, calibration_current_scale_exit);
        const focus_fsm_state_t *calibration_motor_rs = focus_fsm_add_state(
            &cores[i].fsm, calibration_motor_resistance_enter, calibration_motor_resistance_execute,
            calibration_motor_resistance_exit);
        const focus_fsm_state_t *calibration_motor_ld = focus_fsm_add_state(
            &cores[i].fsm, calibration_motor_inductance_d_enter,
            calibration_motor_inductance_d_execute, calibration_motor_inductance_d_exit);
        const focus_fsm_state_t *calibration_motor_lq = focus_fsm_add_state(
            &cores[i].fsm, calibration_motor_inductance_q_enter,
            calibration_motor_inductance_q_execute, calibration_motor_inductance_q_exit);
#ifdef FOCUS_CONFIG_MOTOR_CALIBRATION_KV_ENABLE
        const focus_fsm_state_t *calibration_motor_kv =
            focus_fsm_add_state(&cores[i].fsm, calibration_motor_kv_enter,
                                calibration_motor_kv_execute, calibration_motor_kv_exit);
#endif
#ifdef FOCUS_CONFIG_ENCODER_ENABLE
#ifdef FOCUS_CONFIG_ENCODER_TYPE_ABI
        const focus_fsm_state_t *calibration_encoder_index = focus_fsm_add_state(
            &cores[i].fsm, encoder_index_enter, encoder_index_execute, encoder_index_exit);
#endif
#if defined(FOCUS_CONFIG_ENCODER_TYPE_ABI) || defined(FOCUS_CONFIG_ENCODER_TYPE_ABSOLUTE)
        const focus_fsm_state_t *calibration_encoder_align =
            focus_fsm_add_state(&cores[i].fsm, encoder_align_enter, encoder_align_execute, NULL);
#ifdef FOCUS_CONFIG_ENCODER_ECCENTRICITY_ENABLE
        const focus_fsm_state_t *calibration_encoder_eccentricity = focus_fsm_add_state(
            &cores[i].fsm, encoder_eccentricity_enter, encoder_eccentricity_execute, NULL);
#endif
#endif
#ifdef FOCUS_CONFIG_ENCODER_TYPE_AB
        const focus_fsm_state_t *running_encoder_align =
            focus_fsm_add_state(&cores[i].fsm, encoder_align_enter, encoder_align_execute, NULL);
#ifdef FOCUS_CONFIG_ENCODER_ECCENTRICITY_ENABLE
        const focus_fsm_state_t *running_encoder_eccentricity = focus_fsm_add_state(
            &cores[i].fsm, encoder_eccentricity_enter, encoder_eccentricity_execute, NULL);
#endif
#endif
#ifdef FOCUS_CONFIG_ENCODER_TYPE_ABI
        const focus_fsm_state_t *running_encoder_index = focus_fsm_add_state(
            &cores[i].fsm, encoder_index_enter, encoder_index_execute, encoder_index_exit);
#endif
#endif
#ifdef FOCUS_CONFIG_SENSORLESS_ENABLE
        const focus_fsm_state_t *running_sensorless_align = focus_fsm_add_state(
            &cores[i].fsm, running_sensorless_align_enter, running_sensorless_align_execute, NULL);
        const focus_fsm_state_t *running_sensorless_ramp = focus_fsm_add_state(
            &cores[i].fsm, running_sensorless_ramp_enter, running_sensorless_ramp_execute, NULL);
#endif
        const focus_fsm_state_t *running =
            focus_fsm_add_state(&cores[i].fsm, running_enter, running_execute, NULL);

        focus_fsm_add_transition(&cores[i].fsm, idle, calibration_current_offset,
                                 requested_calibrate_current);
        focus_fsm_add_transition(&cores[i].fsm, calibration_current_offset, idle, requested_idle);
        focus_fsm_add_transition(&cores[i].fsm, calibration_current_offset, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, calibration_current_offset,
                                 calibration_current_scale, calibration_current_offset_ended);
        focus_fsm_add_transition(&cores[i].fsm, calibration_current_scale, idle, requested_idle);
        focus_fsm_add_transition(&cores[i].fsm, calibration_current_scale, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, calibration_current_scale, idle,
                                 calibration_current_scale_ended);

        focus_fsm_add_transition(&cores[i].fsm, idle, calibration_motor_rs,
                                 requested_calibrate_motor);
        focus_fsm_add_transition(&cores[i].fsm, calibration_motor_rs, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, calibration_motor_rs, idle, requested_idle);
        focus_fsm_add_transition(&cores[i].fsm, calibration_motor_rs, calibration_motor_ld,
                                 calibration_motor_resistance_ended);
        focus_fsm_add_transition(&cores[i].fsm, calibration_motor_ld, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, calibration_motor_ld, idle, requested_idle);
        focus_fsm_add_transition(&cores[i].fsm, calibration_motor_ld, calibration_motor_lq,
                                 calibration_motor_inductance_d_ended);
        focus_fsm_add_transition(&cores[i].fsm, calibration_motor_lq, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, calibration_motor_lq, idle, requested_idle);
        focus_fsm_add_transition_begin(&cores[i].fsm, calibration_motor_lq,
                                       calibration_motor_inductance_q_ended);
#ifdef FOCUS_CONFIG_MOTOR_CALIBRATION_KV_ENABLE
#ifdef FOCUS_CONFIG_ENCODER_ENABLE
#ifdef FOCUS_CONFIG_ENCODER_TYPE_AB
        focus_fsm_add_transition_end(&cores[i].fsm, running_encoder_align);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_align, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_align, idle, requested_idle);
        focus_fsm_add_transition_begin(&cores[i].fsm, running_encoder_align, encoder_align_ended);
#ifdef FOCUS_CONFIG_ENCODER_ECCENTRICITY_ENABLE
        focus_fsm_add_transition_end(&cores[i].fsm, running_encoder_eccentricity);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_eccentricity, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_eccentricity, idle, requested_idle);
        focus_fsm_add_transition_begin(&cores[i].fsm, running_encoder_eccentricity,
                                       encoder_eccentricity_ended);
#endif
#endif
#ifdef FOCUS_CONFIG_ENCODER_TYPE_ABI
        focus_fsm_add_transition_end(&cores[i].fsm, running_encoder_index);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_index, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_index, idle, requested_idle);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_index, idle, encoder_index_timeout);
        focus_fsm_add_transition_begin(&cores[i].fsm, running_encoder_index, encoder_index_ended);
#endif
#endif
#ifdef FOCUS_CONFIG_SENSORLESS_ENABLE
        focus_fsm_add_transition_end(&cores[i].fsm, running_sensorless_align);
        focus_fsm_add_transition(&cores[i].fsm, running_sensorless_align, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, running_sensorless_align, idle, requested_idle);
        focus_fsm_add_transition(&cores[i].fsm, running_sensorless_align, running_sensorless_ramp,
                                 running_sensorless_align_ended);
        focus_fsm_add_transition(&cores[i].fsm, running_sensorless_ramp, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, running_sensorless_ramp, idle, requested_idle);
        focus_fsm_add_transition_begin(&cores[i].fsm, running_sensorless_ramp,
                                       running_sensorless_ramp_ended);
#endif
        focus_fsm_add_transition_end(&cores[i].fsm, calibration_motor_kv);
        focus_fsm_add_transition(&cores[i].fsm, calibration_motor_kv, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, calibration_motor_kv, idle, requested_idle);
        focus_fsm_add_transition_begin(&cores[i].fsm, calibration_motor_kv,
                                       calibration_motor_kv_ended);
#endif
        focus_fsm_add_transition_end(&cores[i].fsm, idle);

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
#if defined(FOCUS_CONFIG_ENCODER_TYPE_ABI) || defined(FOCUS_CONFIG_ENCODER_TYPE_ABSOLUTE)
        focus_fsm_add_transition(&cores[i].fsm, idle, calibration_encoder_index,
                                 requested_calibrate_encoder);
        focus_fsm_add_transition(&cores[i].fsm, calibration_encoder_index, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, calibration_encoder_index, idle, requested_idle);
        focus_fsm_add_transition(&cores[i].fsm, calibration_encoder_index, idle,
                                 encoder_index_timeout);
        focus_fsm_add_transition(&cores[i].fsm, calibration_encoder_index,
                                 calibration_encoder_align, encoder_index_ended);
        focus_fsm_add_transition(&cores[i].fsm, calibration_encoder_align, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, calibration_encoder_align, idle, requested_idle);
        focus_fsm_add_transition_begin(&cores[i].fsm, calibration_encoder_align,
                                       encoder_align_ended);
#ifdef FOCUS_CONFIG_ENCODER_ECCENTRICITY_ENABLE
        focus_fsm_add_transition_end(&cores[i].fsm, calibration_encoder_eccentricity);
        focus_fsm_add_transition(&cores[i].fsm, calibration_encoder_eccentricity, idle,
                                 core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, calibration_encoder_eccentricity, idle,
                                 requested_idle);
        focus_fsm_add_transition_begin(&cores[i].fsm, calibration_encoder_eccentricity,
                                       encoder_eccentricity_ended);
#endif
        focus_fsm_add_transition_end(&cores[i].fsm, idle);
#endif
#endif

        focus_fsm_add_transition_begin(&cores[i].fsm, idle, requested_running);
#ifdef FOCUS_CONFIG_ENCODER_ENABLE
#ifdef FOCUS_CONFIG_ENCODER_TYPE_AB
        focus_fsm_add_transition_end(&cores[i].fsm, running_encoder_align);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_align, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_align, idle, requested_idle);
        focus_fsm_add_transition_begin(&cores[i].fsm, running_encoder_align, encoder_align_ended);
#ifdef FOCUS_CONFIG_ENCODER_ECCENTRICITY_ENABLE
        focus_fsm_add_transition_end(&cores[i].fsm, running_encoder_eccentricity);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_eccentricity, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_eccentricity, idle, requested_idle);
        focus_fsm_add_transition_begin(&cores[i].fsm, running_encoder_eccentricity,
                                       encoder_eccentricity_ended);
#endif
#endif
#ifdef FOCUS_CONFIG_ENCODER_TYPE_ABI
        focus_fsm_add_transition_end(&cores[i].fsm, running_encoder_index);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_index, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_index, idle, requested_idle);
        focus_fsm_add_transition(&cores[i].fsm, running_encoder_index, idle, encoder_index_timeout);
        focus_fsm_add_transition_begin(&cores[i].fsm, running_encoder_index, encoder_index_ended);
#endif
#endif
#ifdef FOCUS_CONFIG_SENSORLESS_ENABLE
        focus_fsm_add_transition_end(&cores[i].fsm, running_sensorless_align);
        focus_fsm_add_transition(&cores[i].fsm, running_sensorless_align, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, running_sensorless_align, idle, requested_idle);
        focus_fsm_add_transition(&cores[i].fsm, running_sensorless_align, running_sensorless_ramp,
                                 running_sensorless_align_ended);
        focus_fsm_add_transition(&cores[i].fsm, running_sensorless_ramp, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, running_sensorless_ramp, idle, requested_idle);
        focus_fsm_add_transition_begin(&cores[i].fsm, running_sensorless_ramp,
                                       running_sensorless_ramp_ended);
#endif
        focus_fsm_add_transition_end(&cores[i].fsm, running);
        focus_fsm_add_transition(&cores[i].fsm, running, idle, core_panicked);
        focus_fsm_add_transition(&cores[i].fsm, running, idle, requested_idle);
#ifdef FOCUS_CONFIG_SENSORLESS_ENABLE
        focus_fsm_add_transition(&cores[i].fsm, running, running_sensorless_align,
                                 running_low_velocity);
#endif

        cores[i].calibration.data.motor.rs = 1E-1f;
        cores[i].calibration.data.motor.ld = 1E-4f;
        cores[i].calibration.data.motor.lq = 1E-4f;
#ifdef FOCUS_CONFIG_MOTOR_CALIBRATION_KV_ENABLE
        cores[i].calibration.data.motor.kv = FOCUS_2PI * 1000.f / 60.f;
#endif

        cores[i].calibration.data.current.offset[0] = 0.f;
        cores[i].calibration.data.current.offset[1] = 0.f;
        cores[i].calibration.data.current.offset[2] = 0.f;
        cores[i].calibration.data.current.scale[0] = 1.f;
        cores[i].calibration.data.current.scale[1] = 1.f;
        cores[i].calibration.data.current.scale[2] = 1.f;

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
#ifdef FOCUS_CONFIG_ENCODER_TYPE_ABI
        cores[i].encoder.index_offset = 0;
#endif
        cores[i].calibration.data.encoder.align_offset = 0;
#ifdef FOCUS_CONFIG_ENCODER_ECCENTRICITY_ENABLE
        memset((int16_t *)FOCUS_CONFIG_ENCODER_ECCENTRICITY_LOOKUP(&cores[i]), 0,
               sizeof(FOCUS_CONFIG_ENCODER_ECCENTRICITY_LOOKUP(&cores[i])));
#endif
#endif

        focus_api_calibration_update(i);

        focus_fsm_start(&cores[i].fsm, idle);
    }

    focus_port_init(user);
}

void focus_api_task() {
    for(uint32_t i = 0; i < FOCUS_CONFIG_MOTORS_NUM; i++) {
        focus_fsm_update(&cores[i].fsm);

        cores[i].state_requested = FOCUS_API_STATE_NONE;
    }
}

void focus_api_state_request(const uint32_t motor,
                             const focus_api_state_t state_requested,
                             const focus_api_state_ended_t state_ended_callback) {
    cores[motor].state_requested = state_requested;
    cores[motor].state_ended_callback = state_ended_callback;
}

focus_api_calibration_t *focus_api_calibration(const uint32_t motor) {
    return (focus_api_calibration_t *)&cores[motor].calibration.data;
}

void focus_api_calibration_update(const uint32_t motor) {
    const float w = FOCUS_2PI * FOCUS_CONFIG_FOC_BANDWIDTH;
    const float Kpd = w * cores[motor].calibration.data.motor.ld;
    const float Kpq = w * cores[motor].calibration.data.motor.lq;
    const float Ki = w * cores[motor].calibration.data.motor.rs;

    focus_pid_set_kp(&cores[motor].pid_d, Kpd);
    focus_pid_set_ki(&cores[motor].pid_d, Ki);
    focus_pid_set_kd(&cores[motor].pid_d, 0.f);
    focus_pid_set_ka(&cores[motor].pid_d, 1.f);

    focus_pid_set_kp(&cores[motor].pid_q, Kpq);
    focus_pid_set_ki(&cores[motor].pid_q, Ki);
    focus_pid_set_kd(&cores[motor].pid_q, 0.f);
    focus_pid_set_ka(&cores[motor].pid_q, 1.f);
}

void focus_api_torque_set(const uint32_t motor, const float torque) {
#ifdef FOCUS_CONFIG_MOTOR_CALIBRATION_KV_ENABLE
    const float kt = 1.f / cores[motor].calibration.data.motor.kv;
#else
    const float kt = 60.f / (FOCUS_2PI * FOCUS_CONFIG_MOTOR_KV);
#endif

    cores[motor].iq_setpoint = torque / kt;
}

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
float focus_api_position(const uint32_t motor) {
    return cores[motor].position;
}
#endif

float focus_api_velocity(const uint32_t motor) {
    return cores[motor].velocity;
}

#ifdef FOCUS_CONFIG_ENCODER_ENABLE
#ifdef FOCUS_CONFIG_ENCODER_TYPE_ABI
void focus_port_event_index(const uint32_t motor, const uint32_t encoder_count) {
    cores[motor].calibration.context.encoder.index_offset = encoder_count;
    cores[motor].calibration.context.encoder.index_occurred = true;
}
#endif
#endif

void focus_port_event_sample(const uint32_t motor, const focus_port_sample_t *sample) {
    cores[motor].sample = *sample;

    focus_fsm_execute(&cores[motor].fsm);
}

void focus_port_event_panic(const uint32_t motor) {
    focus_port_shutdown(motor, cores[motor].user);

    cores[motor].state_requested = FOCUS_API_STATE_PANIC;
}
