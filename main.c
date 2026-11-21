/**
 * @file embedded_demo.c
 * @brief Embedded Firmware Architecture Demo: Ring Buffer, Sensor Filter,
 *        Fault State Machine, Watchdog Monitor, and Cooperative Scheduler.
 *
 * Compile (simulation / host test):
 *   gcc -std=c11 -Wall -Wextra -Wpedantic -O2 embedded_demo.c -o embedded_demo
 *   ./embedded_demo
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* =========================================================================
 * 1. HARDWARE / PLATFORM CONFIGURATION CONSTANTS
 * ========================================================================= */

#define SYS_TICK_HZ                 1000U       /* 1 ms system tick */
#define UART_RX_BUF_SIZE            64U         /* Power of two recommended */
#define UART_TX_BUF_SIZE            128U
#define ADC_MAX_COUNTS              4095U       /* 12-bit ADC: 0..4095 */
#define ADC_VREF_MV                 3300U       /* 3.300 V reference */
#define SENSOR_FILTER_WINDOW        8U          /* Moving average tap count */
#define SENSOR_LOW_THRESHOLD_MV     200U        /* Open/short low fault */
#define SENSOR_HIGH_THRESHOLD_MV    3100U       /* Over-voltage fault */
#define WATCHDOG_TIMEOUT_MS         250U        /* Max allowed stall window */
#define MAX_TASKS                   6U
#define DEMO_SIMULATION_TICKS       1200U       /* 1.2 seconds simulated */

/* =========================================================================
 * 2. RING BUFFER (CIRCULAR FIFO) FOR BYTE STREAMS
 * ========================================================================= */

typedef struct {
    uint8_t *buffer;
    uint16_t capacity;
    volatile uint16_t head;       /* Write index */
    volatile uint16_t tail;       /* Read index */
    volatile uint32_t drop_count; /* Bytes dropped due to full buffer */
} ring_buf_t;

static void ring_buf_init(ring_buf_t *rb, uint8_t *storage, uint16_t capacity)
{
    rb->buffer = storage;
    rb->capacity = capacity;
    rb->head = 0U;
    rb->tail = 0U;
    rb->drop_count = 0U;
}

static void ring_buf_reset(ring_buf_t *rb)
{
    rb->head = 0U;
    rb->tail = 0U;
    rb->drop_count = 0U;
}

static uint16_t ring_buf_count(const ring_buf_t *rb)
{
    if (rb->head >= rb->tail) {
        return (uint16_t)(rb->head - rb->tail);
    }
    return (uint16_t)((rb->capacity - rb->tail) + rb->head);
}

static bool ring_buf_is_full(const ring_buf_t *rb)
{
    uint16_t next_head = (uint16_t)((rb->head + 1U) % rb->capacity);
    return (next_head == rb->tail);
}

static bool ring_buf_is_empty(const ring_buf_t *rb)
{
    return (rb->head == rb->tail);
}

static bool ring_buf_push(ring_buf_t *rb, uint8_t byte)
{
    uint16_t next_head = (uint16_t)((rb->head + 1U) % rb->capacity);
    if (next_head == rb->tail) {
        rb->drop_count++;
        return false;
    }
    rb->buffer[rb->head] = byte;
    rb->head = next_head;
    return true;
}

static bool ring_buf_pop(ring_buf_t *rb, uint8_t *out_byte)
{
    if (ring_buf_is_empty(rb)) {
        return false;
    }
    *out_byte = rb->buffer[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) % rb->capacity);
    return true;
}

static uint16_t ring_buf_write(ring_buf_t *rb, const uint8_t *data, uint16_t len)
{
    uint16_t written = 0U;
    for (uint16_t i = 0U; i < len; i++) {
        if (ring_buf_push(rb, data[i])) {
            written++;
        } else {
            break;
        }
    }
    return written;
}

/* =========================================================================
 * 3. SENSOR ACQUISITION, FIXED-POINT FILTER & FAULT DETECTOR
 * ========================================================================= */

typedef enum {
    SENSOR_OK = 0,
    SENSOR_FAULT_UNDERVOLTAGE,
    SENSOR_FAULT_OVERVOLTAGE,
    SENSOR_FAULT_STALE
} sensor_status_t;

typedef struct {
    uint16_t raw_counts;
    uint32_t raw_mv;
    uint32_t filtered_mv;
    uint32_t samples[SENSOR_FILTER_WINDOW];
    uint32_t accumulator;
    uint8_t sample_idx;
    uint8_t sample_count;
    uint32_t last_update_tick;
    uint16_t fault_debounce_cnt;
    sensor_status_t status;
} analog_sensor_t;

static uint32_t adc_counts_to_mv(uint16_t counts)
{
    if (counts > ADC_MAX_COUNTS) {
        counts = ADC_MAX_COUNTS;
    }
    return ((uint32_t)counts * ADC_VREF_MV) / ADC_MAX_COUNTS;
}

static void sensor_init(analog_sensor_t *s, uint32_t now_tick)
{
    memset(s, 0, sizeof(*s));
    s->last_update_tick = now_tick;
    s->status = SENSOR_OK;
}

static void sensor_update_sample(analog_sensor_t *s, uint16_t raw_counts, uint32_t now_tick)
{
    s->raw_counts = raw_counts;
    s->raw_mv = adc_counts_to_mv(raw_counts);
    s->last_update_tick = now_tick;

    if (s->sample_count == SENSOR_FILTER_WINDOW) {
        s->accumulator -= s->samples[s->sample_idx];
    } else {
        s->sample_count++;
    }

    s->samples[s->sample_idx] = s->raw_mv;
    s->accumulator += s->raw_mv;
    s->sample_idx = (uint8_t)((s->sample_idx + 1U) % SENSOR_FILTER_WINDOW);
    s->filtered_mv = s->accumulator / s->sample_count;

    /* Debounced threshold fault detection */
    bool out_of_range_low = (s->filtered_mv < SENSOR_LOW_THRESHOLD_MV);
    bool out_of_range_high = (s->filtered_mv > SENSOR_HIGH_THRESHOLD_MV);

    if (out_of_range_low || out_of_range_high) {
        if (s->fault_debounce_cnt < 5U) {
            s->fault_debounce_cnt++;
        }
        if (s->fault_debounce_cnt >= 3U) {
            s->status = out_of_range_low ? SENSOR_FAULT_UNDERVOLTAGE
                                         : SENSOR_FAULT_OVERVOLTAGE;
        }
    } else {
        if (s->fault_debounce_cnt > 0U) {
            s->fault_debounce_cnt--;
        }
        if (s->fault_debounce_cnt == 0U) {
            s->status = SENSOR_OK;
        }
    }
}

static void sensor_check_staleness(analog_sensor_t *s, uint32_t now_tick, uint32_t max_age_ms)
{
    if ((now_tick - s->last_update_tick) > max_age_ms) {
        s->status = SENSOR_FAULT_STALE;
    }
}

/* =========================================================================
 * 4. SYSTEM STATE MACHINE & WATCHDOG SUPERVISOR
 * ========================================================================= */

typedef enum {
    SYS_STATE_BOOT = 0,
    SYS_STATE_CALIBRATE,
    SYS_STATE_RUN,
    SYS_STATE_DEGRADED,
    SYS_STATE_FAULT
} system_state_t;

typedef struct {
    uint32_t timeout_ms;
    uint32_t last_kick_tick;
    uint32_t kick_count;
    bool expired;
} watchdog_t;

typedef struct {
    system_state_t state;
    system_state_t prev_state;
    uint32_t state_enter_tick;
    uint32_t calibration_offset_mv;
    uint32_t fault_code;
    watchdog_t wdg;
} sys_controller_t;

static const char *sys_state_name(system_state_t st)
{
    switch (st) {
        case SYS_STATE_BOOT:      return "BOOT";
        case SYS_STATE_CALIBRATE: return "CALIBRATE";
        case SYS_STATE_RUN:       return "RUN";
        case SYS_STATE_DEGRADED:  return "DEGRADED";
        case SYS_STATE_FAULT:     return "FAULT";
        default:                  return "UNKNOWN";
    }
}

static void watchdog_init(watchdog_t *wdg, uint32_t timeout_ms, uint32_t now_tick)
{
    wdg->timeout_ms = timeout_ms;
    wdg->last_kick_tick = now_tick;
    wdg->kick_count = 0U;
    wdg->expired = false;
}

static void watchdog_kick(watchdog_t *wdg, uint32_t now_tick)
{
    wdg->last_kick_tick = now_tick;
    wdg->kick_count++;
}

static bool watchdog_check(watchdog_t *wdg, uint32_t now_tick)
{
    if ((now_tick - wdg->last_kick_tick) >= wdg->timeout_ms) {
        wdg->expired = true;
    }
    return wdg->expired;
}

static void sys_transition(sys_controller_t *ctrl, system_state_t next_state, uint32_t now_tick)
{
    if (ctrl->state != next_state) {
        ctrl->prev_state = ctrl->state;
        ctrl->state = next_state;
        ctrl->state_enter_tick = now_tick;
        printf("[%04lu ms] STATE: %s -> %s (fault_code=0x%02lX)\n",
               (unsigned long)now_tick,
               sys_state_name(ctrl->prev_state),
               sys_state_name(ctrl->state),
               (unsigned long)ctrl->fault_code);
    }
}

static void sys_controller_init(sys_controller_t *ctrl, uint32_t now_tick)
{
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->state = SYS_STATE_BOOT;
    ctrl->prev_state = SYS_STATE_BOOT;
    ctrl->state_enter_tick = now_tick;
    watchdog_init(&ctrl->wdg, WATCHDOG_TIMEOUT_MS, now_tick);
}

static void sys_controller_step(sys_controller_t *ctrl, const analog_sensor_t *sensor,
                                const ring_buf_t *tx_buf, uint32_t now_tick)
{
    if (watchdog_check(&ctrl->wdg, now_tick)) {
        ctrl->fault_code |= 0x01U;
        sys_transition(ctrl, SYS_STATE_FAULT, now_tick);
        return;
    }

    switch (ctrl->state) {
        case SYS_STATE_BOOT:
            /* Wait 50 ms for rails and peripherals to settle */
            if ((now_tick - ctrl->state_enter_tick) >= 50U) {
                sys_transition(ctrl, SYS_STATE_CALIBRATE, now_tick);
            }
            break;

        case SYS_STATE_CALIBRATE:
            /* Capture zero-current/mid-rail reference offset over 100 ms */
            if ((now_tick - ctrl->state_enter_tick) >= 100U) {
                ctrl->calibration_offset_mv = (sensor->filtered_mv > 1650U)
                                              ? (sensor->filtered_mv - 1650U)
                                              : 0U;
                sys_transition(ctrl, SYS_STATE_RUN, now_tick);
            }
            break;

        case SYS_STATE_RUN:
            if (sensor->status == SENSOR_FAULT_OVERVOLTAGE ||
                sensor->status == SENSOR_FAULT_STALE) {
                ctrl->fault_code |= (uint32_t)sensor->status;
                sys_transition(ctrl, SYS_STATE_FAULT, now_tick);
            } else if (sensor->status == SENSOR_FAULT_UNDERVOLTAGE ||
                       tx_buf->drop_count > 0U) {
                ctrl->fault_code |= 0x10U;
                sys_transition(ctrl, SYS_STATE_DEGRADED, now_tick);
            }
            break;

        case SYS_STATE_DEGRADED:
            if (sensor->status == SENSOR_FAULT_OVERVOLTAGE ||
                sensor->status == SENSOR_FAULT_STALE) {
                ctrl->fault_code |= (uint32_t)sensor->status;
                sys_transition(ctrl, SYS_STATE_FAULT, now_tick);
            } else if (sensor->status == SENSOR_OK &&
                       (now_tick - ctrl->state_enter_tick) >= 150U) {
                ctrl->fault_code &= ~0x10U;
                sys_transition(ctrl, SYS_STATE_RUN, now_tick);
            }
            break;

        case SYS_STATE_FAULT:
            /* Safe state: hold outputs safe, log fault until external clear/reset */
            break;

        default:
            sys_transition(ctrl, SYS_STATE_FAULT, now_tick);
            break;
    }
}

/* =========================================================================
 * 5. COOPERATIVE TASK SCHEDULER
 * ========================================================================= */

typedef void (*task_fn_t)(void *ctx, uint32_t now_tick);

typedef struct {
    const char *name;
    task_fn_t fn;
    void *ctx;
    uint32_t period_ms;
    uint32_t last_run_tick;
    uint32_t run_count;
    bool enabled;
} sched_task_t;

typedef struct {
    sched_task_t tasks[MAX_TASKS];
    uint8_t task_count;
} scheduler_t;

static void scheduler_init(scheduler_t *sched)
{
    memset(sched, 0, sizeof(*sched));
}

static bool scheduler_add_task(scheduler_t *sched, const char *name, task_fn_t fn,
                               void *ctx, uint32_t period_ms, uint32_t start_tick)
{
    if (sched->task_count >= MAX_TASKS || fn == NULL || period_ms == 0U) {
        return false;
    }
    sched_task_t *t = &sched->tasks[sched->task_count++];
    t->name = name;
    t->fn = fn;
    t->ctx = ctx;
    t->period_ms = period_ms;
    t->last_run_tick = start_tick;
    t->run_count = 0U;
    t->enabled = true;
    return true;
}

static void scheduler_tick(scheduler_t *sched, uint32_t now_tick)
{
    for (uint8_t i = 0U; i < sched->task_count; i++) {
        sched_task_t *t = &sched->tasks[i];
        if (!t->enabled) {
            continue;
        }
        if ((now_tick - t->last_run_tick) >= t->period_ms) {
            t->last_run_tick = now_tick;
            t->run_count++;
            t->fn(t->ctx, now_tick);
        }
    }
}

/* =========================================================================
 * 6. APPLICATION CONTEXT, TASKS & SIMULATED HARDWARE STUBS
 * ========================================================================= */

typedef struct {
    uint32_t tick_ms;
    ring_buf_t rx_buf;
    ring_buf_t tx_buf;
    uint8_t rx_storage[UART_RX_BUF_SIZE];
    uint8_t tx_storage[UART_TX_BUF_SIZE];
    analog_sensor_t sensor;
    sys_controller_t sys;
    scheduler_t sched;
} app_context_t;

/* Simulated ADC hardware register read with synthetic waveform & fault injection */
static uint16_t hal_adc_read_sim(uint32_t tick_ms)
{
    /* Inject a transient undervoltage dip around 450..520 ms */
    if (tick_ms >= 450U && tick_ms <= 520U) {
        return 120U; /* ~96 mV */
    }
    /* Inject a severe overvoltage spike around 900..940 ms */
    if (tick_ms >= 900U && tick_ms <= 940U) {
        return 4050U; /* ~3261 mV */
    }
    /* Nominal operating signal oscillating gently around ~1650 mV (counts ~2048) */
    int32_t wave = (int32_t)((tick_ms % 100U)) - 50;
    int32_t counts = 2048 + (wave * 6);
    if (counts < 0) {
        counts = 0;
    }
    if (counts > (int32_t)ADC_MAX_COUNTS) {
        counts = (int32_t)ADC_MAX_COUNTS;
    }
    return (uint16_t)counts;
}

/* Simulated UART TX hardware drain */
static void hal_uart_tx_drain_sim(ring_buf_t *tx_buf)
{
    uint8_t ch = 0U;
    uint16_t drained = 0U;
    while (drained < 16U && ring_buf_pop(tx_buf, &ch)) {
        drained++;
    }
}

static void task_sensor_sample(void *ctx, uint32_t now_tick)
{
    app_context_t *app = (app_context_t *)ctx;
    uint16_t counts = hal_adc_read_sim(now_tick);
    sensor_update_sample(&app->sensor, counts, now_tick);
}

static void task_supervisor_wdg(void *ctx, uint32_t now_tick)
{
    app_context_t *app = (app_context_t *)ctx;
    sensor_check_staleness(&app->sensor, now_tick, 100U);
    sys_controller_step(&app->sys, &app->sensor, &app->tx_buf, now_tick);

    /* Kick watchdog unless in unrecoverable fault state */
    if (app->sys.state != SYS_STATE_FAULT) {
        watchdog_kick(&app->sys.wdg, now_tick);
    }
}

static void task_telemetry_pack(void *ctx, uint32_t now_tick)
{
    app_context_t *app = (app_context_t *)ctx;
    char frame[96];
    int len = snprintf(frame, sizeof(frame),
                       "T=%lu,ST=%s,RAW=%lu,FILT=%lu,RX=%u,TX=%u\r\n",
                       (unsigned long)now_tick,
                       sys_state_name(app->sys.state),
                       (unsigned long)app->sensor.raw_mv,
                       (unsigned long)app->sensor.filtered_mv,
                       (unsigned int)ring_buf_count(&app->rx_buf),
                       (unsigned int)ring_buf_count(&app->tx_buf));
    if (len > 0) {
        (void)ring_buf_write(&app->tx_buf, (const uint8_t *)frame, (uint16_t)len);
    }
    hal_uart_tx_drain_sim(&app->tx_buf);
}

static void task_cli_poll(void *ctx, uint32_t now_tick)
{
    app_context_t *app = (app_context_t *)ctx;
    (void)now_tick;

    /* Feed a simulated command byte into RX ring buffer every 200 ms */
    ring_buf_push(&app->rx_buf, (uint8_t)'?');

    uint8_t byte = 0U;
    while (ring_buf_pop(&app->rx_buf, &byte)) {
        if (byte == '?') {
            const char *ack = "OK\r\n";
            (void)ring_buf_write(&app->tx_buf, (const uint8_t *)ack, 4U);
        }
    }
}

static void task_periodic_summary(void *ctx, uint32_t now_tick)
{
    app_context_t *app = (app_context_t *)ctx;
    printf("[%04lu ms] SUMMARY | state=%-9s | raw=%4lumV | filt=%4lumV | "
           "wdg_kicks=%lu | tx_drops=%lu\n",
           (unsigned long)now_tick,
           sys_state_name(app->sys.state),
           (unsigned long)app->sensor.raw_mv,
           (unsigned long)app->sensor.filtered_mv,
           (unsigned long)app->sys.wdg.kick_count,
           (unsigned long)app->tx_buf.drop_count);
}

/* =========================================================================
 * 7. ENTRY POINT & MAIN LOOP
 * ========================================================================= */

static void app_init(app_context_t *app)
{
    memset(app, 0, sizeof(*app));
    app->tick_ms = 0U;

    ring_buf_init(&app->rx_buf, app->rx_storage, UART_RX_BUF_SIZE);
    ring_buf_init(&app->tx_buf, app->tx_storage, UART_TX_BUF_SIZE);
    ring_buf_reset(&app->rx_buf);
    ring_buf_reset(&app->tx_buf);

    sensor_init(&app->sensor, app->tick_ms);
    sys_controller_init(&app->sys, app->tick_ms);
    scheduler_init(&app->sched);

    (void)scheduler_add_task(&app->sched, "sensor_10ms", task_sensor_sample,
                             app, 10U, app->tick_ms);
    (void)scheduler_add_task(&app->sched, "wdg_20ms", task_supervisor_wdg,
                             app, 20U, app->tick_ms);
    (void)scheduler_add_task(&app->sched, "cli_50ms", task_cli_poll,
                             app, 50U, app->tick_ms);
    (void)scheduler_add_task(&app->sched, "telem_100ms", task_telemetry_pack,
                             app, 100U, app->tick_ms);
    (void)scheduler_add_task(&app->sched, "summary_200ms", task_periodic_summary,
                             app, 200U, app->tick_ms);
}

int main(void)
{
    app_context_t app;
    app_init(&app);
    printf("Starting Embedded Firmware Demo (%u ms simulation)...\n",
           DEMO_SIMULATION_TICKS);
    for (uint32_t t = 0U; t <= DEMO_SIMULATION_TICKS; t++) {
        app.tick_ms = t;
        scheduler_tick(&app.sched, app.tick_ms);
    }
    printf("Simulation complete. Final state: %s, WD kicks: %lu\n",
           sys_state_name(app.sys.state),
           (unsigned long)app.sys.wdg.kick_count);
    return EXIT_SUCCESS;
}
