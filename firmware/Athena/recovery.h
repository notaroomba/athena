/*
 * recovery.h - SPU flight logic: flight phase from the MPU's navigation state, pyro channels
 * (drogue at apogee, main below a set altitude) and servo outputs. Pure C so it runs in the
 * host self-test; the two Recovery_Hw* functions are supplied by main.c (GPIO/TIM) or the test.
 *
 * Safety model: nothing fires unless the SPU is ARMED (CMD_ARM with CMD_KEY, or 'A' on its USB
 * console) AND the external arming switch on the ARM terminal closes the pyro supply. A corrupt
 * or replayed frame cannot fire: FIRE needs the key, the channel range check and the armed flag.
 */
#ifndef RECOVERY_H
#define RECOVERY_H
#include <stdint.h>
#include "athena_link.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RECOVERY_PYRO_CH   6
#define RECOVERY_SERVO_CH  6

typedef struct {
    float    main_alt_m;        /* main chute below this altitude above the pad while descending (150) */
    uint32_t pyro_pulse_ms;     /* channel on-time (1000) */
    uint32_t apogee_delay_ms;   /* drogue fires this long after apogee is detected (0) */
    uint8_t  drogue_ch;         /* 1-based channel, 0 = none (1) */
    uint8_t  main_ch;           /* 1-based channel, 0 = none (2) */
} Recovery_Params;

typedef struct {
    Recovery_Params p;
    uint8_t  phase;             /* SPU_PHASE_* */
    uint8_t  armed;
    uint8_t  fired, on;         /* channel bit masks */
    uint8_t  auto_done;         /* bit0 drogue event consumed, bit1 main event consumed */
    uint8_t  low_acc_n, descend_n;
    uint32_t pyro_off_ms[RECOVERY_PYRO_CH];
    uint16_t servo_us[RECOVERY_SERVO_CH];
    uint32_t launch_ms, apogee_ms, phase_ms, last_state_ms, still_since_ms;
    float    alt, vz, acc_mag;  /* latest: m above pad, m/s up, m/s^2 specific force */
    float    still_alt;
    float    apogee_m, vmax_ms;
} Recovery;

/* hardware hooks (ch is 0-based) */
extern void Recovery_HwPyro(uint8_t ch, int on);
extern void Recovery_HwServo(uint8_t ch, uint16_t pulse_us);   /* 0 = no pulse */

void        Recovery_Init(Recovery *r, const Recovery_Params *p);          /* p = NULL -> defaults */
void        Recovery_OnState(Recovery *r, const Athena_State *s, uint32_t now_ms);
void        Recovery_Task(Recovery *r, uint32_t now_ms);                   /* every loop: pulse timing, auto-disarm */
int         Recovery_Command(Recovery *r, const Athena_Cmd *c, uint32_t now_ms);   /* 0 ok, -1 rejected */
void        Recovery_Fill(const Recovery *r, Athena_SpuStatus *st, uint32_t now_ms);
const char *Recovery_PhaseName(uint8_t phase);

#ifdef __cplusplus
}
#endif
#endif
