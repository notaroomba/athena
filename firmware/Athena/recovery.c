#include <string.h>
#include <math.h>
#include "recovery.h"

#define G0 9.80665f

static const Recovery_Params defaults = { .main_alt_m = 150.f, .pyro_pulse_ms = 1000, .apogee_delay_ms = 0, .drogue_ch = 1, .main_ch = 2 };
static const char *const phase_names[] = { "pad", "boost", "coast", "apogee", "descent", "landed" };

const char *Recovery_PhaseName(uint8_t ph) { return ph < 6 ? phase_names[ph] : "?"; }

static void set_phase(Recovery *r, uint8_t ph, uint32_t now) { r->phase = ph; r->phase_ms = now; }

static void fire(Recovery *r, uint8_t ch, uint32_t now)
{
    if (ch >= RECOVERY_PYRO_CH) return;
    r->on |= (uint8_t)(1u << ch);
    r->fired |= (uint8_t)(1u << ch);
    r->pyro_off_ms[ch] = now + r->p.pyro_pulse_ms;
    Recovery_HwPyro(ch, 1);
}

static void all_off(Recovery *r)
{
    for (uint8_t i = 0; i < RECOVERY_PYRO_CH; i++) Recovery_HwPyro(i, 0);
    r->on = 0;
}

void Recovery_Init(Recovery *r, const Recovery_Params *p)
{
    memset(r, 0, sizeof *r);
    r->p = p ? *p : defaults;
    all_off(r);
    for (uint8_t i = 0; i < RECOVERY_SERVO_CH; i++) Recovery_HwServo(i, 0);
}

void Recovery_OnState(Recovery *r, const Athena_State *s, uint32_t now)
{
    r->last_state_ms = now;
    r->alt = -s->pos_ned[2];
    r->vz  = -s->vel_ned[2];
    r->acc_mag = sqrtf(s->acc_body[0] * s->acc_body[0] + s->acc_body[1] * s->acc_body[1] + s->acc_body[2] * s->acc_body[2]);
    int in_flight = (s->flags & STATE_FLAG_IN_FLIGHT) != 0;

    if (r->phase != SPU_PHASE_PAD && r->phase != SPU_PHASE_LANDED) {
        if (r->alt > r->apogee_m) r->apogee_m = r->alt;
        if (fabsf(r->vz) > r->vmax_ms) r->vmax_ms = fabsf(r->vz);
    }

    switch (r->phase) {
    case SPU_PHASE_PAD:
        if (in_flight) {                                   /* the MPU's launch detector (1.5 g or +10 m) */
            r->launch_ms = now; r->apogee_m = r->alt; r->vmax_ms = 0; r->low_acc_n = r->descend_n = 0; r->auto_done = 0;
            set_phase(r, SPU_PHASE_BOOST, now);
        }
        break;
    case SPU_PHASE_BOOST:
        /* burnout: only drag left, specific force under 0.5 g for 250 ms (5 frames at 20 Hz); 10 s cap */
        if (r->acc_mag < 0.5f * G0) { if (++r->low_acc_n >= 5) set_phase(r, SPU_PHASE_COAST, now); }
        else r->low_acc_n = 0;
        if (now - r->launch_ms > 10000u) set_phase(r, SPU_PHASE_COAST, now);
        break;
    case SPU_PHASE_COAST:
        /* apogee: descending faster than 1 m/s for 3 consecutive frames, never within 1 s of launch */
        if (now - r->launch_ms > 1000u && r->vz < -1.0f) {
            if (++r->descend_n >= 3) { r->apogee_ms = now; set_phase(r, SPU_PHASE_APOGEE, now); }
        } else r->descend_n = 0;
        break;
    case SPU_PHASE_APOGEE:
        if (now - r->apogee_ms >= r->p.apogee_delay_ms) {
            if (r->armed && r->p.drogue_ch) fire(r, (uint8_t)(r->p.drogue_ch - 1), now);
            r->auto_done |= 1;
            set_phase(r, SPU_PHASE_DESCENT, now);
        }
        break;
    case SPU_PHASE_DESCENT:
        if (!(r->auto_done & 2) && r->alt < r->p.main_alt_m && r->vz < 0.f) {
            if (r->armed && r->p.main_ch) fire(r, (uint8_t)(r->p.main_ch - 1), now);
            r->auto_done |= 2;                             /* one shot, even when it could not fire */
        }
        /* landed: vertical speed under 0.5 m/s and altitude within 3 m for 5 s */
        if (fabsf(r->vz) < 0.5f) {
            if (!r->still_since_ms) { r->still_since_ms = now; r->still_alt = r->alt; }
            else if (now - r->still_since_ms > 5000u) {
                if (fabsf(r->alt - r->still_alt) < 3.f) set_phase(r, SPU_PHASE_LANDED, now);
                else { r->still_since_ms = now; r->still_alt = r->alt; }
            }
        } else r->still_since_ms = 0;
        break;
    default:
        break;
    }
}

void Recovery_Task(Recovery *r, uint32_t now)
{
    for (uint8_t i = 0; i < RECOVERY_PYRO_CH; i++)
        if ((r->on & (1u << i)) && (int32_t)(now - r->pyro_off_ms[i]) >= 0) { r->on &= (uint8_t)~(1u << i); Recovery_HwPyro(i, 0); }
    if (r->phase == SPU_PHASE_LANDED && r->armed && now - r->phase_ms > 10000u) r->armed = 0;   /* nothing left to do on the ground */
}

int Recovery_Command(Recovery *r, const Athena_Cmd *c, uint32_t now)
{
    switch (c->cmd) {
    case CMD_PING:
        return 0;
    case CMD_ARM:
        if (c->key != CMD_KEY) return -1;
        r->armed = 1;
        return 0;
    case CMD_DISARM:
        r->armed = 0;
        all_off(r);
        return 0;
    case CMD_FIRE:
        if (c->key != CMD_KEY || !r->armed || c->arg < 1 || c->arg > RECOVERY_PYRO_CH) return -1;
        fire(r, (uint8_t)(c->arg - 1), now);
        return 0;
    case CMD_SERVO:
        if (c->arg < 1 || c->arg > RECOVERY_SERVO_CH) return -1;
        if (c->value && (c->value < 500 || c->value > 2500)) return -1;
        r->servo_us[c->arg - 1] = c->value;
        Recovery_HwServo((uint8_t)(c->arg - 1), c->value);
        return 0;
    case CMD_SET_MAIN_ALT:
        if (c->value < 30 || c->value > 3000) return -1;
        r->p.main_alt_m = (float)c->value;
        return 0;
    default:
        return -1;                                         /* CMD_RESET_MPU is hardware: handled by the caller */
    }
}

void Recovery_Fill(const Recovery *r, Athena_SpuStatus *st, uint32_t now)
{
    st->phase = r->phase;
    st->flags = (uint8_t)((r->armed ? SPU_FLAG_ARMED : 0) | ((r->last_state_ms && now - r->last_state_ms < 1000u) ? SPU_FLAG_MPU_LINK : 0));
    st->pyro_fired = r->fired;
    st->pyro_on = r->on;
    st->main_alt_m = (uint16_t)r->p.main_alt_m;
    memcpy(st->servo_us, r->servo_us, sizeof st->servo_us);
    st->apogee_m = r->apogee_m;
    st->vmax_ms = r->vmax_ms;
}
