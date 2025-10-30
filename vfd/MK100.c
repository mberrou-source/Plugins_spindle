// mk100.c — grblHAL spindle plugin for MK100 VFD (STM32 build)
// Forces Modbus FC06 (write single register) for 0x2000 (command) and 0x1000 (comm %).

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "shared.h"
#include "modbus.h"
#include "grbl/time.h"

// MK100 registers
#define MK100_REG_CMD_WORD       0x2000u
#define MK100_REG_COMM_VALUE     0x1000u
#define MK100_REG_RUN_FREQ       0x1001u
#define MK100_ADDR_P0_10_MAXF    0xF00Au

// Command values
#define MK100_CMD_RUN_FWD        0x0001u
#define MK100_CMD_RUN_REV        0x0002u
#define MK100_CMD_COAST_STOP     0x0005u
#define MK100_CMD_DECEL_STOP     0x0006u
#define MK100_CMD_FAULT_RESET    0x0007u

typedef struct {
    modbus_stream_t *mb;
    uint8_t slave;
    uint16_t max_hz_x100;   // P0-10 (Hz*100)
    bool max_ok;
    bool fb_x100;
    uint32_t last_poll_ms;
} mk100_ctx_t;

static mk100_ctx_t ctx = {0};

static bool mb_write_u16_fc06 (uint16_t reg, uint16_t value)
{
    return modbus_write_single_register(ctx.mb, ctx.slave, reg, value);
}

static bool mb_read_u16 (uint16_t reg, uint16_t *out)
{
    return modbus_read_holding_registers(ctx.mb, ctx.slave, reg, 1, out);
}

static bool read_max_freq (void)
{
    uint16_t v = 0;
    if(!mb_read_u16(MK100_ADDR_P0_10_MAXF, &v)) return false;
    ctx.max_hz_x100 = v; ctx.max_ok = true; return true;
}

static bool probe_fb_scale (void)
{
    uint16_t f = 0;
    if(!mb_read_u16(MK100_REG_RUN_FREQ, &f)) return false;
    ctx.fb_x100 = (f > 600); // >600 Hz => x100 scaling (MK100 max ≤600 Hz)
    return true;
}

static int16_t rpm_to_pct_x100 (float rpm)
{
    if(!ctx.max_ok && !read_max_freq()) return 0;
    float tgt_hz_x100 = (rpm / 60.0f) * 100.0f;
    float max_hz_x100 = (float)ctx.max_hz_x100; if(max_hz_x100 < 1) max_hz_x100 = 1;
    float pct = (tgt_hz_x100 / max_hz_x100) * 10000.0f; // −10000..10000
    if(pct > 10000) pct = 10000; if(pct < -10000) pct = -10000;
    if(pct >= 0) pct += 0.5f; else pct -= 0.5f;
    return (int16_t)pct;
}

static bool read_rpm (float *rpm_out)
{
    if(!ctx.fb_x100 && !probe_fb_scale()) return false;
    uint16_t f = 0; if(!mb_read_u16(MK100_REG_RUN_FREQ, &f)) return false;
    float hz = ctx.fb_x100 ? (f / 100.0f) : (float)f;
    *rpm_out = hz * 60.0f; return true;
}

static void maybe_poll (void)
{
    const uint32_t now = hal.get_elapsed_ticks();
    if(now - ctx.last_poll_ms < 1500) return;
    uint16_t tmp; mb_read_u16(MK100_REG_RUN_FREQ, &tmp);
    ctx.last_poll_ms = now;
}

// vtable
static bool mk100_init (modbus_stream_t *s, uint8_t slave_addr)
{
    memset(&ctx, 0, sizeof(ctx));
    ctx.mb = s; ctx.slave = slave_addr ? slave_addr : 1;
    read_max_freq(); probe_fb_scale();
    ctx.last_poll_ms = hal.get_elapsed_ticks();
    return true;
}

static bool mk100_set_rpm (float rpm)
{
    int16_t pct = rpm_to_pct_x100(rpm);
    bool ok = mb_write_u16_fc06(MK100_REG_COMM_VALUE, (uint16_t)pct);
    maybe_poll(); return ok;
}

static bool mk100_start (spindle_dir_t dir)
{
    uint16_t cmd = (dir == SpindleDir_CCW) ? MK100_CMD_RUN_REV : MK100_CMD_RUN_FWD;
    bool ok = mb_write_u16_fc06(MK100_REG_CMD_WORD, cmd);
    maybe_poll(); return ok;
}

static bool mk100_stop (bool decel)
{
    uint16_t cmd = decel ? MK100_CMD_DECEL_STOP : MK100_CMD_COAST_STOP;
    bool ok = mb_write_u16_fc06(MK100_REG_CMD_WORD, cmd);
    maybe_poll(); return ok;
}

static void mk100_reset_fault (void)
{ mb_write_u16_fc06(MK100_REG_CMD_WORD, MK100_CMD_FAULT_RESET); }

static bool mk100_get_state (spindle_state_t *st)
{
    float rpm; if(!read_rpm(&rpm)) return false;
    st->on = rpm > 1.0f; st->dir = SpindleDir_CW; return true;
}

static bool mk100_get_rpm (float *rpm_out) { return read_rpm(rpm_out); }

static vfd_spindle_t mk100_desc = {
    .name         = "MK100",
    .id           = 0,
    .type         = VFD_TYPE_MODBUS,
    .init         = mk100_init,
    .set_rpm      = mk100_set_rpm,
    .start        = mk100_start,
    .stop         = mk100_stop,
    .reset_fault  = mk100_reset_fault,
    .get_state    = mk100_get_state,
    .get_rpm      = mk100_get_rpm,
    .flags        = VFD_FLAG_HAS_FEEDBACK
};

void mk100_register (void) { vfd_register_spindle(&mk100_desc); }
