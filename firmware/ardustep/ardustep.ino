/*
 * ardustep.ino  --  USB step generator for LinuxCNC ("for science").
 *
 * Architecture (see README.md):
 *   LinuxCNC motion -> ardustep.py (USB serial) -> THIS firmware -> STEP/DIR.
 *
 * The host streams absolute position targets (in steps) once per cycle. We do
 * the actual pulse timing on-chip with a Timer1 DDS so the jittery USB link
 * only has to deliver setpoints, not microsecond-accurate steps. Each incoming
 * target is converted to a phase increment that sweeps the axis from its
 * current position to the target over one host interval (thin-buffer interp).
 *
 * We reply to every command with a feedback packet (actual step counts +
 * status), so LinuxCNC closes the position loop and enforces following error.
 */
#include <Arduino.h>
#include "config.h"
#include "protocol.h"
#include "stepgen.h"

/* Phase increment per step-of-delta for a one-interval sweep:
 *   inc = delta * 2^32 / (interval_s * ISR_HZ)
 * Precompute the constant factor K. */
static const float K_INC =
    4294967296.0f / (((float)CMD_INTERVAL_US * 1e-6f) * (float)ISR_HZ);

/* ---- incoming-packet framer -------------------------------------------- */
static uint8_t  rxbuf[CMD_PACKET_LEN];
static uint8_t  rxlen = 0;

/* ---- latched state ------------------------------------------------------ */
static uint16_t underrun_count = 0;
static uint32_t last_packet_ms = 0;
static bool     faulted        = false;
static bool     enabled        = false;
static uint8_t  last_seq       = 0;

static inline void set_enable_pin(bool on) {
#if ENABLE_ACTIVE_HIGH
    if (on) PORTB |=  (1 << ENABLE_PIN_BIT); else PORTB &= ~(1 << ENABLE_PIN_BIT);
#else
    if (on) PORTB &= ~(1 << ENABLE_PIN_BIT); else PORTB |=  (1 << ENABLE_PIN_BIT);
#endif
}

static inline void set_spindle_pin(bool on) {
    if (on) PORTB |= (1 << SPINDLE_PIN_BIT); else PORTB &= ~(1 << SPINDLE_PIN_BIT);
}

static uint8_t read_limits(void) {
    /* active-low with pull-ups: pressed == pin low == bit set in result */
    uint8_t pins = (uint8_t)(~PINC) & LIMIT_MASK;
    return pins;   /* LIM_X|LIM_Y|LIM_Z already align to PC0..2 */
}

static void send_feedback(uint8_t seq, uint8_t limits) {
    FeedbackPacket fb;
    fb.magic  = FB_MAGIC;
    fb.seq    = seq;
    fb.status = 0;
    if (stepgen_busy()) fb.status |= ST_RUNNING;
    if (enabled)        fb.status |= ST_ENABLED;
    if (faulted)        fb.status |= ST_FAULT;
    if (underrun_count) fb.status |= ST_UNDERRUN;
    fb.limits = limits;
    for (uint8_t i = 0; i < NUM_AXES; i++) fb.pos_fb[i] = stepgen_position(i);
    fb.underruns = underrun_count;
    fb.crc = crc8((const uint8_t *)&fb, FB_PACKET_LEN - 1);
    Serial.write((const uint8_t *)&fb, FB_PACKET_LEN);
}

/* Act on a validated command packet. */
static void handle_command(const CommandPacket *cmd) {
    last_packet_ms = millis();
    last_seq = cmd->seq;

    bool estop = (cmd->flags & FLAG_ESTOP) != 0;
    enabled    = (cmd->flags & FLAG_ENABLE) != 0;

    if (estop || !enabled) {
        stepgen_stop_all();
        set_enable_pin(false);
        set_spindle_pin(false);
        if (estop) faulted = true;
        return;
    }
    faulted = false;
    set_enable_pin(true);
    set_spindle_pin((cmd->flags & FLAG_SPINDLE) != 0);

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        int32_t target = cmd->pos_cmd[i];
        int32_t delta  = target - stepgen_position(i);
        uint32_t mag   = (delta >= 0) ? (uint32_t)delta : (uint32_t)(-delta);

        uint32_t inc;
        float fi = (float)mag * K_INC;
        if (fi >= 4294967295.0f) {           /* needs > 1 step/tick: clamp */
            inc = 0xFFFFFFFFul;
            if (underrun_count < 0xFFFF) underrun_count++;
        } else {
            inc = (uint32_t)fi;
        }
        bool un = false;
        stepgen_set_target(i, target, inc, &un);
    }
}

/* Feed one received byte into the framer; returns true when a full, CRC-valid
 * CommandPacket has been assembled into rxbuf. */
static bool framer_push(uint8_t b) {
    if (rxlen == 0) {
        if (b != CMD_MAGIC) return false;    /* hunt for magic */
        rxbuf[rxlen++] = b;
        return false;
    }
    rxbuf[rxlen++] = b;
    if (rxlen < CMD_PACKET_LEN) return false;

    rxlen = 0;                               /* full frame captured */
    if (crc8(rxbuf, CMD_PACKET_LEN - 1) != rxbuf[CMD_PACKET_LEN - 1]) {
        return false;                        /* bad CRC -> drop, resync */
    }
    return true;
}

void setup(void) {
    /* control + spindle pins as outputs, de-asserted */
    DDRB  |= (1 << ENABLE_PIN_BIT) | (1 << SPINDLE_PIN_BIT);
    set_enable_pin(false);
    set_spindle_pin(false);

    /* limit inputs with pull-ups */
    DDRC  &= ~LIMIT_MASK;
    PORTC |=  LIMIT_MASK;

    stepgen_init();
    Serial.begin(SERIAL_BAUD);
    last_packet_ms = millis();
}

void loop(void) {
    /* Drain whatever the USB CDC has buffered, acting on each complete frame. */
    while (Serial.available()) {
        if (framer_push((uint8_t)Serial.read())) {
            CommandPacket cmd;
            memcpy(&cmd, rxbuf, CMD_PACKET_LEN);
            uint8_t limits = read_limits();
            handle_command(&cmd);
            send_feedback(cmd.seq, limits);
        }
    }

    /* Watchdog: lost the host -> stop and fault. */
    if ((uint32_t)(millis() - last_packet_ms) > WATCHDOG_MS) {
        if (!faulted) {
            stepgen_stop_all();
            set_enable_pin(false);
            set_spindle_pin(false);
            faulted = true;
        }
    }
}
