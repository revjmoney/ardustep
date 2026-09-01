/* ardustep v2 -- experimental USB step generator for LinuxCNC. */
#include <Arduino.h>
#include "config.h"
#include "protocol.h"
#include "stepgen.h"

#if (ISR_HZ * CMD_INTERVAL_US) % 1000000UL
#error "CMD_INTERVAL_US must contain an integer number of ISR ticks"
#endif
#if TICKS_PER_CMD == 0
#error "command interval must contain at least one ISR tick"
#endif

enum FrameType { FRAME_NONE, FRAME_COMMAND, FRAME_HELLO };
static uint8_t rxbuf[CMD_PACKET_LEN];
static uint8_t rxlen = 0;
static uint8_t rx_expected = 0;

static uint16_t underrun_count = 0;
static uint32_t last_packet_ms = 0;
static bool faulted = false;
static bool enabled = false;          /* actual physical ENABLE state */
static bool protocol_ready = false;

static inline void set_enable_pin(bool on) {
#if ENABLE_ACTIVE_HIGH
    if (on) PORTB |= (1 << ENABLE_PIN_BIT); else PORTB &= ~(1 << ENABLE_PIN_BIT);
#else
    if (on) PORTB &= ~(1 << ENABLE_PIN_BIT); else PORTB |= (1 << ENABLE_PIN_BIT);
#endif
}

static inline void set_spindle_pin(bool on) {
    if (on) PORTB |= (1 << SPINDLE_PIN_BIT); else PORTB &= ~(1 << SPINDLE_PIN_BIT);
}

static void stop_outputs(void) {
    stepgen_stop_all();
    enabled = false;
    set_enable_pin(false);
    set_spindle_pin(false);
}

static uint8_t read_limits(void) {
    return (uint8_t)(~PINC) & LIMIT_MASK;
}

static void send_feedback(uint8_t seq, uint8_t limits) {
    FeedbackPacket fb;
    bool busy = false;
    fb.magic = FB_MAGIC;
    fb.seq = seq;
    fb.status = 0;
    stepgen_snapshot(fb.pos_fb, &busy);
    if (busy)           fb.status |= ST_RUNNING;
    if (enabled)        fb.status |= ST_ENABLED;
    if (faulted)        fb.status |= ST_FAULT;
    if (underrun_count) fb.status |= ST_UNDERRUN;
    fb.limits = limits;
    fb.underruns = underrun_count;
    fb.crc = crc8((const uint8_t *)&fb, FB_PACKET_LEN - 1);
    Serial.write((const uint8_t *)&fb, FB_PACKET_LEN);
}

static void send_info(uint8_t seq) {
    InfoPacket info;
    info.magic = INFO_MAGIC;
    info.seq = seq;
    info.proto_version = PROTO_VERSION;
    info.fw_major = FW_VERSION_MAJOR;
    info.fw_minor = FW_VERSION_MINOR;
    info.axes = NUM_AXES;
    info.isr_hz = ISR_HZ;
    info.command_interval_us = CMD_INTERVAL_US;
    info.max_step_rate = (uint16_t)MAX_STEP_RATE;
    info.watchdog_ms = WATCHDOG_MS;
    info.crc = crc8((const uint8_t *)&info, INFO_PACKET_LEN - 1);
    Serial.write((const uint8_t *)&info, INFO_PACKET_LEN);
}

static uint32_t increment_for_delta(int64_t delta, bool *saturated) {
    uint64_t magnitude = delta < 0 ? (uint64_t)(-delta) : (uint64_t)delta;
    const uint32_t max_steps = MAX_STEP_RATE / (1000000UL / CMD_INTERVAL_US);
    *saturated = magnitude > max_steps;
    if (magnitude > max_steps) magnitude = max_steps;
    return (uint32_t)((magnitude * 0x100000000ULL) / TICKS_PER_CMD);
}

static void handle_command(const CommandPacket *cmd) {
    last_packet_ms = millis();
    bool estop = (cmd->flags & FLAG_ESTOP) != 0;
    bool request_enable = (cmd->flags & FLAG_ENABLE) != 0;
    bool clear_fault = (cmd->flags & FLAG_CLEAR_FAULT) != 0;

    if (!protocol_ready) {
        stop_outputs();
        faulted = true;
        return;
    }
    if (estop) {
        stop_outputs();
        faulted = true;
        return;
    }
    if (clear_fault && !request_enable) {
        stop_outputs();
        faulted = false;
        return;
    }
    if (!request_enable) {
        stop_outputs();
        return;
    }
    if (faulted) {
        stop_outputs();
        return;
    }

    enabled = true;
    set_enable_pin(true);
    set_spindle_pin((cmd->flags & FLAG_SPINDLE) != 0);
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        int32_t position = stepgen_position(i);
        int64_t delta = (int64_t)cmd->pos_cmd[i] - (int64_t)position;
        bool saturated = false;
        uint32_t inc = increment_for_delta(delta, &saturated);
        if (saturated && underrun_count != 0xFFFF) underrun_count++;
        stepgen_set_target(i, cmd->pos_cmd[i], inc);
    }
}

static FrameType resync_after_bad_frame(void) {
    for (uint8_t i = 1; i < rxlen; i++) {
        if (rxbuf[i] == CMD_MAGIC || rxbuf[i] == HELLO_MAGIC) {
            uint8_t keep = rxlen - i;
            memmove(rxbuf, rxbuf + i, keep);
            rxlen = keep;
            rx_expected = rxbuf[0] == CMD_MAGIC ? CMD_PACKET_LEN : HELLO_PACKET_LEN;
            if (rxlen >= rx_expected) {
                FrameType recovered = rxbuf[0] == CMD_MAGIC ? FRAME_COMMAND : FRAME_HELLO;
                if (crc8(rxbuf, rx_expected - 1) == rxbuf[rx_expected - 1]) {
                    rxlen = 0;
                    rx_expected = 0;
                    return recovered;
                }
                rxlen = 0;
                rx_expected = 0;
            }
            return FRAME_NONE;
        }
    }
    rxlen = 0;
    rx_expected = 0;
    return FRAME_NONE;
}

static FrameType framer_push(uint8_t byte) {
    if (rxlen == 0) {
        if (byte == CMD_MAGIC) rx_expected = CMD_PACKET_LEN;
        else if (byte == HELLO_MAGIC) rx_expected = HELLO_PACKET_LEN;
        else return FRAME_NONE;
    }
    rxbuf[rxlen++] = byte;
    if (rxlen < rx_expected) return FRAME_NONE;

    uint8_t complete_len = rx_expected;
    FrameType type = rxbuf[0] == CMD_MAGIC ? FRAME_COMMAND : FRAME_HELLO;
    if (crc8(rxbuf, complete_len - 1) != rxbuf[complete_len - 1]) {
        return resync_after_bad_frame();
    }
    rxlen = 0;
    rx_expected = 0;
    return type;
}

void setup(void) {
    DDRB |= (1 << ENABLE_PIN_BIT) | (1 << SPINDLE_PIN_BIT);
    set_enable_pin(false);
    set_spindle_pin(false);
    DDRC &= ~LIMIT_MASK;
    PORTC |= LIMIT_MASK;
    stepgen_init();
    Serial.begin(SERIAL_BAUD);
    last_packet_ms = millis();
}

void loop(void) {
    while (Serial.available()) {
        FrameType frame = framer_push((uint8_t)Serial.read());
        if (frame == FRAME_HELLO) {
            HelloPacket hello;
            memcpy(&hello, rxbuf, HELLO_PACKET_LEN);
            protocol_ready = hello.proto_version == PROTO_VERSION;
            if (!protocol_ready) {
                stop_outputs();
                faulted = true;
            }
            last_packet_ms = millis();
            send_info(hello.seq);
        } else if (frame == FRAME_COMMAND) {
            CommandPacket cmd;
            memcpy(&cmd, rxbuf, CMD_PACKET_LEN);
            handle_command(&cmd);
            send_feedback(cmd.seq, read_limits());
        }
    }

    if ((uint32_t)(millis() - last_packet_ms) > WATCHDOG_MS && !faulted) {
        stop_outputs();
        faulted = true;
    }
}
