/*
 * stepgen.cpp  --  Timer1 DDS step generator implementation.
 */
#include <Arduino.h>
#include <util/atomic.h>
#include "stepgen.h"

typedef struct {
    volatile int32_t  position;   /* actual steps emitted (signed)           */
    volatile uint32_t remaining;  /* abs steps left to reach target          */
    volatile uint32_t acc;        /* DDS phase accumulator                   */
    volatile uint32_t inc;        /* DDS phase increment (== rate)           */
    volatile int8_t   dir;        /* +1 or -1                                */
} Axis;

static Axis axes[NUM_AXES];

/* DIR bits to apply to PORTD, maintained by the main loop, read by the ISR.
 * A single-byte volatile write is atomic on AVR. */
static volatile uint8_t g_dirbits = 0;

static const uint8_t step_mask_for[NUM_AXES] = {
    (uint8_t)(1u << STEP_BIT(0)),
    (uint8_t)(1u << STEP_BIT(1)),
    (uint8_t)(1u << STEP_BIT(2)),
};

void stepgen_init(void) {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        axes[i].position  = 0;
        axes[i].remaining = 0;
        axes[i].acc       = 0;
        axes[i].inc       = 0;
        axes[i].dir       = 1;
    }
    g_dirbits = 0;

    /* STEP+DIR pins as outputs, driven low. */
    DDRD  |= (STEP_MASK | DIR_MASK);
    PORTD &= ~(STEP_MASK | DIR_MASK);

    /* Timer1: CTC mode, prescaler 1, compare match at ISR_HZ. */
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A  = (uint16_t)((F_CPU / ISR_HZ) - 1);   /* 16e6/30000 - 1 = 532 */
    TCCR1B |= (1 << WGM12);                       /* CTC */
    TCCR1B |= (1 << CS10);                        /* prescaler 1 */
    TIMSK1 |= (1 << OCIE1A);
    interrupts();
}

void stepgen_set_target(uint8_t axis, int32_t target, uint32_t inc,
                        bool *underrun) {
    Axis *a = &axes[axis];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        int32_t delta = target - a->position;
        if (delta >= 0) {
            a->dir = 1;
            a->remaining = (uint32_t)delta;
        } else {
            a->dir = -1;
            a->remaining = (uint32_t)(-delta);
        }
        a->inc = inc;
        a->acc = 0;

        /* Maintain the DIR output bit for this axis. */
        uint8_t bit = (uint8_t)(1u << DIR_BIT(axis));
        bool neg = (a->dir < 0);
        if (DIR_INVERT_MASK & (1u << axis)) neg = !neg;
        if (neg) g_dirbits |= bit; else g_dirbits &= ~bit;
    }
    /* Apply DIR bits to the port now so they are stable before the next pulse.
     * ISR only ever touches STEP_MASK bits, so masking DIR here is safe. */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        PORTD = (uint8_t)((PORTD & ~DIR_MASK) | g_dirbits);
    }
    if (underrun) *underrun = (inc == 0xFFFFFFFFul);
}

void stepgen_stop_all(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            axes[i].remaining = 0;
            axes[i].inc = 0;
        }
    }
}

int32_t stepgen_position(uint8_t axis) {
    int32_t v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        v = axes[axis].position;
    }
    return v;
}

bool stepgen_busy(void) {
    bool busy = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            if (axes[i].remaining) { busy = true; break; }
        }
    }
    return busy;
}

/*
 * The DDS heartbeat. Kept lean: integer-only, no function calls, one PORTD
 * write. At ISR_HZ = 30 kHz this must finish well inside ~33 us.
 *
 * Pulses are one ISR period wide: bits set here are cleared at the top of the
 * next tick. That gives a >30 us high time at 30 kHz, comfortably above any
 * driver's minimum step pulse.
 */
ISR(TIMER1_COMPA_vect) {
    uint8_t stepbits = 0;
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        Axis *a = &axes[i];
        if (a->remaining) {
            uint32_t before = a->acc;
            uint32_t after  = before + a->inc;
            a->acc = after;
            if (after < before) {            /* 32-bit overflow -> one step */
                stepbits |= step_mask_for[i];
                a->position += a->dir;
                a->remaining--;
            }
        }
    }
    /* Clear last tick's pulses, assert this tick's, leave DIR + serial bits. */
#if STEP_ACTIVE_HIGH
    PORTD = (uint8_t)((PORTD & ~STEP_MASK) | stepbits);
#else
    PORTD = (uint8_t)((PORTD | STEP_MASK) & ~stepbits);
#endif
}
