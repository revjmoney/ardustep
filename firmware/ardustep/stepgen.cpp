/*
 * stepgen.cpp -- Timer1 DDS step generator with explicit pulse/DIR timing.
 */
#include <Arduino.h>
#include <util/atomic.h>
#include "stepgen.h"

typedef struct {
    volatile int32_t  position;
    volatile uint32_t remaining;
    volatile uint32_t acc;
    volatile uint32_t inc;
    volatile int8_t   dir;
    volatile int8_t   requested_dir;
    volatile uint8_t  low_ticks;
    volatile uint8_t  dir_setup_ticks;
    volatile uint8_t  dir_hold_ticks;
    volatile bool     step_pending;
} Axis;

static Axis axes[NUM_AXES];

static const uint8_t step_mask_for[NUM_AXES] = {
    (uint8_t)(1u << STEP_BIT(0)),
    (uint8_t)(1u << STEP_BIT(1)),
    (uint8_t)(1u << STEP_BIT(2)),
};

static inline void set_dir_bit(uint8_t axis, int8_t dir) {
    uint8_t bit = (uint8_t)(1u << DIR_BIT(axis));
    bool negative = dir < 0;
    if (DIR_INVERT_MASK & (1u << axis)) negative = !negative;
    if (negative) PORTD |= bit; else PORTD &= (uint8_t)~bit;
}

static inline void deassert_steps(void) {
#if STEP_ACTIVE_HIGH
    PORTD &= (uint8_t)~STEP_MASK;
#else
    PORTD |= STEP_MASK;
#endif
}

void stepgen_init(void) {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        axes[i].position        = 0;
        axes[i].remaining       = 0;
        axes[i].acc             = 0;
        axes[i].inc             = 0;
        axes[i].dir             = 1;
        axes[i].requested_dir   = 1;
        axes[i].low_ticks       = 0;
        axes[i].dir_setup_ticks = 0;
        axes[i].dir_hold_ticks  = 0;
        axes[i].step_pending    = false;
    }

    DDRD |= (STEP_MASK | DIR_MASK);
    PORTD &= (uint8_t)~DIR_MASK;
    deassert_steps();

    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A  = (uint16_t)((F_CPU / ISR_HZ) - 1);
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS10);
    TIMSK1 |= (1 << OCIE1A);
    interrupts();
}

void stepgen_set_target(uint8_t axis, int32_t target, uint32_t inc) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        Axis *a = &axes[axis];
        int64_t delta = (int64_t)target - (int64_t)a->position;
        int8_t new_dir = delta < 0 ? -1 : 1;
        uint32_t magnitude = delta < 0 ? (uint32_t)(-delta) : (uint32_t)delta;

        if (magnitude && new_dir != a->requested_dir) {
            a->acc = 0;
            a->step_pending = false;
        }
        a->requested_dir = new_dir;
        a->remaining = magnitude;
        a->inc = magnitude ? inc : 0;
        if (!magnitude) a->step_pending = false;
    }
}

void stepgen_stop_all(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            axes[i].remaining = 0;
            axes[i].inc = 0;
            axes[i].acc = 0;
            axes[i].step_pending = false;
            axes[i].low_ticks = 0;
            axes[i].dir_setup_ticks = 0;
            axes[i].dir_hold_ticks = 0;
        }
        deassert_steps();
    }
}

int32_t stepgen_position(uint8_t axis) {
    int32_t value;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = axes[axis].position;
    }
    return value;
}

bool stepgen_busy(void) {
    bool busy;
    int32_t ignored[NUM_AXES];
    stepgen_snapshot(ignored, &busy);
    return busy;
}

void stepgen_snapshot(int32_t positions[NUM_AXES], bool *busy) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        bool any = false;
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            positions[i] = axes[i].position;
            if (axes[i].remaining || axes[i].step_pending) any = true;
        }
        if (busy) *busy = any;
    }
}

/* STEP is asserted for one ISR interval. STEP_LOW_TICKS then blocks a new
 * assertion for a complete interval, giving a 15 kstep/s conservative ceiling.
 * A blocked overflow is retained so phase is never silently discarded. */
ISR(TIMER1_COMPA_vect) {
    deassert_steps();
    uint8_t stepbits = 0;

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        Axis *a = &axes[i];
        bool low_block = a->low_ticks != 0;
        bool hold_block = a->dir_hold_ticks != 0;
        bool setup_block = a->dir_setup_ticks != 0;
        if (a->low_ticks) a->low_ticks--;
        if (a->dir_hold_ticks) a->dir_hold_ticks--;
        if (a->dir_setup_ticks) a->dir_setup_ticks--;

        if (a->requested_dir != a->dir) {
            if (!low_block && !hold_block) {
                a->dir = a->requested_dir;
                set_dir_bit(i, a->dir);
                a->dir_setup_ticks = DIR_SETUP_TICKS;
                a->acc = 0;
                a->step_pending = false;
            }
            continue;
        }

        if (!a->remaining) {
            a->step_pending = false;
            continue;
        }

        bool issue = false;
        if (a->step_pending) {
            issue = !low_block && !hold_block && !setup_block;
        } else {
            uint32_t before = a->acc;
            a->acc = before + a->inc;
            if (a->acc < before) {
                if (!low_block && !hold_block && !setup_block) issue = true;
                else a->step_pending = true;
            }
        }

        if (issue) {
            stepbits |= step_mask_for[i];
            a->position += a->dir;
            a->remaining--;
            a->step_pending = false;
            a->low_ticks = STEP_LOW_TICKS;
            a->dir_hold_ticks = DIR_HOLD_TICKS;
        }
    }

#if STEP_ACTIVE_HIGH
    PORTD |= stepbits;
#else
    PORTD &= (uint8_t)~stepbits;
#endif
}
