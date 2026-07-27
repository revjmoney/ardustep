/*
 * stepgen.h  --  per-axis DDS (direct digital synthesis) step generator.
 *
 * A Timer1 CTC interrupt fires at ISR_HZ. On each tick every axis adds its
 * 32-bit phase `inc` to a 32-bit accumulator; an accumulator overflow emits one
 * STEP pulse and advances that axis by one step toward its target. The output
 * step frequency of an axis is therefore  f = inc / 2^32 * ISR_HZ.
 *
 * The host sends absolute position targets; the main loop converts the delta to
 * the new target into a phase increment (see ardustep.ino) so the axis sweeps
 * from its current position to the target over one host interval -- this is the
 * "thin buffer" first-order interpolation.
 */
#ifndef ARDUSTEP_STEPGEN_H
#define ARDUSTEP_STEPGEN_H

#include <stdint.h>
#include "config.h"

void    stepgen_init(void);

/* Atomically retarget one axis. `target` is absolute, in steps. `inc` is the
 * precomputed phase increment for this interval. `underrun` is set true if the
 * move was clamped to the 1-step-per-tick ceiling (the axis cannot keep up). */
void    stepgen_set_target(uint8_t axis, int32_t target, uint32_t inc,
                           bool *underrun);

/* Stop everything immediately (zero all phase increments). */
void    stepgen_stop_all(void);

/* Atomic 32-bit read of an axis' actual generated step count. */
int32_t stepgen_position(uint8_t axis);

/* True if any axis still has steps remaining to its target. */
bool    stepgen_busy(void);

#endif /* ARDUSTEP_STEPGEN_H */
