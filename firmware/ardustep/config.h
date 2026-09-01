/*
 * config.h  --  board / pin / timing configuration for ardustep
 *
 * Target: Arduino Uno / Nano (ATmega328P @ 16 MHz).
 *
 * Pin map (all STEP+DIR live on PORTD so a single port write moves every axis
 * on the same edge; D0/D1 are left for the USART):
 *
 *      Axis    STEP        DIR
 *      X       D2 (PD2)    D5 (PD5)
 *      Y       D3 (PD3)    D6 (PD6)
 *      Z       D4 (PD4)    D7 (PD7)
 *
 *      Limit X = A0 (PC0),  Limit Y = A1 (PC1),  Limit Z = A2 (PC2)
 *          (inputs, internal pull-ups, active-LOW: pressed == pin low)
 *
 *      ENABLE out = D8 (PB0)   (drives stepper-driver /EN, level set by invert)
 *      SPINDLE out= D9 (PB1)   (on/off for now; PWM is future work because
 *                               Timer1 is taken by the step ISR)
 */
#ifndef ARDUSTEP_CONFIG_H
#define ARDUSTEP_CONFIG_H

#include "protocol.h"

/* ---- serial -------------------------------------------------------------- */
#define SERIAL_BAUD     1000000UL   /* 1 Mbaud: 0% error at 16 MHz            */

/* ---- step generator timing ---------------------------------------------- */
#define ISR_HZ          30000UL     /* Timer1 CTC DDS clock                   */
#define CMD_INTERVAL_US 1000UL      /* expected host cycle; sets interp slope */
#define WATCHDOG_MS     50          /* no valid packet for this long -> stop  */

/* One tick high plus one complete tick low guarantees distinct STEP edges.
 * Direction changes are performed by the ISR and get one tick of hold and
 * setup time. At 30 kHz each tick is about 33.3 us. */
#define STEP_LOW_TICKS  1
#define DIR_SETUP_TICKS 1
#define DIR_HOLD_TICKS  1
#define MAX_STEP_RATE   (ISR_HZ / (1UL + STEP_LOW_TICKS))
#define TICKS_PER_CMD   ((ISR_HZ * CMD_INTERVAL_US) / 1000000UL)

/* ---- PORTD bit layout (do not move off PORTD without reworking the ISR) -- */
#define STEP_MASK       0x1C        /* PD2|PD3|PD4                            */
#define DIR_MASK        0xE0        /* PD5|PD6|PD7                            */
#define STEP_BIT(axis)  (2 + (axis))   /* PD2..PD4 */
#define DIR_BIT(axis)   (5 + (axis))   /* PD5..PD7 */

/* Per-axis direction inversion. Set a bit to flip that axis' DIR polarity so
 * positive machine motion matches your wiring. Bit i -> axis i. */
#define DIR_INVERT_MASK 0x00

/* STEP pulse polarity: 0 = active-high pulse (most drivers). */
#define STEP_ACTIVE_HIGH 1

/* ---- control / IO pins on PORTB ----------------------------------------- */
#define ENABLE_PIN_BIT  0           /* PB0 / D8  */
#define SPINDLE_PIN_BIT 1           /* PB1 / D9  */
#define ENABLE_ACTIVE_HIGH 1        /* 1: drive EN high when enabled         */

/* ---- limit inputs on PORTC ---------------------------------------------- */
#define LIMIT_MASK      0x07        /* PC0|PC1|PC2, active-low w/ pull-ups    */

#endif /* ARDUSTEP_CONFIG_H */
