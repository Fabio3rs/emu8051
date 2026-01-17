/* 8051 emulator core
 * Copyright 2006 Jari Komppa
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * (i.e. the MIT License)
 *
 * core.c
 * General emulation functions
 */

#define T0_MODE3_MASK (TMODMASK_M0_0 | TMODMASK_M1_0)

#include <stdlib.h>
#include <string.h>
#include "emu8051.h"

static void serial_tx(struct em8051 *aCPU) {
	// Test if still something to send
	if (! aCPU->serial_out_remaining_bits)
	       return;

	aCPU->serial_out_remaining_bits--;
	bool tx_bit = (aCPU->mSFR[REG_SBUF] >> aCPU->serial_out_remaining_bits);
	// Set P3.1 according to the currently clocked out SERIAL bit
	aCPU->mSFR[REG_P3] &= ~(1 << 1);
	if (tx_bit) aCPU->mSFR[REG_P3] |= (1 << 1);

	// If everything is sent now, add it to the visual buffer & raise interrupt
	if (aCPU->serial_out_remaining_bits == 0) {
		aCPU->serial_out[aCPU->serial_out_idx] = aCPU->mSFR[REG_SBUF];
		aCPU->serial_out_idx = (aCPU->serial_out_idx + 1) % sizeof(aCPU->serial_out);
		aCPU->mSFR[REG_SCON] |= (1<<1); // Set TI bit
		if (aCPU->mSFR[REG_IE] & IEMASK_ES) aCPU->serial_interrupt_trigger = 1; // Trigger Serial Interrupt
	}
}

#ifdef __8052__
static bool timer2_add_increment(struct em8051 *aCPU, int8_t increment) {
    int16_t v = aCPU->mSFR[REG_TL2];
    v+=increment;
    aCPU->mSFR[REG_TL2] = v & 0xff;
    if (v > 0xff || v < 0)
    {
        // TL2 overflowed
        v = aCPU->mSFR[REG_TH2];
        v+=increment;
        aCPU->mSFR[REG_TH2] = v & 0xff;
        if (v > 0xff)
        {
            return true;
        }
    }

    if (increment < 0 && aCPU->mSFR[REG_TL2] == aCPU->mSFR[REG_RCAP2L] && aCPU->mSFR[REG_TH2] == aCPU->mSFR[REG_RCAP2H])
    {
        return true;
    }
    return false;
}

static void timer2_tick(struct em8051 *aCPU){
    static uint8_t prev_t2;
    static uint8_t serial_tx_counter;
    int8_t increment = 0;
    bool overflow = false;

    if (aCPU->mSFR[REG_T2CON] & T2CONMASK_TR2)
    {
        if (aCPU->mSFR[REG_T2CON] & T2CONMASK_CT2)
        {
            if (prev_t2 != 0 && (aCPU->mSFR[REG_P1] & (1)) == 0){
                increment = 1;
                if (aCPU->mSFR[REG_T2CON] & T2CONMASK_EXEN2)
                    aCPU->mSFR[REG_T2CON] |= T2CONMASK_EXF2;
            }
            else
                increment = 0;
        }
        else
        {
            if (aCPU->mSFR[REG_T2CON] & (T2CONMASK_TCLK | T2CONMASK_RCLK) ||
                (aCPU->mSFR[REG_T2MOD] & T2MODMASK_T2OE && !(aCPU->mSFR[REG_T2CON] & T2CONMASK_CT2)))
                // TODO: Due to the lack of accurate emulation of internal CPU states, there is a limitation that if the targeted overflow
                // rate is higher than the machine cycle (RCAP2H/L >= 65533), the effective overflow rate will be less than the machine
                // cycle frequency. This can be worked around for baudrates below, but the clock out wont be right without doing some
                // re-work in the emu.c code that calls the tick function. A simple solution might be add a per clock tick, or change the
                // existing tick functions to accept a number that indicates how far through the machine cycle it is (0-11 === clock%12),
                // then call them every clock so that they can decide if they need to do anything themselves.
                increment = 6;
            else
                increment = 1;
        }
    }

    if (increment && aCPU->mSFR[REG_T2MOD] & T2MODMASK_DCEN && !(aCPU->mSFR[REG_P1] & (1 << 1))) {
        increment = -increment;
    }

    if (increment) {
        overflow = timer2_add_increment(aCPU, increment);
    }

    if (overflow) {
        serial_tx_counter = (serial_tx_counter + 1) % 16;

        aCPU->mSFR[REG_T2CON] |= T2CONMASK_TF2;
        // Clockout
        if (aCPU->mSFR[REG_T2MOD] & T2MODMASK_T2OE && !(aCPU->mSFR[REG_T2CON] & T2CONMASK_CT2)) {
            if (aCPU->mSFR[REG_T2CON] & (T2CONMASK_TCLK | T2CONMASK_RCLK)) {
                if (serial_tx_counter == 0)
                    aCPU->mSFR[REG_P1] ^= (1);
            } else
                aCPU->mSFR[REG_P1] ^= (1);
        }
        // Baud generator
        if (aCPU->mSFR[REG_T2CON] & (T2CONMASK_TCLK | T2CONMASK_RCLK)) {
            aCPU->mSFR[REG_T2CON] &= ~T2CONMASK_TF2;
            if (increment > 0) {
                aCPU->mSFR[REG_TL2] = aCPU->mSFR[REG_RCAP2L];
                aCPU->mSFR[REG_TH2] = aCPU->mSFR[REG_RCAP2H];
            } else {
                aCPU->mSFR[REG_TL2] = 0xff;
                aCPU->mSFR[REG_TH2] = 0xff;
            }
            if (aCPU->mSFR[REG_T2CON] & (T2CONMASK_TCLK)) {
                if (aCPU->mSFR[REG_SCON] & SCONMASK_SM1) {
                    if (serial_tx_counter == 0){
                        serial_tx(aCPU);
                    }
                }
            }
        // Auto-reload
        } else if (aCPU->mSFR[REG_T2CON] & T2CONMASK_CPRL2) {
            if (increment > 0) {
                aCPU->mSFR[REG_TL2] = aCPU->mSFR[REG_RCAP2L];
                aCPU->mSFR[REG_TH2] = aCPU->mSFR[REG_RCAP2H];
            } else {
                aCPU->mSFR[REG_TL2] = 0xff;
                aCPU->mSFR[REG_TH2] = 0xff;
            }
        }
    }

    prev_t2 = aCPU->mSFR[REG_P1] & 1;
}
#endif

static void timer_tick(struct em8051 *aCPU)
{
    uint8_t increment;
    uint16_t v;

    static uint8_t prev_t0;
    static uint8_t prev_t1;

    if ((aCPU->mSFR[REG_TMOD] & (TMODMASK_M0_0 | TMODMASK_M1_0)) == (TMODMASK_M0_0 | TMODMASK_M1_0))
    {
        // timer/counter 0 in mode 3

        increment = 0;

        // Check if we're run enabled
        if (aCPU->mSFR[REG_TCON] & TCONMASK_TR0 &&
            (!(aCPU->mSFR[REG_TMOD] & TMODMASK_GATE_0) ||
            (aCPU->mSFR[REG_TMOD] & TMODMASK_GATE_0 && aCPU->mSFR[REG_P3] & P3MASK_T0)))
        {
            // check timer / counter mode
            if (aCPU->mSFR[REG_TMOD] & TMODMASK_CT_0)
            {
                if (prev_t0 != 0 && (aCPU->mSFR[REG_P3] & P3MASK_T0) == 0)
                    increment = 1;
                else
                    increment = 0;
            }
            else
            {
                increment = 1;
            }
        }
        if (increment)
        {
            v = aCPU->mSFR[REG_TL0];
            v++;
            aCPU->mSFR[REG_TL0] = v & 0xff;
            if (v > 0xff)
            {
                // TL0 overflowed
                aCPU->mSFR[REG_TCON] |= TCONMASK_TF0;
            }
        }

        increment = 0;

        // Check if we're run enabled
        if (aCPU->mSFR[REG_TCON] & TCONMASK_TR1 &&
            (!(aCPU->mSFR[REG_TMOD] & TMODMASK_GATE_1) ||
            (aCPU->mSFR[REG_TMOD] & TMODMASK_GATE_1 && aCPU->mSFR[REG_P3] & P3MASK_T1)))
        {
            // check timer / counter mode
            if (aCPU->mSFR[REG_TMOD] & TMODMASK_CT_1)
            {
                if (prev_t1 != 0 && (aCPU->mSFR[REG_P3] & P3MASK_T1) == 0)
                    increment = 1;
                else
                    increment = 0;
            }
            else
            {
                increment = 1;
            }
        }

        if (increment)
        {
            v = aCPU->mSFR[REG_TH0];
            v++;
            aCPU->mSFR[REG_TH0] = v & 0xff;
            if (v > 0xff)
            {
                // TH0 overflowed
                aCPU->mSFR[REG_TCON] |= TCONMASK_TF1;
            }
        }

    }

    {   // Timer/counter 0

        increment = 0;

        // Check if we're run enabled
        if (aCPU->mSFR[REG_TCON] & TCONMASK_TR0 &&
            (!(aCPU->mSFR[REG_TMOD] & TMODMASK_GATE_0) ||
            (aCPU->mSFR[REG_TMOD] & TMODMASK_GATE_0 && aCPU->mSFR[REG_P3] & P3MASK_T0)))
        {
            // check timer / counter mode
            if (aCPU->mSFR[REG_TMOD] & TMODMASK_CT_0)
            {
                if (prev_t0 != 0 && (aCPU->mSFR[REG_P3] & P3MASK_T0) == 0)
                    increment = 1;
                else
                    increment = 0;
            }
            else
            {
                increment = 1;
            }
        }

        if (increment)
        {
            switch (aCPU->mSFR[REG_TMOD] & (TMODMASK_M0_0 | TMODMASK_M1_0))
            {
            case 0: // 13-bit timer
                v = aCPU->mSFR[REG_TL0] & 0x1f; // lower 5 bits of TL0
                v++;
                aCPU->mSFR[REG_TL0] = (aCPU->mSFR[REG_TL0] & ~0x1f) | (v & 0x1f);
                if (v > 0x1f)
                {
                    // TL0 overflowed
                    v = aCPU->mSFR[REG_TH0];
                    v++;
                    aCPU->mSFR[REG_TH0] = v & 0xff;
                    if (v > 0xff)
                    {
                        // TH0 overflowed; set bit
                        aCPU->mSFR[REG_TCON] |= TCONMASK_TF0;
                    }
                }
                break;
            case TMODMASK_M0_0: // 16-bit timer/counter
                v = aCPU->mSFR[REG_TL0];
                v++;
                aCPU->mSFR[REG_TL0] = v & 0xff;
                if (v > 0xff)
                {
                    // TL0 overflowed
                    v = aCPU->mSFR[REG_TH0];
                    v++;
                    aCPU->mSFR[REG_TH0] = v & 0xff;
                    if (v > 0xff)
                    {
                        // TH0 overflowed; set bit
                        aCPU->mSFR[REG_TCON] |= TCONMASK_TF0;
                    }
                }
                break;
            case TMODMASK_M1_0: // 8-bit auto-reload timer
                v = aCPU->mSFR[REG_TL0];
                v++;
                aCPU->mSFR[REG_TL0] = v & 0xff;
                if (v > 0xff)
                {
                    // TL0 overflowed; reload
                    aCPU->mSFR[REG_TL0] = aCPU->mSFR[REG_TH0];
                    aCPU->mSFR[REG_TCON] |= TCONMASK_TF0;
                }
                break;
            default: // two 8-bit timers
                // mode 3 handled above
                break;
            }
        }
    }

    {   // Timer/counter 1

        increment = 0;

        if (aCPU->mSFR[REG_TCON] & TCONMASK_TR1 &&
            (!(aCPU->mSFR[REG_TMOD] & TMODMASK_GATE_1) ||
            (aCPU->mSFR[REG_TMOD] & TMODMASK_GATE_1 && aCPU->mSFR[REG_P3] & P3MASK_T1)))
        {
            if (aCPU->mSFR[REG_TMOD] & TMODMASK_CT_1)
            {
                if (prev_t1 != 0 && (aCPU->mSFR[REG_P3] & P3MASK_T1) == 0)
                    increment = 1;
                else
                    increment = 0;
            }
            else
            {
                increment = 1;
            }
        }

        if (increment)
        {
            switch (aCPU->mSFR[REG_TMOD] & (TMODMASK_M0_1 | TMODMASK_M1_1))
            {
            case 0: // 13-bit timer
                v = aCPU->mSFR[REG_TL1] & 0x1f; // lower 5 bits of TL0
                v++;
                aCPU->mSFR[REG_TL1] = (aCPU->mSFR[REG_TL1] & ~0x1f) | (v & 0x1f);
                if (v > 0x1f)
                {
                    // TL1 overflowed
                    v = aCPU->mSFR[REG_TH1];
                    v++;
                    aCPU->mSFR[REG_TH1] = v & 0xff;
                    if (v > 0xff)
                    {
                        // TH1 overflowed; set bit
                        // Only update TF1 if timer 0 is not in "mode 3"
                    if (!((aCPU->mSFR[REG_TMOD] & T0_MODE3_MASK ) == T0_MODE3_MASK))
                        aCPU->mSFR[REG_TCON] |= TCONMASK_TF1;

                    }
                }
                break;
            case TMODMASK_M0_1: // 16-bit timer/counter
                v = aCPU->mSFR[REG_TL1];
                v++;
                aCPU->mSFR[REG_TL1] = v & 0xff;
                if (v > 0xff)
                {
                    // TL1 overflowed
                    v = aCPU->mSFR[REG_TH1];
                    v++;
                    aCPU->mSFR[REG_TH1] = v & 0xff;
                    if (v > 0xff)
                    {
                        // TH1 overflowed; set bit
                        // Only update TF1 if timer 0 is not in "mode 3"

                        if (!((aCPU->mSFR[REG_TMOD] & T0_MODE3_MASK)  == T0_MODE3_MASK))
                            aCPU->mSFR[REG_TCON] |= TCONMASK_TF1;

                    }
                }
                break;
            case TMODMASK_M1_1: // 8-bit auto-reload timer
                v = aCPU->mSFR[REG_TL1];
                v++;
                aCPU->mSFR[REG_TL1] = v & 0xff;
                if (v > 0xff)
                {
                    // TL0 overflowed; reload
                    aCPU->mSFR[REG_TL1] = aCPU->mSFR[REG_TH1];
                    // Only update TF1 if timer 0 is not in "mode 3"


                    if (!((aCPU->mSFR[REG_TMOD] & T0_MODE3_MASK ) == T0_MODE3_MASK))
                        aCPU->mSFR[REG_TCON] |= TCONMASK_TF1;

                }
                break;
            default: // disabled
                break;
            }

            // If Timer1 overflowed, see if we need to send a serial bit
            if (aCPU->mSFR[REG_TCON] & TCONMASK_TF1) {
                if (aCPU->mSFR[REG_SCON] & SCONMASK_SM1) {
                    #ifdef __8052__
                    if (!(aCPU->mSFR[REG_T2CON] & T2CONMASK_TCLK)) {
                    #endif
                        serial_tx(aCPU);
                        aCPU->mSFR[REG_TCON] &= ~TCONMASK_TF1; // clear overflow flag
                    #ifdef __8052__
                    }
                    #endif
                }
            }
        }
    }

    prev_t0 = aCPU->mSFR[REG_P3] & P3MASK_T0;
    prev_t1 = aCPU->mSFR[REG_P3] & P3MASK_T1;
}

#ifdef __SAB80C517__
// SAB80C517: SFR write callback for interrupt enable/priority registers
// Sets 1-instruction delay before interrupts can be recognized (per datasheet)
static void sfr_write_irq_inhibit(struct em8051 *aCPU, uint8_t aRegister)
{
    (void)aRegister;  // Unused parameter
    aCPU->irq_inhibit = 1;  // Set 1-instruction delay
}
#endif // __SAB80C517__

void handle_interrupts(struct em8051 *aCPU)
{
    int16_t dest_ip = -1;
    uint8_t hi = 0;
    uint8_t lo = 0;

    // can't interrupt high level
    if (aCPU->mInterruptActive > 1)
        return;

    if (aCPU->mSFR[REG_IE] & IEMASK_EA)
    {
        // Interrupts enabled
        if (aCPU->mSFR[REG_IE] & IEMASK_EX0 && aCPU->mSFR[REG_TCON] & TCONMASK_IE0)
        {
            // External int 0
            dest_ip = ISR_INT0;
            if (aCPU->mSFR[REG_IP] & IPMASK_PX0)
                hi = 1;
            lo = 1;
        }
        if (aCPU->mSFR[REG_IE] & IEMASK_ET0 && aCPU->mSFR[REG_TCON] & TCONMASK_TF0 && !hi)
        {
            // Timer/counter 0
            if (!lo)
            {
                dest_ip = ISR_TF0;
                lo = 1;
            }
            if (aCPU->mSFR[REG_IP] & IPMASK_PT0)
            {
                hi = 1;
                dest_ip = ISR_TF0;
            }
        }
        if (aCPU->mSFR[REG_IE] & IEMASK_EX1 && aCPU->mSFR[REG_TCON] & TCONMASK_IE1 && !hi)
        {
            // External int 1
            if (!lo)
            {
                dest_ip = ISR_INT1;
                lo = 1;
            }
            if (aCPU->mSFR[REG_IP] & IPMASK_PX1)
            {
                hi = 1;
                dest_ip = ISR_INT1;
            }
        }
        if (aCPU->mSFR[REG_IE] & IEMASK_ET1 && aCPU->mSFR[REG_TCON] & TCONMASK_TF1 && !hi)
        {
            // Timer/counter 1 enabled
            if (!lo)
            {
                dest_ip = ISR_TF1;
                lo = 1;
            }
            if (aCPU->mSFR[REG_IP] & IPMASK_PT1)
            {
                hi = 1;
                dest_ip = ISR_TF1;
            }
        }
        if (aCPU->mSFR[REG_IE] & IEMASK_ES && aCPU->serial_interrupt_trigger && !hi)
        {
            // Serial port interrupt
            if (!lo)
            {
                dest_ip = ISR_SR;
                lo = 1;
            }
            if (aCPU->mSFR[REG_IP] & IPMASK_PS)
            {
                hi = 1;
                dest_ip = ISR_SR;
            }
            // TODO
        }
#ifdef __8052__
        if (aCPU->mSFR[REG_IE] & IEMASK_ET2 &&
            (aCPU->mSFR[REG_T2CON] & T2CONMASK_TF2 ||
                (aCPU->mSFR[REG_T2CON] & T2CONMASK_EXF2 && !aCPU->mSFR[REG_T2MOD] & T2MODMASK_DCEN)
            ) && !hi)
        {
            // Timer 2 (8052 only)
            if (!lo)
            {
                dest_ip = ISR_TF2;
                lo = 1;
            }
            if (aCPU->mSFR[REG_IP] & IPMASK_PT2)
            {
                hi = 1;
                dest_ip = ISR_TF2;
            }
            // TODO
        }
#endif // __8052__
    }

    // no interrupt
    if (dest_ip == -1)
        return;

    // can't interrupt same-level
    if (aCPU->mInterruptActive == 1 && !hi)
        return;

    // some interrupt occurs; perform LCALL
    aCPU->mSFR[REG_PCON] &= ~0x01; // clear idle flag, but not Power down flag
    push_to_stack(aCPU, aCPU->mPC & 0xff);
    push_to_stack(aCPU, aCPU->mPC >> 8);
    aCPU->mPC = dest_ip;
    // wait for 2 ticks instead of one since we were not executing
    // this LCALL before.
    aCPU->mTickDelay = 2;
    switch (dest_ip)
    {
    case ISR_TF0:
        aCPU->mSFR[REG_TCON] &= ~TCONMASK_TF0; // clear overflow flag
        break;
    case ISR_TF1:
        aCPU->mSFR[REG_TCON] &= ~TCONMASK_TF1; // clear overflow flag
        break;
    case ISR_SR:
        aCPU->serial_interrupt_trigger = 0; // handled the serial interrupt trigger
        break;
#ifdef __8052__
    case ISR_TF2:
        // Clear Timer 2 overflow flag (TF2) - bit 7 of T2CON
        // CRITICAL: Without this, Timer 2 fires continuously causing stack overflow!
        // Note: EXF2 (bit 6) is NOT automatically cleared, must be cleared by software
        aCPU->mSFR[REG_T2CON] &= ~T2CONMASK_TF2;
        break;
#endif // __8052__
    }

    if (hi)
    {
        aCPU->mInterruptActive |= 2;
    }
    else
    {
        aCPU->mInterruptActive = 1;
    }
    aCPU->int_a[hi] = aCPU->mSFR[REG_ACC];
    aCPU->int_psw[hi] = aCPU->mSFR[REG_PSW];
    aCPU->int_sp[hi] = aCPU->mSFR[REG_SP];
}

#ifdef __SAB80C517__
// SAB80C517: Handle interrupts with 4-level priority system and 14 sources
// Datasheet: Table 8-1 defines priority groups and polling order
// OPTIMIZED: Cache SFR reads and pre-calculate priorities to reduce overhead
void handle_interrupts_80c517(struct em8051 *aCPU)
{
    int16_t dest_ip = -1;
    uint8_t highest_priority = 0;
    uint8_t current_level = aCPU->mInterruptActive;

    // Check if no interrupt is currently active (sentinel value)
    // This allows priority-0 interrupts to be accepted
    bool no_int_active = (current_level == SAB80C517_NO_INT_ACTIVE);

    // =========================================================================
    // OPTIMIZATION: Cache all SFR reads at start (reduces ~60 reads to ~10)
    // =========================================================================
    uint8_t ien0  = aCPU->mSFR[REG_IEN0];
    uint8_t ien1  = aCPU->mSFR[REG_IEN1];
    uint8_t ien2  = aCPU->mSFR[REG_IEN2];
    uint8_t tcon  = aCPU->mSFR[REG_TCON];
    uint8_t ircon = aCPU->mSFR[REG_IRCON];
    uint8_t ip0   = aCPU->mSFR[REG_IP0];
    uint8_t ip1   = aCPU->mSFR[REG_IP1];

    // Global interrupt enable check (early exit)
    if (!(ien0 & IEN0MASK_EA))
        return;

    // Pre-calculate all 6 group priorities (2 SFR reads instead of 30)
    // Priority encoding: level = (IP1.bit << 1) | IP0.bit
    uint8_t group_pri[6];
    for (int g = 0; g < 6; g++)
        group_pri[g] = (uint8_t)(((ip1 >> g) & 1) << 1) | ((ip0 >> g) & 1);

    // =========================================================================
    // Scan all 14 interrupt sources in datasheet polling order
    // Accept if: no interrupt is active OR priority is strictly greater than current
    // Within same priority level: first in polling order wins
    // =========================================================================

    // Group 0 (IP0.0/IP1.0): IE0, SR1, IADC
    if ((ien0 & IEN0MASK_EX0) && (tcon & TCONMASK_IE0)) {
        uint8_t pri = group_pri[0];
        if ((no_int_active || pri > current_level) && (dest_ip < 0 || pri > highest_priority)) {
            highest_priority = pri;
            dest_ip = ISR_517_INT0;
        }
    }
    if ((ien2 & IEN2MASK_ES1) && (aCPU->mSFR[REG_S1CON] & (SCONMASK_RI | SCONMASK_TI))) {
        uint8_t pri = group_pri[0];
        if ((no_int_active || pri > current_level) && pri > highest_priority) {
            highest_priority = pri;
            dest_ip = ISR_517_SR1;
        }
    }
    if ((ien1 & IEN1MASK_EADC) && (ircon & IRCONMASK_IADC)) {
        uint8_t pri = group_pri[0];
        if ((no_int_active || pri > current_level) && pri > highest_priority) {
            highest_priority = pri;
            dest_ip = ISR_517_IADC;
        }
    }

    // Group 1 (IP0.1/IP1.1): TF0, IEX2
    if ((ien0 & IEN0MASK_ET0) && (tcon & TCONMASK_TF0)) {
        uint8_t pri = group_pri[1];
        if ((no_int_active || pri > current_level) && (dest_ip < 0 || pri > highest_priority)) {
            highest_priority = pri;
            dest_ip = ISR_517_TF0;
        }
    }
    if ((ien1 & IEN1MASK_EX2) && (ircon & IRCONMASK_IEX2)) {
        uint8_t pri = group_pri[1];
        if ((no_int_active || pri > current_level) && pri > highest_priority) {
            highest_priority = pri;
            dest_ip = ISR_517_IEX2;
        }
    }

    // Group 2 (IP0.2/IP1.2): IE1, IEX3
    if ((ien0 & IEN0MASK_EX1) && (tcon & TCONMASK_IE1)) {
        uint8_t pri = group_pri[2];
        if ((no_int_active || pri > current_level) && (dest_ip < 0 || pri > highest_priority)) {
            highest_priority = pri;
            dest_ip = ISR_517_INT1;
        }
    }
    if ((ien1 & IEN1MASK_EX3) && (ircon & IRCONMASK_IEX3)) {
        uint8_t pri = group_pri[2];
        if ((no_int_active || pri > current_level) && pri > highest_priority) {
            highest_priority = pri;
            dest_ip = ISR_517_IEX3;
        }
    }

    // Group 3 (IP0.3/IP1.3): TF1, CTF, IEX4
    if ((ien0 & IEN0MASK_ET1) && (tcon & TCONMASK_TF1)) {
        uint8_t pri = group_pri[3];
        if ((no_int_active || pri > current_level) && (dest_ip < 0 || pri > highest_priority)) {
            highest_priority = pri;
            dest_ip = ISR_517_TF1;
        }
    }
    if ((ien2 & IEN2MASK_ECT) && (aCPU->mSFR[REG_CTCON] & CTCONMASK_CTF)) {
        uint8_t pri = group_pri[3];
        if ((no_int_active || pri > current_level) && pri > highest_priority) {
            highest_priority = pri;
            dest_ip = ISR_517_CTF;
        }
    }
    if ((ien1 & IEN1MASK_EX4) && (ircon & IRCONMASK_IEX4)) {
        uint8_t pri = group_pri[3];
        if ((no_int_active || pri > current_level) && pri > highest_priority) {
            highest_priority = pri;
            dest_ip = ISR_517_IEX4;
        }
    }

    // Group 4 (IP0.4/IP1.4): SR0, IEX5
    if ((ien0 & IEN0MASK_ES0) && aCPU->serial_interrupt_trigger) {
        uint8_t pri = group_pri[4];
        if ((no_int_active || pri > current_level) && (dest_ip < 0 || pri > highest_priority)) {
            highest_priority = pri;
            dest_ip = ISR_517_SR0;
        }
    }
    if ((ien1 & IEN1MASK_EX5) && (ircon & IRCONMASK_IEX5)) {
        uint8_t pri = group_pri[4];
        if ((no_int_active || pri > current_level) && pri > highest_priority) {
            highest_priority = pri;
            dest_ip = ISR_517_IEX5;
        }
    }

    // Group 5 (IP0.5/IP1.5): TF2/EXF2, IEX6
    if ((ien0 & IEN0MASK_ET2) &&
        ((ircon & IRCONMASK_TF2) || ((ircon & IRCONMASK_EXF2) && (ien1 & IEN1MASK_EXEN2)))) {
        uint8_t pri = group_pri[5];
        if ((no_int_active || pri > current_level) && (dest_ip < 0 || pri > highest_priority)) {
            highest_priority = pri;
            dest_ip = ISR_517_TF2;
        }
    }
    if ((ien1 & IEN1MASK_EX6) && (ircon & IRCONMASK_IEX6)) {
        uint8_t pri = group_pri[5];
        if ((no_int_active || pri > current_level) && pri > highest_priority) {
            highest_priority = pri;
            dest_ip = ISR_517_IEX6;
        }
    }

    // No interrupt found
    if (dest_ip < 0)
        return;

    // Perform LCALL to ISR
    aCPU->mSFR[REG_PCON] &= ~0x01;  // Clear idle flag
    push_to_stack(aCPU, aCPU->mPC & 0xff);
    push_to_stack(aCPU, aCPU->mPC >> 8);
    aCPU->mPC = dest_ip;
    aCPU->mTickDelay = 2;

    // Clear interrupt flags (hardware auto-clear policy per datasheet)
    switch (dest_ip) {
        case ISR_517_TF0:
            aCPU->mSFR[REG_TCON] &= ~TCONMASK_TF0;
            break;
        case ISR_517_TF1:
            aCPU->mSFR[REG_TCON] &= ~TCONMASK_TF1;
            break;
        case ISR_517_IEX2:
            aCPU->mSFR[REG_IRCON] &= ~IRCONMASK_IEX2;
            break;
        case ISR_517_IEX3:
            aCPU->mSFR[REG_IRCON] &= ~IRCONMASK_IEX3;
            break;
        case ISR_517_IEX4:
            aCPU->mSFR[REG_IRCON] &= ~IRCONMASK_IEX4;
            break;
        case ISR_517_IEX5:
            aCPU->mSFR[REG_IRCON] &= ~IRCONMASK_IEX5;
            break;
        case ISR_517_IEX6:
            aCPU->mSFR[REG_IRCON] &= ~IRCONMASK_IEX6;
            break;
        case ISR_517_INT0:
            // Clear IE0 only if edge-triggered (IT0=1)
            if (aCPU->mSFR[REG_TCON] & TCONMASK_IT0)
                aCPU->mSFR[REG_TCON] &= ~TCONMASK_IE0;
            break;
        case ISR_517_INT1:
            // Clear IE1 only if edge-triggered (IT1=1)
            if (aCPU->mSFR[REG_TCON] & TCONMASK_IT1)
                aCPU->mSFR[REG_TCON] &= ~TCONMASK_IE1;
            break;
        case ISR_517_SR0:
            aCPU->serial_interrupt_trigger = 0;
            break;
        // TF2, EXF2, IADC, RI1/TI1, CTF: Software must clear (no action)
    }

    // Save registers for this priority level
    aCPU->int_a_517[highest_priority] = aCPU->mSFR[REG_ACC];
    aCPU->int_psw_517[highest_priority] = aCPU->mSFR[REG_PSW];
    aCPU->int_sp_517[highest_priority] = aCPU->mSFR[REG_SP];

    // Update active priority level
    aCPU->mInterruptActive = highest_priority;
}
#endif // __SAB80C517__

bool tick(struct em8051 *aCPU)
{
    uint8_t v;
    bool ticked = false;

    if (aCPU->mTickDelay)
    {
        aCPU->mTickDelay--;
    }

    // Test for Power Down
    if (aCPU->mTickDelay == 0 && (aCPU->mSFR[REG_PCON]) & 0x02) {
        aCPU->mTickDelay = 1;
        return 1;
    }

    // Interrupts are sent if the following cases are not true:
    // 1. interrupt of equal or higher priority is in progress (tested inside function)
    // 2. current cycle is not the final cycle of instruction (tickdelay = 0)
    // 3. the instruction in progress is RETI or any write to the IE or IP regs
    if (aCPU->mTickDelay == 0)
    {
#ifdef __SAB80C517__
        // Decrement IRQ inhibit counter
        if (aCPU->irq_inhibit > 0)
            aCPU->irq_inhibit--;

        // Only handle interrupts if not inhibited
        if (aCPU->irq_inhibit == 0)
            handle_interrupts_80c517(aCPU);
#else
        handle_interrupts(aCPU);
#endif
    }

    if (aCPU->mTickDelay == 0)
    {
        // IDL activate the idle mode to save power
        bool is_idle = (aCPU->mSFR[REG_PCON]) & 0x01;
        if (is_idle) {
            aCPU->mTickDelay = 1;
        } else {
            aCPU->mTickDelay = aCPU->op[aCPU->mCodeMem[aCPU->mPC & (aCPU->mCodeMemMaxIdx)]](aCPU);
        }
        ticked = true;
        // update parity bit
        v = aCPU->mSFR[REG_ACC];
        v ^= v >> 4;
        v &= 0xf;
        v = (0x6996 >> v) & 1;
        aCPU->mSFR[REG_PSW] = (aCPU->mSFR[REG_PSW] & ~PSWMASK_P) | (v * PSWMASK_P);
    }

    timer_tick(aCPU);
    #ifdef __8052__
    timer2_tick(aCPU);
    #endif

    return ticked;
}

uint8_t decode(struct em8051 *aCPU, uint16_t aPosition, char *aBuffer)
{
    bool is_idle = (aCPU->mSFR[REG_PCON]) & 0x01;
    if (is_idle) {
        strcpy(aBuffer, "IDLE");
        return 0;
    }
    bool is_powerdown = (aCPU->mSFR[REG_PCON]) & 0x02;
    if (is_powerdown) {
        strcpy(aBuffer, "POWER DOWN");
        return 0;
    }
    return aCPU->dec[aCPU->mCodeMem[aPosition & (aCPU->mCodeMemMaxIdx)]](aCPU, aPosition, aBuffer);
}

void disasm_setptrs(struct em8051 *aCPU);
void op_setptrs(struct em8051 *aCPU);

void reset(struct em8051 *aCPU, bool aWipe)
{
    // clear memory, set registers to bootup values, etc
    if (aWipe)
    {
        memset(aCPU->mCodeMem, 0, aCPU->mCodeMemMaxIdx+1);
        memset(aCPU->mExtData, 0, aCPU->mExtDataMaxIdx+1);
        memset(aCPU->mLowerData, 0, 128);
        if (aCPU->mUpperData)
            memset(aCPU->mUpperData, 0, 128);
    }

    memset(aCPU->mSFR, 0, 128);

    aCPU->mPC = 0;
    aCPU->mTickDelay = 0;
    aCPU->mSFR[REG_SP] = 7;
    aCPU->mSFR[REG_P0] = 0xff;
    aCPU->mSFR[REG_P1] = 0xff;
    aCPU->mSFR[REG_P2] = 0xff;
    aCPU->mSFR[REG_P3] = 0xff;

    // Power-off flag will be 1 only after a power on (cold reset).
    // A warm reset doesn’t affect the value of this bit
    // ... Therefore, we only set it if aWipe is 1
    if (aWipe)
        aCPU->mSFR[REG_PCON] |= (1<<4);

    // Random values
    if (aWipe)
        aCPU->mSFR[REG_SBUF] = rand();

    // build function pointer lists

    disasm_setptrs(aCPU);
    op_setptrs(aCPU);

    // Clean internal variables
#ifdef __SAB80C517__
    aCPU->mInterruptActive = SAB80C517_NO_INT_ACTIVE;  // No interrupt active
    // Initialize SAB80C517 interrupt state
    aCPU->irq_inhibit = 0;
#else
    aCPU->mInterruptActive = 0;
#endif

#ifdef __SAB80C517__

    // Register SFR write callbacks for IRQ inhibit
    // These registers trigger 1-instruction delay before interrupt recognition
    aCPU->sfrwrite[REG_IEN0 - 0x80] = sfr_write_irq_inhibit;
    aCPU->sfrwrite[REG_IEN1 - 0x80] = sfr_write_irq_inhibit;
    aCPU->sfrwrite[REG_IEN2 - 0x80] = sfr_write_irq_inhibit;
    aCPU->sfrwrite[REG_IP0 - 0x80]  = sfr_write_irq_inhibit;
    aCPU->sfrwrite[REG_IP1 - 0x80]  = sfr_write_irq_inhibit;
#endif

    // Clean Serial
    aCPU->serial_interrupt_trigger = 0;
    aCPU->serial_out_remaining_bits = 0;
}
