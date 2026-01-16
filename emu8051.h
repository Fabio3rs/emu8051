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
 * emu8051.h
 * Emulator core header file
 */

#include <stdint.h>
#include <stdbool.h>

struct em8051;

// Operation: returns number of ticks the operation should take
typedef uint8_t (*em8051operation)(struct em8051 *aCPU);

// Decodes opcode at position, and fills the buffer with the assembler code.
// Returns how many bytes the opcode takes.
typedef uint8_t (*em8051decoder)(struct em8051 *aCPU, uint16_t aPosition, char *aBuffer);

// Callback: some exceptional situation occurred. See EM8051_EXCEPTION enum, below
typedef void (*em8051exception)(struct em8051 *aCPU, int aCode);

// Callback: an SFR register is about to be read (not called for 'a' ops nor psw changes)
// Default is to return the value in the SFR register. Ports may act differently.
typedef uint8_t (*em8051sfrread)(struct em8051 *aCPU, uint8_t aRegister);

// Callback: an SFR register has changed (not called for 'a' ops)
// Default is to do nothing
typedef void (*em8051sfrwrite)(struct em8051 *aCPU, uint8_t aRegister);

// Callback: writing to external memory
// Default is to update external memory
// (can be used to control some peripherals)
typedef void (*em8051xwrite)(struct em8051 *aCPU, uint16_t aAddress, uint8_t aValue);

// Callback: reading from external memory
// Default is to return the value in external memory
// (can be used to control some peripherals)
typedef uint8_t (*em8051xread)(struct em8051 *aCPU, uint16_t aAddress);


struct em8051
{
    unsigned char *mCodeMem; // 1k - 64k, must be power of 2
    uint16_t mCodeMemMaxIdx;
    unsigned char *mExtData; // 0 - 64k, must be power of 2
    uint16_t mExtDataMaxIdx;
    unsigned char mLowerData[128]; // 128 bytes
    unsigned char *mUpperData; // 0 or 128 bytes; leave to NULL if none
    unsigned char mSFR[128]; // 128 bytes; (special function registers)
    uint16_t mPC; // Program Counter; outside memory area
    uint8_t mTickDelay; // How many ticks should we delay before continuing
    em8051operation op[256]; // function pointers to opcode handlers
    em8051decoder dec[256]; // opcode-to-string decoder handlers
    em8051exception except; // callback: exceptional situation occurred
    em8051sfrread sfrread[128]; // callback array: SFR register being read
    em8051sfrwrite sfrwrite[128]; // callback array: SFR register written
    em8051xread xread; // callback: external memory being read
    em8051xwrite xwrite; // callback: external memory being written

    // Internal values for interrupt services etc.
#ifdef __SAB80C517__
    // SAB80C517: 4-level priority system (0-3)
    // Sentinel value 0xFF means "no interrupt active"
    uint8_t mInterruptActive;   // Current priority level (0-3) or 0xFF if none
    uint8_t int_a_517[4];       // ACC saved per priority level
    uint8_t int_psw_517[4];     // PSW saved per priority level
    uint8_t int_sp_517[4];      // SP saved per priority level
    uint8_t irq_inhibit;        // IRQ inhibit counter (0=allow, >0=delay)

    // Keep old arrays for ABI compatibility (unused in SAB80C517 mode)
    uint8_t int_a[2];
    uint8_t int_psw[2];
    uint8_t int_sp[2];
#else
    // 8051/8052: 2-level priority system
    uint8_t mInterruptActive;   // 0=none, 1=low active, 2=high active
    uint8_t int_a[2];           // ACC saved for low/high level
    uint8_t int_psw[2];         // PSW saved for low/high level
    uint8_t int_sp[2];          // SP saved for low/high level
#endif

    // Internal handling of UART
    char serial_out[18]; // The shown size is only 18 chars
    uint8_t serial_out_idx;
    uint8_t serial_out_remaining_bits;
    bool serial_interrupt_trigger;
};

// set the emulator into reset state. Must be called before tick(), as
// it also initializes the function pointers. aWipe tells whether to reset
// all memory to zero.
void reset(struct em8051 *aCPU, bool aWipe);

// run one emulator tick, or 12 hardware clock cycles.
// returns "true" if a new operation was executed.
bool tick(struct em8051 *aCPU);

// decode the next operation as character string.
// buffer must be big enough (64 bytes is very safe).
// Returns length of opcode.
uint8_t decode(struct em8051 *aCPU, uint16_t aPosition, char *aBuffer);

// Load an intel hex format object file. Returns negative for errors.
int load_obj(struct em8051 *aCPU, char *aFilename);

// Alternate way to execute an opcode (switch-structure instead of function pointers)
uint8_t do_op(struct em8051 *aCPU);

// Internal: Pushes a value into stack
void push_to_stack(struct em8051 *aCPU, uint8_t aValue);


// SFR register locations
enum SFR_REGS
{
    REG_ACC = 0xE0 - 0x80,
    REG_B   = 0xF0 - 0x80,
    REG_PSW = 0xD0 - 0x80,
    REG_SP  = 0x81 - 0x80,
    REG_DPL = 0x82 - 0x80,
    REG_DPH = 0x83 - 0x80,
    REG_P0  = 0x80 - 0x80,
    REG_P1  = 0x90 - 0x80,
    REG_P2  = 0xA0 - 0x80,
    REG_P3  = 0xB0 - 0x80,
    REG_IP  = 0xB8 - 0x80,
    REG_IE  = 0xA8 - 0x80,
    REG_TMOD = 0x89 - 0x80,
    REG_TCON = 0x88 - 0x80,
    REG_TH0 = 0x8C - 0x80,
    REG_TL0 = 0x8A - 0x80,
    REG_TH1 = 0x8D - 0x80,
    REG_TL1 = 0x8B - 0x80,
    REG_SCON = 0x98 - 0x80,
    REG_SBUF = 0x99 - 0x80,
    REG_PCON = 0x87 - 0x80,
#ifdef __8052__
    REG_T2CON = 0xC8 - 0x80,
    REG_T2MOD = 0xC9 - 0x80,
    REG_TH2 = 0xCD - 0x80,
    REG_TL2 = 0xCC - 0x80,
    REG_RCAP2H = 0xCB - 0x80,
    REG_RCAP2L = 0xCA - 0x80,
#endif // __8052__
#ifdef __SAB80C517__
    // SAB80C517 SFR addresses (NOTE: 0x80 offset in mSFR array indexing)
    // CRITICAL: 0xB8 is IP in 8051 but IEN1 in SAB80C517!
    REG_IEN0  = 0xA8 - 0x80,  // Interrupt Enable 0 (same address as IE)
    REG_IEN1  = 0xB8 - 0x80,  // Interrupt Enable 1 (CONFLICT: IP in 8051!)
    REG_IEN2  = 0x9A - 0x80,  // Interrupt Enable 2
    REG_IP0   = 0xA9 - 0x80,  // Interrupt Priority 0 (low bit)
    REG_IP1   = 0xB9 - 0x80,  // Interrupt Priority 1 (high bit)
    REG_IRCON = 0xC0 - 0x80,  // Interrupt Request Control
    REG_CTCON = 0xE1 - 0x80,  // Compare Timer Control
    REG_S1CON = 0x9B - 0x80,  // Serial 1 Control
    REG_S1BUF = 0x9C - 0x80,  // Serial 1 Buffer
#endif // __SAB80C517__
};

enum PSW_BITS
{
    PSW_P = 0,
    PSW_UNUSED = 1,
    PSW_OV = 2,
    PSW_RS0 = 3,
    PSW_RS1 = 4,
    PSW_F0 = 5,
    PSW_AC = 6,
    PSW_C = 7
};

enum PSW_MASKS
{
    PSWMASK_P = 0x01,
    PSWMASK_UNUSED = 0x02,
    PSWMASK_OV = 0x04,
    PSWMASK_RS0 = 0x08,
    PSWMASK_RS1 = 0x10,
    PSWMASK_F0 = 0x20,
    PSWMASK_AC = 0x40,
    PSWMASK_C = 0x80
};

enum IE_MASKS
{
    IEMASK_EX0 = 0x01,
    IEMASK_ET0 = 0x02,
    IEMASK_EX1 = 0x04,
    IEMASK_ET1 = 0x08,
    IEMASK_ES  = 0x10,
    IEMASK_ET2 = 0x20,
    IEMASK_UNUSED = 0x40,
    IEMASK_EA  = 0x80
};

enum PT_MASKS
{
    PTMASK_PX0 = 0x01,
    PTMASK_PT0 = 0x02,
    PTMASK_PX1 = 0x04,
    PTMASK_PT1 = 0x08,
    PTMASK_PS  = 0x10,
    PTMASK_PT2 = 0x20,
    PTMASK_UNUSED1 = 0x40,
    PTMASK_UNUSED2 = 0x80
};

enum TCON_MASKS
{
    TCONMASK_IT0 = 0x01,
    TCONMASK_IE0 = 0x02,
    TCONMASK_IT1 = 0x04,
    TCONMASK_IE1 = 0x08,
    TCONMASK_TR0 = 0x10,
    TCONMASK_TF0 = 0x20,
    TCONMASK_TR1 = 0x40,
    TCONMASK_TF1 = 0x80
};

enum TMOD_MASKS
{
    TMODMASK_M0_0 = 0x01,
    TMODMASK_M1_0 = 0x02,
    TMODMASK_CT_0 = 0x04,
    TMODMASK_GATE_0 = 0x08,
    TMODMASK_M0_1 = 0x10,
    TMODMASK_M1_1 = 0x20,
    TMODMASK_CT_1 = 0x40,
    TMODMASK_GATE_1 = 0x80
};

#ifdef __8052__
enum T2CON_MASKS
{
    T2CONMASK_CPRL2 = 0x01,
    T2CONMASK_CT2   = 0x02,
    T2CONMASK_TR2   = 0x04,
    T2CONMASK_EXEN2 = 0x08,
    T2CONMASK_TCLK  = 0x10,
    T2CONMASK_RCLK  = 0x20,
    T2CONMASK_EXF2  = 0x40,
    T2CONMASK_TF2   = 0x80
};

enum T2MOD_MASKS
{
    T2MODMASK_DCEN = 0x01,
    T2MODMASK_T2OE = 0x02
};
#endif

#ifdef __SAB80C517__
// SAB80C517: Sentinel value for "no interrupt active"
// Must be distinct from valid priority levels 0-3
#define SAB80C517_NO_INT_ACTIVE 0xFF

// SAB80C517 Interrupt Enable Register 0 (IEN0) bit masks
enum IEN0_MASKS
{
    IEN0MASK_EX0 = 0x01,  // External interrupt 0 enable
    IEN0MASK_ET0 = 0x02,  // Timer 0 interrupt enable
    IEN0MASK_EX1 = 0x04,  // External interrupt 1 enable
    IEN0MASK_ET1 = 0x08,  // Timer 1 interrupt enable
    IEN0MASK_ES0 = 0x10,  // Serial port 0 interrupt enable
    IEN0MASK_ET2 = 0x20,  // Timer 2 interrupt enable
    IEN0MASK_WDT = 0x40,  // Watchdog timer refresh flag
    IEN0MASK_EA  = 0x80   // Global interrupt enable
};

// SAB80C517 Interrupt Enable Register 1 (IEN1) bit masks
enum IEN1_MASKS
{
    IEN1MASK_EX2  = 0x01,  // External interrupt 2 enable
    IEN1MASK_EX3  = 0x02,  // External interrupt 3 enable
    IEN1MASK_EX4  = 0x04,  // External interrupt 4 enable
    IEN1MASK_EX5  = 0x08,  // External interrupt 5 enable
    IEN1MASK_EX6  = 0x10,  // External interrupt 6 enable
    IEN1MASK_ES1  = 0x20,  // Serial port 1 interrupt enable
    IEN1MASK_SWDT = 0x40,  // Software watchdog enable
    IEN1MASK_EXF2 = 0x80   // Timer 2 external reload enable
};

// SAB80C517 Interrupt Enable Register 2 (IEN2) bit masks
enum IEN2_MASKS
{
    IEN2MASK_IADC = 0x01,  // ADC interrupt enable
    IEN2MASK_ECT  = 0x10   // Compare timer interrupt enable
};

// SAB80C517 Interrupt Request Control (IRCON) bit masks
enum IRCON_MASKS
{
    IRCONMASK_IADC = 0x01,  // ADC conversion complete flag
    IRCONMASK_IEX2 = 0x02,  // External interrupt 2 edge flag
    IRCONMASK_IEX3 = 0x04,  // External interrupt 3 edge flag
    IRCONMASK_IEX4 = 0x08,  // External interrupt 4 edge flag
    IRCONMASK_IEX5 = 0x10,  // External interrupt 5 edge flag
    IRCONMASK_IEX6 = 0x20,  // External interrupt 6 edge flag
    IRCONMASK_TF2  = 0x40,  // Timer 2 overflow flag
    IRCONMASK_EXF2 = 0x80   // Timer 2 external reload flag
};

// SAB80C517 Compare Timer Control (CTCON) bit masks
enum CTCON_MASKS
{
    CTCONMASK_CTF = 0x01  // Compare timer flag
};
#endif // __SAB80C517__

enum IP_MASKS
{
    IPMASK_PX0 = 0x01,
    IPMASK_PT0 = 0x02,
    IPMASK_PX1 = 0x04,
    IPMASK_PT1 = 0x08,
    IPMASK_PS  = 0x10,
    IPMASK_PT2 = 0x20
};

enum SCON_MASKS
{
    SCONMASK_RI   = 0x01,
    SCONMASK_TI   = 0x02,
    SCONMASK_RB8  = 0x04,
    SCONMASK_TB8  = 0x08,
    SCONMASK_REN  = 0x10,
    SCONMASK_SM2  = 0x20,
    SCONMASK_SM1  = 0x40,
    SCONMASK_SM0  = 0x80,
};

enum P3_MASKS
{
    P3MASK_INT0 = (1 << 2),
    P3MASK_INT1 = (1 << 2),
    P3MASK_T0 = (1 << 4),
    P3MASK_T1 = (1 << 5),
};

enum ISR_VECTORS
{
    ISR_RST  = 0x00,
    ISR_INT0 = 0x03,
    ISR_TF0  = 0x0B,
    ISR_INT1 = 0x13,
    ISR_TF1  = 0x1B,
    ISR_SR   = 0x23,
#ifdef __8052__
    ISR_TF2  = 0x2B,
#endif // __8052__
};

#ifdef __SAB80C517__
// SAB80C517 ISR vectors (14 interrupt sources)
enum ISR_VECTORS_80C517
{
    ISR_517_RST  = 0x00,  // Reset
    ISR_517_INT0 = 0x03,  // External interrupt 0
    ISR_517_TF0  = 0x0B,  // Timer 0 overflow
    ISR_517_INT1 = 0x13,  // External interrupt 1
    ISR_517_TF1  = 0x1B,  // Timer 1 overflow
    ISR_517_SR0  = 0x23,  // Serial port 0 (RI0/TI0)
    ISR_517_TF2  = 0x2B,  // Timer 2 overflow/external reload
    ISR_517_IADC = 0x43,  // A/D converter
    ISR_517_IEX2 = 0x4B,  // External interrupt 2
    ISR_517_IEX3 = 0x53,  // External interrupt 3
    ISR_517_IEX4 = 0x5B,  // External interrupt 4
    ISR_517_IEX5 = 0x63,  // External interrupt 5
    ISR_517_IEX6 = 0x6B,  // External interrupt 6
    ISR_517_SR1  = 0x83,  // Serial port 1 (RI1/TI1)
    ISR_517_CTF  = 0x9B   // Compare timer overflow
};
#endif // __SAB80C517__

enum EM8051_EXCEPTION
{
    EXCEPTION_STACK,  // stack address > 127 with no upper memory, or roll over
    EXCEPTION_ACC_TO_A, // acc-to-a move operation; illegal (acc-to-acc is ok, a-to-acc is ok..)
    EXCEPTION_IRET_PSW_MISMATCH, // psw not preserved over interrupt call (doesn't care about P, F0 or UNUSED)
    EXCEPTION_IRET_SP_MISMATCH,  // sp not preserved over interrupt call
    EXCEPTION_IRET_ACC_MISMATCH, // acc not preserved over interrupt call
    EXCEPTION_ILLEGAL_OPCODE     // for the single 'reserved' opcode in the architecture
};

