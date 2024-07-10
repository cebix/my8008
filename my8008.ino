//
// Intel 8008 System Emulator for Adafruit ItsyBitsy M4
//
// Copyright 2024 Christian Bauer
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
//

//
// Wiring:
//
// ItsyBitsy  8008
// D5_5V      INTERRUPT
//            READY tied to +5V
// D7         Phi1
// D9         Phi2
// D10        S0
// D11        S1
// D12        S2
// MISO       SYNC
// MOSI       D0
// SCK        D1
// A0         D2
// A2         D3  (being able to use AREF would be nicer...)
// A4         D4
// A1         D5
// A5         D6
// D2         D7
//
// All signals except for D5_5V go through 3.3V-5V level shifters.
//
// I/O ports 0/1 of the 8008 simulate a 6551-ish USART connected to RX/TX
// on the ItsyBitsy.
//


#include "mon.h"

#include <Adafruit_DotStar.h>


Adafruit_DotStar strip(DOTSTAR_NUM, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BRG);

// Turn off DotStar LED.
static void initLED(void)
{
    strip.begin();
    strip.setBrightness(20);
    strip.show();
}

// Set DotStar LED to green.
static void setLEDGreen(void)
{
    strip.setPixelColor(0, strip.gamma32(strip.ColorHSV(0)));
    strip.show();
}

// Set DotStar LED to orange.
static void setLEDOrange(void)
{
    strip.setPixelColor(0, strip.gamma32(strip.ColorHSV(16384)));
    strip.show();
}


// Disable all interrupts on the ItsyBitsy (necessary because timer
// or USB interrupts may disrupt the timing of the bus routines).
static void disableInterrupts(void)
{
   __asm__ volatile("cpsid i");
}


// Enable all interrupts.
static void enableInterrupts(void)
{
   __asm__ volatile("cpsie i");
}


// Initialize I/O pins.
static void initPinmux(void)
{
    // Enable D5 for INT output
    pinMode(5, OUTPUT);

    // Enable D10..D12 for S0..S2 input
    pinMode(10, INPUT);
    pinMode(11, INPUT);
    pinMode(12, INPUT);

    // Enable MISO for SYNC input
    pinMode(PIN_SPI_MISO, INPUT);

    // Enable D13 (on-board LED) for output
    pinMode(13, OUTPUT);

    // Enable the peripheral multiplexer on pins D7/D9 for clock output
    PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.DRVSTR = 1;
    PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.DRVSTR = 1;
 
    // Set the D7 (PORT_PA18) mux to peripheral (even pin number) E(6): TCC0/WO[6] (Channel 0)
    PORT->Group[g_APinDescription[7].ulPort].PMUX[g_APinDescription[7].ulPin >> 1].reg |= PORT_PMUX_PMUXE(6);

    // Set the D9 (PORT_PA19) mux to peripheral (odd pin number) O(5): TCC1/WO[3] (Channel 3)
    PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO(5);

    // Disable the peripheral multiplexer on pins MOSI/SCK/A0/(AREF)/A4/A1/A5/D2 for data I/O
    for (unsigned pin = 0; pin < 8; ++pin) {
        PORT->Group[0].PINCFG[pin].bit.PMUXEN = 0;
        PORT->Group[0].PINCFG[pin].bit.INEN = 1;
        PORT->Group[0].PINCFG[pin].bit.PULLEN = 0;
        PORT->Group[0].PINCFG[pin].bit.DRVSTR = 0;
    }

    PORT->Group[1].PINCFG[8].bit.PMUXEN = 0;  // A2
    PORT->Group[1].PINCFG[8].bit.INEN = 1;
    PORT->Group[1].PINCFG[8].bit.PULLEN = 0;
    PORT->Group[1].PINCFG[8].bit.DRVSTR = 0;

    PORT->Group[0].OUTCLR.reg = 0xFF;
    PORT->Group[0].DIRCLR.reg = 0xFF;

    PORT->Group[1].OUTCLR.reg = (1UL << 8);  // A2
    PORT->Group[1].DIRCLR.reg = (1UL << 8);
}


// Set up timers for two-phase clock generation.
static void initPhiClock(void)
{
    // Set up the generic clock (GCLK7) to clock timer TCC0
    GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                           GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                           GCLK_GENCTRL_GENEN |        // Enable GCLK7
                           GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
    while (GCLK->SYNCBUSY.bit.GENCTRL7) { }            // Wait for synchronization 

    GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC0 peripheral channel
                            GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TCC0

    // Set up TCC0
    TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 |       // Set prescaler to 1, 48MHz/1 = 48MHz
                      TCC_CTRLA_PRESCSYNC_PRESC;       // Set the reset/reload to trigger on prescaler clock                 

    TCC0->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
    while (TCC0->SYNCBUSY.bit.WAVE) { }                // Wait for synchronization

    TCC0->PER.reg = 59;                                // Set-up the PER (period) register 800kHz PWM
    while (TCC0->SYNCBUSY.bit.PER) { }                 // Wait for synchronization

    TCC0->CC[0].reg = 16;                              // Set-up the CC (counter compare), channel 0 register for 28% duty-cycle
    while (TCC0->SYNCBUSY.bit.CC0) { }                 // Wait for synchronization

    // Set up TCC1
    TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 |       // Set prescaler to 1, 48MHz/1 = 48MHz
                      TCC_CTRLA_PRESCSYNC_PRESC;       // Set the reset/reload to trigger on prescaler clock

    TCC1->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
    while (TCC1->SYNCBUSY.bit.WAVE) { }                // Wait for synchronization

    TCC1->PER.reg = 59;                                // Set-up the PER (period) register 800kHz PWM
    while (TCC1->SYNCBUSY.bit.PER) { }                 // Wait for synchronization

    TCC1->CC[3].reg = 16;                              // Set-up the CC (counter compare), channel 3 register for 28% duty-cycle
    while (TCC1->SYNCBUSY.bit.CC3) { }                 // Wait for synchronization

    TCC1->COUNT.reg = 36;                              // Shift TCC1

    // Enable them
    TCC0->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC0
    TCC1->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC1
    while (TCC0->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
    while (TCC1->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
}


// Init USB and USART serial ports.
static void initSerial(void)
{
    // USB
    Serial.begin(115200);

    // USART (RX/TX)
    Serial1.begin(115200);
    SERCOM3->USART.INTENCLR.reg = 0xFF;
}


// Set INT to high.
static void assertINT(void)
{
    digitalWrite(5, HIGH);
}


// Set INT to low.
static void deassertINT(void)
{
    digitalWrite(5, LOW);
}


// Wait for falling edge of SYNC.
static void waitForSYNC(void)
{
    while (digitalRead(PIN_SPI_MISO) == LOW) { }
    while (digitalRead(PIN_SPI_MISO) == HIGH) { }
}


// Read the S0..S2 state lines.
static uint8_t readSTATE(void)
{
    uint32_t data = PORT->Group[0].IN.reg;
    uint8_t v = 0;
    if (data & (1UL << 20)) {  // D10
        v |= 1;
    }
    if (data & (1UL << 21)) {  // D11
        v |= 2;
    }
    if (data & (1UL << 23)) {  // D12
        v |= 4;
    }
    return v;
}


// Processor state
enum {
    STATE_WAIT = 0,
    STATE_T3 = 1,
    STATE_T1 = 2,
    STATE_STOPPED = 3,
    STATE_T2 = 4,
    STATE_T5 = 5,
    STATE_T1I = 6,
    STATE_T4 = 7,
};


// String representation of processor states
static const char * stateText[8] = {
    "WAIT",
    "T3",
    "T1",
    "STOPPED",
    "T2",
    "T5",
    "T1I",
    "T4",
};


// Bus access types
enum {
    TYPE_PCI = 0,  // Read first byte of instruction
    TYPE_PCC = 1,  // I/O command
    TYPE_PCR = 2,  // Read additional bytes of instruction or data
    TYPE_PCW = 3,  // Write data
};


// Instruction opcode inserted on interrupt request
#define INT_INSTR 0x05  // RST 0 (start execution at address 0)


// Emulated 16K memory
static uint8_t mem[0x4000] = {
#if 0

    // result = op1 + op2
    0x36, 0x0A,  // 000  MVI L,0A
    0x2E, 0x00,  // 002  MVI H,00
    0xC7,        // 004  MOV A,M
    0x30,        // 005  INR L
    0x87,        // 006  ADD M
    0x30,        // 007  INR L
    0xF8,        // 008  MOV M,A
    0x00,        // 009  HLT

    0x15,        // 00A  data (op1)
    0x51,        // 00B  data (op2)
    0x00,        // 00C  data (result)

#elif 0

    // Output string on USART
    0x2e, 0x00,       // 000        LHI >string
    0x36, 0x21,       // 002        LLI <string
    0xc7,             // 004  loop: LAM
    0xa0,             // 005        NDA
    0x68, 0x12, 0x00, // 006        JTZ end
    0x46, 0x13, 0x00, // 009        CAL cout
    0x46, 0x1d, 0x00, // 00C        CAL incr
    0x44, 0x04, 0x00, // 00F        JMP loop
    0x01,             // 012  end:  HLT
    0xc8,             // 013  cout: LBA
    0x43,             // 014        INP 1
    0x24, 0x10,       // 015        NDI 0x10
    0x68, 0x13, 0x00, // 017        JTZ cout
    0xc1,             // 01A        LAB
    0x51,             // 01B        OUT 8
    0x07,             // 01C        RET
    0x30,             // 01D  incr: INL
    0x0b,             // 01E        RFZ
    0x28,             // 01F        INH
    0x07,             // 020        RET
                      // 021  string:
    0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x77, 0x6f, 0x72, 0x6c, 0x64, 0x21, 0x00,

#elif 0

    // USART echo
    0x46, 0x09, 0x00,  // 000 loop: CAL cin
    0x46, 0x11, 0x00,  // 003       CAL cout
    0x44, 0x00, 0x00,  // 006       JMP loop
    0x43,              // 009 cin:  INP 1
    0x24, 0x08,        // 00A       NDI 0x08
    0x68, 0x09, 0x00,  // 00C       JTZ cin
    0x41,              // 00F       INP 0
    0x07,              // 010       RET
    0xc8,              // 011 cout: LBA
    0x43,              // 012       INP 1
    0x24, 0x10,        // 013       NDI 0x10
    0x68, 0x11, 0x00,  // 015       JTZ cout
    0xc1,              // 018       LAB
    0x51,              // 019       OUT 8
    0x07,              // 01A       RET

#else

    // SCELBAL BASIC interpreter
#include "scelbal.h"

#endif
};


// Use SCELBI character encoding?
#define SCELBI


// Input value from I/O port (0..7).
static uint8_t inPort(unsigned port)
{
    switch (port) {
    case 0: {
        // USART receive data register
        uint8_t v = SERCOM3->USART.DATA.bit.DATA;
#ifdef SCELBI
        if (v < 0x40 || v == 0x7F) {
            v |= 0x80;
        }
#endif
        return v;
    }
    case 1: {
        // USART status register (like 6551)
        uint8_t intflags = SERCOM3->USART.INTFLAG.reg;
        uint8_t v = 0;
        if (intflags & 0x04) {
            v |= 0x08;  // Receive data register full
        }
        if (intflags & 0x01) {
            v |= 0x10;  // Transmit data register empty
        }
        return v;
    }
    default:
        return 0xFF;
    }
}


// Output value to I/O port (8..31).
static void outPort(unsigned port, uint8_t v)
{
    // TODO
    switch (port) {
    case 8:
        // USART transmit data register
#ifdef SCELBI
         v &= 0x7F;
#endif
        SERCOM3->USART.DATA.reg = v;
        break;
    default:
        break;
    }
}


// Report a bus state sequence error.
static void seqError(uint8_t from, uint8_t to)
{
    enableInterrupts();

    Serial.print("seq error ");
    Serial.print(stateText[from]);
    Serial.print(" -> ");
    Serial.println(stateText[to]);

    return;
}


#define TRACE
#define TRACE_SIZE 64

#ifdef TRACE
static uint8_t stateBuf[TRACE_SIZE];
static uint8_t busBuf[TRACE_SIZE];
static uint32_t traceCycle;
#endif


// Run the 8008 bus emulation.
static void runBus(void)
{
    uint32_t cycle = 0;
    uint16_t addr = 0;
    uint8_t type = 0;

    bool interrupted = false;
    bool first = true;

    uint8_t lastState = STATE_STOPPED;

    // Execute and trace
    while (true) {
        waitForSYNC();

        // Read state and bus lines
        uint8_t state = readSTATE();
        uint8_t bus = (PORT->Group[0].IN.reg & 0xFF) | ((PORT->Group[1].IN.reg & 0x100) >> 5);

#ifdef TRACE
        stateBuf[traceCycle] = state;
        busBuf[traceCycle] = bus;
        ++traceCycle;
        if (traceCycle >= TRACE_SIZE) {
            traceCycle = 0;
        }
#endif

        switch (state) {
        case STATE_T1I:
            // Acknowledge interrupt, insert instruction in T3 state
            interrupted = true;
            deassertINT();

            // fallthrough

        case STATE_T1:
            // Latch lower 8 bits of address
            addr = bus;
            break;

        case STATE_T2:
            if (lastState != STATE_T1 && lastState != STATE_T1I) {
                seqError(lastState, state);
                return;
            }

            // Latch upper 6 bits of address and access type
            addr |= (bus & 0x3F) << 8;
            type = bus >> 6;

            if (type == TYPE_PCI || type == TYPE_PCR || (type == TYPE_PCC && (addr & 0x3000) == 0)) {

                // Put value on bus for state T3
                uint8_t v;
                if (interrupted) {

                    // Insert opcode for interrupt
                    v = INT_INSTR;
                    interrupted = false;

                } else if (type == TYPE_PCC) {

                    // Read from input port
                    v = inPort((addr >> 9) & 0x07);

                } else {

                    // Read from memory
                    v = mem[addr];
                }

                PORT->Group[0].OUT.reg = (PORT->Group[0].OUT.reg & 0xFFFFFF00) | v;
                PORT->Group[1].OUT.reg = (PORT->Group[1].OUT.reg & 0xFFFFFEFF) | ((v & 0x08) << 5);

                // Set bus to output
                PORT->Group[0].DIRSET.reg = 0xFF;
                PORT->Group[1].DIRSET.reg = 1UL << 8;
            }
            break;

        case STATE_T3:
            if (lastState != STATE_T2) {
                seqError(lastState, state);
                return;
            }

            // Set bus back to input
            PORT->Group[0].DIRCLR.reg = 0xFF;
            PORT->Group[1].DIRCLR.reg = 1UL << 8;

            switch (type) {
            case TYPE_PCW:
                // Write to memory
                mem[addr] = bus;
                break;

            case TYPE_PCC:
                // Write to output port
                if (addr & 0x3000) {
                    outPort((addr >> 9) & 0x1F, addr & 0xFF);
                }
                break;

            default:
                break;
            }

            break;

        case STATE_T4:
            if (lastState != STATE_T3) {
                seqError(lastState, state);
                return;
            }
            break;

        case STATE_T5:
            if (lastState != STATE_T4) {
                seqError(lastState, state);
                return;
            }
            break;

        case STATE_STOPPED:
            // Stop emulation when 8008 has halted
            // (ignore first cycle when reset interrupt is asserted)
            if (! first) {
                return;
            }
            break;

        default:
            break;
        }

        first = false;
        lastState = state;
        ++cycle;
    }
}


// Output traceback of bus activities.
#ifdef TRACE
static void dumpBusTrace(void)
{
    Serial.println("Traceback:");

    uint16_t addr;

    // Dump trace buffer
    for (int i = 0; i < TRACE_SIZE; ++i) {
        uint8_t state = stateBuf[(traceCycle + i) % TRACE_SIZE];
        uint8_t bus = busBuf[(traceCycle + i) % TRACE_SIZE];
        uint8_t type = 0;

        if (state == STATE_T1 || state == STATE_T1I) {
            addr = bus;
        } else if (state == STATE_T2) {
            addr |= (bus & 0x3F) << 8;
            type = bus >> 6;
        }

        Serial.print(stateText[state]);
        Serial.print(' ');
        Serial.print(bus, HEX);

        if (state == STATE_T2) {
            switch (type) {
            case TYPE_PCI:
                Serial.print(" -> read instr ");
                break;
            case TYPE_PCC:
                Serial.print(" -> I/O command ");
                break;
            case TYPE_PCR:
                Serial.print(" -> read data ");
                break;
            case TYPE_PCW:
                Serial.print(" -> write data ");
                break;
            }
            Serial.println(addr, HEX);
        } else {
            Serial.println();
        }
    }
}
#endif


// Set up everything.
void setup(void)
{
    initLED();
    initPinmux();
    deassertINT();
    initSerial();

    // Wait for USB ACM device to settle before doing output
    delay(2000);
    setLEDOrange();

    Serial.println("\n\ni8008 System Emulator by Christian Bauer\n");
    Serial.println("Enter 'h' for help");

    // Start 8008 clock and wait for >=16 clock periods to give 8008 time to reset
    initPhiClock();
    delay(10);
}


// Main loop.
void loop(void)
{
    // Enter monitor mode until 'go' command is issued
    monitor(mem);

    // Issue reset interrupt to 8008
    Serial.println("Running...");

    setLEDGreen();
    disableInterrupts();

    waitForSYNC();
    assertINT();

    // Run bus emulation until processor stops
    runBus();

    enableInterrupts();
    setLEDOrange();

    Serial.println("Stopped.");

#ifdef TRACE
    // Dump bus traceback
    dumpBusTrace();
#endif
}
