//
// Simple Intel 8008 machine language monitor
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

#include "mon.h"

#include <Arduino.h>


static uint16_t getArg(String & args)
{
    String arg;

    int spaceIdx = args.indexOf(' ');
    if (spaceIdx >= 0) {
        arg = args.substring(0, spaceIdx);
        args = args.substring(spaceIdx);
        args.trim();
    } else {
        arg = args;
        args = "";
    }

    return strtoul(arg.c_str(), 0, 16);
}


static char char2print(uint8_t c)
{
   return (c >= 0x20 && c <= 0x7e) ? c : '.';
}


static void dumpMem(String & args, const uint8_t * mem)
{
    if (args.length() == 0) {
        Serial.println("Missing argument");
        return;
    }

    uint16_t startAdr = getArg(args);
    uint16_t endAdr;

    const unsigned bytesPerLine = 16;

    if (args.length() > 0) {
        endAdr = getArg(args);
    } else {
        endAdr = startAdr + 8 * bytesPerLine - 1;
    }

    startAdr &= 0x3FFF;
    endAdr &= 0x3FFF;

    for (uint16_t adr = startAdr; adr <= endAdr; adr += bytesPerLine) {
        char buf[256];
        char ascii[bytesPerLine + 1];
        int l = 0;

        l += sprintf(buf + l, "%04x:", adr & 0x3FFF);

        for (unsigned i = 0; i < bytesPerLine; ++i) {
            uint8_t c = mem[(adr + i) & 0x3FFF];
            l += sprintf(buf + l, " %02x", c);
            ascii[i] = char2print(c);
        }

        ascii[bytesPerLine] = '\0';
        l += sprintf(buf + l, "  '%s'", ascii);

        Serial.println(buf);
    }
}


static const char * opcodeText[256] = {
  // x0     x1        x2     x3        x4     x5        x6     x7        x8     x9        xA     xB        xC     xD        xE     xF
    "HLT", "HLT",    "RLC", "RFC",    "ADI", "RST 0",  "LAI", "RET",    "INB", "DCB",    "RRC", "RFZ",    "ACI", "RST 1",  "LBI", "RET",     // 00..0F
    "INC", "DCC",    "RAL", "RFN",    "SUI", "RST 2",  "LCI", "RET",    "IND", "DCD",    "RAR", "RFP",    "SBI", "RST 3",  "LDI", "RET",     // 10..1F
    "INE", "DCE",    "???", "RTC",    "NDI", "RST 4",  "LEI", "RET",    "INH", "DCH",    "???", "RTZ",    "XRI", "RST 5",  "LHI", "RET",     // 20..2F
    "INL", "DCL",    "???", "RTN",    "ORI", "RST 6",  "LLI", "RET",    "???", "???",    "???", "RTP",    "CPI", "RST 7",  "LMI", "RET",     // 30..3F
    "JFC", "INP 0",  "CFC", "INP 1",  "JMP", "INP 2",  "CAL", "INP 3",  "JFZ", "INP 4",  "CFZ", "INP 5",  "JMP", "INP 6",  "CAL", "INP 7",   // 40..4F
    "JFN", "OUT 8",  "CFN", "OUT 9",  "JMP", "OUT A",  "CAL", "OUT B",  "JFP", "OUT C",  "CFP", "OUT D",  "JMP", "OUT E",  "CAL", "OUT F",   // 50..5F
    "JTC", "OUT 10", "CTC", "OUT 11", "JMP", "OUT 12", "CAL", "OUT 13", "JTZ", "OUT 14", "CTZ", "OUT 15", "JMP", "OUT 16", "CAL", "OUT 17",  // 60..6F
    "JTN", "OUT 18", "CTN", "OUT 19", "JMP", "OUT 1A", "CAL", "OUT 1B", "JTP", "OUT 1C", "CTP", "OUT 1D", "JMP", "OUT 1E", "CAL", "OUT 1F",  // 70..7F
    "ADA", "ADB",    "ADC", "ADD",    "ADE", "ADH",    "ADL", "ADM",    "ACA", "ACB",    "ACC", "ACD",    "ACE", "ACH",    "ACL", "ACM",     // 80..8F
    "SUA", "SUB",    "SUC", "SUD",    "SUE", "SUH",    "SUL", "SUM",    "SBA", "SBB",    "SBC", "SBD",    "SBE", "SBH",    "SBL", "SBM",     // 90..9F
    "NDA", "NDB",    "NDC", "NDD",    "NDE", "NDH",    "NDL", "NDM",    "XRA", "XRB",    "XRC", "XRD",    "XRE", "XRH",    "XRL", "XRM",     // A0..AF
    "ORA", "ORB",    "ORC", "ORD",    "ORE", "ORH",    "ORL", "ORM",    "CPA", "CPB",    "CPC", "CPD",    "CPE", "CPH",    "CPL", "CPM",     // B0..BF
    "NOP", "LAB",    "LAC", "LAD",    "LAE", "LAH",    "LAL", "LAM",    "LBA", "LBB",    "LBC", "LBD",    "LBE", "LBH",    "LBL", "LBM",     // C0..CF
    "LCA", "LCB",    "LCC", "LCD",    "LCE", "LCH",    "LCL", "LCM",    "LDA", "LDB",    "LDC", "LDD",    "LDE", "LDH",    "LDL", "LDM",     // D0..DF
    "LEA", "LEB",    "LEC", "LED",    "LEE", "LEH",    "LEL", "LEM",    "LHA", "LHB",    "LHC", "LHD",    "LHE", "LHH",    "LHL", "LHM",     // E0..EF
    "LLA", "LLB",    "LLC", "LLD",    "LLE", "LLH",    "LLL", "LLM",    "LMA", "LMB",    "LMC", "LMD",    "LME", "LMH",    "LML", "HLT",     // F0..FF
};


static const uint8_t opcodeLen[256] = {
 // x0 x1 x2 x3 x4 x5 x6 x7 x8 x9 xA xB xC xD xE xF
    1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1, 2, 1,  // 00..0F
    1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1, 2, 1,  // 10..1F
    1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1, 2, 1,  // 20..2F
    1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1, 2, 1,  // 30..3F
    3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1,  // 40..4F
    3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1,  // 50..5F
    3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1,  // 60..6F
    3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1,  // 70..7F
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 80..8F
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 90..9F
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // A0..AF
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // B0..BF
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // C0..CF
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // D0..DF
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // E0..EF
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // F0..FF
};


static void disassemble(String & args, const uint8_t * mem)
{
    if (args.length() == 0) {
        Serial.println("Missing argument");
        return;
    }

    uint16_t startAdr = getArg(args);
    uint16_t endAdr;

    if (args.length() > 0) {
        endAdr = getArg(args);
    } else {
        endAdr = startAdr + 32;
    }

    startAdr &= 0x3FFF;
    endAdr &= 0x3FFF;

    for (uint16_t adr = startAdr; adr <= endAdr; ) {
        uint8_t op = mem[adr & 0x3FFF];
        uint8_t len = opcodeLen[op];

        char buf[256];
        int l = 0;

        l += sprintf(buf + l, "%04x:", adr & 0x3FFF);

        for (unsigned i = 0; i < len; ++i) {
            l += sprintf(buf + l, " %02x", mem[(adr + i) & 0x3FFF]);
        }
        for (unsigned i = len; i < 3; ++i) {
            l += sprintf(buf + l, "   ");
        }

        l += sprintf(buf + l, "  %s", opcodeText[op]);
        if (len == 2) {
            l += sprintf(buf + l, " %02x", mem[(adr + 1) & 0x3FFF]);
        } else if (len == 3) {
            l += sprintf(buf + l, " %02x%02x", mem[(adr + 2) & 0x3FFF] & 0x3F, mem[(adr + 1) & 0x3FFF]);
        }

        Serial.println(buf);
        adr += len;
    }
}


void monitor(uint8_t * mem)
{
    bool done = false;

    while (! done) {

        // Print prompt
        Serial.print(".");

        // Read and parse command line
        String input;
        while (! Serial.available()) { }
        input = Serial.readStringUntil('\n');
        input.trim();
        Serial.println(input);

        if (input.length() == 0) {
            continue;
        }

        String cmd;
        String args;

        int spaceIdx = input.indexOf(' ');
        if (spaceIdx > 0) {
            cmd = input.substring(0, spaceIdx);
            args = input.substring(spaceIdx);
            args.trim();
        } else {
            cmd = input;
        }

        if (cmd.equals("h") || cmd.equals("??")) {
            Serial.println("h              Show list of commands");
            Serial.println("g              Run 8008 program");
            Serial.println("m xxxx [yyyy]  Dump memory");
            Serial.println("d xxxx [yyyy]  Disassemble");
        } else if (cmd.equals("g")) {
            done = true;
        } else if (cmd.equals("m")) {
            dumpMem(args, mem);
        } else if (cmd.equals("d")) {
            disassemble(args, mem);
        } else {
            Serial.println("Unknown command");
        }
    }
}
