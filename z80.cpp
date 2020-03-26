#include "z80.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Some useful macros

//send a byte to a specific spot in a word
#define bytetoHigh(word, byte) (((byte) << 8) | ((word) & 0x00FF))
#define bytetoLow(word, byte) ((byte) | ((word) & 0xFF00))

//send a byte from a word to a specific spot in a word
#define HightoHigh(dstword, srcword) (((srcword) & 0xFF00) | ((dstword) & 0x00FF))
#define LowtoHigh(dstword, srcword) (((srcword) << 8 ) | ((dstword) & 0x00FF))
#define HightoLow(dstword, srcword) (((srcword) >> 8 ) | ((dstword) & 0xFF00))
#define LowtoLow(dstword, srcword) (((srcword) & 0x00FF ) | ((dstword) & 0xFF00))


// Main Memory
uint8_t *memory = (uint8_t*) malloc((0xFFFF) * sizeof(uint8_t));

// Registers
uint16_t AF = 0x01B0; // AAAAAAAAZNHCxxxx
uint16_t BC = 0x0804; // BBBBBBBBCCCCCCCC
uint16_t DE = 0x0201; // DDDDDDDDEEEEEEEE
uint16_t HL = 0x0000; // HHHHHHHHLLLLLLLL
uint16_t SP = 0xFFFE; // Stack Pointer
uint16_t PC = 0x0100; // Program Counter

// Clocks per instruction lookup table
uint8_t *clockCycles = (uint8_t*) malloc((0xFF) * sizeof(uint8_t));

// Turn on
volatile bool z80::cpuPower = false;

// Keep count of cycles
volatile uint64_t z80::totalClockCycles = 0;

// Keep count of instructions per type
uint32_t *instructionsCount = (uint32_t*) malloc((0xFF) * sizeof(uint32_t));

// Initial instruction
uint8_t instruction = 0x00;

// HALT
bool halted = 0;

void writeByte(unsigned int location, uint8_t data) {
    memory[location] = data;
}

uint8_t readByte(unsigned int location)
{
   return memory[location];
}

void z80::initMemory() {
    int i;
    // Reset memory to zero
    memset(memory, 0x00, 0xFFFF);
    // default clocks per instruction to 1
    for (i=0; i<255;i++) {
      clockCycles[i]=1;
      instructionsCount[i]=0;
    }
    clockCycles[0x3C] = 4;
    clockCycles[0x76] = 4;
    clockCycles[0x03] = 4;
    clockCycles[0x80] = 4;
    clockCycles[0x04] = 4;
    clockCycles[0x05] = 4;
    clockCycles[0x0C] = 4;
    clockCycles[0x0D] = 4;
    clockCycles[0x14] = 4;
    clockCycles[0x15] = 4;
    clockCycles[0x1C] = 4;
    clockCycles[0x1D] = 4;
    clockCycles[0x24] = 4;
    clockCycles[0x25] = 4;
    clockCycles[0x2C] = 4;
    clockCycles[0x2D] = 4;
    clockCycles[0x3D] = 4;
    clockCycles[0xA0] = 4;
    clockCycles[0xA1] = 4;
    clockCycles[0xA2] = 4;
    clockCycles[0xA3] = 4;
    clockCycles[0xA4] = 4;
    clockCycles[0xA5] = 4;
    clockCycles[0xA7] = 4;
    clockCycles[0xB0] = 4;
    clockCycles[0xB1] = 4;
    clockCycles[0xB2] = 4;
    clockCycles[0xB3] = 4;
    clockCycles[0xB4] = 4;
    clockCycles[0xB5] = 4;
    clockCycles[0xB7] = 4;
    clockCycles[0xA8] = 4;
    clockCycles[0xA9] = 4;
    clockCycles[0xAA] = 4;
    clockCycles[0x81] = 4;
    clockCycles[0x82] = 4;
    clockCycles[0x83] = 4;
    clockCycles[0x84] = 4;
    clockCycles[0x85] = 4;
    clockCycles[0x87] = 4;
    clockCycles[0x90] = 4;
    clockCycles[0x91] = 4;
    clockCycles[0x92] = 4;
    clockCycles[0x93] = 4;
    clockCycles[0x94] = 4;
    clockCycles[0x95] = 4;
    clockCycles[0x97] = 4;
    clockCycles[0x41] = 4;
    clockCycles[0x4A] = 4;
    clockCycles[0x53] = 4;
    clockCycles[0x78] = 4;
    clockCycles[0x06] = 7;
    clockCycles[0x0E] = 7;
    clockCycles[0x16] = 7;
    clockCycles[0x1E] = 7;
    clockCycles[0x26] = 7;
    clockCycles[0x2E] = 7;
    clockCycles[0x3E] = 7;
    clockCycles[0x21] = 10;
    clockCycles[0x34] = 11;
    clockCycles[0x35] = 11;
    clockCycles[0x36] = 10;
    clockCycles[0x70] = 7;
    clockCycles[0x46] = 7;
}

void z80::loadTest() {
    writeByte(0x0100, 0x3C);
    writeByte(0x0101, 0x3C);
    writeByte(0x0102, 0x3C);
    writeByte(0x0103, 0x80);
    writeByte(0x0104, 0x76);
}

void z80::loadFirst() {
    writeByte(0x0100, 0x3C);
    writeByte(0x0101, 0x3C);
    writeByte(0x0102, 0x3C);
    writeByte(0x0103, 0x80);
    writeByte(0x0104, 0xA0);
    writeByte(0x0105, 0xB3);
    writeByte(0x0106, 0x1C);
    writeByte(0x0107, 0xB3);
    writeByte(0x0108, 0xA1);
    writeByte(0x0109, 0x0C);
    writeByte(0x010A, 0xA1);
    writeByte(0x010B, 0xAA);
    writeByte(0x010C, 0x15);
    writeByte(0x010D, 0xAA);
    writeByte(0x010E, 0x76);
}

void z80::loadSecond() {
  writeByte(0x0100, 0x3C);
  writeByte(0x0101, 0x3C);
  writeByte(0x0102, 0x53);
  writeByte(0x0103, 0x4A);
  writeByte(0x0104, 0x41);
  writeByte(0x0105, 0x78);
  writeByte(0x0106, 0x06);
  writeByte(0x0107, 0x47);
  writeByte(0x0108, 0x26);
  writeByte(0x0109, 0x02);
  writeByte(0x010A, 0x2E);
  writeByte(0x010B, 0x00);
  writeByte(0x010C, 0x36);
  writeByte(0x010D, 0x99);
  writeByte(0x010E, 0x34);
  writeByte(0x010F, 0x34);
  writeByte(0x010F, 0x46);
  writeByte(0x0110, 0x76);
}

uint8_t fetch() {
    //read the next byte and increment the program counter
    return readByte(PC++);
}

uint16_t readWord()
{
    // little endian
    uint8_t low = readByte(PC++);
    uint8_t high = readByte(PC++);
    return low | (high << 8);
}

void printRegisters() {
    printf("PC: %04x, AF: %04x, BC: %04x, DE: %04x, HL: %04x, SP: %04x\n", PC, AF, BC, DE, HL, SP);
}

void halt() {
        int i=0;
        printf("Total Clock Cycles: %lu\n", z80::totalClockCycles);
        for (i=0;i<255; i++){
	   if (instructionsCount[i]) {
              printf("Instruction 0x%02x count is  %04x\n", i, instructionsCount[i]);
	   }
	}
        printf("Halting now.\n");
	exit(1);
}

void z80::cpuStep() {
    uint8_t n, n1, n2, interrupt;
    int8_t sn;
    uint16_t nn, nn1, nn2;
    bool c;

    if (!cpuPower) return;
    printRegisters();
    printf("totalClockCycles at %08lu\n", totalClockCycles);

    // Check if halted
    if (halted) {
        halt();
    }

    //fetch
    instruction = fetch();
    printf("instruction = %04x\n ", instruction);

    //decode
    totalClockCycles += clockCycles[instruction];
    instructionsCount[instruction]++;
    switch (instruction)
    {


    // NOP
    case 0x0:
        break;

    // HALT
    case 0x76:
        halted = 1;
        break;
    // INC B
    case 0x04:
  BC=HightoHigh(BC, BC+0x0100);
    break;

    //DEC B
    case 0x05:
  BC=HightoHigh(BC, BC-0x0100);
    break;

    //INC C
    case 0x0C:
    BC=LowtoLow(BC, BC+0x0001);
    break;

    //DEC C
    case 0x0D:
    BC=LowtoLow(BC, BC-0x0001);
    break;

    //Inc d
    case 0x14:
    DE=HightoHigh(DE, DE+0x0100);
    break;

    //Dec d
    case 0x15:
    DE=HightoHigh(DE, DE-0x0100);
    break;

    //Inc E
    case 0x1C:
    DE=LowtoLow(DE, DE+0x0001);
    break;

    //Dec E
    case 0x1D:
    DE=LowtoLow(DE, DE-0x0001);
    break;

    //INC H
    case 0x24:
    HL=HightoHigh(HL, HL+0x0100);
    break;

    //DEC H
    case 0x25:
    HL=HightoHigh(HL, HL-0x0100);
    break;

    //Inc L
    case 0x2C:
    HL=LowtoLow(HL, HL+0x0100);
    break;

    //DEC L
    case 0x2D:
    HL=LowtoLow(HL, HL-0x0100);
    break;

    // INC A    Example for increments a particular byte
    case 0x3C:
  	AF=HightoHigh(AF, AF+0x0100);
    //ignore setting flags for now
    break;

    //DEC A
    case 0x3D:
    AF=HightoHigh(AF, AF-0x0100);
    break;

    //And B with A Result in A
    case 0xA0:
    AF=bytetoHigh(AF, (AF >> 8) & (BC >> 8));
    break;

    //And C with A Result in A
    case 0xA1:
    AF=bytetoHigh(AF, (AF >> 8) & (BC << 8));
    break;

    //And D with A Result in A
    case 0xA2:
    AF=bytetoHigh(AF, (AF >> 8) & (DE >> 8));
    break;

    //And E with A Result in A
    case 0xA3:
    AF=bytetoHigh(AF, (AF >> 8) & (DE << 8));
    break;

    //AND H with A Result in A
    case 0xA4:
    AF=bytetoHigh(AF, (AF >> 8) & (HL >> 8));
    break;

    //AND L with A Result in A
    case 0xA5:
    AF=bytetoHigh(AF, (AF >> 8) & (HL << 8));
    break;

    //And A with A Result in A
    case 0xA7:
    AF=bytetoHigh(AF, (AF >> 8) & (AF >> 8));
    break;

    //OR B with A Result in A
    case 0xB0:
    AF=bytetoHigh(AF, (AF >> 8) | (BC >> 8));
    break;

    //OR C with A Result in A
    case 0xB1:
    AF=bytetoHigh(AF, (AF >> 8) | (BC << 8));
    break;

    //OR D with A Result in A
    case 0xB2:
    AF=bytetoHigh(AF, (AF >> 8) | (DE >> 8));
    break;

    //OR E with A Result in A
    case 0xB3:
    AF=bytetoHigh(AF, (AF >> 8) | (DE << 8));
    break;

    //OR H with A Result in A
    case 0xB4:
    AF=bytetoHigh(AF, (AF >> 8) | (HL >> 8));
    break;

    //OR L with A Result in A
    case 0xB5:
    AF=bytetoHigh(AF, (AF >> 8) | (HL << 8));
    break;

    //OR A with A Result in A
    case 0xB7:
    AF=bytetoHigh(AF, (AF >> 8) | (AF >> 8));
    break;

    //XOR B with A Result in A
    case 0xA8:
    AF=bytetoHigh(AF, (AF >> 8) ^ (BC >> 8));
    break;

    //XOR C with A Result in A
    case 0xA9:
    AF=bytetoHigh(AF, (AF >> 8) ^ (BC << 8));
    break;

    //XOR D with A Result in A
    case 0xAA:
    AF=bytetoHigh(AF, (AF >> 8) ^ (DE >> 8));
    break;

    //XOR E with A Result in A
    case 0xAB:
    AF=bytetoHigh(AF, (AF >> 8) ^ (DE << 8));
    break;

    //XOR H with A Result in A
    case 0xAC:
    AF=bytetoHigh(AF, (AF >> 8) ^ (HL >> 8));
    break;

    //XOR L with A Result in A
    case 0xAD:
    AF=bytetoHigh(AF, (AF >> 8) ^ (HL << 8));
    break;

    //XOR A with A Result in A
    case 0xAF:
    AF=bytetoHigh(AF, (AF >> 8) ^ (AF >> 8));
    break;

    // ADD A,B   Add  byte to a byte  result in A
    case 0x80:
    AF = bytetoHigh(AF, (AF >> 8) + (BC >> 8));
    //ignore setting flags for now
    break;

    // ADD A,C   Add  byte to a byte  result in A
    case 0x81:
    AF = bytetoHigh(AF, (AF >> 8) + (BC << 8));
    break;

    // ADD A,D   Add  byte to a byte  result in A
    case 0x82:
    AF = bytetoHigh(AF, (AF >> 8) + (DE >> 8));
    break;

    // ADD A,E   Add  byte to a byte  result in A
    case 0x83:
    AF = bytetoHigh(AF, (AF >> 8) + (DE << 8));
    break;

    // ADD A,H   Add  byte to a byte  result in A
    case 0x84:
    AF = bytetoHigh(AF, (AF >> 8) + (HL >> 8));
    break;

    // ADD A,L   Add  byte to a byte  result in A
    case 0x85:
    AF = bytetoHigh(AF, (AF >> 8) + (HL << 8));
    break;

    // ADD A,F   Add  byte to a byte  result in A
    case 0x87:
    AF = bytetoHigh(AF, (AF >> 8) + (AF >> 8));
    break;

      // SUB B from A   sub  byte to a byte  result in A
      case 0x90:
      AF = bytetoHigh(AF, (AF >> 8) - (BC >> 8));
      break;

      // SUB C from A   sub  byte to a byte  result in A
      case 0x91:
      AF = bytetoHigh(AF, (AF >> 8) - (BC << 8));
      break;

      // SUB D from A   sub  byte to a byte  result in A
      case 0x92:
      AF = bytetoHigh(AF, (AF >> 8) - (DE >> 8));
      break;

      // SUB E from A   sub  byte to a byte  result in A
      case 0x93:
      AF = bytetoHigh(AF, (AF >> 8) - (DE << 8));
      break;

      // SUB H from A   sub  byte to a byte  result in A
      case 0x94:
      AF = bytetoHigh(AF, (AF >> 8) - (HL >> 8));
      break;

      // SUB L from A   sub  byte to a byte  result in A
      case 0x95:
      AF = bytetoHigh(AF, (AF >> 8) - (HL << 8));
      break;

      // SUB A from A   sub  byte to a byte  result in A
      case 0x97:
      AF = bytetoHigh(AF, (AF >> 8) - (AF >> 8));
      break;

      // Copy C into B
      case 0x41:
      BC = LowtoHigh(BC,BC);
      break;

      // Copy D into C
      case 0x4A:
      BC = HightoLow(DE,BC);
      break;

      //Copy E into D
      case 0x53:
      DE = LowtoHigh(DE,DE);
      break;

      //Copy B into A
      case 0x78:
      AF = HightoHigh(BC,AF);
      break;

      //Load following byte into B
      case 0x06:
      BC = bytetoHigh(BC,fetch());
      break;

      //Load the following byte into C
      case 0x0E:
      BC = bytetoLow(BC, fetch());
      break;

      //load the following byte into D
      case 0x16:
      DE = bytetoHigh(DE, fetch());
      break;

      //load the following byte into E
      case 0x1E:
      DE = bytetoLow(DE, fetch());
      break;

      //load the following byte into H
      case 0x26:
      HL = bytetoHigh(HL, fetch());
      break;

      //load the following byte into L
      case 0x2E:
      HL = bytetoLow(HL, fetch());
      break;

      //load the following byte into A
      case 0x3E:
      AF = bytetoHigh(AF, fetch());
      break;

      //load the following byte into L then into H
      case 0x21:
      HL = bytetoLow(HL, fetch());
      HL = bytetoHigh(HL, fetch());
      break;

      //Increment the data stored in memory at address HL
      case 0x34:
      memory[HL]++;
      break;

      //DEC the data stored in memory at address HL
      case 0x35:
      memory[HL]--;
      break;

      //Take the following byte and put it in memory at address HL
      case 0x36:
      memory[HL]==fetch();
      break;

      //Store B in the memory at address HL
      case 0x70:
      memory[HL] = (BC >> 8);
      break;

      //Store B in the memory at address HL
      case 0x46:
      BC=LowtoHigh(BC, memory[HL]);
      break;

    // INC BC    Example for increments a word
    case 0x03:
	BC++;
        //ignore setting flags for now
        break;


    default:
        printf("Instruction %02x not valid (at %04x)\n\n", instruction, PC - 1);
        halt();
        break;
    }

}
