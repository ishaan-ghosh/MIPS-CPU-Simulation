/*
File: sim3.c
Author: Ishaan Ghosh
Purpose: This program simulates a single cycle CPU, it does this by breaking
the task down into smaller tasks which are then performed using helper functions.
Helper functions which do things like: Extracting the information from the 
instruction, extracting instruction from memory, filling CPU control bits, etc..
*/

#include "sim3.h"

/*
This helper function extracts the fields from the instruction with no intelligence,
meaning it will extract both the I and R format fields from the instruction regardless
of which one it is. It then initializes the InstructionFields struct with the fields.

@param: instruction - the actual instruction bits
        fieldsOut - the part of the instructionfields struct we need to initialize
*/
void extract_instructionFields(WORD instruction, InstructionFields *fieldsOut)
{
    // Extracting both I and R-format fields from the instruction using bit shift
    // Then utilize the correct fields depending on whether the instruction
    // is actually I or R-format
    fieldsOut -> opcode = (unsigned) ((instruction & 0xfc000000) >> 26);
    fieldsOut -> rs = (unsigned) ((instruction & 0x03e00000) >> 21);
    fieldsOut -> rt = (unsigned) ((instruction & 0x001f0000) >> 16);
    fieldsOut -> rd = (unsigned) ((instruction & 0x0000f800) >> 11);
    fieldsOut -> shamt = (unsigned) ((instruction & 0x000007c0) >> 6);
    fieldsOut -> funct = (unsigned) (instruction & 0x0000003f);
    fieldsOut -> address = (unsigned) (instruction & 0x03ffffff);
    fieldsOut -> imm16 = ((unsigned) (instruction << 16)) >> 16;
    // Using helper function to extend 16 bit immediate then bit shifting
    fieldsOut -> imm32 = signExtend16to32((instruction << 16) >> 16);
}

/*
This extracts the information from the information memory, as the PC counter
gives the address in bytes and we are indexing into an array of words, we
need to divide by 4 to find the true address. A word = 4 bytes in MIPS.

@param: curPC - the current program counter
        instructionMemory - Array of instruction memory
@ return: returns the instruction from memory given the curPC
*/
WORD getInstruction(WORD curPC, WORD *instructionMemory)
{
    return instructionMemory[curPC / 4];
}

/*
This helper function sets the control bits for the CPU, reading the information
from the instructionfields struct that has aleady been initialized into the
CPUControl struct. This is the actual intelligence part that we were missing
when initially extracting the information for the instructionfields struct.
It checks the opcode to determine whether it is an I or R format instruction
and then the funct field to check which specific instruction the programmer
wants performed.

@param: fields - the previously initialized fields from the instruction
        controlOut - the actual control bits that we will be initializing for the CPU
@return: 1 if the instruction is recognized, 0 otherwise
*/
int  fill_CPUControl(InstructionFields *fields, CPUControl *controlOut)
{
    int opcode = fields -> opcode;
    int funct = fields -> funct;

    controlOut->ALUsrc = 0;
    controlOut->ALU.op = 0;
    controlOut->ALU.bNegate = 0;
    controlOut->branch = 0;
    controlOut->jump = 0;
    controlOut->memWrite = 0;
    controlOut->memRead = 0;
    controlOut->memToReg = 0;
    controlOut->regDst = 0;
    controlOut->regWrite = 0;
    controlOut->extra1 = 0;
    controlOut->extra2 = 0;
    controlOut->extra3 = 0;

    if (opcode == 0) {
        // If opCode is 0 it is always R format
        // Use R-format fields for calculation
        // The code below just checks what specific instruction needs to be
        // performed by checking the funct field. The relation between
        // the funct value and the value the ALU op and bNegate need to be 
        // set to is detailed in a Appendix A Piazza document
        
        if (funct == 34 || funct == 35) {
            // IF operation is SUB or SUBU
            controlOut -> ALU.op = 2;
            controlOut -> ALU.bNegate = 1;
        } else if (funct == 7) {
            // If operation is SRAV
            controlOut -> ALU.op = 2;
            controlOut -> extra3 = 1;
        } else if (funct == 42) {
            // if operation is SLT
            controlOut -> ALU.op = 3;
            controlOut -> ALU.bNegate = 1;
        } else if (funct == 32 || funct == 33) {
            // if operation is ADD or ADDU
            controlOut -> ALU.op = 2; 
        } else if (funct == 36) {
            // if operation is AND
            controlOut -> ALU.op = 0;
        } else if (funct == 37) {
            // if operation is OR
            controlOut -> ALU.op = 1;
        } else if (funct == 38) {
            // if operation is XOR
            controlOut -> ALU.op = 4;
        } else if (funct == 3) {
            // IF operation is SRA
            controlOut -> ALU.op = 2;
            controlOut -> extra1 = 1;
        } else {
            return 0;
        }
        // Write to register and set dest register to bits 15-11
        controlOut -> regDst = 1;
        controlOut -> regWrite = 1;
        
    } else {
        // Else, it is probably an I-format instruction
        // Use I-format fields
        if (opcode == 35) {
            // If operation is LW
            controlOut -> ALUsrc = 1;
            controlOut -> ALU.op = 2;
            controlOut -> memRead = 1;
            controlOut -> memToReg = 1;
            controlOut -> regWrite = 1;
        } else if (opcode == 43) {
            // If operation is SW
            controlOut -> ALUsrc = 1;
            controlOut -> ALU.op = 2;
            controlOut -> memWrite = 1;
        } else if (opcode == 4) {
            // If operation is BEQ
            controlOut -> ALU.op = 2;
            controlOut -> ALU.bNegate = 1;
            controlOut -> branch = 1;
        } else if (opcode == 14) {
            // If operation is XORI
            controlOut -> ALUsrc = 1;
            controlOut -> regWrite = 1;
            controlOut -> ALU.op = 4;
            controlOut -> extra2 = 1;
        } else if (opcode == 8) {
            // If operation is ADDI
            controlOut -> ALUsrc = 1;
            controlOut -> regWrite = 1;
            controlOut -> ALU.op = 2;
        } else if (opcode == 9) {
            // If operation is ADDIU
            controlOut -> ALUsrc = 1;
            controlOut -> regWrite = 1;
            controlOut -> ALU.op = 2;
        } else if (opcode == 10) {
            // If operation is SLTI
            controlOut -> ALUsrc = 1;
            controlOut -> regWrite = 1;
            controlOut -> ALU.op = 3;
            controlOut -> ALU.bNegate = 1;
        } else if (opcode == 2) {
            // If operation is JUMP
            controlOut -> jump = 1;
        } else {
            return 0;
        }
    }
    return 1;
}

/*
This extracts the correct first input for the ALU depending on whether operation is SRA or not

@param: fields - the previously initialized fields from the instruction
        controlIn - the actual control bits for the CPU
        rsVal - rsVal from instruction
        rtVal - rtVal from instruction
        oldPC - address of previous PC
@return: first input of ALU, usually $rs but can be shamt for SRA op
*/
WORD getALUinput1(CPUControl *controlIn,
                  InstructionFields *fieldsIn,
                  WORD rsVal, WORD rtVal, WORD reg32, WORD reg33,
                  WORD oldPC)
{
    if (controlIn->extra1 == 1) {
        // If operation is SRA
        return fieldsIn -> shamt;
    } else {
        return rsVal;
    }
}

/*
This helper function extracts the correct second input for the ALU depending on
what the ALUsrc is and if operation is XORI

@param: fields - the previously initialized fields from the instruction
        controlIn - the actual control bits for the CPU
        rsVal - rsVal from instruction
        rtVal - rtVal from instruction
        oldPC - address of previous PC
@return: second input of ALU, can be immediate constant or rtVal depending on op
*/
WORD getALUinput2(CPUControl *controlIn,
                  InstructionFields *fieldsIn,
                  WORD rsVal, WORD rtVal, WORD reg32, WORD reg33,
                  WORD oldPC)
{
    if (controlIn -> ALUsrc == 0) {
        // If we want to read from register and not immediate
        return rtVal;
    } else if (controlIn->ALU.op == 4 && controlIn->extra2 == 1) {
        // If we are doing XORI
        return (unsigned) fieldsIn -> imm16;
    } else {
        // For other immediate operations
        return fieldsIn -> imm32;
    }
}

/*
This helper function performs the performs the actual ALU operation
and stores the results in the appropriate fields. It reads various
fields in the controlIn parameter such as the ALU operation and
bNegate to identify which operation needs to be performed then executes
and saves to the approriate field in aluResultOut.

@param:     controlIn - struct that contains all control bits
            input1 - contains the bits for the first input into ALU
            input2 - contains the bits for the second inpiut into ALU
            aluResultOut - struct that will be edited to reflect ALU operation result
*/
void execute_ALU(CPUControl *controlIn,
                 WORD input1, WORD input2,
                 ALUResult  *aluResultOut)
{
    if (controlIn -> ALU.op == 0) {
        // If operation is AND
        aluResultOut -> result = input1 & input2;
        if (aluResultOut -> result == 0) {
            aluResultOut -> zero = 1;
        } else {
            aluResultOut->zero = 0;
        }
    } else if (controlIn -> ALU.op == 1) {
        // If operation is OR
        aluResultOut -> result = input1 | input2;
        if (aluResultOut -> result == 0) {
            aluResultOut -> zero = 1;
        } else {
            aluResultOut->zero = 0;
        }
    } else if (controlIn -> ALU.op == 2) {
        if (controlIn -> ALU.bNegate == 0 && controlIn -> extra1 == 0 && controlIn->extra3 == 0) {
            // If operation is ADD
            aluResultOut -> result = input1 + input2;
            if (aluResultOut -> result == 0) {
                aluResultOut -> zero = 1;
            } else {
            aluResultOut->zero = 0;
            }
        } else if (controlIn -> ALU.bNegate == 1 && controlIn -> extra1 == 0 && controlIn->extra3 == 0) {
            // If operation is SUB
            aluResultOut -> result = input1 - input2;
            if (aluResultOut -> result == 0) {
                aluResultOut -> zero = 1;
            } else {
            aluResultOut->zero = 0;
            }
        } else {
            // If operation is SRA or SRAV
            aluResultOut -> result = input2 >> input1;
            if (aluResultOut -> result == 0) {
                aluResultOut -> zero = 1;
            } else {
            aluResultOut->zero = 0;
            }
        }
    } else if (controlIn -> ALU.op == 3) {
        // If operation is LESS
        aluResultOut -> result = input1 < input2;
        if (aluResultOut -> result == 0) {
            aluResultOut -> zero = 1;
        } else {
            aluResultOut->zero = 0;
        }
    } else if (controlIn -> ALU.op == 4) {
        // If operation is XOR
        aluResultOut -> result = input1 ^ input2;
        if (aluResultOut -> result == 0) {
            aluResultOut -> zero = 1;
        } else {
            aluResultOut->zero = 0;
        }
    } 
}

/*
This helper function implements the data memory unit. It first checks
if we are supposed to write to memory and if so, we take the result address
and divide by 4 (to get WORD addresses) then sets the rtVal to that. It then
checks whether we are supposed to read from memory or not, correctly setting
resultOut of so or setting it to 0 if not.

@param:     controlIn - array of control bits
            aluResultIn - the resulting struct from execute ALU
            rsVal - rs register value
            rtVal - rt register value
            memory - Array of WORDS, represents data memory
            resultOut - read data from memory, if not supposed to read, = 0
*/
void execute_MEM(CPUControl *controlIn,
                 ALUResult  *aluResultIn,
                 WORD        rsVal, WORD rtVal,
                 WORD       *memory,
                 MemResult  *resultOut)
{   
    WORD result = aluResultIn->result;
    if (controlIn -> memWrite == 1) {
        // If we want to write to memory
        memory[result / 4] = rtVal;
    }

    if (controlIn -> memRead == 1) {
        // If we want to read from memory
        resultOut -> readVal = memory[result / 4];
    } else {
        // If we do not need to read from Memory set to 0
        resultOut -> readVal = 0;
    }

}

/*
This helper function decides the address of the next PC

@param :    fields - struct with instruction information
            controlIn - control bits for input
            aluZero - output from ALU
            rsVal - register used for execution
            rtVal - register used for execution
            oldPC - old program counter, needs to be changed

@return: current program counter after change
*/
WORD getNextPC(InstructionFields *fields, CPUControl *controlIn, int aluZero,
               WORD rsVal, WORD rtVal,
               WORD oldPC)
{
    if (controlIn -> jump == 1) {
        // Jump
        return fields -> address * 4 | oldPC & 0xf0000000;
    } else if (controlIn -> branch == 1 && (aluZero == 1 || (aluZero == 0 && controlIn->extra1 == 1))) {
        // beq or bne
        return fields -> imm32 * 4 + (oldPC+4);
    } else {
        // Normal instructions with instruction jumps, branch or otherwise
        return oldPC + 4;
    }
}

/*
This helper method performs the register saving, only if it needs to be done.

@param: fields - struct with instruction fields
        controlIn - struct of control bits
        aluResultIN - struct of the ALU result
        memResultIn - struct of the memory result
        regs - struct of current set of registers
*/
void execute_updateRegs(InstructionFields *fields, CPUControl *controlIn,
                        ALUResult  *aluResultIn, MemResult *memResultIn,
                        WORD       *regs)
{
    if (controlIn->regWrite == 1) {
        // Checks whether we actually have to write to registers
        WORD readVal = memResultIn -> readVal;
        if (controlIn->regDst == 1) {
            // If R-format instruction
            regs[fields->rd] = aluResultIn->result;
        } else {
            if (controlIn->memRead == 1) {
                // If operation is LW
                regs[fields->rt] = readVal;
            } else {
                regs[fields->rt] = aluResultIn->result;
            }
        }
    }
}

