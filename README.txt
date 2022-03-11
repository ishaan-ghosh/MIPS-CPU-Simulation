XORI - exclusive or with an immediate value, R[$rt] ← R[$rs] ^ {0 × 16, imm}, when opcode = 14. 
CPU changes are: adding additional if statement for control bits, setting extra2 to 1 which is then used in getALUinput2 to return the 16 bit immediate instead of 32.



SRA - shift right artithmetic is a signed bit shift to the right R[$rd] ← R[$rt] >> shamt, when opcode = 0 and funct = 3. 
CPU changes are: additional if statement for control bits, setting extra1 to 1, then returning shamt in getALUinput1, and finally shifting by shamt in executeALU if extra1 == 1.



SRAV - exact same as SRA except shifting by rsVal instead of shamt, R[$rd] ← R[$rt] >> R[$rs], when opcode = 0 and funct = 7. 
CPU changes are: additional if statement for control bits, setting extra3 to 1, and shifting by rs in executeALU if extra3 == 1.