li t4, 0
li t5, 264
li a6, 0
li t6, 1
L3:
li a2, 0x80400000
li t2, 0
li t3, 480000
L1:
li a3, 0
li a4, 800
L0:
beq a3, a4, L2
add a5, t4, a2
lw t1, 0(a5)
sw t1, 0(t2)
addi a2, a2, 4
addi t2, t2, 4
addi a3, a3, 4
bne t2, t3, L0
beq a6, t6, L6
beq a6, x0, L4
L5:
beq t4, t5, L7
beq t4, x0, L8
bne t4, t5, L3
bne t4, x0, L3
ret
L2:
addi a2, a2, 264
j L1
L4:
addi t4, t4, 4
j L5
L6:
addi t4, t4, -4
j L5
L7:
li a6, 1
j L3
L8:
li a6, 0
j L3