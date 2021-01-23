.syntax unified
.cpu cortex-m3
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler
.global Reset_Handler

.section .text

.thumb_func
Default_Handler:
Infinite_Loop:
    b   Infinite_Loop

.thumb_func
Reset_Handler:
    bl main_func
    b .

// ISR vecotor data
.section .isr_vector, "a"
g_pfnVectors:
    .word stack_top
    .word Reset_Handler
    .word Default_Handler // NMI
    .word Default_Handler // HardFault
    .word Default_Handler // MemManage
    .word Default_Handler // BusFault
    .word Default_Handler // UsageFault
    .word 0
    .word 0
    .word 0
    .word 0
	.word Default_Handler // SVC
    // and a lot more ...