.syntax unified
.cpu cortex-m3
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler
.global Reset_Handler
.global demo_irq_handler

.section .text

.thumb_func
Default_Handler:
Infinite_Loop:
    b   Infinite_Loop

.thumb_func
Reset_Handler:
    @ MOV   R0,   #0
    @ MSR  PRIMASK, R0 
    bl main_func
    b .

.thumb_func
demo_irq_handler:
    bl demo_irq_func
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
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0 
    .word demo_irq_handler // CAN1_RX0 for demo_irq