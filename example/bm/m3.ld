ENTRY(Reset_Handler)

MEMORY {
    FLASH (RX) : ORIGIN = 0, LENGTH = 1024K
    SRAM (RWX) : ORIGIN = 0x20000000, LENGTH = 320K
}

stack_top = 0x20001000;

SECTIONS {
    .isr_vector : {
        . = ALIGN(8);
        KEEP(*(.isr_vector))
        . = ALIGN(8);
    } > FLASH

    .text : {
        . = ALIGN(8);
        *(.text)
        . = ALIGN(8);
        text_end = .;
    } > FLASH

    .data : { *(.data) } >SRAM AT>FLASH
    .bss : { *(.bss COMMON) } > SRAM
    . = ALIGN(8);
}
