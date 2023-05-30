ENTRY(reset_handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

MEMORY
{
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
    CCMRAM (rwx): ORIGIN = 0x10000000, LENGTH = 32K
    FLASH  (rx) : ORIGIN = 0x08000000, LENGTH = 512K
}

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        . = ALIGN(4);
        _etext = .;

    } >FLASH

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /*Uninitialized data section */
    . = ALIGN(4);
    .bss :
    {
        _sbss = .;
        *(.bss)
        *(.bss*)
        . = ALIGN(4);
        _ebss = .;
    } >RAM
}