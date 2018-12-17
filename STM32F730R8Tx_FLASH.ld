_estack = 0x2003C000;
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;
MEMORY
{
RAM (xrw) : ORIGIN = 0x20000004, LENGTH = 256K - 4 - 16K
RAME (xrw) : ORIGIN = 0x2003C000, LENGTH = 16K
}
SECTIONS
{    
    DevInfo 0x2003f000 :
    {
        KEEP(build/dev_info.o(*))
    } >RAME

    PrgCode :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
        . = ALIGN(4);
        *(.text)
        KEEP(build/ext_ldr.o(.text))
        *(.text*)
        KEEP(build/ext_ldr.o(.text*))
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP (*(.init))
        KEEP (*(.fini))
        . = ALIGN(4);
        _etext = .;
        . = ALIGN(4);
        *(.rodata)
        KEEP(build/ext_ldr.o(.rodata))
        *(.rodata*)
        KEEP(build/ext_ldr.o(.rodata*))
        . = ALIGN(4);
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        *(.got)
        *(.got*)
    } >RAM
    
    PrgData :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        KEEP(build/ext_ldr.o(.data))
        *(.data*)
        KEEP(build/ext_ldr.o(.data*))
        . = ALIGN(4);
        _edata = .;
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        KEEP(build/ext_ldr.o(.bss))
        *(.bss*)
        KEEP(build/ext_ldr.o(.bss*))
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >RAM

    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 :
    {
        *(.ARM.attributes)
    }
}
