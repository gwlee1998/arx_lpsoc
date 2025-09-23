/*======================================================================*/
/* Proxy kernel linker script                                           */
/*======================================================================*/
/* This is the linker script used when building the proxy kernel. */

/*----------------------------------------------------------------------*/
/* Setup                                                                */
/*----------------------------------------------------------------------*/

/* The OUTPUT_ARCH command specifies the machine architecture where the
   argument is one of the names used in the BFD library. More
   specifically one of the entires in bfd/cpu-mips.c */

OUTPUT_ARCH( "riscv" )

MEMORY 
{
  sram (rwx)  : ORIGIN = 0xe0000000, LENGTH = 0x10000
}
MEMORY 
{
  dram (rwx)  : ORIGIN = 0x0, LENGTH = 0x40000000
}
ENTRY(_start)

SECTIONS
{
	.sram_start :
	{
    _sram_start_ = .;
	} >sram
	.notcached :
	{
		. = ALIGN(8);
		*(.notcached)
		. = ALIGN(8);
	} >sram
  .heap_sram :
	{
		. = ALIGN(16);
		_heap_sram_baseaddr_ = .;
	} >sram
	.sram_end :
	{
    _sram_end_ = .;
	} >sram
	.dram_start :
	{
    _dram_start_ = .;
	} >dram
	.init	:
  {
    _text_start_ = .;
		_cacheable_start_ = .;
		*(.text.init)
		. = ALIGN(8);
		*(.init)
		. = ALIGN(8);
  } >dram

	.text :
  {
    *(.text.unlikely .text.unlikely.*)
    *(.text.startup .text.startup.*)
    *(.text .text.*)
    *(.gnu.linkonce.t.*)
		. = ALIGN(8);
  } >dram

  .fini           :
	{
		KEEP (*(SORT_NONE(.fini)))
	} >dram

	.preinit_array  :
	{
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	} >dram

	.init_array     :
	{
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
		KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
		PROVIDE_HIDDEN (__init_array_end = .);
	} >dram

	.fini_array     :
	{
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
		KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} >dram

	.ctors          :
	{
		KEEP (*crtbegin.o(.ctors))
		KEEP (*crtbegin?.o(.ctors))
		KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
		KEEP (*(SORT(.ctors.*)))
		KEEP (*(.ctors))
	} >dram

	.dtors          :
	{
		KEEP (*crtbegin.o(.dtors))
		KEEP (*crtbegin?.o(.dtors))
		KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*(.dtors))
    _text_end_ = .;
	} >dram

  .rodata	:
  {
    *(.rdata)
    *(.rodata .rodata.*)
    *(.gnu.linkonce.r.*)
    *(.non_volatile)
		. = ALIGN(8);
  } >dram
	
	.stack :
	{
		. = ALIGN(16);
		. += (0x10000*1);
		. = ALIGN(16);
		PROVIDE( _sp = . );
	} >dram

  .bigdata_bss :
	{
		. = ALIGN(16);
		_bigdata_bss_start_ = .;
		*(.bigdata_bss)
		_bigdata_bss_end_ = .;
		_cacheable_end_ = NEXT(16);
	} >dram

	.bigdata :
	{
    _backup_start_ = .;
		. = ALIGN(16);
		*(.bigdata)
	} >dram

	.once :
	{
		. = ALIGN(16);
		*(.once)
	} >dram

  .cached :
	{
		. = ALIGN(16);
		*(.cached)
		*(.exclusive)
  } >dram
	.data :
	{
		. = ALIGN(8);
		*(.data .data.*)
    *(.gnu.linkonce.d.*)
		. = ALIGN(8);
	} >dram

	.sdata :
	{
		. = ALIGN(8);
		_gp = . + 0x800;
		*(.sdata .sdata.*)
    *(.gnu.linkonce.s.*)
		. = ALIGN(8);
		*(.srodata .srodata.*)
		. = ALIGN(8);
  } >dram

  .sbss :
	{
		. = ALIGN(8);
		PROVIDE( _fbss = . );
		_bss_start_ = .;
    *(.sbss .sbss.* .gnu.linkonce.sb.*)
    *(.scommon)
		. = ALIGN(8);
	} >dram

	.bss :
	{
		. = ALIGN(8);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(8);
		_bss_end_ = NEXT(4);
	} >dram
  .heap_dram :
	{
		. = ALIGN(16);
		_heap_dram_baseaddr_ = .;
	} >dram
	.dram_end :
	{
    _dram_end_ = .;
	} >dram
	/DISCARD/ : {}
}
