# Non-hardware regression guard for the shared MT29 raw NAND geometry.
# It examines source/DTS inputs only; it never opens a storage device.

if(NOT DEFINED MSENSE_SOURCE_ROOT)
	message(FATAL_ERROR "MSENSE_SOURCE_ROOT is required")
endif()

function(require_match file pattern description)
	file(READ "${file}" contents)
	if(NOT contents MATCHES "${pattern}")
		message(FATAL_ERROR "${description}: ${file}")
	endif()
endfunction()

set(PPG_DTS "${MSENSE_SOURCE_ROOT}/boards/senselab/ppgv2/ppgv2_nrf5340_cpuapp.dts")
set(ECG_DTS "${MSENSE_SOURCE_ROOT}/boards/senselab/ecgv0/ecgv0_nrf5340_cpuapp.dts")
set(NAND_SOURCE "${MSENSE_SOURCE_ROOT}/shared/drivers/nand/spi_nand.c")

foreach(dts IN ITEMS "${PPG_DTS}" "${ECG_DTS}")
	require_match("${dts}" "individual-size[ \\t]*=[ \\t]*<1073741824>"
		"each package must be exactly 1 GiB")
	require_match("${dts}" "dies-per-flash[ \\t]*=[ \\t]*<2>"
		"each package must have two dies")
	require_match("${dts}" "page-size[ \\t]*=[ \\t]*<4096>"
		"each die must use 4096-byte pages")
	require_match("${dts}" "pages-per-erase-block[ \\t]*=[ \\t]*<64>"
		"each erase block must contain 64 pages")
endforeach()

require_match("${ECG_DTS}" "num-flashchips[ \\t]*=[ \\t]*<2>"
	"ECGv0 must configure two packages")
require_match("${PPG_DTS}" "num-flashchips[ \\t]*=[ \\t]*<4>"
	"PPGv2 must configure four packages")

# 1 GiB / 4096 bytes per page / 2 dies / 64 pages per block = 2048 blocks/die.
math(EXPR BLOCKS_PER_DIE "1073741824 / 4096 / 2 / 64")
if(NOT BLOCKS_PER_DIE EQUAL 2048)
	message(FATAL_ERROR "MT29 geometry must produce 2048 erase blocks per die")
endif()

require_match("${NAND_SOURCE}"
	"for[ \\t]*\\(int current_block = 0; current_block < block_count; current_block\\+\\+\\)"
	"die erase must stop before block_count (block 2048 is invalid)")
file(READ "${NAND_SOURCE}" nand_contents)
if(nand_contents MATCHES "current_block <= block_count")
	message(FATAL_ERROR "die erase loop must not address block_count")
endif()
if(nand_contents MATCHES "current_die\\[[0-9]\\]")
	message(FATAL_ERROR "NAND driver must not use fixed package-array indices")
endif()
require_match("${NAND_SOURCE}" "current_flash >= cfg->num_flashes"
	"current_flash must be bounds-checked before package-array access")

string(FIND "${nand_contents}" "int spi_nand_multi_chip_erase" multi_erase_start)
string(FIND "${nand_contents}" "static int spi_read_jedec_id" multi_erase_end)
if((multi_erase_start EQUAL -1) OR (multi_erase_end EQUAL -1))
	message(FATAL_ERROR "unable to locate multi-package erase implementation")
endif()
math(EXPR multi_erase_length "${multi_erase_end} - ${multi_erase_start}")
string(SUBSTRING "${nand_contents}" ${multi_erase_start} ${multi_erase_length} multi_erase_source)
string(FIND "${multi_erase_source}" "ret = erase_file_table();" file_table_erase_pos)
if(file_table_erase_pos EQUAL -1)
	message(FATAL_ERROR "multi-package erase must erase the NOR file table")
endif()
string(SUBSTRING "${multi_erase_source}" ${file_table_erase_pos} -1 file_table_erase_source)
string(FIND "${file_table_erase_source}" "LOG_ERR(\"failed to erase file table\")" file_table_error_pos)
string(FIND "${file_table_erase_source}" "return ret;" file_table_return_pos)
string(FIND "${file_table_erase_source}" "LOG_INF(\"all erase complete!\")" all_complete_pos)
if((file_table_error_pos EQUAL -1) OR (file_table_return_pos EQUAL -1) OR
   (all_complete_pos EQUAL -1) OR NOT (file_table_error_pos LESS file_table_return_pos) OR
   NOT (file_table_return_pos LESS all_complete_pos))
	message(FATAL_ERROR "NOR file-table erase failure must return before all-erase success is logged")
endif()
