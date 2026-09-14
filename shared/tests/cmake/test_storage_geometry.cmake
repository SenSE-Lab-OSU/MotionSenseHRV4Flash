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
set(NAND_HEADER "${MSENSE_SOURCE_ROOT}/shared/drivers/nand/spi_nand.h")
set(NAND_DISK_SOURCE "${MSENSE_SOURCE_ROOT}/shared/drivers/nand/nand_disk.c")
set(BAD_PAGE_SOURCE "${MSENSE_SOURCE_ROOT}/shared/drivers/nand/bad_page.c")
set(NAND_BINDING "${MSENSE_SOURCE_ROOT}/shared/dts/bindings/senselab,nanddisk.yaml")
set(STORAGE_LOG_SOURCE "${MSENSE_SOURCE_ROOT}/shared/storage_log_backend.c")

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

function(require_shared_storage_bus dts nor_index cs_count cs_order)
	file(READ "${dts}" contents)
	require_match("${dts}" "&spi0[ \\t\\r\\n]*\\{[^}]*status[ \\t]*=[ \\t]*\"disabled\""
		"unused SPIM0 must be disabled")

	string(FIND "${contents}" "&spi4 {" spi4_start)
	string(FIND "${contents}" "&qspi" spi4_end)
	if((spi4_start EQUAL -1) OR (spi4_end EQUAL -1) OR
	   NOT (spi4_start LESS spi4_end))
		message(FATAL_ERROR "unable to isolate SPIM4 storage bus: ${dts}")
	endif()
	math(EXPR spi4_length "${spi4_end} - ${spi4_start}")
	string(SUBSTRING "${contents}" ${spi4_start} ${spi4_length} spi4_source)
	if(NOT spi4_source MATCHES "mx25u80:[ \\t]*mx25u8035f@${nor_index}[ \\t\\r\\n]*\\{[^}]*reg[ \\t]*=[ \\t]*<${nor_index}>")
		message(FATAL_ERROR "NOR must use appended SPIM4 CS index ${nor_index}: ${dts}")
	endif()
	string(REGEX MATCH "cs-gpios[ \\t\\r\\n]*=[^;]*;" cs_property "${spi4_source}")
	string(REGEX MATCHALL "GPIO_ACTIVE_LOW" cs_entries "${cs_property}")
	list(LENGTH cs_entries actual_cs_count)
	if(NOT actual_cs_count EQUAL cs_count)
		message(FATAL_ERROR "SPIM4 must have ${cs_count} storage chip selects: ${dts}")
	endif()
	if(NOT cs_property MATCHES "${cs_order}")
		message(FATAL_ERROR "SPIM4 NAND/NOR chip-select order is wrong: ${dts}")
	endif()
endfunction()

require_shared_storage_bus("${ECG_DTS}" 2 3
	"gpio0[ \\t]+18[^;]*gpio0[ \\t]+4[^;]*gpio0[ \\t]+5")
require_shared_storage_bus("${PPG_DTS}" 4 5
	"gpio0[ \\t]+18[^;]*gpio0[ \\t]+4[^;]*gpio0[ \\t]+21[^;]*gpio0[ \\t]+19[^;]*gpio0[ \\t]+5")

require_match("${NAND_DISK_SOURCE}"
	"DT_PROP_LEN\\(DT_BUS\\(DT_DRV_INST\\(0\\)\\), cs_gpios\\)[^=]*>=[^D]*DT_INST_PROP\\(0, num_flashchips\\)"
	"the controller may have sibling CS entries after the NAND entries")
foreach(index RANGE 0 3)
	require_match("${NAND_DISK_SOURCE}"
		"GPIO_DT_SPEC_GET_BY_IDX\\(DT_BUS\\(DT_DRV_INST\\(0\\)\\), cs_gpios, ${index}\\)"
		"NAND package CS ${index} must retain its array index")
endforeach()
file(READ "${NAND_DISK_SOURCE}" nand_disk_contents)
if(nand_disk_contents MATCHES "cs_gpios, 4\\)")
	message(FATAL_ERROR "the sibling NOR CS must not enter the NAND package array")
endif()
require_match("${NAND_BINDING}" "first num-flashchips entries"
	"the binding must define the first-N NAND CS contract")
require_match("${NAND_SOURCE}" "K_MUTEX_DEFINE\\(storage_spi_bus_mutex\\)"
	"shared storage bus operations need a mutex")
require_match("${NAND_SOURCE}" "static void acquire_device[^}]*storage_spi_bus_lock\\(\\)"
	"complete NAND operations must acquire the shared storage bus")
require_match("${NAND_SOURCE}" "storage_spi_bus_unlock\\(\\);"
	"complete NAND operations must release the shared storage bus")
require_match("${NAND_DISK_SOURCE}" "storage_spi_bus_lock\\(\\);"
	"NOR file-table operations must acquire the shared storage bus")
require_match("${BAD_PAGE_SOURCE}" "storage_spi_bus_lock\\(\\);"
	"NOR bad-sector metadata operations must acquire the shared storage bus")

# 1 GiB / 4096 bytes per page / 2 dies / 64 pages per block = 2048 blocks/die.
math(EXPR BLOCKS_PER_DIE "1073741824 / 4096 / 2 / 64")
if(NOT BLOCKS_PER_DIE EQUAL 2048)
	message(FATAL_ERROR "MT29 geometry must produce 2048 erase blocks per die")
endif()

require_match("${NAND_SOURCE}"
	"for[ \\t]*\\(int current_block = 0; current_block < block_count; current_block\\+\\+\\)"
	"die erase must stop before block_count (block 2048 is invalid)")
file(READ "${NAND_SOURCE}" nand_contents)
file(READ "${NAND_HEADER}" nand_header_contents)
if(nand_contents MATCHES "return[ \t]+253" OR
   nand_contents MATCHES "2000000000|1900000000" OR
   nand_header_contents MATCHES "spi_flash_wait_until_ready")
	message(FATAL_ERROR "NAND status transport errors and waits must remain bounded")
endif()
require_match("${NAND_SOURCE}"
	"static int get_features\\([^;]*uint8_t \\*data\\)"
	"GET FEATURE must return errno separately from the status byte")
require_match("${NAND_SOURCE}" "#define NAND_STATUS_POLL_INTERVAL_US 100U"
	"NAND readiness polling must yield between status reads")
require_match("${NAND_SOURCE}" "#define NAND_PAGE_PROGRAM_TIMEOUT_US 750U"
	"page program completion must have a bounded operation-specific timeout")
require_match("${NAND_SOURCE}" "#define NAND_BLOCK_ERASE_TIMEOUT_US 12000U"
	"block erase completion must have a bounded operation-specific timeout")
require_match("${NAND_SOURCE}" "disable_ret = write_disable\\(dev\\)"
	"NAND write operations must attempt WRDI in their common cleanup path")
require_match("${NAND_SOURCE}" "#define NAND_STATUS_PROGRAM_FAIL BIT\\(3\\)"
	"page program must use the NAND P_FAIL bit")
require_match("${NAND_SOURCE}" "#define NAND_STATUS_ERASE_FAIL BIT\\(2\\)"
	"block erase must use the NAND E_FAIL bit")
require_match("${NAND_SOURCE}" "#define NAND_RESET_NO_COMMAND_US 1500U"
	"stacked-die RESET needs a command-free interval")
require_match("${NAND_SOURCE}" "#define NAND_RESET_POLL_TIMEOUT_US 1000U"
	"RESET polling must use only the remaining bounded budget")
require_match("${NAND_SOURCE}" "return flash_reset_and_unlock\\(dev\\);"
	"NAND configuration must propagate reset and unlock failures")

string(FIND "${nand_contents}" "static int flash_reset_and_unlock" reset_start)
string(FIND "${nand_contents}" "static int spi_configure" reset_end)
if((reset_start EQUAL -1) OR (reset_end EQUAL -1) OR
   NOT (reset_start LESS reset_end))
	message(FATAL_ERROR "unable to locate NAND reset/unlock implementation")
endif()
math(EXPR reset_length "${reset_end} - ${reset_start}")
string(SUBSTRING "${nand_contents}" ${reset_start} ${reset_length} reset_source)
string(FIND "${reset_source}" "ret = spi_nand_reset(dev);" reset_command)
string(FIND "${reset_source}" "k_sleep(K_USEC(NAND_RESET_NO_COMMAND_US));" reset_delay)
string(FIND "${reset_source}" "spi_nand_wait_until_ready" reset_wait)
string(FIND "${reset_source}" "die < cfg->dies_per_flash" reset_die_loop)
string(FIND "${reset_source}" "restore_ret = set_die(dev, 0);" reset_restore)
if((reset_command EQUAL -1) OR (reset_delay EQUAL -1) OR
   (reset_wait EQUAL -1) OR (reset_die_loop EQUAL -1) OR
   (reset_restore EQUAL -1) OR NOT (reset_command LESS reset_delay) OR
   NOT (reset_delay LESS reset_wait) OR NOT (reset_wait LESS reset_die_loop) OR
   NOT (reset_die_loop LESS reset_restore))
	message(FATAL_ERROR "NAND reset must delay, poll, unlock each die, and restore die 0")
endif()

if(nand_disk_contents MATCHES "disk_nand_access_init")
	message(FATAL_ERROR "NAND initialization must not be hidden by a success wrapper")
endif()
string(FIND "${nand_disk_contents}" "static int nand_disk_device_init" disk_init_start)
if(disk_init_start EQUAL -1)
	message(FATAL_ERROR "unable to locate NAND disk registration")
endif()
string(SUBSTRING "${nand_disk_contents}" ${disk_init_start} -1 disk_init_source)
string(FIND "${disk_init_source}" "ret = spi_init(dev);" disk_spi_init)
string(FIND "${disk_init_source}" "if (ret != 0)" disk_init_check)
string(FIND "${disk_init_source}" "return disk_access_register(&nand_disk);" disk_register)
if((disk_spi_init EQUAL -1) OR (disk_init_check EQUAL -1) OR
   (disk_register EQUAL -1) OR NOT (disk_spi_init LESS disk_init_check) OR
   NOT (disk_init_check LESS disk_register))
	message(FATAL_ERROR "NAND disk registration must follow successful hardware initialization")
endif()
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

# log_output_write() retries its callback until the requested length is
# consumed. The storage backend must therefore report intentionally discarded
# bytes as consumed instead of returning zero and blocking later backends.
file(READ "${STORAGE_LOG_SOURCE}" storage_log_contents)
string(FIND "${storage_log_contents}" "int write_log_to_file" storage_write_start)
string(FIND "${storage_log_contents}" "BUILD_ASSERT" storage_write_end)
if((storage_write_start EQUAL -1) OR (storage_write_end EQUAL -1))
	message(FATAL_ERROR "unable to locate storage log output callback")
endif()
math(EXPR storage_write_length "${storage_write_end} - ${storage_write_start}")
string(SUBSTRING "${storage_log_contents}" ${storage_write_start} ${storage_write_length}
	storage_write_source)
string(FIND "${storage_write_source}" "if (!msense_storage_log_write_enabled())" disabled_write_start)
string(FIND "${storage_write_source}" "return msense_storage_log_append" storage_append_start)
if((disabled_write_start EQUAL -1) OR (storage_append_start EQUAL -1))
	message(FATAL_ERROR "storage log callback must retain its policy and append paths")
endif()
math(EXPR disabled_write_length "${storage_append_start} - ${disabled_write_start}")
string(SUBSTRING "${storage_write_source}" ${disabled_write_start} ${disabled_write_length}
	disabled_write_source)
if(disabled_write_source MATCHES "return[ \t]+0;")
	message(FATAL_ERROR "disabled storage logging must not return zero to log_output_write")
endif()
if(NOT disabled_write_source MATCHES "return[ \t]+\\(int\\)length;")
	message(FATAL_ERROR "disabled storage logging must report every discarded byte as consumed")
endif()
