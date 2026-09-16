#include <zephyr/usb/class/usb_msc.h>

#include "msense_msc_media.h"
#if defined(CONFIG_DISK_DRIVER_RAW_NAND)
#include "drivers/nand/nand_disk.h"
#endif

int msense_msc_media_initialize_absent(void)
{
	int ret;

	ret = usb_mass_storage_set_medium_present(false);
	if (ret != 0) {
		return ret;
	}

	return usb_mass_storage_set_read_only(true);
}

int msense_msc_media_claim_for_firmware(void)
{
	int ret = usb_mass_storage_set_medium_present(false);

	if (ret != 0) {
		return ret;
	}

#if defined(CONFIG_DISK_DRIVER_RAW_NAND)
	disk_nand_read_ahead_quiesce();
#endif
	return 0;
}

int msense_msc_media_publish_to_host(void)
{
	int ret;

	ret = usb_mass_storage_set_read_only(true);
	if (ret != 0) {
		return ret;
	}

	return usb_mass_storage_set_medium_present(true);
}
