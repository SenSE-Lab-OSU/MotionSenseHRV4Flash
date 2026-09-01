#include <zephyr/usb/class/usb_msc.h>

#include "msense_msc_media.h"

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
	return usb_mass_storage_set_medium_present(false);
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
