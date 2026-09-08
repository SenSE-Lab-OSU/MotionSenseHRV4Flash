#ifndef MSENSE_MSC_MEDIA_H_
#define MSENSE_MSC_MEDIA_H_

/**
 * Initialize the legacy MSC LUN in its safe host-facing state.
 *
 * This function must be called from thread context after usb_enable() has
 * initialized legacy MSC. It leaves the medium absent if setting host
 * read-only fails.
 */
int msense_msc_media_initialize_absent(void);

/**
 * Exclude the host from the MSC medium before firmware mounts or writes it.
 *
 * This function must be called from thread context.
 */
int msense_msc_media_claim_for_firmware(void);

/**
 * Publish the MSC medium only after firmware filesystem teardown completes.
 *
 * This function reasserts host read-only before making the medium present and
 * must be called from thread context.
 */
int msense_msc_media_publish_to_host(void);

#endif /* MSENSE_MSC_MEDIA_H_ */
