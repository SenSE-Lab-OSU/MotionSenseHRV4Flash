#include "ecgRecorder.h"

#include "drivers/ecg/max30001.h"
#include "zephyrfilesystem.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include "BLEService.h"

LOG_MODULE_REGISTER(ecg_recorder, CONFIG_LOG_LEVEL_MAX30001);

#define ECG_RECORD_MAX30001_NODE DT_NODELABEL(max30001)

#define ECG_RECORD_FRAME_SIZE 12
#define ECG_RECORD_SYNC0 0xA5u
#define ECG_RECORD_SYNC1 0xECu
#define ECG_RECORD_TYPE_SAMPLE 0x01u
#define ECG_RECORD_CRC_POLY 0x07u

#define ECG_RECORD_THREAD_STACK_SIZE 4096
#define ECG_RECORD_THREAD_PRIORITY 5

static const struct gpio_dt_spec ecg_intb =
	GPIO_DT_SPEC_GET(ECG_RECORD_MAX30001_NODE, intb_gpios);

static K_SEM_DEFINE(ecg_record_start_sem, 0, 1);
static K_SEM_DEFINE(ecg_fifo_sem, 0, 1);
static K_SEM_DEFINE(ecg_record_stopped_sem, 0, 1);

static struct gpio_callback ecg_intb_callback;
static atomic_t ecg_record_requested;
static atomic_t ecg_record_active;
static bool ecg_intb_callback_added;
static uint32_t ecg_sequence;
uint32_t ecg_total_samples = 0;
uint32_t ecg_last_update_samples = 0;

float random_bar;
uint8_t ecg_packet[ENMO_DATA_LEN] = {0};
#define ECG_COUNTER (sizeof(random_bar))


static void ecg_record_thread(void *arg1, void *arg2, void *arg3);

K_THREAD_DEFINE(ecg_record_thread_id,
		ECG_RECORD_THREAD_STACK_SIZE,
		ecg_record_thread,
		NULL,
		NULL,
		NULL,
		ECG_RECORD_THREAD_PRIORITY,
		0,
		0);

/**
 * @brief Compute a CRC-8 checksum over a byte buffer.
 *
 * Bitwise CRC-8 with polynomial 0x07 (CRC-8/ATM) and initial value 0x00.
 * Used to protect the payload portion of each 12-byte ECG storage frame so
 * that corruption can be detected when frames are later parsed off the NAND
 * filesystem.
 *
 * @param data Buffer to checksum.
 * @param len  Number of bytes to include.
 *
 * @return The 8-bit CRC value.
 */
static uint8_t ecg_record_crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++) {
			if ((crc & 0x80u) != 0u) {
				crc = (uint8_t)((crc << 1) ^ ECG_RECORD_CRC_POLY);
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

/**
 * @brief GPIO interrupt handler for the MAX30001 INTB line.
 *
 * Runs in interrupt context when the MAX30001 asserts INTB (the ECG FIFO has
 * reached its configured threshold). It does the minimum possible work:
 * giving ecg_fifo_sem to wake the recorder thread, which performs the actual
 * SPI FIFO drain in thread context where blocking is allowed.
 *
 * @param port GPIO port device (unused).
 * @param cb   Callback structure (unused).
 * @param pins Bitmask of triggering pins (unused).
 */
static void ecg_record_intb_handler(const struct device *port,
				    struct gpio_callback *cb,
				    uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_sem_give(&ecg_fifo_sem);
}

/**
 * @brief Prepare the MAX30001 INTB GPIO for use as a data-ready interrupt.
 *
 * Configures the INTB pin (taken from the max30001 devicetree node) as an
 * input and registers ecg_record_intb_handler() as its callback. The
 * callback is only added once per boot (tracked by ecg_intb_callback_added)
 * so repeated start/stop cycles do not accumulate duplicate callbacks. The
 * pin interrupt itself is left DISABLED; ecg_record_run() enables edge
 * triggering only after the sensor is fully configured, avoiding spurious
 * wakeups during bring-up.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the GPIO controller is not ready.
 * @retval Other negative errno from GPIO configuration calls.
 */
static int ecg_record_configure_intb(void)
{
	int ret;

	if (!gpio_is_ready_dt(&ecg_intb)) {
		LOG_ERR("MAX30001 INTB GPIO is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&ecg_intb, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	if (!ecg_intb_callback_added) {
		gpio_init_callback(&ecg_intb_callback,
				   ecg_record_intb_handler,
				   BIT(ecg_intb.pin));

		ret = gpio_add_callback(ecg_intb.port, &ecg_intb_callback);
		if (ret != 0) {
			return ret;
		}
		ecg_intb_callback_added = true;
	}

	return gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_DISABLE);
}

/**
 * @brief Serialize one ECG sample into a framed record and queue it for
 *        storage.
 *
 * Packs the sample into a fixed 12-byte frame designed to be robust when
 * read back from raw storage:
 *
 *   [0]  sync byte 0xA5        [1]  sync byte 0xEC
 *   [2]  record type (0x01)    [3]  ETAG (bits 0-2) | PTAG (bits 3-5)
 *   [4-7]  32-bit sample sequence number, little-endian
 *   [8-10] 24-bit raw ECG sample, big-endian
 *   [11] CRC-8 over bytes 2-10
 *
 * The two sync bytes let a parser resynchronize mid-stream, the sequence
 * number exposes dropped samples, and the CRC catches corruption. The frame
 * is handed to store_data() which buffers it for the ECG file on the NAND
 * filesystem.
 *
 * @param sample Decoded ECG sample to store.
 */
static void ecg_record_store_sample(const struct max30001_ecg_sample *sample)
{
	uint8_t frame[ECG_RECORD_FRAME_SIZE];

	frame[0] = ECG_RECORD_SYNC0;
	frame[1] = ECG_RECORD_SYNC1;
	frame[2] = ECG_RECORD_TYPE_SAMPLE;
	frame[3] = (uint8_t)((sample->etag & 0x07u) |
			     ((sample->ptag & 0x07u) << 3));
	frame[4] = (uint8_t)(ecg_sequence & 0xFFu);
	frame[5] = (uint8_t)((ecg_sequence >> 8) & 0xFFu);
	frame[6] = (uint8_t)((ecg_sequence >> 16) & 0xFFu);
	frame[7] = (uint8_t)((ecg_sequence >> 24) & 0xFFu);
	frame[8] = (uint8_t)((sample->raw >> 16) & 0xFFu);
	frame[9] = (uint8_t)((sample->raw >> 8) & 0xFFu);
	frame[10] = (uint8_t)(sample->raw & 0xFFu);
	frame[11] = ecg_record_crc8(&frame[2], 9);

	store_data(frame, sizeof(frame), ecg);
}

/**
 * @brief Store a batch of ECG samples read from the FIFO.
 *
 * Iterates over a batch produced by max30001_ecg_read_fifo(), skipping any
 * sample whose time_valid flag is false (words that do not represent a real
 * sample slot), and writes each remaining sample to storage via
 * ecg_record_store_sample(). The global ecg_sequence counter is incremented
 * once per stored sample so the sequence numbers embedded in the frames stay
 * contiguous.
 *
 * @param samples Array of decoded samples.
 * @param count   Number of valid entries in samples.
 */
static void ecg_record_process_samples(const struct max30001_ecg_sample *samples,
				       size_t count)
{
	for (size_t i = 0; i < count; i++) {
		if (!samples[i].time_valid) {
			continue;
		}

		ecg_record_store_sample(&samples[i]);
		ecg_sequence++;
	}
}



/**
 * @brief Empty the MAX30001 FIFO and dispatch the samples.
 *
 * Called from the recorder thread each time the INTB interrupt fires (and
 * once more at shutdown). Reads the FIFO in bursts of up to
 * MAX30001_ECG_FIFO_MAX_SAMPLES, for at most 4 passes, stopping early when a
 * read returns fewer than a full burst or ends on an EOF-tagged sample —
 * both signs the FIFO is empty. The pass limit bounds time spent here if
 * samples arrive as fast as they are drained.
 *
 * Each burst is persisted via ecg_record_process_samples(). Running totals
 * are also maintained, and once at least 1024 new samples have accumulated,
 * the cumulative sample count is copied into ecg_packet and submitted to the
 * BLE work queue as a lightweight progress notification for the connected
 * host.
 *
 * FIFO read errors are logged and abort the drain; the next interrupt
 * retries naturally.
 */
static void ecg_record_drain_fifo(void)
{
	int ret;

	for (int pass = 0; pass < 4; pass++) {
		struct max30001_ecg_sample samples[MAX30001_ECG_FIFO_MAX_SAMPLES];
		size_t count = 0;

		ret = max30001_ecg_read_fifo(samples, ARRAY_SIZE(samples), &count);
		if (ret != 0) {
			LOG_ERR("MAX30001 ECG FIFO read failed: %d", ret);
			return;
		}

		if (count == 0) {
			return;
		}
		
		
		
		ecg_record_process_samples(samples, count);

		ecg_total_samples += count;
		ecg_last_update_samples += count;
		if (ecg_last_update_samples >= 1024){
			LOG_INF("count: %d", ecg_total_samples);
			
			memcpy(&ecg_packet[4], &ecg_total_samples, sizeof(ecg_total_samples));
			my_motionData.dataPacket = ecg_packet;
      		my_motionData.packetLength = sizeof(ecg_packet);
			ecg_last_update_samples = 0;
      		k_work_submit(&my_motionData.work);
		}

		if (count < ARRAY_SIZE(samples) || samples[count - 1].eof) {
			return;
		}
	}
}

/**
 * @brief Execute one complete ECG recording session.
 *
 * The main body of a recording: sets up the INTB GPIO, resets the sample
 * sequence counter, initializes the MAX30001 for 512 Hz acquisition, starts
 * streaming, and enables the edge interrupt on INTB. It then loops for as
 * long as ecg_record_requested remains set, sleeping on ecg_fifo_sem and
 * draining the FIFO each time the interrupt (or a 1 s timeout, as a safety
 * net against a missed edge) wakes it.
 *
 * On exit — whether from a stop request or a setup failure — the interrupt
 * is disabled, one final FIFO drain captures any remaining samples, and the
 * sensor is powered down via max30001_ecg_stop(). Requires the NAND
 * filesystem to be ready before starting.
 *
 * @retval 0 on a clean stop.
 * @retval -ENODEV if the filesystem is not ready.
 * @retval Other negative errno if sensor or GPIO setup fails.
 */
static int ecg_record_run(void)
{
	int ret;

	if (!file_system_ready) {
		LOG_ERR("Filesystem is not ready for ECG recording");
		return -ENODEV;
	}

	ret = ecg_record_configure_intb();
	if (ret != 0) {
		LOG_ERR("Failed to configure MAX30001 INTB: %d", ret);
		return ret;
	}

	ecg_sequence = 0;
	k_sem_reset(&ecg_fifo_sem);

	ret = max30001_ecg_init_512();
	if (ret != 0) {
		LOG_ERR("MAX30001 ECG init failed: %d", ret);
		(void)max30001_ecg_stop();
		return ret;
	}

	ret = max30001_ecg_start();
	if (ret != 0) {
		LOG_ERR("MAX30001 ECG start failed: %d", ret);
		(void)max30001_ecg_stop();
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to enable MAX30001 INTB interrupt: %d", ret);
		(void)max30001_ecg_stop();
		return ret;
	}

	k_sem_give(&ecg_fifo_sem);
	LOG_INF("ECG NAND recording active");

	while (atomic_get(&ecg_record_requested) != 0) {
		ret = k_sem_take(&ecg_fifo_sem, K_SECONDS(1));
		if (ret == 0) {
			ecg_record_drain_fifo();
		}
	}

	(void)gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_DISABLE);
	ecg_record_drain_fifo();
	(void)max30001_ecg_stop();

	return 0;
}

/**
 * @brief Dedicated recorder thread: supervises ECG recording sessions.
 *
 * Created at boot by K_THREAD_DEFINE and never exits. It blocks on
 * ecg_record_start_sem until ecg_recorder_start() signals a session, then
 * repeatedly invokes ecg_record_run() while ecg_record_requested is set.
 * Around each run it maintains ecg_record_active (so ecg_recorder_stop() can
 * tell whether a session is in flight) and gives ecg_record_stopped_sem so
 * the stopper can synchronize with session teardown.
 *
 * If a session ends while a recording is still requested (i.e. it aborted on
 * error rather than being stopped), the error is logged and the session is
 * retried after a 1-second backoff, making recording self-healing across
 * transient sensor or bus failures.
 *
 * @param arg1 Unused.
 * @param arg2 Unused.
 * @param arg3 Unused.
 */
static void ecg_record_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for (;;) {
		(void)k_sem_take(&ecg_record_start_sem, K_FOREVER);

		while (atomic_get(&ecg_record_requested) != 0) {
			int ret;

			atomic_set(&ecg_record_active, 1);
			ret = ecg_record_run();
			atomic_clear(&ecg_record_active);
			k_sem_give(&ecg_record_stopped_sem);

			if (atomic_get(&ecg_record_requested) == 0) {
				break;
			}

			LOG_ERR("ECG recording stopped after error: %d", ret);
			k_sleep(K_SECONDS(1));
		}
	}
}

/**
 * @brief Request that ECG recording begin (public API).
 *
 * Non-blocking entry point called from application code (e.g. when the user
 * enters ECG collection mode). After confirming the filesystem is ready, it
 * atomically transitions ecg_record_requested from 0 to 1 and wakes the
 * recorder thread, which performs all the actual sensor setup and streaming.
 * The compare-and-set makes the call idempotent: if a recording is already
 * requested, nothing happens and 0 is returned.
 *
 * @retval 0 if recording was started or was already running.
 * @retval -ENODEV if the filesystem is not ready.
 */
int ecg_recorder_start(void)
{
	if (!file_system_ready) {
		return -ENODEV;
	}

	k_sem_reset(&ecg_record_stopped_sem);

	if (atomic_cas(&ecg_record_requested, 0, 1)) {
		k_sem_give(&ecg_record_start_sem);
	}

	return 0;
}

/**
 * @brief Request that ECG recording stop and wait for it to finish
 *        (public API).
 *
 * Clears ecg_record_requested so the recorder thread's session loop exits,
 * and gives ecg_fifo_sem to wake the thread immediately rather than letting
 * it wait out its 1-second semaphore timeout. If no session was active the
 * call returns at once; otherwise it blocks (up to 3 seconds) on
 * ecg_record_stopped_sem until the thread has drained the final samples and
 * powered down the sensor, so callers know storage is quiescent when this
 * returns.
 *
 * @retval 0 once recording has stopped (or none was active).
 * @retval -EAGAIN if the active session did not confirm shutdown within 3 s.
 */
int ecg_recorder_stop(void)
{
	atomic_clear(&ecg_record_requested);
	k_sem_give(&ecg_fifo_sem);

	if (atomic_get(&ecg_record_active) == 0) {
		return 0;
	}

	return k_sem_take(&ecg_record_stopped_sem, K_SECONDS(3));
}
