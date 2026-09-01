#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <nrfx_qspi.h>


struct qspi_nor_data {
#ifdef CONFIG_MULTITHREADING
	/* The semaphore to control exclusive access on write/erase. */
	struct k_sem trans;
	/* The semaphore to control exclusive access to the device. */
	struct k_sem sem;
	/* The semaphore to indicate that transfer has completed. */
	struct k_sem sync;
	/* The semaphore to control driver init/uninit. */
	struct k_sem count;
#else /* CONFIG_MULTITHREADING */
	/* A flag that signals completed transfer when threads are
	 * not enabled.
	 */
	volatile bool ready;
#endif /* CONFIG_MULTITHREADING */
	bool xip_enabled;
};


struct qspi_nor_config {
	nrfx_qspi_config_t nrfx_cfg;

	/* Size from devicetree, in bytes */
	uint32_t size;

	/* JEDEC id from devicetree */
	uint8_t id[3];

	const struct pinctrl_dev_config *pcfg;
};


/**
 * @brief QSPI buffer structure
 * Structure used both for TX and RX purposes.
 *
 * @param buf is a valid pointer to a data buffer.
 * Can not be NULL.
 * @param len is the length of the data to be handled.
 * If no data to transmit/receive - pass 0.
 */

struct qspi_buf {
	uint8_t *buf;
	size_t len;
};




/**
 * @brief QSPI command structure
 * Structure used for custom command usage.
 *
 * @param op_code is a command value (i.e 0x9F - get Jedec ID)
 * @param tx_buf structure used for TX purposes. Can be NULL if not used.
 * @param rx_buf structure used for RX purposes. Can be NULL if not used.
 */

struct qspi_cmd {
	uint8_t op_code;
	const struct qspi_buf *tx_buf;
	const struct qspi_buf *rx_buf;
};

int custom_qspi_send_cmd(const struct device *dev, const struct qspi_cmd *cmd, bool wren);