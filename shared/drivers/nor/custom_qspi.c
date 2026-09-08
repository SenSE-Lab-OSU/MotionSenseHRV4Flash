#include <hal/nrf_qspi.h>
#include "custom_qspi.h"



int custom_qspi_send_cmd(const struct device *dev, const struct qspi_cmd *cmd,
			 bool wren)
{
	
	
	/* Check input parameters */
	if (!cmd) {
		return -1;
	}

	const void *tx_buf = NULL;
	size_t tx_len = 0;
	void *rx_buf = NULL;
	size_t rx_len = 0;
	size_t xfer_len = sizeof(cmd->op_code);

	if (cmd->tx_buf) {
		tx_buf = cmd->tx_buf->buf;
		tx_len = cmd->tx_buf->len;
	}

	if (cmd->rx_buf) {
		rx_buf = cmd->rx_buf->buf;
		rx_len = cmd->rx_buf->len;
	}

	if ((rx_len != 0) && (tx_len != 0)) {
		if (rx_len != tx_len) {
			return -1;
		}

		xfer_len += tx_len;
	} else {
		/* At least one of these is zero. */
		xfer_len += tx_len + rx_len;
	}

	if (xfer_len > NRF_QSPI_CINSTR_LEN_9B) {
		return -1;
	}
	
	nrf_qspi_cinstr_conf_t cinstr_cfg = {
		.opcode = cmd->op_code,
		.length = xfer_len,
		.io2_level = true,
		.io3_level = true,
		.wipwait = false,
		.wren = wren,
	};
	struct qspi_nor_data* data = dev->data;
	k_sem_take(&data->sem, K_FOREVER);
	//qspi_lock(dev);

	int res = nrfx_qspi_cinstr_xfer(&cinstr_cfg, tx_buf, rx_buf);
	k_sem_take(&data->sync, K_FOREVER);
	k_sem_give(&data->sem);
	//qspi_unlock(dev);
	return res;
}


int custom_qspi_send_address_cmd(const struct device *dev, const struct qspi_cmd *cmd,
			uint8_t* addres_ptr, uint8_t address_len, bool wren)
{
	
	
	/* Check input parameters */
	if (!cmd) {
		return -1;
	}

	const void *tx_buf = NULL;
	size_t tx_len = 0;
	void *rx_buf = NULL;
	size_t rx_len = 0;
	size_t xfer_len = sizeof(cmd->op_code);

	if (cmd->tx_buf) {
		tx_buf = cmd->tx_buf->buf;
		tx_len = cmd->tx_buf->len;
	}

	if (cmd->rx_buf) {
		rx_buf = cmd->rx_buf->buf;
		rx_len = cmd->rx_buf->len;
	}

	if ((rx_len != 0) && (tx_len != 0)) {
		if (rx_len != tx_len) {
			return -1;
		}

		xfer_len += tx_len;
	} else {
		/* At least one of these is zero. */
		xfer_len += tx_len + rx_len;
	}

	if (xfer_len > NRF_QSPI_CINSTR_LEN_9B) {
		return -1;
	}
	
	nrf_qspi_cinstr_conf_t cinstr_cfg = {
		.opcode = cmd->op_code,
		.length = NRF_QSPI_CINSTR_LEN_1B,
		.io2_level = true,
		.io3_level = true,
		.wipwait = false,
		.wren = wren,
	};

	struct qspi_nor_data* data = dev->data;
	k_sem_take(&data->sem, K_FOREVER);
	//qspi_lock(dev);
	nrfx_qspi_lfm_start(&cinstr_cfg);


	int rest = nrfx_qspi_lfm_xfer(addres_ptr, NULL, address_len, false);

	int res = nrfx_qspi_cinstr_xfer(&cinstr_cfg, tx_buf, rx_buf);
	k_sem_take(&data->sync, K_FOREVER);
	k_sem_give(&data->sem);
	//qspi_unlock(dev);
	return res;
}