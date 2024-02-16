// This is an example flash driver to show how the devcie driver model works.


#include <zephyr/drivers/flash.h>

//must put this with the compatable: vnd, customflash in device tree

#define DT_DRV_COMPAT vnd_customflash


/* Define data (RAM) and configuration (ROM) structures: */
struct my_dev_data {


     /* per-device values to store in RAM */
};
struct my_dev_cfg {
     uint32_t freq; /* Just an example: initial clock frequency in Hz */
     /* other configuration to store in ROM */
};

/**
 * @brief Initialize and configure the flash
 *
 * @param name The flash name
 * @return 0 on success, negative errno code otherwise
 */
static int qspi_nor_init(const struct device *dev)
{
    return 0;
}




static int custom_erase(const struct device *dev, off_t addr, size_t size){
    
    return 0;
}

static int custom_write(const struct device *dev, off_t addr,
			const void *src,
			size_t size)
{
            return 0;
}

static int custom_read(const struct device *dev, off_t addr, void *dest,
			size_t size){


                return 0;
            }



static const struct flash_driver_api custom_api = {
    .read = custom_read,
    .write = custom_write,
    .erase = custom_erase,
};




#define CREATE_MY_DEVICE(inst)                                       \
     static struct my_dev_data my_data_##inst = {                    \
             /* initialize RAM values as needed, e.g.: */            \
                                  \
                                                                     \
     };                                                              \
     static const struct my_dev_cfg my_cfg_##inst = {                \
             /* initialize ROM values as needed. */                  \
     };                                                              \
     DEVICE_DT_INST_DEFINE(inst,                                     \
                           qspi_nor_init,                     \
                           NULL,                                     \
                           &my_data_##inst,                          \
                           &my_cfg_##inst,                           \
                           POST_KERNEL, 5,  \
                           &custom_api);



/* Call the device creation macro for each instance: */
DT_INST_FOREACH_STATUS_OKAY(CREATE_MY_DEVICE)


/* I love writing low level code! */



