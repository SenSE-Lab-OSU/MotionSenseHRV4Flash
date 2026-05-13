#ifndef BAD_PAGE_H
#define BAD_PAGE_H


#include <stddef.h>


int save_bad_sectors_arr();
int load_bad_sectors_arr();
int register_bad_sector(uint32_t sector_num);
int get_sector_offset(int sector_num);


int spi_nand_bad_page_init(const struct device* dev);
int spi_nand_page_read_badsim_wrapper(const struct device* dev, off_t page_addr, void* dest);
int spi_nand_page_write_badsim_wrapper(const struct device* dev, off_t page_address, const void* src, size_t size);
int add_bad_page(off_t page);
int remove_bad_page(off_t page);

#endif /* BAD_PAGE_H */