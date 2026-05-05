
#include "spi_nand.h"
#include "bad_page.h"

#define MAX_BAD_PAGES 100
#define PAGE_SIZE 4096

static off_t bad_pages[MAX_BAD_PAGES] = {}; // Global array of bad pages, initialize to 0 (invalid)

/**
 * Wrapper for spi_nand_page_read that simulates bad pages.
 * For pages in the bad_pages array, fills dest with 0xFF (bad output).
 * Otherwise, calls the original spi_nand_page_read.
 */
int spi_nand_page_read_badsim_wrapper(const struct device* dev, off_t page_addr, void* dest) {
    for (int i = 0; i < MAX_BAD_PAGES; i++) {
        if (bad_pages[i] == page_addr && bad_pages[i] != 0) {
            // Bad page: fill with 0xFF
            memset(dest, 0xFF, PAGE_SIZE);
            return 0; // Success, but with bad data
        }
    }
    // Not bad: call original
    return spi_nand_page_read(dev, page_addr, dest);
}

/**
 * Wrapper for spi_nand_page_write that simulates bad pages.
 * For pages in the bad_pages array, returns an error without writing.
 * Otherwise, calls the original spi_nand_page_write.
 */
int spi_nand_page_write_badsim_wrapper(const struct device* dev, off_t page_address, const void* src, size_t size) {
    for (int i = 0; i < MAX_BAD_PAGES; i++) {
        if (bad_pages[i] == page_address && bad_pages[i] != 0) {
            return 0; //  Fail Silently
        }
    }
    // Not bad: call original
    return spi_nand_page_write(dev, page_address, src, size);
}

/**
 * Function to add a bad page to the global array.
 * Returns 0 on success, -1 if array is full.
 */
int add_bad_page(off_t page) {
    for (int i = 0; i < MAX_BAD_PAGES; i++) {
        if (bad_pages[i] == 0) {
            bad_pages[i] = page;
            return 0;
        }
    }
    return -1; // Array full
}

/**
 * Function to remove a bad page from the global array.
 * Returns 0 on success, -1 if not found.
 */
int remove_bad_page(off_t page) {
    for (int i = 0; i < MAX_BAD_PAGES; i++) {
        if (bad_pages[i] == page) {
            bad_pages[i] = 0;
            return 0;
        }
    }
    return -1; // Not found
}


int spi_nand_bad_page_init(const struct device* dev) {
    
    add_bad_page(126);
    add_bad_page(10895);
    return 0;
}