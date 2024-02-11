#include "nand.h"


dhara_nand a;




int dhara_nand_is_bad(const dhara_nand* n, dhara_block_t b)
{
    return 0;
}

void dhara_nand_mark_bad(const dhara_nand* n, dhara_block_t b)
{

}

int dhara_nand_erase(const dhara_nand* n, dhara_block_t b, dhara_error_t* err)
{
    return 0;
}

int dhara_nand_prog(const dhara_nand* n, dhara_page_t p, const uint8_t* data, dhara_error_t* err)
{
    return 0;
}

int dhara_nand_is_free(const dhara_nand* n, dhara_page_t p)
{
    return 0;
}

int dhara_nand_read(const dhara_nand* n, dhara_page_t p, size_t offset, size_t length, uint8_t* data, dhara_error_t* err)
{
    return 0;
}

int dhara_nand_copy(const dhara_nand* n, dhara_page_t src, dhara_page_t dst, dhara_error_t* err)
{
    return 0;
}
