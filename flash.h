#ifndef __FLASH_H
#define __FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

void flash_read(uint32_t *dst, uint32_t size);
void flash_write(uint32_t *src, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif
