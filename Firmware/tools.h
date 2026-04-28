#ifndef TOOLS_H
#define TOOLS_H

#include <stdint.h>

				   
#define SWAP16(x) (uint16_t) ( ((((uint16_t)(x) & 0x00ff)) << 8) |(((uint16_t)(x) & 0xff00) >> 8) )


#define SWAP32(A) ((((uint32_t)(A) & 0xff000000) >> 24) | \
				   (((uint32_t)(A) & 0x00ff0000) >>  8) | \
				   (((uint32_t)(A) & 0x0000ff00) <<  8) | \
				   (((uint32_t)(A) & 0x000000ff) << 24))
#define ENDIANNESS ((char)endian_test.mylong)

uint32_t htonl(uint32_t );
uint32_t ntohl(uint32_t );

uint16_t htons(uint16_t hs);
uint16_t ntohs(uint16_t ns);

char * mac_ntoh(unsigned char *addr);
uint16_t crc16_xmodem(uint8_t *data, int32_t len);

#endif
