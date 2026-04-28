#include "stdio.h"
#include "tools.h"

static union {   
    char c[4];   
    unsigned long mylong;
} endian_test = {{ 'l', '?', '?', 'b' } };

uint32_t htonl(uint32_t hl)
{
	return (ENDIANNESS=='l') ? SWAP32(hl): hl;
} 
uint32_t ntohl(uint32_t nl)
{
	return (ENDIANNESS=='l') ? SWAP32(nl): nl;	
}

uint16_t htons(uint16_t hs)
{
    return (ENDIANNESS=='l') ? SWAP16(hs):hs;
}
uint16_t ntohs(uint16_t ns)
{
    return (ENDIANNESS=='l') ? SWAP16(ns):ns;
}


/*
 * Convert Ethernet address to standard hex-digits.
 */
char *
mac_ntoh(unsigned char *addr)
{
    static char buf[6];
	buf[0] = addr[5],
	buf[1] = addr[4],
	buf[2] = addr[3],
	buf[3] = addr[2],
	buf[4] = addr[1],
	buf[5] = addr[0];
    return buf;
}



/****************************Info********************************************** 
 * Name:    CRC-16/XMODEM       x16+x12+x5+1 
 * Width:	16 
 * Poly:    0x1021 
 * Init:    0x0000 
 * Refin:   False 
 * Refout:  False 
 * Xorout:  0x0000 
 * Alias:   CRC-16/ZMODEM,CRC-16/ACORN 
 *****************************************************************************/
uint16_t crc16_xmodem(uint8_t *data, int32_t len) {
    uint16_t CRCin = 0x0000;
	uint16_t CPoly = 0x1021;
	
	while (len--) 	
	{
		CRCin ^= (*(data++) << 8);
		for(int i = 0;i < 8;i++)
		{
			if(CRCin & 0x8000)
				CRCin = (CRCin << 1) ^ CPoly;
			else
				CRCin = CRCin << 1;
		}
	}

    return CRCin;
}



