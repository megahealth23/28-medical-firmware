#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>
/*溢出标志：0-正常，-1-溢出*/  
#define FLAGS_OVERRUN		0x0001  

typedef struct{  
	int *buf;  
	int putP;// 下一个数据写入位置
	int size;// 大小
	int flags;  
} FIFO32;  

typedef struct{

    unsigned short *buf;
	int putP;
	int size;
	int flags;
}FIFO16;

// global functions declaration 
extern void fifo32_init(FIFO32 *fifo,int size, int *buf);  
extern int fifo32_putPut(FIFO32 *fifo,int data);  
extern int fifo32_recover(FIFO32 *fifo, int num);
extern int fifo32_status(FIFO32 *fifo);


extern void fifo16_init(FIFO16 *fifo,int size, unsigned short *buf);  
extern int fifo16_putPut(FIFO16 *fifo, unsigned short data);  
extern int fifo16_recover(FIFO16 *fifo, int num);
extern int fifo16_status(FIFO16 *fifo);


#endif /*FIFO8_H_*/
