
#include "fifo.h"

/*********************************************************************
 * @fn      fifo8_init()
 *
 * @brief   fifo init
 *
 * @param   FIFO8 *fifo, int size, int *buf
 *
 * @return  none..
 */
void fifo32_init(FIFO32 *fifo, int size, int *buf)
{
	fifo->buf = buf;
	fifo->flags = 0;          
	fifo->size = size;
	fifo->putP = 0;                  

	return;
}

/*********************************************************************
 * @fn      fifo8_putPut()
 *
 * @brief   向FIFO 中写入数据
 *
 * @param   FIFO8 *fifo, int data.
 *
 * @return  status..
 */
int fifo32_putPut(FIFO32 *fifo, int data)
{
	if (fifo->putP >= fifo->size){
		fifo->flags |= FLAGS_OVERRUN;
		return -1;
	}
	fifo->buf[fifo->putP] = data;
	fifo->putP++;
	
	if(fifo->putP >= fifo->size){
		fifo->putP = fifo->size;
	}
	
	return 0;
}

int fifo32_status(FIFO32 *fifo)
{
    return fifo->putP;
}

/*********************************************************************
 * @fn      fifo8_recover()
 *
 * @brief   FIFO 指针归位 
 *
 * @param   FIFO8 *fifo
 *
 * @return  status..
 */
int fifo32_recover(FIFO32 *fifo,int num)
{
    if (num < 0 || num >= fifo->putP)
    {
        return -1;
    }
    int moveofflen = (fifo->putP - num);
    for(unsigned int i=0;i<num;i++)
    {
        fifo->buf[i] = fifo->buf[i+moveofflen];
    }
    fifo->putP = num;
    return 0;

}


void fifo16_init(FIFO16 *fifo, int size, uint16_t *buf)
{
	fifo->buf = buf;
	fifo->flags = 0;          
	fifo->size = size;
	fifo->putP = 0;                                   

	return;
}

/*********************************************************************
 * @fn      fifo8_putPut()
 *
 * @brief   向FIFO 中写入数据
 *
 * @param   FIFO8 *fifo, int data.
 *
 * @return  status..
 */
int fifo16_putPut(FIFO16 *fifo, uint16_t data)
{
	if (fifo->putP >= fifo->size){
		fifo->flags |= FLAGS_OVERRUN;
		return -1;
	}
	fifo->buf[fifo->putP] = data;
	fifo->putP++;
	
	if(fifo->putP >= fifo->size){
		fifo->putP = fifo->size;
	}
	
	return 0;
}



/*********************************************************************
 * @fn      fifo8_recover()
 *
 * @brief   FIFO 指针归位 
 *
 * @param   FIFO8 *fifo
 *
 * @return  status..
 */
int fifo16_recover(FIFO16 *fifo,int num)
{
    if (num < 0 || num >= fifo->putP)
    {
        return -1;
    }
    int moveofflen = (fifo->putP - num);
    for(unsigned int i=0;i<num;i++)
    {
        fifo->buf[i] = fifo->buf[i+moveofflen];
    }
    fifo->putP = num;
    return 0;

}

int fifo16_status(FIFO16 *fifo)
{
    return fifo->putP;
}


