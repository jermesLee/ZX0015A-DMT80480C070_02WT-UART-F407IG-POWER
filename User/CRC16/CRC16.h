#ifndef _CRC16_H_
#define	_CRC16_H_

#include "includes.h"


extern void CRC16_CheckCompute(uint8_t* DataHeader, uint16_t DataLen,uint8_t* ReturnData);
extern ErrorStatus CRC16_Check(uint8_t* DataHeader1,uint8_t* DataHeader2);

#endif
