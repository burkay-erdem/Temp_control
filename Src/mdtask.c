#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "mb.h"
#include "mbport.h"
#include "mbrtu.h"

#include "MAX31865.h"

#define REG_INPUT_START   1000
#define REG_INPUT_NREGS   8
#define REG_HOLDING_START 1000
#define REG_HOLDING_NREGS 130
 eMBErrorCode  eeStatus ;
 eMBErrorCode  eStatus ;
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
void read_temps();
void ModbusRTUTask(void const * argument)
{ 
  /* ABCDEF */
  usRegInputBuf[0] = 11;
  usRegInputBuf[1] = 33;
  usRegInputBuf[2] = 33;
  usRegInputBuf[3] = 44;
  usRegInputBuf[4] = 55;
  usRegInputBuf[5] = 66;
  usRegInputBuf[6] = 77;
  usRegInputBuf[7] = 88;  
	usRegHoldingBuf[1]=0;
 static UCHAR ucRcvAddress, *ucMBFrame, usLength;

 const UCHAR     ucSlaveID[] = { 0x1 };
 eStatus = eMBInit( MB_RTU, 2, 2, 9600, MB_PAR_NONE );
 eeStatus = eMBSetSlaveID( 0x1, TRUE, ucSlaveID, 1 );

 eStatus = eMBEnable();
  while(1) {
		
		*((uint32_t *)&GPIOC->BSRR) = ((uint32_t)GPIO_PIN_4|GPIO_PIN_5) << 16; //reset bit
    eMBPoll();
		
		
usRegHoldingBuf[1]++;
  }
}
void read_temps(){
	MAX31865_init(2);
	while(1){
		
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		usRegInputBuf[1] = (MAX31865_readTemp()-7)*100;
	osDelay(100);
	}
}
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
	usAddress = REG_INPUT_START+usAddress;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
				
				HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    }
    else
    {
			  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
        eStatus = MB_ENOREG;			
    }

    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
usAddress = REG_INPUT_START+usAddress;
    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}
