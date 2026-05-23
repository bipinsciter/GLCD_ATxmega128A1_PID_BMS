
#ifndef __OS_API_H__
#define __OS_API_H__

//#define OS_AVRX      1
//#define OS_FREERTOS    1

char * strtrim( char * str );

#ifdef OS_AVRX
#include "avrx/Avrx.h"


void AvrXInit();
unsigned char AvrXReadEEProm( unsigned char * address );
unsigned int AvrXReadEEPromWord( unsigned int * address );
void AvrXWriteEEPromWord(unsigned int * address, unsigned int value);

#define OSInit                   AvrXInit
#define OSStart                  Epilog
#define OSSleep(time)            { TimerControlBlock   DeviceIOTimer;  AvrXDelay(&DeviceIOTimer, time);}
#define OSSemaMutex              Mutex
#define OSSemaBinary             Mutex
#define OSSemaCreateMutex( sema )
#define OSSemaCreateBinary( sema )
#define OSSemaGive( sema )       AvrXSetSemaphore( &sema )
#define OSSemaTakeEver( sema )   AvrXWaitSemaphore( &sema )
#define OSReadEEPromByte         AvrXReadEEProm
#define OSReadEEPromWord         AvrXReadEEPromWord
#define OSWriteEEPromWord        AvrXWriteEEPromWord

#else

#include "FreeRTOS.h"
#include "task.h"
#include "portable.h"
#include "semphr.h"
#include "queue.h"


void FreeRTOSInit();
unsigned char FreeRTOSReadEEPromByte( unsigned char * address );
unsigned int FreeRTOSReadEEPromWord( unsigned int * address );
void FreeRTOSWriteEEPromWord(unsigned int * address, unsigned int value);
void FreeRTOSReadEEPromBlock( uint8_t * address, uint8_t *value, uint8_t size);
void FreeRTOSWriteEEPromBlock(uint8_t * address, uint8_t *value, uint8_t size);

#define OSInit                      FreeRTOSInit
#define OSStart                     vTaskStartScheduler
#define OSSleep(time)               vTaskDelay( time )//vTaskDelay( pdMS_TO_TICKS(time) )
#define OSSemaMutex                 xSemaphoreHandle
#define OSSemaBinary                xSemaphoreHandle
#define OSSemaCreateMutex( sema )   {sema = xSemaphoreCreateMutex();}
#define OSSemaCreateBinary( sema )  vSemaphoreCreateBinary( sema )
#define OSSemaGive(sema)            xSemaphoreGive(sema)
#define OSSemaTakeEver(sema)        xSemaphoreTake(sema, portMAX_DELAY)
#define OSReadEEPromByte            FreeRTOSReadEEPromByte
#define OSReadEEPromWord            FreeRTOSReadEEPromWord
#define OSWriteEEPromWord			FreeRTOSWriteEEPromWord
#define OSReadEEPromBlock           FreeRTOSReadEEPromBlock
#define OSWriteEEPromBlock			FreeRTOSWriteEEPromBlock

#endif /* OS_AVRX */

#endif /* __OS_API_H__ */
