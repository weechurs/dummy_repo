#include "main.h"
#include "cmsis_os2.h"
#include "freertos.h"
#include <stdio.h>
#include <string.h>
#include "appUSART.h"
#include "appLSM9DS1.h"
#include "memsTask.h"
#include "timers.h"
#include "console.h"

extern osThreadId_t memsTaskHandle;
extern osTimerId_t taskTimerHandle;

#define FUSION_HEADER_STR "Quaternion0,Quaternion1,Quaternion2,Quaternion3,Rotationn0,Rotation1,Rotation2,Gravity0,Gravity1,Gravity2,LinearAcceleration0,LinearAcceleration1,LinearAcceleration2,Heading,HeadingError"
#define RAW_HEADER_STR "AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ"
#define COMBINED_HEADER_STR "AccelX(mg),AccelY,AccelZ,GyroX(mdps),GyroY,GyroZ,MagX(mG),MagY,MagZ,Quaternion0,Quaternion1,Quaternion2,Quaternion3,Rotationn0,Rotation1,Rotation2,Gravity0,Gravity1,Gravity2,LinearAcceleration0,LinearAcceleration1,LinearAcceleration2,Heading,HeadingError"

static char lineBuffer[1024];
static char outBuffer[1024];

static uint16_t lineIndex = 0;
static uint8_t streamActiveFlag = 0;
static uint8_t fusionSet = 0;

static void processLine(void);

uint8_t isFusionSet(void)
{
  return fusionSet;
}

uint8_t isStreamActive(void)
{
  return streamActiveFlag;
}

static void processLine(void)
{
  switch(lineBuffer[0])
  {
    case 'B':
    	if(streamActiveFlag==0)
    	{
    	  streamActiveFlag=1;
    	  sprintf(outBuffer, "%s\r\n", COMBINED_HEADER_STR); //(fusionSet==1)?(FUSION_HEADER_STR):(RAW_HEADER_STR));
    	  USART1TxStr(outBuffer);
    	}
	    break;
    case 'E':
    	if(streamActiveFlag==1)
    	{
      	  streamActiveFlag=0;
//      	  osTimerStop(taskTimerHandle);
    	}
    	break;
#if 0
    case 'F':
    	if(streamActiveFlag==0)
    	{
    	  fusionSet=(fusionSet==1)?0:1;
    	  sprintf(outBuffer, "Fusion is %s\r\n",(fusionSet==1)?("set"):("not set"));
    	  USART1TxStr(outBuffer);
    	}
    	break;
#endif
    case 'V':
    	if(streamActiveFlag==0)
    	{
    	  sprintf(outBuffer, "Version: %s\r\n", VERSION_STR);
    	  USART1TxStr(outBuffer);
    	}
    	break;
    case '?':
    	if(streamActiveFlag==0)
    	{
          USART1TxStr("B - Begin streaming\r\nE - End streaming\r\nV - Version\r\n? - This menu\r\n");
    	}
    default:
    	break;
  }
}

void ConsoleTask(void *argument)
{
  (void) argument;

  uint8_t data;

  for(;;)
  {
	USART1RxDataWait();
    while(USART1Rx(&data))
    {
      if((data=='\n')||(data=='\r'))
      {
    	lineBuffer[lineIndex]='\0';
        processLine();
        lineIndex=0;
      }
      else
      {
        lineBuffer[lineIndex]=data;
        lineIndex++;
      }
    }
  }
}
