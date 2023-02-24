#include "main.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include "appUSART.h"
#include "appLSM9DS1.h"
#include "appMotionFX.h"
#include "console.h"
#include "memsTask.h"

#define ALGO_FREQ  100U /* Algorithm frequency 100Hz */
#define ACC_ODR  ((float)ALGO_FREQ)
#define ACC_FS  4 /* FS = <-4g, 4g> */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
#define MOTION_FX_ENGINE_DELTATIME  0.01f
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f
#define FROM_MGAUSS_TO_UT50  (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS  500.0f

extern osTimerId_t taskTimerHandle;

static void acquireMEMS(void);
static void runFusion(void);
static void transmitFusionResults(void);
static void transmitIMUReadings(void);
static void transmitAllData(void);

static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];

static char outBuffer[1024];

extern osThreadId_t memsTaskHandle;

uint32_t AlgoFreq = ALGO_FREQ;

MFX_input_t data_in;
MFX_input_t *pdata_in = &data_in;
MFX_output_t data_out;
MFX_output_t *pdata_out = &data_out;

static void acquireMEMS(void)
{
  GetIMUReading(acceleration_mg, angular_rate_mdps, magnetic_field_mgauss);
}

static void runFusion(void)
{
  /* Convert angular velocity from [mdps] to [dps] */
  data_in.gyro[0] = (float)angular_rate_mdps[0] * FROM_MDPS_TO_DPS;
  data_in.gyro[1] = (float)angular_rate_mdps[1] * FROM_MDPS_TO_DPS;
  data_in.gyro[2] = (float)angular_rate_mdps[2] * FROM_MDPS_TO_DPS;

  /* Convert acceleration from [mg] to [g] */
  data_in.acc[0] = (float)acceleration_mg[0] * FROM_MG_TO_G;
  data_in.acc[1] = (float)acceleration_mg[1] * FROM_MG_TO_G;
  data_in.acc[2] = (float)acceleration_mg[2] * FROM_MG_TO_G;

  /* Convert magnetic field intensity from [mGauss] to [uT / 50] */
  data_in.mag[0] = (float)magnetic_field_mgauss[0] * FROM_MGAUSS_TO_UT50;
  data_in.mag[1] = (float)magnetic_field_mgauss[1] * FROM_MGAUSS_TO_UT50;
  data_in.mag[2] = (float)magnetic_field_mgauss[2] * FROM_MGAUSS_TO_UT50;

  motionFXRun(pdata_in, pdata_out, MOTION_FX_ENGINE_DELTATIME);
}

static void transmitAllData(void)
{
  sprintf((char *)outBuffer,
	          "%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\r\n",
	          acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
	          angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
	          magnetic_field_mgauss[0], magnetic_field_mgauss[1], magnetic_field_mgauss[2],
		         pdata_out->quaternion[0], pdata_out->quaternion[1], pdata_out->quaternion[2],pdata_out->quaternion[3],
		         pdata_out->rotation[0], pdata_out->rotation[1], pdata_out->rotation[2],
		         pdata_out->gravity[0], pdata_out->gravity[1], pdata_out->gravity[2],
		         pdata_out->linear_acceleration[0], pdata_out->linear_acceleration[1], pdata_out->linear_acceleration[2],
				 pdata_out->heading, pdata_out->headingErr);
  USART1TxStr(outBuffer);
}

static void transmitIMUReadings(void)
{
  sprintf((char *)outBuffer,
	          "%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\r\n",
	          acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
	          angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
	          magnetic_field_mgauss[0], magnetic_field_mgauss[1], magnetic_field_mgauss[2]);
  USART1TxStr(outBuffer);
}


static void transmitFusionResults(void)
{
  sprintf((char *)outBuffer,
         "%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\r\n",
         pdata_out->quaternion[0], pdata_out->quaternion[1], pdata_out->quaternion[2],pdata_out->quaternion[3],
         pdata_out->rotation[0], pdata_out->rotation[1], pdata_out->rotation[2],
         pdata_out->gravity[0], pdata_out->gravity[1], pdata_out->gravity[2],
         pdata_out->linear_acceleration[0], pdata_out->linear_acceleration[1], pdata_out->linear_acceleration[2],
		 pdata_out->heading, pdata_out->headingErr);
  USART1TxStr(outBuffer);
}

void MEMSTask(void *argument)
{
  (void) argument;

  IMUInit();
  motionFXInit();
  motionFXStart();
  osDelay(1000);
  osTimerStart(taskTimerHandle, 25);
  for(;;)
  {
    osThreadSuspend(memsTaskHandle);
    acquireMEMS();
    runFusion();
    if(isStreamActive())
    {
#if 0
      if (isFusionSet())
      {
        transmitFusionResults();
      }
      else
      {
        transmitIMUReadings();
      }
#else
      transmitAllData();
#endif
    }
  }
}
