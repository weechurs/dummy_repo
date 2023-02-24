#include "main.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include "appUSART.h"
#include "appLSM9DS1.h"
#include "appMotionFX.h"

#define STATE_SIZE                      (size_t)(2432)

#define SAMPLETODISCARD                 15

#define GBIAS_ACC_TH_SC                 (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC                (2.0f*0.002f)
#define GBIAS_MAG_TH_SC                 (2.0f*0.001500f)

#define DECIMATION                      1U

static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;

static volatile int sampleToDiscard = SAMPLETODISCARD;
static int discardedCount = 0;

uint8_t mfxstate[STATE_SIZE];

void motionFXInit(void)
{
  if (STATE_SIZE < MotionFX_GetStateSize())
    Error_Handler();

  MotionFX_initialize((MFXState_t *)mfxstate);

  MotionFX_getKnobs(mfxstate, ipKnobs);

  //BSP_SENSOR_ACC_GetOrientation(ipKnobs->acc_orientation);
  //BSP_SENSOR_GYR_GetOrientation(ipKnobs->gyro_orientation);
  //BSP_SENSOR_MAG_GetOrientation(ipKnobs->mag_orientation);

  ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC;
  ipKnobs->gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;
  ipKnobs->gbias_mag_th_sc = GBIAS_MAG_TH_SC;

  ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
  ipKnobs->LMode = 1;
  ipKnobs->modx = DECIMATION;

  MotionFX_setKnobs(mfxstate, ipKnobs);

  MotionFX_enable_6X(mfxstate, MFX_ENGINE_DISABLE);
  MotionFX_enable_9X(mfxstate, MFX_ENGINE_DISABLE);
}

void motionFXRun(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time)
{
  if (discardedCount == sampleToDiscard)
  {
    MotionFX_propagate(mfxstate, data_out, data_in, &delta_time);
	MotionFX_update(mfxstate, data_out, data_in, &delta_time, NULL);
  }
  else
  {
    discardedCount++;
  }
}

void motionFXStart(void)
{
  MotionFX_enable_9X(mfxstate, MFX_ENGINE_ENABLE);
}
