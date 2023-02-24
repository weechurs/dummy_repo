#pragma once

#include "motion_fx.h"
#include "main.h"

void motionFXInit(void);
void motionFXRun(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time);
void motionFXStart(void);
