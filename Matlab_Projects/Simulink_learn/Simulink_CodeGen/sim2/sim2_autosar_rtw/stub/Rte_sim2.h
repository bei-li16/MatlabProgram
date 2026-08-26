/* This file contains stub implementations of the AUTOSAR RTE functions.
   The stub implementations can be used for testing the generated code in
   Simulink, for example, in SIL/PIL simulations of the component under
   test. Note that this file should be replaced with an appropriate RTE
   file when deploying the generated code outside of Simulink.

   This file is generated for:
   Atomic software component:  "sim2"
   ARXML schema: "R22-11"
   File generated on: "Sun Jun 29 22:05:23 2025"  */

#ifndef Rte_sim2_h
#define Rte_sim2_h
#include "Rte_Type.h"
#include "Compiler.h"

/* Data access functions */
#define Rte_IRead_sim2_Step_shift_sw_shift_sw Rte_IRead_sim2_sim2_Step_shift_sw_shift_sw

uint8 Rte_IRead_sim2_Step_shift_sw_shift_sw(void);

#define Rte_IRead_sim2_Step_speed_speed Rte_IRead_sim2_sim2_Step_speed_speed

float32 Rte_IRead_sim2_Step_speed_speed(void);

/* Entry point functions */
extern FUNC(void, sim2_CODE) sim2_Init(void);
extern FUNC(void, sim2_CODE) sim2_Step(void);

#endif
