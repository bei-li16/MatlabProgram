/*
 * File: sim2.c
 *
 * Code generated for Simulink model 'sim2'.
 *
 * Model version                  : 1.16
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Sun Jun 29 22:05:17 2025
 *
 * Target selection: autosar.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "sim2.h"
#include "Platform_Types.h"

/* Named constants for Chart: '<Root>/Chart1' */
#define sim2_IN_D                      ((uint8)1U)
#define sim2_IN_D1                     ((uint8)1U)
#define sim2_IN_D2                     ((uint8)2U)
#define sim2_IN_D3                     ((uint8)3U)
#define sim2_IN_D4                     ((uint8)4U)
#define sim2_IN_N                      ((uint8)2U)
#define sim2_IN_NO_ACTIVE_CHILD        ((uint8)0U)
#define sim2_IN_P                      ((uint8)3U)
#define sim2_IN_R                      ((uint8)4U)

/* PublicStructure Variables for Internal Data */
ARID_DEF_sim2_T sim2_ARID_DEF;         /* '<Root>/Chart1' */

/* Model step function */
void sim2_Step(void)
{
  float32 tmp;
  sint32 tmp_0;

  /* Chart: '<Root>/Chart1' incorporates:
   *  Inport: '<Root>/shift_sw'
   *  Inport: '<Root>/speed'
   */
  if (sim2_ARID_DEF.is_active_c1_sim2 == 0U) {
    sim2_ARID_DEF.is_active_c1_sim2 = 1U;
    sim2_ARID_DEF.is_c1_sim2 = sim2_IN_P;
  } else {
    switch (sim2_ARID_DEF.is_c1_sim2) {
     case sim2_IN_D:
      if (Rte_IRead_sim2_Step_shift_sw_shift_sw() == 2) {
        sim2_ARID_DEF.is_D = sim2_IN_NO_ACTIVE_CHILD;
        sim2_ARID_DEF.is_c1_sim2 = sim2_IN_N;
      } else {
        switch (sim2_ARID_DEF.is_D) {
         case sim2_IN_D1:
          if (Rte_IRead_sim2_Step_speed_speed() > 20.0F) {
            sim2_ARID_DEF.is_D = sim2_IN_D2;
          }
          break;

         case sim2_IN_D2:
          tmp = Rte_IRead_sim2_Step_speed_speed();
          if (tmp > 40.0F) {
            sim2_ARID_DEF.is_D = sim2_IN_D3;
          } else if (tmp < 15.0F) {
            sim2_ARID_DEF.is_D = sim2_IN_D1;
          }
          break;

         case sim2_IN_D3:
          tmp = Rte_IRead_sim2_Step_speed_speed();
          if (tmp > 60.0F) {
            sim2_ARID_DEF.is_D = sim2_IN_D4;
          } else if (tmp < 35.0F) {
            sim2_ARID_DEF.is_D = sim2_IN_D2;
          }
          break;

         default:
          /* case IN_D4: */
          if (Rte_IRead_sim2_Step_speed_speed() < 55.0F) {
            sim2_ARID_DEF.is_D = sim2_IN_D3;
          }
          break;
        }
      }
      break;

     case sim2_IN_N:
      tmp_0 = Rte_IRead_sim2_Step_shift_sw_shift_sw();
      if (tmp_0 == 3) {
        sim2_ARID_DEF.is_c1_sim2 = sim2_IN_D;
        sim2_ARID_DEF.is_D = sim2_IN_D1;
      } else if (tmp_0 == 1) {
        sim2_ARID_DEF.is_c1_sim2 = sim2_IN_R;
      }
      break;

     case sim2_IN_P:
      if (Rte_IRead_sim2_Step_shift_sw_shift_sw() == 1) {
        sim2_ARID_DEF.is_c1_sim2 = sim2_IN_R;
      }
      break;

     default:
      /* case IN_R: */
      tmp_0 = Rte_IRead_sim2_Step_shift_sw_shift_sw();
      if (tmp_0 == 2) {
        sim2_ARID_DEF.is_c1_sim2 = sim2_IN_N;
      } else if (tmp_0 == 0) {
        sim2_ARID_DEF.is_c1_sim2 = sim2_IN_P;
      }
      break;
    }
  }

  /* End of Chart: '<Root>/Chart1' */
}

/* Model initialize function */
void sim2_Init(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
