/* This file contains stub implementations of the AUTOSAR RTE functions.
   The stub implementations can be used for testing the generated code in
   Simulink, for example, in SIL/PIL simulations of the component under
   test. Note that this file should be replaced with an appropriate RTE
   file when deploying the generated code outside of Simulink.

   This file is generated for:
   Atomic software component:  "sim1"
   ARXML schema: "R22-11"
   File generated on: "Thu Jan 22 20:38:10 2026"  */

#ifndef Rte_sim1_h
#define Rte_sim1_h
#include "Rte_Type.h"
#include "Compiler.h"

/* Data access functions */
#define Rte_IRead_sim1_Step_In1_In1    Rte_IRead_sim1_sim1_Step_In1_In1

float64 Rte_IRead_sim1_Step_In1_In1(void);

#define Rte_IWrite_sim1_Step_Out1_Out1 Rte_IWrite_sim1_sim1_Step_Out1_Out1

void Rte_IWrite_sim1_Step_Out1_Out1(float64 u);

#define Rte_IWriteRef_sim1_Step_Out1_Out1 Rte_IWriteRef_sim1_sim1_Step_Out1_Out1

float64* Rte_IWriteRef_sim1_Step_Out1_Out1(void);

#define Rte_IRead_sim1_Step_In2_In2    Rte_IRead_sim1_sim1_Step_In2_In2

float64 Rte_IRead_sim1_Step_In2_In2(void);

#define Rte_IRead_sim1_Step_In3_In3    Rte_IRead_sim1_sim1_Step_In3_In3

float64 Rte_IRead_sim1_Step_In3_In3(void);

#define Rte_IWrite_sim1_Step_zout_zout Rte_IWrite_sim1_sim1_Step_zout_zout

void Rte_IWrite_sim1_Step_zout_zout(float64 u);

#define Rte_IWriteRef_sim1_Step_zout_zout Rte_IWriteRef_sim1_sim1_Step_zout_zout

float64* Rte_IWriteRef_sim1_Step_zout_zout(void);

/* Entry point functions */
extern FUNC(void, sim1_CODE) sim1_Init(void);
extern FUNC(void, sim1_CODE) sim1_Step(void);

#endif
