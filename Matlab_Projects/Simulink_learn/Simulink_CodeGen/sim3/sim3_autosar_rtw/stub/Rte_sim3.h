/* This file contains stub implementations of the AUTOSAR RTE functions.
   The stub implementations can be used for testing the generated code in
   Simulink, for example, in SIL/PIL simulations of the component under
   test. Note that this file should be replaced with an appropriate RTE
   file when deploying the generated code outside of Simulink.

   This file is generated for:
   Atomic software component:  "sim3"
   ARXML schema: "R22-11"
   File generated on: "Thu Jan 29 23:45:10 2026"  */

#ifndef Rte_sim3_h
#define Rte_sim3_h
#include "Rte_Type.h"
#include "Compiler.h"

/* Data access functions */
#define Rte_IRead_sim3_Step_In1_In1    Rte_IRead_sim3_sim3_Step_In1_In1

uint8 Rte_IRead_sim3_Step_In1_In1(void);

#define Rte_IRead_sim3_Step_In2_In2    Rte_IRead_sim3_sim3_Step_In2_In2

uint8 Rte_IRead_sim3_Step_In2_In2(void);

#define Rte_IWrite_sim3_Step_zout_zout Rte_IWrite_sim3_sim3_Step_zout_zout

void Rte_IWrite_sim3_Step_zout_zout(UFIX16_EN6 u);

#define Rte_IWriteRef_sim3_Step_zout_zout Rte_IWriteRef_sim3_sim3_Step_zout_zout

UFIX16_EN6* Rte_IWriteRef_sim3_Step_zout_zout(void);

/* Entry point functions */
extern FUNC(void, sim3_CODE) sim3_Init(void);
extern FUNC(void, sim3_CODE) sim3_Step(void);

#endif
