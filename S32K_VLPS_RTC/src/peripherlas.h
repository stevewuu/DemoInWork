/*******************************************************************************
* NXP Semiconductors
* (c) Copyright 2018 NXP Semiconductors
* ALL RIGHTS RESERVED.
********************************************************************************
Services performed by NXP in this matter are performed AS IS and without any
warranty. CUSTOMER retains the final decision relative to the total design
and functionality of the end product. NXP neither guarantees nor will be held
liable by CUSTOMER for the success of this project.
NXP DISCLAIMS ALL WARRANTIES, EXPRESSED, IMPLIED OR STATUTORY INCLUDING,
BUT NOT LIMITED TO, IMPLIED WARRANTY OF MERCHANTABILITY OR FITNESS FOR
A PARTICULAR PURPOSE ON ANY HARDWARE, SOFTWARE ORE ADVISE SUPPLIED
TO THE PROJECT BY NXP, AND OR NAY PRODUCT RESULTING FROM NXP SERVICES.
IN NO EVENT SHALL NXP BE LIABLE FOR INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING
OUT OF THIS AGREEMENT.
CUSTOMER agrees to hold NXP harmless against any and all claims demands
or actions by anyone on account of any damage, or injury, whether commercial,
contractual, or tortuous, rising directly or indirectly as a result
of the advise or assistance supplied CUSTOMER in connection with product,
services or goods supplied under this Agreement.
********************************************************************************
* File:             peripherals.h
* Owner:            Daniel Martynek
* Version:          1.0
* Date:             Sep-28-2018
* Classification:   General Business Information
* Brief:            RTC in VLPS
********************************************************************************/

#ifndef PERIPHERLAS_H_
#define PERIPHERLAS_H_

void init_RTC(void);
void init_PORT(void);
void clkout_bus(void);
void init_SIRC(void);
void init_VLPR(void);
void enter_VLPR(void);
void enter_VLPS(void);
void SlowRUN_to_VLPS(void);
void switch_to_SIRC_in_RUN(void);
void disable_FIRC_in_RUN(void);
void init_NVIC(void);

#endif /* PERIPHERLAS_H_ */
