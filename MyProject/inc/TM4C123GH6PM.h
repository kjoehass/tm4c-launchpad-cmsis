/**
 * @file inc/TM4C123GH6PM.h
 *
 * @brief CMSIS Cortex-M4 Core Peripheral Access Layer Header File for
 *        Texas Instruments TM4C123GH6PM
 *
 * @author K. Joseph Hass
 * @date Created: 2014-11-11T08:51:35-0500
 * @date Last modified: 2014-11-11T09:05:50-0500
 *
 * @copyright Copyright (C) 2014 Kenneth Joseph Hass
 *
 * @copyright This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * @copyright This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 */

/** @addtogroup TI
  * @{
  */

/** @addtogroup TM4C123GH6PM
  * @{
  */

#ifndef TM4C123GH6PM_H
#define TM4C123GH6PM_H

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------  Interrupt Number Definition  ------------------- */

typedef enum {
/* --------------  Cortex-M4 Processor Exceptions Numbers  -------------- */
  Reset_IRQn            = -15, /*!< Reset Vector, Power up and warm reset */
  NonMaskableInt_IRQn   = -14, /*!< Non maskable Interrupt                */
  HardFault_IRQn        = -13, /*!< Hard Fault                            */
  MemoryManagement_IRQn = -12, /*!< Memory Management, MPU mismatch,
                                    including Access Violation & No Match */
  BusFault_IRQn         = -11, /*!< Bus Fault, Pre-Fetch-, Memory Access
                                    Fault, other address/memory fault     */
  UsageFault_IRQn       = -10, /*!< Usage Fault                           */
  SVCall_IRQn           =  -5, /*!< System Service Call via SVC inst      */
  DebugMonitor_IRQn     =  -4, /*!< Debug Monitor                         */
  PendSV_IRQn           =  -2, /*!< Pendable request for system service   */
  SysTick_IRQn          =  -1, /*!< System Tick Timer                     */
/* --------------  TM4C123GH6PM Specific Interrupt Numbers  ------------- */
  GPIOA_IRQn            =   0, /*!<  GPIOA                                */
  GPIOB_IRQn            =   1, /*!<  GPIOB                                */
  GPIOC_IRQn            =   2, /*!<  GPIOC                                */
  GPIOD_IRQn            =   3, /*!<  GPIOD                                */
  GPIOE_IRQn            =   4, /*!<  GPIOE                                */
  UART0_IRQn            =   5, /*!<  UART0                                */
  UART1_IRQn            =   6, /*!<  UART1                                */
  SSI0_IRQn             =   7, /*!<  SSI0                                 */
  I2C0_IRQn             =   8, /*!<  I2C0                                 */
  PWM0_FAULT_IRQn       =   9, /*!<  PWM0_FAULT                           */
  PWM0_0_IRQn           =  10, /*!<  PWM0_0                               */
  PWM0_1_IRQn           =  11, /*!<  PWM0_1                               */
  PWM0_2_IRQn           =  12, /*!<  PWM0_2                               */
  QEI0_IRQn             =  13, /*!<  QEI0                                 */
  ADC0SS0_IRQn          =  14, /*!<  ADC0SS0                              */
  ADC0SS1_IRQn          =  15, /*!<  ADC0SS1                              */
  ADC0SS2_IRQn          =  16, /*!<  ADC0SS2                              */
  ADC0SS3_IRQn          =  17, /*!<  ADC0SS3                              */
  WATCHDOG_IRQn         =  18, /*!<  WATCHDOG0 and WATCHDOG1              */
  TIMER0A_IRQn          =  19, /*!<  TIMER0A                              */
  TIMER0B_IRQn          =  20, /*!<  TIMER0B                              */
  TIMER1A_IRQn          =  21, /*!<  TIMER1A                              */
  TIMER1B_IRQn          =  22, /*!<  TIMER1B                              */
  TIMER2A_IRQn          =  23, /*!<  TIMER2A                              */
  TIMER2B_IRQn          =  24, /*!<  TIMER2B                              */
  COMP0_IRQn            =  25, /*!<  COMP0                                */
  COMP1_IRQn            =  26, /*!<  COMP1                                */
  SYSCTL_IRQn           =  28, /*!<  SYSCTL                               */
  FLASH_CTRL_IRQn       =  29, /*!<  FLASH_CTRL                           */
  GPIOF_IRQn            =  30, /*!<  GPIOF                                */
  UART2_IRQn            =  33, /*!<  UART2                                */
  SSI1_IRQn             =  34, /*!<  SSI1                                 */
  TIMER3A_IRQn          =  35, /*!<  TIMER3A                              */
  TIMER3B_IRQn          =  36, /*!<  TIMER3B                              */
  I2C1_IRQn             =  37, /*!<  I2C1                                 */
  QEI1_IRQn             =  38, /*!<  QEI1                                 */
  CAN0_IRQn             =  39, /*!<  CAN0                                 */
  CAN1_IRQn             =  40, /*!<  CAN1                                 */
  HIB_IRQn              =  43, /*!<  HIB                                  */
  USB0_IRQn             =  44, /*!<  USB0                                 */
  PWM0_3_IRQn           =  45, /*!<  PWM0_3                               */
  UDMA_IRQn             =  46, /*!<  UDMA                                 */
  UDMAERR_IRQn          =  47, /*!<  UDMAERR                              */
  ADC1SS0_IRQn          =  48, /*!<  ADC1SS0                              */
  ADC1SS1_IRQn          =  49, /*!<  ADC1SS1                              */
  ADC1SS2_IRQn          =  50, /*!<  ADC1SS2                              */
  ADC1SS3_IRQn          =  51, /*!<  ADC1SS3                              */
  SSI2_IRQn             =  57, /*!<  SSI2                                 */
  SSI3_IRQn             =  58, /*!<  SSI3                                 */
  UART3_IRQn            =  59, /*!<  UART3                                */
  UART4_IRQn            =  60, /*!<  UART4                                */
  UART5_IRQn            =  61, /*!<  UART5                                */
  UART6_IRQn            =  62, /*!<  UART6                                */
  UART7_IRQn            =  63, /*!<  UART7                                */
  I2C2_IRQn             =  68, /*!<  I2C2                                 */
  I2C3_IRQn             =  69, /*!<  I2C3                                 */
  TIMER4A_IRQn          =  70, /*!<  TIMER4A                              */
  TIMER4B_IRQn          =  71, /*!<  TIMER4B                              */
  TIMER5A_IRQn          =  92, /*!<  TIMER5A                              */
  TIMER5B_IRQn          =  93, /*!<  TIMER5B                              */
  WTIMER0A_IRQn         =  94, /*!<  WTIMER0A                             */
  WTIMER0B_IRQn         =  95, /*!<  WTIMER0B                             */
  WTIMER1A_IRQn         =  96, /*!<  WTIMER1A                             */
  WTIMER1B_IRQn         =  97, /*!<  WTIMER1B                             */
  WTIMER2A_IRQn         =  98, /*!<  WTIMER2A                             */
  WTIMER2B_IRQn         =  99, /*!<  WTIMER2B                             */
  WTIMER3A_IRQn         = 100, /*!<  WTIMER3A                             */
  WTIMER3B_IRQn         = 101, /*!<  WTIMER3B                             */
  WTIMER4A_IRQn         = 102, /*!<  WTIMER4A                             */
  WTIMER4B_IRQn         = 103, /*!<  WTIMER4B                             */
  WTIMER5A_IRQn         = 104, /*!<  WTIMER5A                             */
  WTIMER5B_IRQn         = 105, /*!<  WTIMER5B                             */
  SYSEXC_IRQn           = 106, /*!<  SYSEXC                               */
  PWM1_0_IRQn           = 134, /*!<  PWM1_0                               */
  PWM1_1_IRQn           = 135, /*!<  PWM1_1                               */
  PWM1_2_IRQn           = 136, /*!<  PWM1_2                               */
  PWM1_3_IRQn           = 137, /*!<  PWM1_3                               */
  PWM1_FAULT_IRQn       = 138  /*!<  PWM1_FAULT                           */
} IRQn_Type;

/** @addtogroup Configuration_of_CMSIS
  * @{
  */

/* ====================================================================== */
/* ===========      Processor and Core Peripheral Section     =========== */
/* ====================================================================== */

#define __CM4_REV          0x0102 /*!< Cortex-M4 Core Revision            */
#define __MPU_PRESENT           1 /*!< MPU present or not                 */
#define __NVIC_PRIO_BITS        3 /*!< Number of Bits for Priority Levels */
#define __Vendor_SysTickConfig  0 /*!< 1 if different SysTick Cfg is used */
#define __FPU_PRESENT           1 /*!< FPU present or not                 */

/** @} */ /* End of group Configuration_of_CMSIS */

#include <core_cm4.h>            /*!< Cortex-M4 processor and core        */
#include "system_TM4C.h"         /*!< TM4C123GH6PM System                 */

/* ====================================================================== */
/* ===========       Device Specific Peripheral Section       =========== */
/* ====================================================================== */

/** @addtogroup Device_Peripheral_Registers
  * @{
  */

/**
  * @brief Register map for WATCHDOG peripheral (WATCHDOG)
  */

typedef struct {                 /*!< WATCHDOG0 Structure                 */
  __IO uint32_t LOAD;            /*!< Watchdog Load                       */
  __I  uint32_t VALUE;           /*!< Watchdog Value                      */
  __IO uint32_t CTL;             /*!< Watchdog Control                    */
  __O  uint32_t ICR;             /*!< Watchdog Interrupt Clear            */
  __I  uint32_t RIS;             /*!< Watchdog Raw Interrupt Status       */
  __I  uint32_t MIS;             /*!< Watchdog Masked Interrupt Status    */
  __I  uint32_t RSRVD0[(0x418-0x014)/4 - 1];
  __IO uint32_t TEST;            /*!< Watchdog Test                       */
  __I  uint32_t RSRVD1[(0xC00-0x418)/4 - 1];
  __IO uint32_t LOCK;            /*!< Watchdog Lock                       */
  __I  uint32_t RSRVD2[(0xFD0-0xC00)/4 - 1];
  __I  uint32_t PID4;            /*!< Peripheral Identification 4         */
  __I  uint32_t PID5;            /*!< Peripheral Identification 5         */
  __I  uint32_t PID6;            /*!< Peripheral Identification 6         */
  __I  uint32_t PID7;            /*!< Peripheral Identification 7         */
  __I  uint32_t PID0;            /*!< Peripheral Identification 0         */
  __I  uint32_t PID1;            /*!< Peripheral Identification 1         */
  __I  uint32_t PID2;            /*!< Peripheral Identification 2         */
  __I  uint32_t PID3;            /*!< Peripheral Identification 3         */
  __I  uint32_t PCID0;           /*!< PrimeCell Identification 0          */
  __I  uint32_t PCID1;           /*!< PrimeCell Identification 1          */
  __I  uint32_t PCID2;           /*!< PrimeCell Identification 2          */
  __I  uint32_t PCID3;           /*!< PrimeCell Identification 3          */
} WATCHDOG_Type;

/**
  * @brief Register map for GPIO peripheral (GPIO)
  */

typedef struct {                 /*!< GPIOA Structure                     */
  __IO uint32_t MASKEDACCESS[255];
  __IO uint32_t DATA;            /*!< GPIO Data                           */
  __IO uint32_t DIR;             /*!< GPIO Direction                      */
  __IO uint32_t IS;              /*!< GPIO Interrupt Sense                */
  __IO uint32_t IBE;             /*!< GPIO Interrupt Both Edges           */
  __IO uint32_t IEV;             /*!< GPIO Interrupt Event                */
  __IO uint32_t IM;              /*!< GPIO Interrupt Mask                 */
  __I  uint32_t RIS;             /*!< GPIO Raw Interrupt Status           */
  __I  uint32_t MIS;             /*!< GPIO Masked Interrupt Status        */
  __O  uint32_t ICR;             /*!< GPIO Interrupt Clear                */
  __IO uint32_t AFSEL;           /*!< GPIO Alternate Function Select      */
  __I  uint32_t RSRVD1[(0x500-0x420)/4 - 1];
  __IO uint32_t DR2R;            /*!< GPIO 2-mA Drive Select              */
  __IO uint32_t DR4R;            /*!< GPIO 4-mA Drive Select              */
  __IO uint32_t DR8R;            /*!< GPIO 8-mA Drive Select              */
  __IO uint32_t ODR;             /*!< GPIO Open Drain Select              */
  __IO uint32_t PUR;             /*!< GPIO Pull-Up Select                 */
  __IO uint32_t PDR;             /*!< GPIO Pull-Down Select               */
  __IO uint32_t SLR;             /*!< GPIO Slew Rate Control Select       */
  __IO uint32_t DEN;             /*!< GPIO Digital Enable                 */
  __IO uint32_t LOCK;            /*!< GPIO Lock                           */
  __I  uint32_t CR;              /*!< GPIO Commit                         */
  __IO uint32_t AMSEL;           /*!< GPIO Analog Mode Select             */
  __IO uint32_t PCTL;            /*!< GPIO Port Control                   */
  __IO uint32_t ADCCTL;          /*!< GPIO ADC Control                    */
  __IO uint32_t DMACTL;          /*!< GPIO DMA Control                    */
  __I  uint32_t RSRVD2[(0xFD0-0x534)/4 - 1];
  __I  uint32_t PID4;            /*!< Peripheral Identification 4         */
  __I  uint32_t PID5;            /*!< Peripheral Identification 5         */
  __I  uint32_t PID6;            /*!< Peripheral Identification 6         */
  __I  uint32_t PID7;            /*!< Peripheral Identification 7         */
  __I  uint32_t PID0;            /*!< Peripheral Identification 0         */
  __I  uint32_t PID1;            /*!< Peripheral Identification 1         */
  __I  uint32_t PID2;            /*!< Peripheral Identification 2         */
  __I  uint32_t PID3;            /*!< Peripheral Identification 3         */
  __I  uint32_t PCID0;           /*!< PrimeCell Identification 0          */
  __I  uint32_t PCID1;           /*!< PrimeCell Identification 1          */
  __I  uint32_t PCID2;           /*!< PrimeCell Identification 2          */
  __I  uint32_t PCID3;           /*!< PrimeCell Identification 3          */
} GPIO_Type;

/**
  * @brief Register map for SSI peripheral (SSI)
  */

typedef struct {                  /*!< SSI Structure                      */
  __IO uint32_t CR0;              /*!< SSI Control 0                      */
  __IO uint32_t CR1;              /*!< SSI Control 1                      */
  __IO uint32_t DR;               /*!< SSI Data                           */
  __I  uint32_t SR;               /*!< SSI Status                         */
  __IO uint32_t CPSR;             /*!< SSI Clock Prescale                 */
  __IO uint32_t IM;               /*!< SSI Interrupt Mask                 */
  __I  uint32_t RIS;              /*!< SSI Raw Interrupt Status           */
  __I  uint32_t MIS;              /*!< SSI Masked Interrupt Status        */
  __O  uint32_t ICR;              /*!< SSI Interrupt Clear                */
  __IO uint32_t DMACTL;           /*!< SSI DMA Control                    */
  __I  uint32_t RSRVD1[(0xFC8-0x024)/4 - 1];
  __IO uint32_t CC;               /*!< SSI Clock Configuration            */
  __I  uint32_t RSRVD0XFCC;
  __I  uint32_t PID4;            /*!< Peripheral Identification 4         */
  __I  uint32_t PID5;            /*!< Peripheral Identification 5         */
  __I  uint32_t PID6;            /*!< Peripheral Identification 6         */
  __I  uint32_t PID7;            /*!< Peripheral Identification 7         */
  __I  uint32_t PID0;            /*!< Peripheral Identification 0         */
  __I  uint32_t PID1;            /*!< Peripheral Identification 1         */
  __I  uint32_t PID2;            /*!< Peripheral Identification 2         */
  __I  uint32_t PID3;            /*!< Peripheral Identification 3         */
  __I  uint32_t PCID0;           /*!< PrimeCell Identification 0          */
  __I  uint32_t PCID1;           /*!< PrimeCell Identification 1          */
  __I  uint32_t PCID2;           /*!< PrimeCell Identification 2          */
  __I  uint32_t PCID3;           /*!< PrimeCell Identification 3          */
} SSI_Type;

/**
  * @brief Register map for UART peripheral (UART)
  */

typedef struct {                 /*!< UART Structure                      */
  __IO uint32_t DR;              /*!< UART Data                           */
  union {
    __O  uint32_t ECR;           /*!< UART Error Clear                    */
    __I  uint32_t RSR;           /*!< UART Receive Status                 */
  } ;
  __I  uint32_t RSRVD0[(0x018-0x004)/4 - 1];
  __IO uint32_t FR;              /*!< UART Flag                           */
  __I  uint32_t RSRVD1;
  __IO uint32_t ILPR;            /*!< UART IrDA Low-Power Register        */
  __IO uint32_t IBRD;            /*!< UART Integer Baud-Rate Divisor      */
  __IO uint32_t FBRD;            /*!< UART Fractional Baud-Rate Divisor   */
  __IO uint32_t LCRH;            /*!< UART Line Control                   */
  __IO uint32_t CTL;             /*!< UART Control                        */
  __IO uint32_t IFLS;            /*!< UART Interrupt FIFO Level Select    */
  __IO uint32_t IM;              /*!< UART Interrupt Mask                 */
  __I  uint32_t RIS;             /*!< UART Raw Interrupt Status           */
  __I  uint32_t MIS;             /*!< UART Masked Interrupt Status        */
  __O  uint32_t ICR;             /*!< UART Interrupt Clear                */
  __IO uint32_t DMACTL;          /*!< UART DMA Control                    */
  __I  uint32_t RSRVD2[(0x0A4-0x048)/4 - 1];
  __IO uint32_t _9BITADDR;       /*!< UART 9-Bit Self Address             */
  __IO uint32_t _9BITAMASK;      /*!< UART 9-Bit Self Address Mask        */
  __I  uint32_t RSRVD4[(0xFC0-0x0A8)/4 - 1];
  __I  uint32_t PP;              /*!< UART Peripheral Properties          */
  __I  uint32_t RSRVD5;
  __IO uint32_t CC;              /*!< UART Clock Configuration            */
  __I  uint32_t RSRVD0XFCC;
  __I  uint32_t PID4;            /*!< Peripheral Identification 4         */
  __I  uint32_t PID5;            /*!< Peripheral Identification 5         */
  __I  uint32_t PID6;            /*!< Peripheral Identification 6         */
  __I  uint32_t PID7;            /*!< Peripheral Identification 7         */
  __I  uint32_t PID0;            /*!< Peripheral Identification 0         */
  __I  uint32_t PID1;            /*!< Peripheral Identification 1         */
  __I  uint32_t PID2;            /*!< Peripheral Identification 2         */
  __I  uint32_t PID3;            /*!< Peripheral Identification 3         */
  __I  uint32_t PCID0;           /*!< PrimeCell Identification 0          */
  __I  uint32_t PCID1;           /*!< PrimeCell Identification 1          */
  __I  uint32_t PCID2;           /*!< PrimeCell Identification 2          */
  __I  uint32_t PCID3;           /*!< PrimeCell Identification 3          */
} UART_Type;

/**
  * @brief Register map for I2C peripheral (I2C)
  */

typedef struct {                  /*!< I2C Structure                      */
  __IO uint32_t MSA;              /*!< I2C Master Slave Address           */
  __IO uint32_t MCS;              /*!< I2C Master Control/Status          */
  __IO uint32_t MDR;              /*!< I2C Master Data                    */
  __IO uint32_t MTPR;             /*!< I2C Master Timer Period            */
  __IO uint32_t MIMR;             /*!< I2C Master Interrupt Mask          */
  __IO uint32_t MRIS;             /*!< I2C Master Raw Interrupt Status    */
  __IO uint32_t MMIS;             /*!< I2C Master Masked Interrupt Status */
  __O  uint32_t MICR;             /*!< I2C Master Interrupt Clear         */
  __IO uint32_t MCR;              /*!< I2C Master Configuration           */
  __IO uint32_t MCLKOCNT;         /*!< I2C Master Clock Low Timeout Count */
  __I  uint32_t RSRVD0;
  __I  uint32_t MBMON;            /*!< I2C Master Bus Monitor             */
  __I  uint32_t RSRVD1[(0x038-0x02C)/4-1];
  __IO uint32_t MCR2;             /*!< I2C Master Configuration 2         */
  __I  uint32_t RSRVD2[(0x800-0x038)/4-1];
  __IO uint32_t SOAR;             /*!< I2C Slave Own Address              */
  __I  uint32_t SCSR;             /*!< I2C Slave Control/Status           */
  __IO uint32_t SDR;              /*!< I2C Slave Data                     */
  __IO uint32_t SIMR;             /*!< I2C Slave Interrupt Mask           */
  __I  uint32_t SRIS;             /*!< I2C Slave Raw Interrupt Status     */
  __I  uint32_t SMIS;             /*!< I2C Slave Masked Interrupt Status  */
  __O  uint32_t SICR;             /*!< I2C Slave Interrupt Clear          */
  __IO uint32_t SOAR2;            /*!< I2C Slave Own Address 2            */
  __IO uint32_t SACKCTL;          /*!< I2C ACK Control                    */
  __I  uint32_t RSRVD3[(0xFC0-0x820)/4-1];
  __I  uint32_t PP;               /*!< I2C Peripheral Properties          */
  __I  uint32_t PC;               /*!< I2C Peripheral Properties          */
} I2C_Type;

/**
  * @brief Register map for PWM peripheral (PWM)
  */

typedef struct {                 /*!< PWM Structure                      */
  __IO uint32_t CTL;             /*!< PWM Master Control                  */
  __IO uint32_t SYNC;            /*!< PWM Time Base Sync                  */
  __IO uint32_t ENABLE;          /*!< PWM Output Enable                   */
  __IO uint32_t INVERT;          /*!< PWM Output Inversion                */
  __IO uint32_t FAULT;           /*!< PWM Output Fault                    */
  __IO uint32_t INTEN;           /*!< PWM Interrupt Enable                */
  __I  uint32_t RIS;             /*!< PWM Raw Interrupt Status            */
  __IO uint32_t ISC;             /*!< PWM Interrupt Status and Clear      */
  __I  uint32_t STATUS;          /*!< PWM Status                          */
  __IO uint32_t FAULTVAL;        /*!< PWM Fault Condition Value           */
  __IO uint32_t ENUPD;           /*!< PWM Enable Update                   */
  __I  uint32_t RSRVD0[(0x040-0x028)/4-1];
  __IO uint32_t _0_CTL;          /*!< PWM0 Control                        */
  __IO uint32_t _0_INTEN;        /*!< PWM0 Interrupt and Trigger Enable   */
  __I  uint32_t _0_RIS;          /*!< PWM0 Raw Interrupt Status           */
  __IO uint32_t _0_ISC;          /*!< PWM0 Interrupt Status and Clear     */
  __IO uint32_t _0_LOAD;         /*!< PWM0 Load                           */
  __I  uint32_t _0_COUNT;        /*!< PWM0 Counter                        */
  __IO uint32_t _0_CMPA;         /*!< PWM0 Compare A                      */
  __IO uint32_t _0_CMPB;         /*!< PWM0 Compare B                      */
  __IO uint32_t _0_GENA;         /*!< PWM0 Generator A Control            */
  __IO uint32_t _0_GENB;         /*!< PWM0 Generator B Control            */
  __IO uint32_t _0_DBCTL;        /*!< PWM0 Dead-Band Control              */
  __IO uint32_t _0_DBRISE;       /*!< PWM0 Dead-Band Rising-Edge Delay    */
  __IO uint32_t _0_DBFALL;       /*!< PWM0 Dead-Band Falling-Edge-Delay   */
  __IO uint32_t _0_FLTSRC0;      /*!< PWM0 Fault Source 0                 */
  __IO uint32_t _0_FLTSRC1;      /*!< PWM0 Fault Source 1                 */
  __IO uint32_t _0_MINFLTPER;    /*!< PWM0 Minimum Fault Period           */
  __IO uint32_t _1_CTL;          /*!< PWM1 Control                        */
  __IO uint32_t _1_INTEN;        /*!< PWM1 Interrupt and Trigger Enable   */
  __I  uint32_t _1_RIS;          /*!< PWM1 Raw Interrupt Status           */
  __IO uint32_t _1_ISC;          /*!< PWM1 Interrupt Status and Clear     */
  __IO uint32_t _1_LOAD;         /*!< PWM1 Load                           */
  __I  uint32_t _1_COUNT;        /*!< PWM1 Counter                        */
  __IO uint32_t _1_CMPA;         /*!< PWM1 Compare A                      */
  __IO uint32_t _1_CMPB;         /*!< PWM1 Compare B                      */
  __IO uint32_t _1_GENA;         /*!< PWM1 Generator A Control            */
  __IO uint32_t _1_GENB;         /*!< PWM1 Generator B Control            */
  __IO uint32_t _1_DBCTL;        /*!< PWM1 Dead-Band Control              */
  __IO uint32_t _1_DBRISE;       /*!< PWM1 Dead-Band Rising-Edge Delay    */
  __IO uint32_t _1_DBFALL;       /*!< PWM1 Dead-Band Falling-Edge-Delay   */
  __IO uint32_t _1_FLTSRC0;      /*!< PWM1 Fault Source 0                 */
  __IO uint32_t _1_FLTSRC1;      /*!< PWM1 Fault Source 1                 */
  __IO uint32_t _1_MINFLTPER;    /*!< PWM1 Minimum Fault Period           */
  __IO uint32_t _2_CTL;          /*!< PWM2 Control                        */
  __IO uint32_t _2_INTEN;        /*!< PWM2 Interrupt and Trigger Enable   */
  __I  uint32_t _2_RIS;          /*!< PWM2 Raw Interrupt Status           */
  __IO uint32_t _2_ISC;          /*!< PWM2 Interrupt Status and Clear     */
  __IO uint32_t _2_LOAD;         /*!< PWM2 Load                           */
  __I  uint32_t _2_COUNT;        /*!< PWM2 Counter                        */
  __IO uint32_t _2_CMPA;         /*!< PWM2 Compare A                      */
  __IO uint32_t _2_CMPB;         /*!< PWM2 Compare B                      */
  __IO uint32_t _2_GENA;         /*!< PWM2 Generator A Control            */
  __IO uint32_t _2_GENB;         /*!< PWM2 Generator B Control            */
  __IO uint32_t _2_DBCTL;        /*!< PWM2 Dead-Band Control              */
  __IO uint32_t _2_DBRISE;       /*!< PWM2 Dead-Band Rising-Edge Delay    */
  __IO uint32_t _2_DBFALL;       /*!< PWM2 Dead-Band Falling-Edge-Delay   */
  __IO uint32_t _2_FLTSRC0;      /*!< PWM2 Fault Source 0                 */
  __IO uint32_t _2_FLTSRC1;      /*!< PWM2 Fault Source 1                 */
  __IO uint32_t _2_MINFLTPER;    /*!< PWM2 Minimum Fault Period           */
  __IO uint32_t _3_CTL;          /*!< PWM3 Control                        */
  __IO uint32_t _3_INTEN;        /*!< PWM3 Interrupt and Trigger Enable   */
  __I  uint32_t _3_RIS;          /*!< PWM3 Raw Interrupt Status           */
  __IO uint32_t _3_ISC;          /*!< PWM3 Interrupt Status and Clear     */
  __IO uint32_t _3_LOAD;         /*!< PWM3 Load                           */
  __I  uint32_t _3_COUNT;        /*!< PWM3 Counter                        */
  __IO uint32_t _3_CMPA;         /*!< PWM3 Compare A                      */
  __IO uint32_t _3_CMPB;         /*!< PWM3 Compare B                      */
  __IO uint32_t _3_GENA;         /*!< PWM3 Generator A Control            */
  __IO uint32_t _3_GENB;         /*!< PWM3 Generator B Control            */
  __IO uint32_t _3_DBCTL;        /*!< PWM3 Dead-Band Control              */
  __IO uint32_t _3_DBRISE;       /*!< PWM3 Dead-Band Rising-Edge Delay    */
  __IO uint32_t _3_DBFALL;       /*!< PWM3 Dead-Band Falling-Edge-Delay   */
  __IO uint32_t _3_FLTSRC0;      /*!< PWM3 Fault Source 0                 */
  __IO uint32_t _3_FLTSRC1;      /*!< PWM3 Fault Source 1                 */
  __IO uint32_t _3_MINFLTPER;    /*!< PWM3 Minimum Fault Period           */
  __I  uint32_t RSRVD1[(0x800-0x13C)/4-1];
  __IO uint32_t _0_FLTSEN;       /*!< PWM0 Fault Pin Logic Sense          */
  __I  uint32_t _0_FLTSTAT0;     /*!< PWM0 Fault Status 0                 */
  __I  uint32_t _0_FLTSTAT1;     /*!< PWM0 Fault Status 1                 */
  __I  uint32_t RSRVD2[(0x880-0x808)/4-1];
  __IO uint32_t _1_FLTSEN;       /*!< PWM1 Fault Pin Logic Sense          */
  __I  uint32_t _1_FLTSTAT0;     /*!< PWM1 Fault Status 0                 */
  __I  uint32_t _1_FLTSTAT1;     /*!< PWM1 Fault Status 1                 */
  __I  uint32_t RSRVD3[(0x904-0x888)/4-1];
  __I  uint32_t _2_FLTSTAT0;     /*!< PWM2 Fault Status 0                 */
  __I  uint32_t _2_FLTSTAT1;     /*!< PWM2 Fault Status 1                 */
  __I  uint32_t RSRVD4[(0x984-0x908)/4-1];
  __I  uint32_t _3_FLTSTAT0;     /*!< PWM3 Fault Status 0                 */
  __I  uint32_t _3_FLTSTAT1;     /*!< PWM3 Fault Status 1                 */
  __I  uint32_t RSRVD5[(0xFC0-0x988)/4-1];
  __I  uint32_t PP;              /*!< PWM Peripheral Properties           */
} PWM_Type;

/**
  * @brief Register map for QEI peripheral (QEI)
  */

typedef struct {             /*!< QEI Structure                           */
  __IO uint32_t CTL;         /*!< QEI Control                             */
  __I  uint32_t STAT;        /*!< QEI Status                              */
  __IO uint32_t POS;         /*!< QEI Position                            */
  __IO uint32_t MAXPOS;      /*!< QEI Maximum Position                    */
  __IO uint32_t LOAD;        /*!< QEI Timer Load                          */
  __I  uint32_t TIME;        /*!< QEI Timer                               */
  __I  uint32_t COUNT;       /*!< QEI Velocity Counter                    */
  __I  uint32_t SPEED;       /*!< QEI Velocity                            */
  __IO uint32_t INTEN;       /*!< QEI Interrupt Enable                    */
  __I  uint32_t RIS;         /*!< QEI Raw Interrupt Status                */
  __IO uint32_t ISC;         /*!< QEI Interrupt Status and Clear          */
} QEI_Type;

/**
  * @brief Register map for GPTM peripheral (GPTM)
  */

typedef struct {                  /*!< GPTM Structure                     */
  __IO uint32_t CFG;              /*!< GPTM Configuration                 */
  __IO uint32_t TAMR;             /*!< GPTM Timer A Mode                  */
  __IO uint32_t TBMR;             /*!< GPTM Timer B Mode                  */
  __IO uint32_t CTL;              /*!< GPTM Control                       */
  __IO uint32_t SYNC;             /*!< GPTM Synchronize                   */
  __I  uint32_t RSRVD0X014;
  __IO uint32_t IMR;              /*!< GPTM Interrupt Mask                */
  __I  uint32_t RIS;              /*!< GPTM Raw Interrupt Status          */
  __I  uint32_t MIS;              /*!< GPTM Masked Interrupt Status       */
  __O  uint32_t ICR;              /*!< GPTM Interrupt Clear               */
  __IO uint32_t TAILR;            /*!< GPTM Timer A Interval Load         */
  __IO uint32_t TBILR;            /*!< GPTM Timer B Interval Load         */
  __IO uint32_t TAMATCHR;         /*!< GPTM Timer A Match                 */
  __IO uint32_t TBMATCHR;         /*!< GPTM Timer B Match                 */
  __IO uint32_t TAPR;             /*!< GPTM Timer A Prescale              */
  __IO uint32_t TBPR;             /*!< GPTM Timer B Prescale              */
  __IO uint32_t TAPMR;            /*!< GPTM TimerA Prescale Match         */
  __IO uint32_t TBPMR;            /*!< GPTM TimerB Prescale Match         */
  __IO uint32_t TAR;              /*!< GPTM Timer A                       */
  __I  uint32_t TBR;              /*!< GPTM Timer B                       */
  __IO uint32_t TAV;              /*!< GPTM Timer A Value                 */
  __IO uint32_t TBV;              /*!< GPTM Timer B Value                 */
  __I  uint32_t RTCPD;            /*!< GPTM RTC Predivide                 */
  __I  uint32_t TAPS;             /*!< GPTM Timer A Prescale Snapshot     */
  __I  uint32_t TBPS;             /*!< GPTM Timer B Prescale Snapshot     */
  __I  uint32_t TAPV;             /*!< GPTM Timer A Prescale Value        */
  __I  uint32_t TBPV;             /*!< GPTM Timer B Prescale Value        */
  __I  uint32_t RSRVD1[(0xFC0 - 0x068)/4 -1];
  __I  uint32_t PP;               /*!< GPTM Peripheral Properties         */
} GPTM_Type;

/**
  * @brief Register map for ADC peripheral (ADC)
  */

typedef struct {           /*!< ADC Structure                            */
  __IO uint32_t ACTSS;     /*!< Active Sample Sequencer                   */
  __I  uint32_t RIS;       /*!< Raw Interrupt Status                      */
  __IO uint32_t IM;        /*!< Interrupt Mask                            */
  __IO uint32_t ISC;       /*!< Interrupt Status and Clear                */
  __IO uint32_t OSTAT;     /*!< Overflow Status                           */
  __IO uint32_t EMUX;      /*!< Event Multiplexer Select                  */
  __IO uint32_t USTAT;     /*!< Underflow Status                          */
  __IO uint32_t TSSEL;     /*!< Trigger Source Select                     */
  __IO uint32_t SSPRI;     /*!< Sample Sequencer Priority                 */
  __IO uint32_t SPC;       /*!< Sample Phase Control                      */
  __IO uint32_t PSSI;      /*!< Processor Sample Sequence Initiate        */
  __I  uint32_t RSRVD0X02C;
  __IO uint32_t SAC;       /*!< Sample Averaging Control                  */
  __IO uint32_t DCISC;     /*!< Digital Comparator Interrupt Status & Clr */
  __IO uint32_t CTL;       /*!< ADC Control                               */
  __I  uint32_t RSRVD0X03C;
  __IO uint32_t SSMUX0;    /*!< Sample Sequence Input Mux Select 0        */
  __IO uint32_t SSCTL0;    /*!< Sample Sequence Control 0                 */
  __I  uint32_t SSFIFO0;   /*!< Sample Sequence Result FIFO 0             */
  __I  uint32_t SSFSTAT0;  /*!< Sample Sequence FIFO 0 Status             */
  __IO uint32_t SSOP0;     /*!< Sample Sequence 0 Operation               */
  __IO uint32_t SSDC0;     /*!< Sample Sequence 0 Digital Cmprtr Select   */
  __I  uint32_t RSRVD2[2];
  __IO uint32_t SSMUX1;    /*!< Sample Sequence Input Mux Select 1        */
  __IO uint32_t SSCTL1;    /*!< Sample Sequence Control 1                 */
  __I  uint32_t SSFIFO1;   /*!< Sample Sequence Result FIFO 1             */
  __I  uint32_t SSFSTAT1;  /*!< Sample Sequence FIFO 1 Status             */
  __IO uint32_t SSOP1;     /*!< Sample Sequence 1 Operation               */
  __IO uint32_t SSDC1;     /*!< Sample Sequence 1 Digital Cmprtr Select   */
  __I  uint32_t RSRVD3[2];
  __IO uint32_t SSMUX2;    /*!< Sample Sequence Input Mux Select 2        */
  __IO uint32_t SSCTL2;    /*!< Sample Sequence Control 2                 */
  __I  uint32_t SSFIFO2;   /*!< Sample Sequence Result FIFO 2             */
  __I  uint32_t SSFSTAT2;  /*!< Sample Sequence FIFO 2 Status             */
  __IO uint32_t SSOP2;     /*!< Sample Sequence 2 Operation               */
  __IO uint32_t SSDC2;     /*!< Sample Sequence 2 Digital Cmprtr Select   */
  __I  uint32_t RSRVD4[2];
  __IO uint32_t SSMUX3;    /*!< Sample Sequence Input Mux Select 3        */
  __IO uint32_t SSCTL3;    /*!< Sample Sequence Control 3                 */
  __I  uint32_t SSFIFO3;   /*!< Sample Sequence Result FIFO 3             */
  __I  uint32_t SSFSTAT3;  /*!< Sample Sequence FIFO 3 Status             */
  __IO uint32_t SSOP3;     /*!< Sample Sequence 3 Operation               */
  __IO uint32_t SSDC3;     /*!< Sample Sequence 3 Digital Cmprtr Select   */
  __I  uint32_t RSRVD5[(0xD00 - 0x0B4)/4 - 1];
  __O  uint32_t DCRIC;     /*!< Digital Comparator Reset Init Conditions  */
  __I  uint32_t RSRVD6[(0xE00 - 0xD00)/4 - 1];
  __IO uint32_t DCCTL0;    /*!< Digital Comparator Control 0              */
  __IO uint32_t DCCTL1;    /*!< Digital Comparator Control 1              */
  __IO uint32_t DCCTL2;    /*!< Digital Comparator Control 2              */
  __IO uint32_t DCCTL3;    /*!< Digital Comparator Control 3              */
  __IO uint32_t DCCTL4;    /*!< Digital Comparator Control 4              */
  __IO uint32_t DCCTL5;    /*!< Digital Comparator Control 5              */
  __IO uint32_t DCCTL6;    /*!< Digital Comparator Control 6              */
  __IO uint32_t DCCTL7;    /*!< Digital Comparator Control 7              */
  __I  uint32_t RSRVD7[(0xE40 - 0xE1C)/4 - 1];
  __IO uint32_t DCCMP0;    /*!< Digital Comparator Range 0                */
  __IO uint32_t DCCMP1;    /*!< Digital Comparator Range 1                */
  __IO uint32_t DCCMP2;    /*!< Digital Comparator Range 2                */
  __IO uint32_t DCCMP3;    /*!< Digital Comparator Range 3                */
  __IO uint32_t DCCMP4;    /*!< Digital Comparator Range 4                */
  __IO uint32_t DCCMP5;    /*!< Digital Comparator Range 5                */
  __IO uint32_t DCCMP6;    /*!< Digital Comparator Range 6                */
  __IO uint32_t DCCMP7;    /*!< Digital Comparator Range 7                */
  __I  uint32_t RSRVD8[(0xFC0 - 0xE5C)/4 - 1];
  __I  uint32_t PP;        /*!< Peripheral Properties                     */
  __IO uint32_t PC;        /*!< Peripheral Configuration                  */
  __IO uint32_t CC;        /*!< Clock Configuration                       */
} ADC_Type;

/**
  * @brief Register map for COMPARATOR peripheral (COMP)
  */

typedef struct {            /*!< COMP Structure                           */
  __IO uint32_t ACMIS;      /*!< Analog Comp Masked Interrupt Status      */
  __I  uint32_t ACRIS;      /*!< Analog Comp Raw Interrupt Status         */
  __IO uint32_t ACINTEN;    /*!< Analog Comp Interrupt Enable             */
  __I  uint32_t RSRVD0X00C;
  __IO uint32_t ACREFCTL;   /*!< Analog Comp Reference Voltage Control    */
  __I  uint32_t RSRVD1[3];
  __I  uint32_t ACSTAT0;    /*!< Analog Comp Status 0                     */
  __IO uint32_t ACCTL0;     /*!< Analog Comp Control 0                    */
  __I  uint32_t RSRVD2[(0x040 - 0x024)/4 - 1];
  __I  uint32_t ACSTAT1;    /*!< Analog Comp Status 1                     */
  __IO uint32_t ACCTL1;     /*!< Analog Comp Control 1                    */
  __I  uint32_t RSRVD3[(0xFC0 - 0x044)/4 - 1];
  __I  uint32_t PP;         /*!< Analog Comp Peripheral Properties        */
} COMP_Type;

/**
  * @brief Register map for CAN peripheral (CAN)
  */

typedef struct {                    /*!< CAN Structure                   */
  __IO uint32_t CTL;                /*!< CAN Control                      */
  __IO uint32_t STS;                /*!< CAN Status                       */
  __I  uint32_t ERR;                /*!< CAN Error Counter                */
  __IO uint32_t BIT;                /*!< CAN Bit Timing                   */
  __I  uint32_t INT;                /*!< CAN Interrupt                    */
  __IO uint32_t TST;                /*!< CAN Test                         */
  __IO uint32_t BRPE;               /*!< CAN Baud Rate Prescaler Extension*/
  __I  uint32_t RSRVD0X01C;
  __IO uint32_t IF1CRQ;             /*!< CAN IF1 Command Request          */
  __IO uint32_t IF1CMSK;            /*!< CAN IF1 Command Mask             */
  __IO uint32_t IF1MSK1;            /*!< CAN IF1 Mask 1                   */
  __IO uint32_t IF1MSK2;            /*!< CAN IF1 Mask 2                   */
  __IO uint32_t IF1ARB1;            /*!< CAN IF1 Arbitration 1            */
  __IO uint32_t IF1ARB2;            /*!< CAN IF1 Arbitration 2            */
  __IO uint32_t IF1MCTL;            /*!< CAN IF1 Message Control          */
  __IO uint32_t IF1DA1;             /*!< CAN IF1 Data A1                  */
  __IO uint32_t IF1DA2;             /*!< CAN IF1 Data A2                  */
  __IO uint32_t IF1DB1;             /*!< CAN IF1 Data B1                  */
  __IO uint32_t IF1DB2;             /*!< CAN IF1 Data B2                  */
  __I  uint32_t RSRVD1[(0x080 - 0x048)/4 - 1];
  __IO uint32_t IF2CRQ;             /*!< CAN IF2 Command Request          */
  __IO uint32_t IF2CMSK;            /*!< CAN IF2 Command Mask             */
  __IO uint32_t IF2MSK1;            /*!< CAN IF2 Mask 1                   */
  __IO uint32_t IF2MSK2;            /*!< CAN IF2 Mask 2                   */
  __IO uint32_t IF2ARB1;            /*!< CAN IF2 Arbitration 1            */
  __IO uint32_t IF2ARB2;            /*!< CAN IF2 Arbitration 2            */
  __IO uint32_t IF2MCTL;            /*!< CAN IF2 Message Control          */
  __IO uint32_t IF2DA1;             /*!< CAN IF2 Data A1                  */
  __IO uint32_t IF2DA2;             /*!< CAN IF2 Data A2                  */
  __IO uint32_t IF2DB1;             /*!< CAN IF2 Data B1                  */
  __IO uint32_t IF2DB2;             /*!< CAN IF2 Data B2                  */
  __I  uint32_t RSRVD2[(0x100 - 0x0A8)/4 - 1];
  __I  uint32_t TXRQ1;              /*!< CAN Transmission Request 1       */
  __I  uint32_t TXRQ2;              /*!< CAN Transmission Request 2       */
  __I  uint32_t RSRVD3[(0x120 - 0x104)/4 - 1];
  __I  uint32_t NWDA1;              /*!< CAN New Data 1                   */
  __I  uint32_t NWDA2;              /*!< CAN New Data 2                   */
  __I  uint32_t RSRVD4[(0x140 - 0x124)/4 - 1];
  __I  uint32_t MSG1INT;            /*!< CAN Message 1 Interrupt Pending  */
  __I  uint32_t MSG2INT;            /*!< CAN Message 2 Interrupt Pending  */
  __I  uint32_t RSRVD5[(0x160 - 0x144)/4 - 1];
  __I  uint32_t MSG1VAL;            /*!< CAN Message 1 Valid              */
  __I  uint32_t MSG2VAL;            /*!< CAN Message 2 Valid              */
} CAN_Type;

/**
  * @brief Register map for USB0 peripheral (USB0)
  */

typedef struct {                            /*!< USB0 Structure              */
  __IO uint8_t  FADDR;                      /*!< Device Functional Address   */
  __IO uint8_t  POWER;                      /*!< Power                       */
  __I  uint16_t TXIS;                       /*!< Transmit Interrupt Status   */
  __I  uint16_t RXIS;                       /*!< Receive Interrupt Status    */
  __IO uint16_t TXIE;                       /*!< Transmit Interrupt Enable   */
  __IO uint16_t RXIE;                       /*!< Receive Interrupt Enable    */
  __I  uint8_t  IS;                         /*!< General Interrupt Status    */
  __I  uint8_t  IE;                         /*!< Interrupt Enable            */
  __IO uint16_t FRAME;                      /*!< Frame Value                 */
  __IO uint8_t  EPIDX;                      /*!< Endpoint Index              */
  __IO uint8_t  TEST;                       /*!< Test Mode                   */
  __I  uint32_t RSRVD0[4];
  __IO uint32_t FIFO0;                      /*!< FIFO Endpoint 0             */
  __IO uint32_t FIFO1;                      /*!< FIFO Endpoint 1             */
  __IO uint32_t FIFO2;                      /*!< FIFO Endpoint 2             */
  __IO uint32_t FIFO3;                      /*!< FIFO Endpoint 3             */
  __IO uint32_t FIFO4;                      /*!< FIFO Endpoint 4             */
  __IO uint32_t FIFO5;                      /*!< FIFO Endpoint 5             */
  __IO uint32_t FIFO6;                      /*!< FIFO Endpoint 6             */
  __IO uint32_t FIFO7;                      /*!< FIFO Endpoint 7             */
  __I  uint32_t RSRVD1[(0x060 - 0x03C)/4 - 1];
  __IO uint8_t  DEVCTL;                     /*!< Device Control              */
  __I  uint8_t  RSRVD0X061;
  __IO uint8_t  TXFIFOSZ;                   /*!< Transmit Dynamic FIFO Sizing*/
  __IO uint8_t  RXFIFOSZ;                   /*!< Rcv Dynamic FIFO Sizing */
  __IO uint16_t TXFIFOADD;                  /*!< Transmit FIFO Start Address */
  __IO uint16_t RXFIFOADD;                  /*!< Rcv FIFO Start Address  */
  __I  uint8_t  RSRVD3[18];
  __IO uint8_t  CONTIM;                     /*!< Connect Timing              */
  __IO uint8_t  VPLEN;                      /*!< OTG VBUS Pulse Timing       */
  __I  uint8_t  RSRVD0X07C;
  __IO uint8_t  FSEOF;                      /*!< Full-Speed Last Transaction
                                                  to End of Frame Timing     */
  __IO uint8_t  LSEOF;                      /*!< Low-Speed Last Transaction
                                                  to End of Frame Timing     */
  __I  uint8_t  RSRVD0X07F;
  __IO uint8_t  TXFUNCADDR0;                /*!< TX Func Addr Endpoint 0     */
  __I  uint8_t  RSRVD0X081;
  __IO uint8_t  TXHUBADDR0;                 /*!< TX Hub Address Endpoint 0   */
  __IO uint8_t  TXHUBPORT0;                 /*!< TX Hub Port Endpoint 0      */
  __I  uint8_t  RSRVD7[(0x088 - 0x083) - 1];
  __IO uint8_t  TXFUNCADDR1;                /*!< TX Func Addr Endpoint 1     */
  __I  uint8_t  RSRVD0X089;
  __IO uint8_t  TXHUBADDR1;                 /*!< TX Hub Address Endpoint 1   */
  __IO uint8_t  TXHUBPORT1;                 /*!< TX Hub Port Endpoint 1      */
  __IO uint8_t  RXFUNCADDR1;                /*!< Rcv Func Addr Endpoint 1    */
  __I  uint8_t  RSRVD0X08D;
  __IO uint8_t  RXHUBADDR1;                 /*!< Rcv Hub Addr Endpoint 1     */
  __IO uint8_t  RXHUBPORT1;                 /*!< Rcv Hub Port Endpoint 1     */
  __IO uint8_t  TXFUNCADDR2;                /*!< TX Func Address Endpoint 2  */
  __I  uint8_t  RSRVD0X091;
  __IO uint8_t  TXHUBADDR2;                 /*!< TX Hub Address Endpoint 2   */
  __IO uint8_t  TXHUBPORT2;                 /*!< TX Hub Port Endpoint 2      */
  __IO uint8_t  RXFUNCADDR2;                /*!< Rcv Func Address Endpoint 2 */
  __I  uint8_t  RSRVD0X095;
  __IO uint8_t  RXHUBADDR2;                 /*!< Rcv Hub Address Endpoint 2  */
  __IO uint8_t  RXHUBPORT2;                 /*!< Rcv Hub Port Endpoint 2     */
  __IO uint8_t  TXFUNCADDR3;                /*!< TX Func Address Endpoint 3  */
  __I  uint8_t  RSRVD0X099;
  __IO uint8_t  TXHUBADDR3;                 /*!< TX Hub Address Endpoint 3   */
  __IO uint8_t  TXHUBPORT3;                 /*!< TX Hub Port Endpoint 3      */
  __IO uint8_t  RXFUNCADDR3;                /*!< Rcv Func Address Endpoint 3 */
  __I  uint8_t  RSRVD0X09D;
  __IO uint8_t  RXHUBADDR3;                 /*!< Rcv Hub Address Endpoint 3  */
  __IO uint8_t  RXHUBPORT3;                 /*!< Rcv Hub Port Endpoint 3     */
  __IO uint8_t  TXFUNCADDR4;                /*!< TX Func Address Endpoint 4  */
  __I  uint8_t  RSRVD0X0A1;
  __IO uint8_t  TXHUBADDR4;                 /*!< TX Hub Address Endpoint 4   */
  __IO uint8_t  TXHUBPORT4;                 /*!< TX Hub Port Endpoint 4      */
  __IO uint8_t  RXFUNCADDR4;                /*!< Rcv Func Address Endpoint 4 */
  __I  uint8_t  RSRVD0X0A5;
  __IO uint8_t  RXHUBADDR4;                 /*!< Rcv Hub Address Endpoint 4  */
  __IO uint8_t  RXHUBPORT4;                 /*!< Rcv Hub Port Endpoint 4     */
  __IO uint8_t  TXFUNCADDR5;                /*!< TX Func Address Endpoint 5  */
  __I  uint8_t  RSRVD0X0A9;
  __IO uint8_t  TXHUBADDR5;                 /*!< TX Hub Address Endpoint 5   */
  __IO uint8_t  TXHUBPORT5;                 /*!< TX Hub Port Endpoint 5      */
  __IO uint8_t  RXFUNCADDR5;                /*!< Rcv Func Address Endpoint 5 */
  __I  uint8_t  RSRVD0X0AD;
  __IO uint8_t  RXHUBADDR5;                 /*!< Rcv Hub Address Endpoint 5  */
  __IO uint8_t  RXHUBPORT5;                 /*!< Rcv Hub Port Endpoint 5     */
  __IO uint8_t  TXFUNCADDR6;                /*!< TX Func Address Endpoint 6  */
  __I  uint8_t  RSRVD0X0B1;
  __IO uint8_t  TXHUBADDR6;                 /*!< TX Hub Address Endpoint 6   */
  __IO uint8_t  TXHUBPORT6;                 /*!< TX Hub Port Endpoint 6      */
  __IO uint8_t  RXFUNCADDR6;                /*!< Rcv Func Address Endpoint 6 */
  __I  uint8_t  RSRVD0X0B5;
  __IO uint8_t  RXHUBADDR6;                 /*!< Rcv Hub Address Endpoint 6  */
  __IO uint8_t  RXHUBPORT6;                 /*!< Rcv Hub Port Endpoint 6     */
  __IO uint8_t  TXFUNCADDR7;                /*!< TX Func Address Endpoint 7  */
  __I  uint8_t  RSRVD0X0B9;
  __IO uint8_t  TXHUBADDR7;                 /*!< TX Hub Address Endpoint 7   */
  __IO uint8_t  TXHUBPORT7;                 /*!< TX Hub Port Endpoint 7      */
  __IO uint8_t  RXFUNCADDR7;                /*!< Rcv Func Address Endpoint 7 */
  __I  uint8_t  RSRVD0X0BD;
  __IO uint8_t  RXHUBADDR7;                 /*!< Rcv Hub Address Endpoint 7  */
  __IO uint8_t  RXHUBPORT7;                 /*!< Rcv Hub Port Endpoint 7     */
  __I  uint8_t  RSRVD22[(0x102 - 0x0BF) - 1];
  __O  uint8_t  CSRL0;                      /*!< Cntrl/Stat Endpt 0 Low      */
  __O  uint8_t  CSRH0;                      /*!< Cntrl/Stat Endpt 0 High     */
  __I  uint8_t  RSRVD23[(0x108 - 0x103) - 1];
  __I  uint8_t  COUNT0;                     /*!< Rcv Byte Count Endpt 0      */
  __I  uint8_t  RSRVD0X109;
  __IO uint8_t  TYPE0;                      /*!< Type Endpt 0                */
  __IO uint8_t  NAKLMT;                     /*!< NAK Limit                   */
  __I  uint8_t  RSRVD25[(0x110 - 0x10B) - 1];
  __IO uint16_t TXMAXP1;                    /*!< Maximum TX Data Endpt 1     */
  __IO uint8_t  TXCSRL1;                    /*!< TX Cntrl/Stat Endpt 1 Low   */
  __IO uint8_t  TXCSRH1;                    /*!< TX Cntrl/Stat Endpt 1 High  */
  __IO uint16_t RXMAXP1;                    /*!< Maximum Rcv Data Endpt 1    */
  __IO uint8_t  RXCSRL1;                    /*!< Rcv Cntrl/Stat Endpt 1 Low  */
  __IO uint8_t  RXCSRH1;                    /*!< Rcv Cntrl/Stat Endpt 1 High */
  __I  uint16_t RXCOUNT1;                   /*!< Rcv Byte Count Endpt 1      */
  __IO uint8_t  TXTYPE1;                    /*!< Host TX Config Type Endpt 1 */
  __IO uint8_t  TXINTERVAL1;                /*!< Host TX Interval Endpt 1    */
  __IO uint8_t  RXTYPE1;                    /*!< Host Config Rcv Type Endpt 1*/
  __IO uint8_t  RXINTERVAL1;                /*!< Host Rcv Polling Interval
                                                 Endpt 1                     */
  __I  uint16_t RSRVD0X11E;
  __IO uint16_t TXMAXP2;                    /*!< Maximum TX Data Endpt 2     */
  __IO uint8_t  TXCSRL2;                    /*!< TX Cntrl/Stat Endpt 2 Low   */
  __IO uint8_t  TXCSRH2;                    /*!< TX Cntrl/Stat Endpt 2 High  */
  __IO uint16_t RXMAXP2;                    /*!< Maximum Rcv Data Endpt 2    */
  __IO uint8_t  RXCSRL2;                    /*!< Rcv Cntrl/Stat Endpt 2 Low  */
  __IO uint8_t  RXCSRH2;                    /*!< Rcv Cntrl/Stat Endpt 2 High */
  __I  uint16_t RXCOUNT2;                   /*!< Rcv Byte Count Endpt 2      */
  __IO uint8_t  TXTYPE2;                    /*!< Host TX Config Type Endpt 2 */
  __IO uint8_t  TXINTERVAL2;                /*!< Host TX Interval Endpt 2    */
  __IO uint8_t  RXTYPE2;                    /*!< Host Config Rcv Type Endpt 2*/
  __IO uint8_t  RXINTERVAL2;                /*!< Host Rcv Polling Interval
                                                 Endpt 2                     */
  __I  uint16_t RSRVD0X12E;
  __IO uint16_t TXMAXP3;                    /*!< Maximum TX Data Endpt 3     */
  __IO uint8_t  TXCSRL3;                    /*!< TX Cntrl/Stat Endpt 3 Low   */
  __IO uint8_t  TXCSRH3;                    /*!< TX Cntrl/Stat Endpt 3 High  */
  __IO uint16_t RXMAXP3;                    /*!< Maximum Rcv Data Endpt 3    */
  __IO uint8_t  RXCSRL3;                    /*!< Rcv Cntrl/Stat Endpt 3 Low  */
  __IO uint8_t  RXCSRH3;                    /*!< Rcv Cntrl/Stat Endpt 3 High */
  __I  uint16_t RXCOUNT3;                   /*!< Rcv Byte Count Endpt 3      */
  __IO uint8_t  TXTYPE3;                    /*!< Host TX Config Type Endpt 3 */
  __IO uint8_t  TXINTERVAL3;                /*!< Host TX Interval Endpt 3    */
  __IO uint8_t  RXTYPE3;                    /*!< Host Config Rcv Type Endpt 3*/
  __IO uint8_t  RXINTERVAL3;                /*!< Host Rcv Polling Interval
                                                 Endpt 3                     */
  __I  uint16_t RSRVD0X13E;
  __IO uint16_t TXMAXP4;                    /*!< Maximum TX Data Endpt 4     */
  __IO uint8_t  TXCSRL4;                    /*!< TX Cntrl/Stat Endpt 4 Low   */
  __IO uint8_t  TXCSRH4;                    /*!< TX Cntrl/Stat Endpt 4 High  */
  __IO uint16_t RXMAXP4;                    /*!< Maximum Rcv Data Endpt 4    */
  __IO uint8_t  RXCSRL4;                    /*!< Rcv Cntrl/Stat Endpt 4 Low  */
  __IO uint8_t  RXCSRH4;                    /*!< Rcv Cntrl/Stat Endpt 4 High */
  __I  uint16_t RXCOUNT4;                   /*!< Rcv Byte Count Endpt 4      */
  __IO uint8_t  TXTYPE4;                    /*!< Host TX Config Type Endpt 4 */
  __IO uint8_t  TXINTERVAL4;                /*!< Host TX Interval Endpt 4    */
  __IO uint8_t  RXTYPE4;                    /*!< Host Config Rcv Type Endpt 4*/
  __IO uint8_t  RXINTERVAL4;                /*!< Host Rcv Polling Interval
                                                 Endpt 4                     */
  __I  uint16_t RSRVD0X14E;
  __IO uint16_t TXMAXP5;                    /*!< Maximum TX Data Endpt 5     */
  __IO uint8_t  TXCSRL5;                    /*!< TX Cntrl/Stat Endpt 5 Low   */
  __IO uint8_t  TXCSRH5;                    /*!< TX Cntrl/Stat Endpt 5 High  */
  __IO uint16_t RXMAXP5;                    /*!< Maximum Rcv Data Endpt 5    */
  __IO uint8_t  RXCSRL5;                    /*!< Rcv Cntrl/Stat Endpt 5 Low  */
  __IO uint8_t  RXCSRH5;                    /*!< Rcv Cntrl/Stat Endpt 5 High */
  __I  uint16_t RXCOUNT5;                   /*!< Rcv Byte Count Endpt 5      */
  __IO uint8_t  TXTYPE5;                    /*!< Host TX Config Type Endpt 5 */
  __IO uint8_t  TXINTERVAL5;                /*!< Host TX Interval Endpt 5    */
  __IO uint8_t  RXTYPE5;                    /*!< Host Config Rcv Type Endpt 5*/
  __IO uint8_t  RXINTERVAL5;                /*!< Host Rcv Polling Interval
                                                 Endpt 5                     */
  __I  uint16_t RSRVD0X15E;
  __IO uint16_t TXMAXP6;                    /*!< Maximum TX Data Endpt 6     */
  __IO uint8_t  TXCSRL6;                    /*!< TX Cntrl/Stat Endpt 6 Low   */
  __IO uint8_t  TXCSRH6;                    /*!< TX Cntrl/Stat Endpt 6 High  */
  __IO uint16_t RXMAXP6;                    /*!< Maximum Rcv Data Endpt 6    */
  __IO uint8_t  RXCSRL6;                    /*!< Rcv Cntrl/Stat Endpt 6 Low  */
  __IO uint8_t  RXCSRH6;                    /*!< Rcv Cntrl/Stat Endpt 6 High */
  __I  uint16_t RXCOUNT6;                   /*!< Rcv Byte Count Endpt 6      */
  __IO uint8_t  TXTYPE6;                    /*!< Host TX Config Type Endpt 6 */
  __IO uint8_t  TXINTERVAL6;                /*!< Host TX Interval Endpt 6    */
  __IO uint8_t  RXTYPE6;                    /*!< Host Config Rcv Type Endpt 6*/
  __IO uint8_t  RXINTERVAL6;                /*!< Host Rcv Polling Interval
                                                 Endpt 6                     */
  __I  uint16_t RSRVD0X16E;
  __IO uint16_t TXMAXP7;                    /*!< Maximum TX Data Endpt 7     */
  __IO uint8_t  TXCSRL7;                    /*!< TX Cntrl/Stat Endpt 7 Low   */
  __IO uint8_t  TXCSRH7;                    /*!< TX Cntrl/Stat Endpt 7 High  */
  __IO uint16_t RXMAXP7;                    /*!< Maximum Rcv Data Endpt 7    */
  __IO uint8_t  RXCSRL7;                    /*!< Rcv Cntrl/Stat Endpt 7 Low  */
  __IO uint8_t  RXCSRH7;                    /*!< Rcv Cntrl/Stat Endpt 7 High */
  __I  uint16_t RXCOUNT7;                   /*!< Rcv Byte Count Endpt 7      */
  __IO uint8_t  TXTYPE7;                    /*!< Host TX Config Type Endpt 7 */
  __IO uint8_t  TXINTERVAL7;                /*!< Host TX Interval Endpt 7    */
  __IO uint8_t  RXTYPE7;                    /*!< Host Config Rcv Type Endpt 7*/
  __IO uint8_t  RXINTERVAL7;                /*!< Host Rcv Polling Interval
                                                 Endpt 7                     */
  __I  uint16_t RSRVD0X17E;
  __I  uint16_t RSRVD32[(0x304 - 0x17e)/2 - 1];
  __IO uint16_t RQPKTCOUNT1;                /*!< Reqst Packet Count in Block
                                                 Transfer Endpt 1            */
  __I  uint16_t RSRVD0X306;
  __IO uint16_t RQPKTCOUNT2;                /*!< Reqst Packet Count in Block
                                                 Transfer Endpt 2            */
  __I  uint16_t RSRVD0X30A;
  __IO uint16_t RQPKTCOUNT3;                /*!< Reqst Packet Count in Block
                                                 Transfer Endpt 3            */
  __I  uint16_t RSRVD0X30E;
  __IO uint16_t RQPKTCOUNT4;                /*!< Reqst Packet Count in Block
                                                 Transfer Endpt 4            */
  __I  uint16_t RSRVD0X312;
  __IO uint16_t RQPKTCOUNT5;                /*!< Reqst Packet Count in Block
                                                 Transfer Endpt 5            */
  __I  uint16_t RSRVD0X316;
  __IO uint16_t RQPKTCOUNT6;                /*!< Reqst Packet Count in Block 
                                                 Transfer Endpt 6            */
  __I  uint16_t RESERVE0X31A;
  __IO uint16_t RQPKTCOUNT7;                /*!< Reqst Packet Count in Block
                                                 Transfer Endpt 7            */
  __I  uint16_t RSRVD39[(0x340 - 0x31C)/2 - 1];
  __IO uint16_t RXDPKTBUFDIS;               /*!< Rcv Dbl Packet Bffr Disable */
  __IO uint16_t TXDPKTBUFDIS;               /*!< TX Dbl Packet Bffr Disable  */
  
  __I  uint16_t RSRVD40[(0x400 - 0x342)/2 - 1];
  __IO uint32_t EPC;                        /*!< External Power Control     */
  __I  uint32_t EPCRIS;                     /*!< External Power Control
                                                  Raw Interrupt Status       */
  __IO uint32_t EPCIM;                      /*!< External Power Control
                                                  Interrupt Mask             */
  __IO uint32_t EPCISC;                     /*!< External Power Control
                                                  Interrupt Status and Clear */
  __I  uint32_t DRRIS;                      /*!< Device RESUME Raw Interrupt
                                                  Status                     */
  __IO uint32_t DRIM;                       /*!< Device RESUME Interrupt Msk*/
  __O  uint32_t DRISC;                      /*!< Device RESUME Interrupt
                                                  Status and Clear           */
  __IO uint32_t GPCS;                       /*!< General-Purpose Cntrl/Stat */
  __I  uint32_t RSRVD88[4];
  __IO uint32_t VDC;                        /*!< VBUS Droop Control         */
  __I  uint32_t VDCRIS;                     /*!< VBUS Droop Control
                                                  Raw Interrupt Status       */
  __IO uint32_t VDCIM;                      /*!< VBUS Droop Control
                                                  Interrupt Mask             */
  __IO uint32_t VDCISC;                     /*!< VBUS Droop Control
                                                  Interrupt Status and Clear */
  __I  uint32_t RSRVD0X440;
  __I  uint32_t IDVRIS;                     /*!< ID Valid Detect
                                                  Raw Interrupt Status       */
  __IO uint32_t IDVIM;                      /*!< ID Valid Detect
                                                  Interrupt Mask             */
  __IO uint32_t IDVISC;                     /*!< ID Valid Detect
                                                  Interrupt Status and Clear */
  __IO uint32_t DMASEL;                     /*!< DMA Select                 */
  __I  uint32_t RSRVD0X454[(0xFC0 - 0x450)/4 - 1];
  __I  uint32_t PP;                         /*!< USB Peripheral Properties  */
} __attribute__((packed)) USB_Type;


/**
  * @brief Register map for EEPROM peripheral (EEPROM)
  */

typedef struct {               /*!< EEPROM Structure                   */
  __I  uint32_t EESIZE;        /*!< EEPROM Size Information            */
  __IO uint32_t EEBLOCK;       /*!< EEPROM Current Block               */
  __IO uint32_t EEOFFSET;      /*!< EEPROM Current Offset              */
  __I  uint32_t RSRVD0X00C;
  __IO uint32_t EERDWR;        /*!< EEPROM Read-Write                  */
  __IO uint32_t EERDWRINC;     /*!< EEPROM Read-Write with Increment   */
  __I  uint32_t EEDONE;        /*!< EEPROM Done Status                 */
  __IO uint32_t EESUPP;        /*!< EEPROM Support Control and Status  */
  __IO uint32_t EEUNLOCK;      /*!< EEPROM Unlock                      */
  __I  uint32_t RSRVD0x024[3];
  __IO uint32_t EEPROT;        /*!< EEPROM Protection                  */
  __IO uint32_t EEPASS0;       /*!< EEPROM Password                    */
  __IO uint32_t EEPASS1;       /*!< EEPROM Password                    */
  __IO uint32_t EEPASS2;       /*!< EEPROM Password                    */
  __IO uint32_t EEINT;         /*!< EEPROM Interrupt                   */
  __I  uint32_t RSRVD0X044[3];
  __IO uint32_t EEHIDE;        /*!< EEPROM Block Hide                  */
  __I  uint32_t RSRVD0X054[11];
  __IO uint32_t EEDBGME;       /*!< EEPROM Debug Mass Erase            */
  __I  uint32_t RSRVD0X084[(0xFC0 - 0x080)/4 - 1];
  __I  uint32_t EEPROMPP;      /*!< EEPROM                             */
} EEPROM_Type;

/**
  * @brief Register map for SYSEXC peripheral (SYSEXC)
  */

typedef struct {            /*!< SYSEXC Structure                       */
  __I  uint32_t RIS;        /*!< System Exception Raw Interrupt Status  */
  __IO uint32_t IM;         /*!< System Exception Interrupt Mask        */
  __I  uint32_t MIS;        /*!< System Exception Raw Interrupt Status  */
  __O  uint32_t IC;         /*!< System Exception Interrupt Clear       */
} SYSEXC_Type;

/**
  * @brief Register map for HIB peripheral (HIB)
  */

typedef struct {                 /*!< HIB Structure                     */
  __I  uint32_t RTCC;            /*!< Hibernation RTC Counter           */
  __IO uint32_t RTCM0;           /*!< Hibernation RTC Match 0           */
  __I  uint32_t RSRVD0X008;
  __IO uint32_t RTCLD;           /*!< Hibernation RTC Load              */
  __IO uint32_t CTL;             /*!< Hibernation Control               */
  __IO uint32_t IM;              /*!< Hibernation Interrupt Mask        */
  __I  uint32_t RIS;             /*!< Hibernation Raw Interrupt Status  */
  __I  uint32_t MIS;             /*!< Hibernation Mskd Interrupt Status */
  __IO uint32_t IC;              /*!< Hibernation Interrupt Clear       */
  __IO uint32_t RTCT;            /*!< Hibernation RTC Trim              */
  __IO uint32_t RTCSS;           /*!< Hibernation RTC Sub Seconds       */
  __I  uint32_t RSRVD0X02C;
  __IO uint32_t DATA[16];        /*!< Hibernation Data, 16 double words */
} HIB_Type;

/**
  * @brief Register map for FLASH_CTRL peripheral (FLASH_CTRL)
  */

typedef struct {                 /*!< FLASH_CTRL Structure              */
  __IO uint32_t FMA;             /*!< Flash Memory Address              */
  __IO uint32_t FMD;             /*!< Flash Memory Data                 */
  __IO uint32_t FMC;             /*!< Flash Memory Control              */
  __I  uint32_t FCRIS;           /*!< Flash Controller Raw Interrupt Sta*/
  __IO uint32_t FCIM;            /*!< Flash Controller Interrupt Mask   */
  __IO uint32_t FCMISC;          /*!< Flash Controller Masked Interrupt */
  __I  uint32_t RSRVD0X018[2];
  __IO uint32_t FMC2;            /*!< Flash Memory Control 2            */
  __I  uint32_t RSRVD0X024[3];
  __IO uint32_t FWBVAL;          /*!< Flash Write Buffer Valid          */
  __I  uint32_t RSRVD0X034[(0x100 - 0x030)/4 - 1];
  __IO uint32_t FWBN[32];        /*!< Flash Write Buffer n              */
  __I  uint32_t RSRVD0X180[(0xFC0 - 0x17C)/4 - 1];
  __I  uint32_t FSIZE;           /*!< Flash Size                        */
  __I  uint32_t SSIZE;           /*!< SRAM Size                         */
  __I  uint32_t RSRVD0XFC8;
  __IO uint32_t ROMSWMAP;        /*!< ROM Software Map                  */
  __I  uint32_t RSRVD0XFD0[(0XE0F0 - 0XDFCC)/4 - 1];
  __IO uint32_t RMCTL;           /*!< ROM Control                       */
  __I  uint32_t RSRVD0X0F4[(0X130 - 0X0F0)/4 - 1];
  __IO uint32_t FMPRE0_ALT;      /*!< Flash Mem Protection Read Enbl 0  */
  __IO uint32_t FMPPE0_ALT;      /*!< Flash Mem Protection Prog Enbl 0  */
  __I  uint32_t RSRVD0X138[(0X1D0 - 0X134)/4 - 1];
  __IO uint32_t BOOTCFG;         /*!< Boot Configuration                */
  __I  uint32_t RSRVD0X1D4[3];
  __IO uint32_t USERREG0;        /*!< User Register 0                   */
  __IO uint32_t USERREG1;        /*!< User Register 1                   */
  __IO uint32_t USERREG2;        /*!< User Register 2                   */
  __IO uint32_t USERREG3;        /*!< User Register 3                   */
  __I  uint32_t RSRVD0X1F0[(0X200 - 0X1EC)/4 -1];
  __IO uint32_t FMPRE0;          /*!< Flash Mem Protection Read Enbl 0  */
  __IO uint32_t FMPRE1;          /*!< Flash Mem Protection Read Enbl 1  */
  __IO uint32_t FMPRE2;          /*!< Flash Mem Protection Read Enbl 2  */
  __IO uint32_t FMPRE3;          /*!< Flash Mem Protection Read Enbl 3  */
  __I  uint32_t RSRVD0X300[(0X400 - 0X20C)/4 - 1];
  __IO uint32_t FMPPE0;          /*!< Flash Mem Protection Prog Enbl 0  */
  __IO uint32_t FMPPE1;          /*!< Flash Mem Protection Prog Enbl 1  */
  __IO uint32_t FMPPE2;          /*!< Flash Mem Protection Prog Enbl 2  */
  __IO uint32_t FMPPE3;          /*!< Flash Mem Protection Prog Enbl 3  */
} FLASH_CTRL_Type;

/**
  * @brief Register map for SYSCTL peripheral (SYSCTL)
  */

typedef struct {            /*!< SYSCTL Structure                            */
  __I  uint32_t DID0;       /*!< Device Identification 0                     */
  __I  uint32_t DID1;       /*!< Device Identification 1                     */
  __I  uint32_t DC0;        /*!< Device Capabilities 0                       */
  __I  uint32_t RSRVD0x00C;
  __I  uint32_t DC1;        /*!< Device Capabilities 1                       */
  __I  uint32_t DC2;        /*!< Device Capabilities 2                       */
  __I  uint32_t DC3;        /*!< Device Capabilities 3                       */
  __I  uint32_t DC4;        /*!< Device Capabilities 4                       */
  __I  uint32_t DC5;        /*!< Device Capabilities 5                       */
  __I  uint32_t DC6;        /*!< Device Capabilities 6                       */
  __I  uint32_t DC7;        /*!< Device Capabilities 7                       */
  __I  uint32_t DC8;        /*!< Device Capabilities 8 ADC Channels          */
  __IO uint32_t PBORCTL;    /*!< Brown-Out Reset Control                     */
  __I  uint32_t RSRVD1[3];
  __I  uint32_t SRCR0;      /*!< Software Reset Control 0                    */
  __I  uint32_t SRCR1;      /*!< Software Reset Control 1                    */
  __I  uint32_t SRCR2;      /*!< Software Reset Control 2                    */
  __I  uint32_t RSRVD0x04C;
  __I  uint32_t RIS;        /*!< Raw Interrupt Status                        */
  __IO uint32_t IMC;        /*!< Interrupt Mask Control                      */
  __IO uint32_t MISC;       /*!< Masked Interrupt Status and Clear           */
  __IO uint32_t RESC;       /*!< Reset Cause                                 */
  __IO uint32_t RCC;        /*!< Run-Mode Clock Configuration                */
  __I  uint32_t RSRVD3[2];
  __IO uint32_t GPIOHBCTL;  /*!< GPIO High-Performance Bus Control           */
  __IO uint32_t RCC2;       /*!< Run-Mode Clock Configuration 2              */
  __I  uint32_t RSRVD4[2];
  __IO uint32_t MOSCCTL;    /*!< Main Oscillator Control                     */
  __I  uint32_t RSRVD5[(0x100 - 0x07C)/4 - 1];
  __IO uint32_t RCGC0;      /*!< Run Mode Clock Gating Control Register 0    */
  __IO uint32_t RCGC1;      /*!< Run Mode Clock Gating Control Register 1    */
  __IO uint32_t RCGC2;      /*!< Run Mode Clock Gating Control Register 2    */
  __I  uint32_t RSRVD6;
  __IO uint32_t SCGC0;      /*!< Sleep Mode Clock Gating Control Register 0  */
  __IO uint32_t SCGC1;      /*!< Sleep Mode Clock Gating Control Register 1  */
  __IO uint32_t SCGC2;      /*!< Sleep Mode Clock Gating Control Register 2  */
  __I  uint32_t RSRVD7;
  __IO uint32_t DCGC0;      /*!< Deep-Sleep Clock Gating Control Register    */
  __IO uint32_t DCGC1;      /*!< Deep-Sleep Clock Gating Control Register    */
  __IO uint32_t DCGC2;      /*!< Deep-Sleep Clock Gating Control Register    */
  __I  uint32_t RSRVD8[(0x144 - 0x128)/4 - 1];
  __IO uint32_t DSLPCLKCFG; /*!< Deep-Sleep Clock Configuration              */
  __I  uint32_t RSRVD9;
  __I  uint32_t SYSPROP;    /*!< System Properties                           */
  __IO uint32_t PIOSCCAL;   /*!< Precision Internal Oscillator Calibration   */
  __I  uint32_t PIOSCSTAT;  /*!< Precision Internal Oscillator Statistics    */
  __I  uint32_t RSRVD10[2];
  __I  uint32_t PLLFREQ0;   /*!< PLL Frequency 0                             */
  __I  uint32_t PLLFREQ1;   /*!< PLL Frequency 1                             */
  __I  uint32_t PLLSTAT;    /*!< PLL Status                                  */
  __I  uint32_t RSRVD11[(0x188 - 0x168)/4 - 1];
  __IO uint32_t SLPPWRCFG;  /*!< Sleep Power Configuration                   */
  __IO uint32_t DSLPPWRCFG; /*!< Deep-Sleep Power Configuration              */
  __I  uint32_t DC9;        /*!< Device Capabilities 9 ADC Digital Comparator*/
  __I  uint32_t RSRVD12[(0x1A0 - 0x190)/4 - 1];
  __I  uint32_t NVMSTAT;    /*!< Non-Volatile Memory Information             */
  __I  uint32_t RSRVD99[(0x1B4 - 0x1A0)/4 - 1];
  __IO uint32_t LDOSPCTL;   /*!< LDO Sleep Power Control                     */
  __I  uint32_t LDOSPCAL;   /*!< LDO Sleep Power Calibration                 */
  __IO uint32_t LDODPCTL;   /*!< LDO Deep-Sleep Power Control                */
  __I  uint32_t LDODPCAL;   /*!< LDO Deep-Sleep Power Calibration            */
  __I  uint32_t RSRVD98[(0x1CC - 0x1C0)/4 - 1];
  __I  uint32_t SDPMST;     /*!< Sleep/Deep-Sleep Power Mode Status          */
  __I  uint32_t RSRVD13[(0x300 - 0x1CC)/4 - 1];
  __I  uint32_t PPWD;       /*!< Watchdog Timer Peripheral Present           */
  __I  uint32_t PPTIMER;    /*!< Timer Peripheral Present                    */
  __I  uint32_t PPGPIO;     /*!< GPIO Peripheral Present                     */
  __I  uint32_t PPDMA;      /*!< Micro DMA Peripheral Present                */
  __I  uint32_t RSRVD0X0310;
  __I  uint32_t PPHIB;      /*!< Hibernation Peripheral Present              */
  __I  uint32_t PPUART;     /*!< UART Peripheral Present                     */
  __I  uint32_t PPSSI;      /*!< SSI Peripheral Present                      */
  __I  uint32_t PPI2C;      /*!< I2C Peripheral Present                      */
  __I  uint32_t RSRVD0X0324;
  __I  uint32_t PPUSB;      /*!< USB Peripheral Present                      */
  __I  uint32_t RSRVD16[2];
  __I  uint32_t PPCAN;      /*!< Controller Area Network Peripheral Present  */
  __I  uint32_t PPADC;      /*!< ADC Peripheral Present                      */
  __I  uint32_t PPACMP;     /*!< Analog Comparator Peripheral Present        */
  __I  uint32_t PPPWM;      /*!< PWM Peripheral Present                      */
  __I  uint32_t PPQEI;      /*!< Quadrature Encoder Peripheral Present       */
  __I  uint32_t RSRVD17[4];
  __I  uint32_t PPEEPROM;   /*!< EEPROM Peripheral Present                   */
  __I  uint32_t PPWTIMER;   /*!< Wide Timer Peripheral Present               */
  __I  uint32_t RSRVD18[(0x500 - 0x35C)/4 - 1];
  __IO uint32_t SRWD;       /*!< Watchdog Timer Software Reset               */
  __IO uint32_t SRTIMER;    /*!< Timer Software Reset                        */
  __IO uint32_t SRGPIO;     /*!< GPIO Software Reset                         */
  __IO uint32_t SRDMA;      /*!< Micro DMA Software Reset                    */
  __I  uint32_t RSRVD0X0510;
  __IO uint32_t SRHIB;      /*!< Hibernation Software Reset                  */
  __IO uint32_t SRUART;     /*!< UART Software Reset                         */
  __IO uint32_t SRSSI;      /*!< SSI Software Reset                          */
  __IO uint32_t SRI2C;      /*!< I2C Software Reset                          */
  __I  uint32_t RSRVD0X0524;
  __IO uint32_t SRUSB;      /*!< USB Software Reset                          */
  __I  uint32_t RSRVD21[2];
  __IO uint32_t SRCAN;      /*!< Controller Area Network Software Reset      */
  __IO uint32_t SRADC;      /*!< ADC Software Reset                          */
  __IO uint32_t SRACMP;     /*!< Analog Comparator Software Reset            */
  __IO uint32_t SRPWM;      /*!< PWM Software Reset                          */
  __IO uint32_t SRQEI;      /*!< Quadrature Encoder Interface Software Reset */
  __I  uint32_t RSRVD22[(0x558 - 0x544)/4 - 1];
  __IO uint32_t SREEPROM;   /*!< EEPROM Software Reset                       */
  __IO uint32_t SRWTIMER;   /*!< Wide Timer Software Reset                   */
  __I  uint32_t RSRVD23[(0x600 - 0x55C)/4 - 1];
  __IO uint32_t RCGCWD;     /*!< Watchdog Timer Run Mode Clock Gating Cntrl  */
  __IO uint32_t RCGCTIMER;  /*!< Timer Run Mode Clock Gating Control         */
  __IO uint32_t RCGCGPIO;   /*!< GPIO Run Mode Clock Gating Control          */
  __IO uint32_t RCGCDMA;    /*!< Micro DMA Run Mode Clock Gating Control     */
  __I  uint32_t RSRVD24;
  __IO uint32_t RCGCHIB;    /*!< Hibernation Run Mode Clock Gating Control   */
  __IO uint32_t RCGCUART;   /*!< UART Run Mode Clock Gating Control          */
  __IO uint32_t RCGCSSI;    /*!< SSI Run Mode Clock Gating Control           */
  __IO uint32_t RCGCI2C;    /*!< I2C Run Mode Clock Gating Control           */
  __I  uint32_t RSRVD0X0624;
  __IO uint32_t RCGCUSB;    /*!< USB Run Mode Clock Gating Control           */
  __I  uint32_t RSRVD26[2];
  __IO uint32_t RCGCCAN;    /*!< Controller Area Network Run Mode Clk Gating */
  __IO uint32_t RCGCADC;    /*!< ADC Run Mode Clock Gating Control           */
  __IO uint32_t RCGCACMP;   /*!< Analog Comparator Run Mode Clock Gating     */
  __IO uint32_t RCGCPWM;    /*!< PWM Run Mode Clock Gating Control           */
  __IO uint32_t RCGCQEI;    /*!< Quadrature Encoder Run Mode Clock Gating    */
  __I  uint32_t RSRVD27[4];
  __IO uint32_t RCGCEEPROM; /*!< EEPROM Run Mode Clock Gating Control        */
  __IO uint32_t RCGCWTIMER; /*!< Wide Timer Run Mode Clock Gating Control    */
  __I  uint32_t RSRVD28[(0x700 - 0x65C)/4 - 1];
  __IO uint32_t SCGCWD;     /*!< Watchdog Timer Sleep Mode Clock Gating      */
  __IO uint32_t SCGCTIMER;  /*!< Timer Sleep Mode Clock Gating Control       */
  __IO uint32_t SCGCGPIO;   /*!< GPIO Sleep Mode Clock Gating Control        */
  __IO uint32_t SCGCDMA;    /*!< Micro DMA Sleep Mode Clock Gating Control   */
  __I  uint32_t RSRVD0X0710;
  __IO uint32_t SCGCHIB;    /*!< Hibernation Sleep Mode Clock Gating Control */
  __IO uint32_t SCGCUART;   /*!< UART Sleep Mode Clock Gating Control        */
  __IO uint32_t SCGCSSI;    /*!< SSI Sleep Mode Clock Gating Control         */
  __IO uint32_t SCGCI2C;    /*!< I2C Sleep Mode Clock Gating Control         */
  __I  uint32_t RSRVD0X0724;
  __IO uint32_t SCGCUSB;    /*!< USB Sleep Mode Clock Gating Control         */
  __I  uint32_t RSRVD31[2];
  __IO uint32_t SCGCCAN;    /*!< CAN Sleep Mode Clock Gating Control         */
  __IO uint32_t SCGCADC;    /*!< ADC Sleep Mode Clock Gating Control         */
  __IO uint32_t SCGCACMP;   /*!< Analog Comparator Sleep Mode Clock Gating   */
  __IO uint32_t SCGCPWM;    /*!< PWM Sleep Mode Clock Gating Control         */
  __IO uint32_t SCGCQEI;    /*!< Quadrature Encoder Sleep Mode Clock Gating  */
  __I  uint32_t RSRVD32[(0x758 - 0x744)/4 - 1];
  __IO uint32_t SCGCEEPROM; /*!< EEPROM Sleep Mode Clock Gating Control      */
  __IO uint32_t SCGCWTIMER; /*!< Wide Timer Sleep Mode Clock Gating Control  */
  __I  uint32_t RSRVD33[(0x800 - 0x75C)/4 - 1];
  __IO uint32_t DCGCWD;     /*!< Watchdog Timer Deep-Sleep Clock Gating      */
  __IO uint32_t DCGCTIMER;  /*!< Timer Deep-Sleep Mode Clock Gating Control  */
  __IO uint32_t DCGCGPIO;   /*!< GPIO Deep-Sleep Mode Clock Gating Control   */
  __IO uint32_t DCGCDMA;    /*!< Micro DMA Deep-Sleep Clock Gating Control   */
  __I  uint32_t RSRVD0X0810;
  __IO uint32_t DCGCHIB;    /*!< Hibernation Deep-Sleep Clock Gating Control */
  __IO uint32_t DCGCUART;   /*!< UART Deep-Sleep Mode Clock Gating Control   */
  __IO uint32_t DCGCSSI;    /*!< SSI Deep-Sleep Mode Clock Gating Control    */
  __IO uint32_t DCGCI2C;    /*!< I2C Deep-Sleep Mode Clock Gating Control    */
  __I  uint32_t RSRVD0X0824;
  __IO uint32_t DCGCUSB;    /*!< USB Deep-Sleep Mode Clock Gating Control    */
  __I  uint32_t RSRVD36[2];
  __IO uint32_t DCGCCAN;    /*!< CAN Deep-Sleep Mode Clock Gating Control    */
  __IO uint32_t DCGCADC;    /*!< ADC Deep-Sleep Mode Clock Gating Control    */
  __IO uint32_t DCGCACMP;   /*!< Analog Comparator Deep-Sleep Clock Gating   */
  __IO uint32_t DCGCPWM;    /*!< PWM Deep-Sleep Mode Clock Gating Control    */
  __IO uint32_t DCGCQEI;    /*!< Quadrature Encoder Deep-Sleep Clock Gating  */
  __I  uint32_t RSRVD37[4];
  __IO uint32_t DCGCEEPROM; /*!< EEPROM Deep-Sleep Mode Clock Gating Control */
  __IO uint32_t DCGCWTIMER; /*!< Wide Timer Deep-Sleep Clock Gating Control  */
  __I  uint32_t RSRVD38[(0xA00 - 0x85C)/4 - 1];
  __I  uint32_t PRWD;       /*!< Watchdog Timer Peripheral Ready             */
  __I  uint32_t PRTIMER;    /*!< Timer Peripheral Ready                      */
  __I  uint32_t PRGPIO;     /*!< GPIO Peripheral Ready                       */
  __I  uint32_t PRDMA;      /*!< Micro DMA Peripheral Ready                  */
  __I  uint32_t RSRVD0X0A10;
  __I  uint32_t PRHIB;      /*!< Hibernation Peripheral Ready                */
  __I  uint32_t PRUART;     /*!< UART Peripheral Ready                       */
  __I  uint32_t PRSSI;      /*!< SSI Peripheral Ready                        */
  __I  uint32_t PRI2C;      /*!< I2C Peripheral Ready                        */
  __I  uint32_t RSRVD0X0A24;
  __I  uint32_t PRUSB;      /*!< USB Peripheral Ready                        */
  __I  uint32_t RSRVD46[2];
  __I  uint32_t PRCAN;      /*!< Controller Area Network Peripheral Ready    */
  __I  uint32_t PRADC;      /*!< ADC Peripheral Ready                        */
  __I  uint32_t PRACMP;     /*!< Analog Comparator Peripheral Ready          */
  __I  uint32_t PRPWM;      /*!< PWM Peripheral Ready                        */
  __I  uint32_t PRQEI;      /*!< Quadrature Encoder Interface Ready          */
  __I  uint32_t RSRVD47[4];
  __I  uint32_t PREEPROM;   /*!< EEPROM Peripheral Ready                     */
  __I  uint32_t PRWTIMER;   /*!< Wide Timer Peripheral Ready                 */
} SYSCTL_Type;

/**
  * @brief Register map for UDMA peripheral (UDMA)
  */
typedef struct {            /*!< UDMA Structure                             */
  __I  uint32_t STAT;       /*!< DMA Status                                 */
  __O  uint32_t CFG;        /*!< DMA Configuration                          */
  __IO uint32_t CTLBASE;    /*!< DMA Channel Control Base Pointer           */
  __I  uint32_t ALTBASE;    /*!< DMA Alternate Channel Control Base Pointer */
  __I  uint32_t WAITSTAT;   /*!< DMA Channel Wait-on-Request Status         */
  __O  uint32_t SWREQ;      /*!< DMA Channel Software Request               */
  __IO uint32_t USEBURSTSET;/*!< DMA Channel Useburst Set                   */
  __O  uint32_t USEBURSTCLR;/*!< DMA Channel Useburst Clear                 */
  __IO uint32_t REQMASKSET; /*!< DMA Channel Request Mask Set               */
  __O  uint32_t REQMASKCLR; /*!< DMA Channel Request Mask Clear             */
  __IO uint32_t ENASET;     /*!< DMA Channel Enable Set                     */
  __O  uint32_t ENACLR;     /*!< DMA Channel Enable Clear                   */
  __IO uint32_t ALTSET;     /*!< DMA Channel Primary Alternate Set          */
  __O  uint32_t ALTCLR;     /*!< DMA Channel Primary Alternate Clear        */
  __IO uint32_t PRIOSET;    /*!< DMA Channel Priority Set                   */
  __O  uint32_t PRIOCLR;    /*!< DMA Channel Priority Clear                 */
  __I  uint32_t RSRVD0X040[3];
  __IO uint32_t ERRCLR;     /*!< DMA Bus Error Clear                        */
  __I  uint32_t RSRVD0X050[(0X500 - 0X04C)/4 - 1];
  __IO uint32_t CHASGN;     /*!< DMA Channel Assignment                     */
  __IO uint32_t CHIS;       /*!< DMA Channel Interrupt Status               */
  __I  uint32_t RSRVD0X508[2];
  __IO uint32_t CHMAP0;     /*!< DMA Channel Map Select 0                   */
  __IO uint32_t CHMAP1;     /*!< DMA Channel Map Select 1                   */
  __IO uint32_t CHMAP2;     /*!< DMA Channel Map Select 2                   */
  __IO uint32_t CHMAP3;     /*!< DMA Channel Map Select 3                   */
} UDMA_Type;

/* ====================================================================== */
/* ===========              Peripheral memory map             =========== */
/* ====================================================================== */

#define WATCHDOG0_BASE    0x40000000UL
#define WATCHDOG1_BASE    0x40001000UL
#define GPIOA_BASE        0x40004000UL
#define GPIOB_BASE        0x40005000UL
#define GPIOC_BASE        0x40006000UL
#define GPIOD_BASE        0x40007000UL
#define SSI0_BASE         0x40008000UL
#define SSI1_BASE         0x40009000UL
#define SSI2_BASE         0x4000A000UL
#define SSI3_BASE         0x4000B000UL
#define UART0_BASE        0x4000C000UL
#define UART1_BASE        0x4000D000UL
#define UART2_BASE        0x4000E000UL
#define UART3_BASE        0x4000F000UL
#define UART4_BASE        0x40010000UL
#define UART5_BASE        0x40011000UL
#define UART6_BASE        0x40012000UL
#define UART7_BASE        0x40013000UL
#define I2C0_BASE         0x40020000UL
#define I2C1_BASE         0x40021000UL
#define I2C2_BASE         0x40022000UL
#define I2C3_BASE         0x40023000UL
#define GPIOE_BASE        0x40024000UL
#define GPIOF_BASE        0x40025000UL
#define PWM0_BASE         0x40028000UL
#define PWM1_BASE         0x40029000UL
#define QEI0_BASE         0x4002C000UL
#define QEI1_BASE         0x4002D000UL
#define TIMER0_BASE       0x40030000UL
#define TIMER1_BASE       0x40031000UL
#define TIMER2_BASE       0x40032000UL
#define TIMER3_BASE       0x40033000UL
#define TIMER4_BASE       0x40034000UL
#define TIMER5_BASE       0x40035000UL
#define WTIMER0_BASE      0x40036000UL
#define WTIMER1_BASE      0x40037000UL
#define ADC0_BASE         0x40038000UL
#define ADC1_BASE         0x40039000UL
#define COMP_BASE         0x4003C000UL
#define CAN0_BASE         0x40040000UL
#define CAN1_BASE         0x40041000UL
#define WTIMER2_BASE      0x4004C000UL
#define WTIMER3_BASE      0x4004D000UL
#define WTIMER4_BASE      0x4004E000UL
#define WTIMER5_BASE      0x4004F000UL
#define USB0_BASE         0x40050000UL
#define GPIOA_AHB_BASE    0x40058000UL
#define GPIOB_AHB_BASE    0x40059000UL
#define GPIOC_AHB_BASE    0x4005A000UL
#define GPIOD_AHB_BASE    0x4005B000UL
#define GPIOE_AHB_BASE    0x4005C000UL
#define GPIOF_AHB_BASE    0x4005D000UL
#define EEPROM_BASE       0x400AF000UL
#define SYSEXC_BASE       0x400F9000UL
#define HIB_BASE          0x400FC000UL
#define FLASH_CTRL_BASE   0x400FD000UL
#define SYSCTL_BASE       0x400FE000UL
#define UDMA_BASE         0x400FF000UL

/* ====================================================================== */
/* ================        Peripheral declaration        ================ */
/* ====================================================================== */

#define WATCHDOG0     ((WATCHDOG_Type   *) WATCHDOG0_BASE)
#define WATCHDOG1     ((WATCHDOG_Type   *) WATCHDOG1_BASE)
#define GPIOA         ((GPIO_Type       *) GPIOA_BASE)
#define GPIOB         ((GPIO_Type       *) GPIOB_BASE)
#define GPIOC         ((GPIO_Type       *) GPIOC_BASE)
#define GPIOD         ((GPIO_Type       *) GPIOD_BASE)
#define SSI0          ((SSI_Type        *) SSI0_BASE)
#define SSI1          ((SSI_Type        *) SSI1_BASE)
#define SSI2          ((SSI_Type        *) SSI2_BASE)
#define SSI3          ((SSI_Type        *) SSI3_BASE)
#define UART0         ((UART_Type       *) UART0_BASE)
#define UART1         ((UART_Type       *) UART1_BASE)
#define UART2         ((UART_Type       *) UART2_BASE)
#define UART3         ((UART_Type       *) UART3_BASE)
#define UART4         ((UART_Type       *) UART4_BASE)
#define UART5         ((UART_Type       *) UART5_BASE)
#define UART6         ((UART_Type       *) UART6_BASE)
#define UART7         ((UART_Type       *) UART7_BASE)
#define I2C0          ((I2C_Type        *) I2C0_BASE)
#define I2C1          ((I2C_Type        *) I2C1_BASE)
#define I2C2          ((I2C_Type        *) I2C2_BASE)
#define I2C3          ((I2C_Type        *) I2C3_BASE)
#define GPIOE         ((GPIO_Type       *) GPIOE_BASE)
#define GPIOF         ((GPIO_Type       *) GPIOF_BASE)
#define PWM0          ((PWM_Type        *) PWM0_BASE)
#define PWM1          ((PWM_Type        *) PWM1_BASE)
#define QEI0          ((QEI_Type        *) QEI0_BASE)
#define QEI1          ((QEI_Type        *) QEI1_BASE)
#define TIMER0        ((GPTM_Type       *) TIMER0_BASE)
#define TIMER1        ((GPTM_Type       *) TIMER1_BASE)
#define TIMER2        ((GPTM_Type       *) TIMER2_BASE)
#define TIMER3        ((GPTM_Type       *) TIMER3_BASE)
#define TIMER4        ((GPTM_Type       *) TIMER4_BASE)
#define TIMER5        ((GPTM_Type       *) TIMER5_BASE)
#define WTIMER0       ((GPTM_Type       *) WTIMER0_BASE)
#define WTIMER1       ((GPTM_Type       *) WTIMER1_BASE)
#define ADC0          ((ADC_Type        *) ADC0_BASE)
#define ADC1          ((ADC_Type        *) ADC1_BASE)
#define COMP          ((COMP_Type       *) COMP_BASE)
#define CAN0          ((CAN_Type        *) CAN0_BASE)
#define CAN1          ((CAN_Type        *) CAN1_BASE)
#define WTIMER2       ((GPTM_Type       *) WTIMER2_BASE)
#define WTIMER3       ((GPTM_Type       *) WTIMER3_BASE)
#define WTIMER4       ((GPTM_Type       *) WTIMER4_BASE)
#define WTIMER5       ((GPTM_Type       *) WTIMER5_BASE)
#define USB0          ((USB_Type        *) USB0_BASE)
#define GPIOA_AHB     ((GPIO_Type       *) GPIOA_AHB_BASE)
#define GPIOB_AHB     ((GPIO_Type       *) GPIOB_AHB_BASE)
#define GPIOC_AHB     ((GPIO_Type       *) GPIOC_AHB_BASE)
#define GPIOD_AHB     ((GPIO_Type       *) GPIOD_AHB_BASE)
#define GPIOE_AHB     ((GPIO_Type       *) GPIOE_AHB_BASE)
#define GPIOF_AHB     ((GPIO_Type       *) GPIOF_AHB_BASE)
#define EEPROM        ((EEPROM_Type     *) EEPROM_BASE)
#define SYSEXC        ((SYSEXC_Type     *) SYSEXC_BASE)
#define HIB           ((HIB_Type        *) HIB_BASE)
#define FLASH_CTRL    ((FLASH_CTRL_Type *) FLASH_CTRL_BASE)
#define SYSCTL        ((SYSCTL_Type     *) SYSCTL_BASE)
#define UDMA          ((UDMA_Type       *) UDMA_BASE)

/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group TM4C123GH6PM */
/** @} */ /* End of group TI */

#ifdef __cplusplus
}
#endif

#endif  /* TM4C123GH6PM_H */

