/**************************************************************************//**
 * @file     system_TM4C.c
 * @brief    CMSIS Cortex-M4 Device Peripheral Access Layer Source File for
 *           TI Stellaris Devices
 * @version  V3.00
 * @date     19. December 2011
 * @date     Last modified: 2014-09-04T14:59:17-0400
 *
 * @brief    Changed part name from LM4F to TM4C KJH
 *
 * @note
 * Copyright (C) 2010-2011 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

#include <stdint.h>
#include "TM4C123GH6PM.h"

//--------------------- Clock Configuration ----------------------------------
//
// The following controls whether the system clock is configured in the
// SystemInit() function.  If it is defined to be 1 then the system clock
// will be configured according to the macros in the rest of this file.
// If it is defined to be 0, then the system clock configuration is bypassed.
//
#define CLOCK_SETUP 1

//********************************* RCC ***************************************
//  Run-Mode Clock Configuration (RCC)
//
// The following value is the system clock divisor.  This will be applied if
// USESYSDIV (see below) is enabled.  The valid range of dividers is 2-16.
//
#define INIT_RCC_SYSDIV 4

//
// If the value is 1, then the system clock divider is used, and the value of
// the system divider is defined by SYSDIV (see above).  If the value is 0, then
// the system clock divider is not used.
//
#define INIT_RCC_USESYSDIV 1

//
// If the following value is 1, then the PLL is powered down.  Keep this value
// as 1 if you do not need to use the PLL.  In this case, BYPASS (see below)
// must also be set to 1.  If you are using the PLL, then this value must be
// set to 0.
//
#define INIT_RCC_PWRDN 0

//
// Set BYPASS to 1 to bypass the PLL and not use it for the system clock.
// You _must_ set this to 1 if PWRDN (above) is set to 1. Setting BYPASS
// to 0 causes the PLL output to be used for the system clock.
//
#define INIT_RCC_BYPASS 0

//
// Specify the frequency of the external crystal used with the main
// oscillator. If an external crystal is used, then this value _must_ be set
// to match the value of the crystal.
//
//           6: 4.0000 MHz         17: 12.0 MHz    
//           7: 4.096 MHz          18: 12.288 MHz  
//           8: 4.9152 MHz         19: 13.56 MHz   
//           9: 5.0000 MHz         20: 14.31818 MHz
//          10: 5.12 MHz           21: 16.0 MHz    
//          11: 6.0000 MHz         22: 16.384 MHz  
//          12: 6.144 MHz          23: 18.0 MHz    
//          13: 7.3728 MHz         24: 20.0 MHz    
//          14: 8.0000 MHz         25: 24.0 MHz    
//          15: 8.192 MHz          26: 25.0 MHz    
//          16: 10.0 MHz
//              
#define INIT_RCC_XTAL 21

//
//  Select the clock source for the system clock.
//    0: MOSC Main crystal oscillator
//    1: IOSC Internal 16 MHz oscillator
//    2: IOSC/4 Internal oscillator/4 (necessary if IOSC used as input to PLL)
//    3: 30kHz 30 kHz internal oscillator
//
#define INIT_RCC_OSCSRC 0

//
// Set the MOSCDIS to 1 to power down the main oscillator.
//
#define INIT_RCC_MOSCDIS 0

//********************************* RCC2 **************************************
//
//   Run-Mode Clock Configuration 2 (RCC2)
//
// Set to 1 to use the RCC2 register, overriding some of the fields in the
// RCC register.
//
#define INIT_RCC2_USERCC2 0

// 
// Specify the divisor used to generate the system clock from either the PLL
// output or the oscillator. The valid range of divisors is 2-64.
// USESYSDIV in RCC must be enabled, else this value is ignored.
//
#define INIT_RCC_SYSDIV2 3

//
// If DIV400 is set to 1 then the clock divider divides the PLL 400 MHz output
// (rather than the 200 MHz output) and the SYSDIV2LSB bit is used as the LSB
// of a 7-bit clock divider. If DIV400 is set to 0 then SYSDIV2LSB is ignored.
//
#define INIT_RCC_DIV400     0
#define INIT_RCC_SYSDIV2LSB 0

//  
//  Select the clock source for the system clock. Note that when using RCC2
//  there is one additional choice, the 32kHz oscillator.
//    0: MOSC Main crystal oscillator
//    1: IOSC Internal 16 MHz oscillator
//    2: IOSC/4 Internal oscillator/4 (necessary if IOSC used as input to PLL)
//    3: 30kHz 30 kHz internal oscillator
//    7: 32kHz 32.768 kHz external crystal oscillator
//
#define INIT_RCC_OSCSRC2 0

//-------- <<< end of configuration section >>> ------------------------------
#define DEFAULT_RCC         (0x078E3AD1uL)
#define RCC_SYSDIV_MSK      (0xFuL                    << 23)
#define CFG_RCC_SYSDIV      (((INIT_RCC_SYSDIV - 1)   << 23) & RCC_SYSDIV_MSK)
#define RCC_USESYSDIV_MSK   (0x1uL                    << 22)
#define CFG_RCC_USESYSDIV   (INIT_RCC_USESYSDIV       << 22)
#define RCC_PWRDN_MSK       (0x1uL                    << 13)
#define CFG_RCC_PWRDN       (INIT_RCC_PWRDN           << 13)
#define RCC_BYPASS_MSK      (1uL                      << 11)
#if (CFG_RCC_PWRDN)
#  if (! CFG_RCC_BYPASS)
#    warning "PLL bypass forced on, PLL is not powered!"
#  endif
#  define CFG_RCC_BYPASS    (1uL                      << 11)
#else
#  define CFG_RCC_BYPASS    (INIT_RCC_BYPASS          << 11)
#endif
#define RCC_XTAL_MSK        (0x1FuL                   << 6)
#define CFG_RCC_XTAL        ((INIT_RCC_XTAL           << 6) & RCC_XTAL_MSK)
#define RCC_OSCSRC_MSK      (0x3uL                    << 4)
#define CFG_RCC_OSCSRC      ((INIT_RCC_OSCSRC         << 4) & RCC_OSCSRC_MSK)
#define RCC_MOSCDIS_MSK     (0x1uL                    << 0)
#define CFG_RCC_MOSCDIS     (INIT_RCC_MOSCDIS         << 0)

#define DEFAULT_RCC2        (0x07C06810uL)
#define RCC2_USERCC2_MSK    (0x1uL                    << 31)
#define CFG_RCC2_USERCC2    (INIT_RCC2_USERCC2        << 31)
#define RCC2_DIV400_MSK     (0x1uL                    << 30)
#define CFG_RCC2_DIV400     (INIT_RCC2_DIV400         << 30)
#define RCC2_SYSDIV2_MSK    (0x3FuL                   << 23)
#define CFG_RCC2_SYSDIV2    (((INIT_RCC2_SYSDIV2 - 1) << 23) & RCC2_SYSDIV2_MSK)
#define RCC2_SYSDIV2LSB_MSK (0x1uL                    << 22)
#define CFG_RCC2_SYSDIV2LSB (INIT_RCC2_SYSDIV2LSB     << 22)
#define CFG_RCC2_PWRDN2     INIT_RCC_PWRDN
#define RCC2_BYPASS2_MSK    RCC_BYPASS_MSK
#define CFG_RCC_PWRDN       (INIT_RCC_PWRDN           << 13)
#define RCC2_OSCSRC2_MSK    (0x7uL                    << 4)
#define CFG_RCC2_OSCSRC2    ((INIT_RCC2_OSCSRC2       << 4) & RCC2_OSCSRC2_MSK)

//
// Define the bit locations used to check the status of the main oscillator
// power and the lock state of the PLL.
//
#define MOSC_PWR_UP         (1uL                      << 8)
#define PLL_LOCK            (1uL                      << 6)
//
// Define the frequencies of those clock signals that are not configurable by
// the user.
//
#define XTALI       (16000000UL)    // Precision Internal Oscillator
#define XTAL30K     (   30000UL)    // Internal 30K oscillator
#define XTAL32K     (   32768UL)    // external 32K crystal oscillator
#define PLL_CLK    (400000000UL)    // PLL always runs at 400 MHz

/* Determine clock frequency according to clock register values */
#if (! CLOCK_SETUP)
#  define __CORE_CLK  XTALI
#  warning "Clock setup is not performed"
#elif (INIT_RCC2_USERCC2)
#  if (INIT_RCC2_BYPASS2)
#    if   (INIT_RCC2_OSCSRC2 == 0x0)
#      if (INIT_RCC_XTAL <= 0x5)
#        error "INIT_RCC_XTAL value is too small!"
#      elif (INIT_RCC_XTAL == 0x6)
#        define __CORE_CLK_PRE  4000000UL
#      elif (INIT_RCC_XTAL == 0x7)
#        define __CORE_CLK_PRE  4096000UL
#      elif (INIT_RCC_XTAL == 0x8)
#        define __CORE_CLK_PRE  4915200UL
#      elif (INIT_RCC_XTAL == 0x9)
#        define __CORE_CLK_PRE  5000000UL
#      elif (INIT_RCC_XTAL == 0xA)
#        define __CORE_CLK_PRE  5120000UL
#      elif (INIT_RCC_XTAL == 0xB)
#        define __CORE_CLK_PRE  6000000UL
#      elif (INIT_RCC_XTAL == 0xC)
#        define __CORE_CLK_PRE  6144000UL
#      elif (INIT_RCC_XTAL == 0xD)
#        define __CORE_CLK_PRE  7372800UL
#      elif (INIT_RCC_XTAL == 0xE)
#        define __CORE_CLK_PRE  8000000UL
#      elif (INIT_RCC_XTAL == 0xF)
#        define __CORE_CLK_PRE  8192000UL
#      elif (INIT_RCC_XTAL == 0x10)
#        define __CORE_CLK_PRE  10000000UL
#      elif (INIT_RCC_XTAL == 0x11)
#        define __CORE_CLK_PRE  12000000UL
#      elif (INIT_RCC_XTAL == 0x12)
#        define __CORE_CLK_PRE  12288000UL
#      elif (INIT_RCC_XTAL == 0x13)
#        define __CORE_CLK_PRE  13560000UL
#      elif (INIT_RCC_XTAL == 0x14)
#        define __CORE_CLK_PRE  14318180UL
#      elif (INIT_RCC_XTAL == 0x15)
#        define __CORE_CLK_PRE  16000000UL
#      elif (INIT_RCC_XTAL == 0x16)
#        define __CORE_CLK_PRE  16384000UL
#      elif (INIT_RCC_XTAL == 0x17)
#        define __CORE_CLK_PRE  18000000UL
#      elif (INIT_RCC_XTAL == 0x18)
#        define __CORE_CLK_PRE  20000000UL
#      elif (INIT_RCC_XTAL == 0x19)
#        define __CORE_CLK_PRE  24000000UL
#      elif (INIT_RCC_XTAL == 0x1A)
#        define __CORE_CLK_PRE  25000000UL
#      else
#        error "INIT_RCC_XTAL value is too large!"
#      endif
#    elif (INIT_RCC2_OSCSRC2 == 0x1)
#      define __CORE_CLK_PRE  XTALI
#    elif (INIT_RCC2_OSCSRC2 == 0x2)
#      define __CORE_CLK_PRE  (XTALI/4)
#    elif (INIT_RCC2_OSCSRC2 == 0x3)
#      define __CORE_CLK_PRE  XTAL30K
#    elif (INIT_RCC2_OSCSRC2 == 0x7)
#      define __CORE_CLK_PRE  XTAL32K
#    else
#      error "INIT_RCC2_OSCSRC2 has an invalid value!"
#      define __CORE_CLK_PRE  0
#    endif
#  else                         /* end PLL bypassed */
#    if (INIT_RCC2_DIV400)
#      define __CORE_CLK_PRE   PLL_CLK
#    else
#      define __CORE_CLK_PRE   (PLL_CLK/2)
#    endif
#    if (INIT_RCC_USESYSDIV)
#      if (INIT_RCC2_DIV400)
#        define __CORE_CLK  (__CORE_CLK_PRE / ((INIT_RCC2_SYSDIV2 * 2) + INIT_RCC2_SYSDIV2LSB + 1))
#      else
#        define __CORE_CLK  (__CORE_CLK_PRE / (INIT_RCC2_SYSDIV2 + 1))
#      endif
#    else
#      define __CORE_CLK  __CORE_CLK_PRE
#    endif
#  endif                        /* end using RCC2 */
#else
#  if (INIT_RCC_BYPASS)         /* If bypassing PLL, determine clock source frequency */
#    if (INIT_RCC_OSCSRC == 0x0)
#      if (INIT_RCC_XTAL <= 0x5)
#        error "INIT_RCC_XTAL value is too small!"
#      elif (INIT_RCC_XTAL == 0x6)
#        define __CORE_CLK_PRE  4000000UL
#      elif (INIT_RCC_XTAL == 0x7)
#        define __CORE_CLK_PRE  4096000UL
#      elif (INIT_RCC_XTAL == 0x8)
#        define __CORE_CLK_PRE  4915200UL
#      elif (INIT_RCC_XTAL == 0x9)
#        define __CORE_CLK_PRE  5000000UL
#      elif (INIT_RCC_XTAL == 0xA)
#        define __CORE_CLK_PRE  5120000UL
#      elif (INIT_RCC_XTAL == 0xB)
#        define __CORE_CLK_PRE  6000000UL
#      elif (INIT_RCC_XTAL == 0xC)
#        define __CORE_CLK_PRE  6144000UL
#      elif (INIT_RCC_XTAL == 0xD)
#        define __CORE_CLK_PRE  7372800UL
#      elif (INIT_RCC_XTAL == 0xE)
#        define __CORE_CLK_PRE  8000000UL
#      elif (INIT_RCC_XTAL == 0xF)
#        define __CORE_CLK_PRE  8192000UL
#      elif (INIT_RCC_XTAL == 0x10)
#        define __CORE_CLK_PRE  10000000UL
#      elif (INIT_RCC_XTAL == 0x11)
#        define __CORE_CLK_PRE  12000000UL
#      elif (INIT_RCC_XTAL == 0x12)
#        define __CORE_CLK_PRE  12288000UL
#      elif (INIT_RCC_XTAL == 0x13)
#        define __CORE_CLK_PRE  13560000UL
#      elif (INIT_RCC_XTAL == 0x14)
#        define __CORE_CLK_PRE  14318180UL
#      elif (INIT_RCC_XTAL == 0x15)
#        define __CORE_CLK_PRE  16000000UL
#      elif (INIT_RCC_XTAL == 0x16)
#        define __CORE_CLK_PRE  16384000UL
#      elif (INIT_RCC_XTAL == 0x17)
#        define __CORE_CLK_PRE  18000000UL
#      elif (INIT_RCC_XTAL == 0x18)
#        define __CORE_CLK_PRE  20000000UL
#      elif (INIT_RCC_XTAL == 0x19)
#        define __CORE_CLK_PRE  24000000UL
#      elif (INIT_RCC_XTAL == 0x1A)
#        define __CORE_CLK_PRE  25000000UL
#      else
#        error "INIT_RCC_XTAL value is too large!"
#      endif
#    elif (INIT_RCC_OSCSRC == 0x1)
#      define __CORE_CLK_PRE  XTALI
#    elif (INIT_RCC_OSCSRC == 0x2)
#      define __CORE_CLK_PRE  (XTALI/4)
#    elif (INIT_RCC_OSCSRC == 0x3)
#      define __CORE_CLK_PRE  XTAL30K
#    else
#      error "INIT_RCC_OSCSRC has an invalid value!"
#      define __CORE_CLK_PRE  0
#    endif
#  else
#    define __CORE_CLK_PRE   PLL_CLK
#  endif
#  if (INIT_RCC_USESYSDIV)
#    if (INIT_RCC_BYPASS)
#      define __CORE_CLK  (__CORE_CLK_PRE / (INIT_RCC_SYSDIV + 1))
#    else
#      define __CORE_CLK  (__CORE_CLK_PRE / (INIT_RCC_SYSDIV + 1) / 2)
#    endif
#  else
#    define __CORE_CLK  __CORE_CLK_PRE
#  endif
#endif

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = XTALI;  /*!< System Core Clock Frequency */

/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Get the OSC clock
 *----------------------------------------------------------------------------*/
static inline uint32_t GetOscFreq(uint32_t OscSrc)
{
  uint32_t OscFreq = XTALI;

  switch (OscSrc) {
    case 0:                    /* MOSC Main oscillator */
#if (INIT_RCC_XTAL <= 0x5)
      OscFreq = 0UL;
#elif (INIT_RCC_XTAL == 0x6)
      OscFreq = 4000000UL;
#elif (INIT_RCC_XTAL == 0x7)
      OscFreq = 4096000UL;
#elif (INIT_RCC_XTAL == 0x8)
      OscFreq = 4915200UL;
#elif (INIT_RCC_XTAL == 0x9)
      OscFreq = 5000000UL;
#elif (INIT_RCC_XTAL == 0xA)
      OscFreq = 5120000UL;
#elif (INIT_RCC_XTAL == 0xB)
      OscFreq = 6000000UL;
#elif (INIT_RCC_XTAL == 0xC)
      OscFreq = 6144000UL;
#elif (INIT_RCC_XTAL == 0xD)
      OscFreq = 7372800UL;
#elif (INIT_RCC_XTAL == 0xE)
      OscFreq = 8000000UL;
#elif (INIT_RCC_XTAL == 0xF)
      OscFreq = 8192000UL;
#elif (INIT_RCC_XTAL == 0x10)
      OscFreq = 10000000UL;
#elif (INIT_RCC_XTAL == 0x11)
      OscFreq = 12000000UL;
#elif (INIT_RCC_XTAL == 0x12)
      OscFreq = 12288000UL;
#elif (INIT_RCC_XTAL == 0x13)
      OscFreq = 13560000UL;
#elif (INIT_RCC_XTAL == 0x14)
      OscFreq = 14318180UL;
#elif (INIT_RCC_XTAL == 0x15)
      OscFreq = 16000000UL;
#elif (INIT_RCC_XTAL == 0x16)
      OscFreq = 16384000UL;
#elif (INIT_RCC_XTAL == 0x17)
      OscFreq = 18000000UL;
#elif (INIT_RCC_XTAL == 0x18)
      OscFreq = 20000000UL;
#elif (INIT_RCC_XTAL == 0x19)
      OscFreq = 24000000UL;
#elif (INIT_RCC_XTAL == 0x1A)
      OscFreq = 25000000UL;
#else
      OscFreq = 0UL;
#endif
      break;
    case 1:                    /* IOSC Internal oscillator */
      OscFreq = XTALI;
      break;
    case 2:                    /* IOSC/4 Internal oscillator/4 */
      OscFreq = XTALI / 4;
      break;
    case 3:                    /* 30kHz internal oscillator  */
      OscFreq = XTAL30K;
      break;
    case 7:                    /* 32.768kHz crystal oscillator  */
      OscFreq = XTAL32K;
      break;
  }

  return OscFreq;
}

void SystemCoreClockUpdate(void)
{
  uint32_t rcc, rcc2, RawClock, ClockDivisor;

  //
  // Get the clock configuration information from RCC and RCC2
  //
  rcc = SYSCTL->RCC;
  rcc2 = SYSCTL->RCC2;

  //
  // Assume that the clock divider is NOT used. Assume that the PLL is
  // used with the default divide by 2
  //
  ClockDivisor = 1;
  RawClock = PLL_CLK / 2;

  if (rcc2 & RCC2_USERCC2_MSK) {
    //
    // RCC2 is being used. Get the bypass and divider information from
    // that register.
    //
    if (rcc2 & RCC_BYPASS_MSK) {
      RawClock = GetOscFreq((rcc2 & RCC2_OSCSRC2_MSK) >> 4);
    } else {
      if (rcc2 & RCC2_DIV400_MSK) {
        RawClock = PLL_CLK;
      }
    }

    if (rcc & RCC_USESYSDIV_MSK) {
      if (rcc2 & RCC2_DIV400_MSK) {
        ClockDivisor =
            ((rcc2 & (RCC2_SYSDIV2_MSK | RCC2_SYSDIV2LSB_MSK)) >> 22) + 1;
      } else {
        ClockDivisor = (((rcc2 & RCC2_SYSDIV2_MSK) >> 23) + 1);
      }
    }
  } else {
    //
    // RCC2 is not used, so get bypass and divider information from the
    // RCC register. If using the PLL then the VCO frequency is always
    // divided by 2 before applying the SYSDIV divisor.
    //
    if (rcc & RCC_BYPASS_MSK) {
      RawClock = GetOscFreq((rcc & RCC_OSCSRC_MSK) >> 4);
    }
    if (rcc & RCC_USESYSDIV_MSK) {
      ClockDivisor = ((rcc & RCC_SYSDIV_MSK) >> 23) + 1;
    }
  }
  SystemCoreClock = RawClock / ClockDivisor;
}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System. The main system clock is configured and
 *         enabled, but the PWM and USB clocks are not.
 */
void SystemInit(void)
{
#if(CLOCK_SETUP)
  uint32_t i, rcc, rcc2;

  /*
   * Set default values for RCC and RCC2
   * These should be identical to the reset state of these registers
   * The PLL is bypassed, the clock dividers are disabled, the main
   * and USB oscillators are powered down.
   */
  rcc = DEFAULT_RCC;
  rcc2 = DEFAULT_RCC2;
  SYSCTL->RCC = rcc;
  SYSCTL->RCC2 = rcc2;

  /* wait a while */
  for (i = 0; i < 1000; i++) rcc = SYSCTL->RCC;

  //
  // See if the main oscillator needs to be enabled.
  //
#  if (! CFG_RCC_MOSCDIS)

  // Clear the MOSC power up raw interrupt status to be sure it is not
  // set when waiting below.
  SYSCTL->MISC = MOSC_PWR_UP;

  // Clear the disable bit and write the new value
  rcc &= ~RCC_MOSCDIS_MSK;
  SYSCTL->RCC = rcc;

  // Wait for the main oscillator to power up. If it never does then
  // we will loop here forever.
  while ((SYSCTL->RIS & MOSC_PWR_UP) == 0) {
  }
#  endif
  //
  // Set the new crystal value and oscillator source.
  // Write the new values
  //
  rcc &= ~(RCC_XTAL_MSK | RCC_OSCSRC_MSK);
  rcc |= (CFG_RCC_XTAL | CFG_RCC_OSCSRC);
  SYSCTL->RCC = rcc;
#  if (CFG_RCC2_USERCC2)
  rcc2 &= ~RCC2_OSCSRC2_MSK;
  rcc2 |= (CFG_RCC2_USERCC2 | CFG_RCC2_OSCSRC2);
  SYSCTL->RCC2 = rcc2;
#  endif

#  if (! CFG_RCC_PWRDN)

  //
  // Clear the PLL lock interrupt.
  //
  SYSCTL->MISC = PLL_LOCK;

  //
  // Power up the PLL
  //
  rcc &= ~RCC_PWRDN_MSK;
#    if (CFG_RCC2_USERCC2)
  rcc2 &= ~RCC2_PWRDN2_MSK;
  SYSCTL->RCC2 = rcc2;
  SYSCTL->RCC = rcc;
#    else
  SYSCTL->RCC = rcc;
#    endif
#  endif

  //
  // Set the requested system divider 
  // FIXME The datasheet says that SYSDIV2LSB may only be set or cleared
  // when DIV400 == 1. Can both bits be set simultaneously?
  //
  rcc &= ~RCC_SYSDIV_MSK;
  rcc |= CFG_RCC_SYSDIV | CFG_RCC_USESYSDIV;
#  if (CFG_RCC2_USERCC2)
  rcc2 &= ~RCC2_SYSDIV2_MSK;
  rcc2 |= CFG_RCC2_SYSDIV2;
#    if (CFG_RCC2_DIV400) {
    rcc2 &= ~(RCC2_SYSDIV2LSB_MSK);
    rcc2 |= (CFG_RCC2_DIV400 | CFG_RCC2_SYSDIV2LSB);
#    endif
#  endif
  //
  // See if the PLL output is being used to clock the system.
  //
#  if (!CFG_RCC_BYPASS)
  //
  // Wait until the PLL has locked.
  //
  while (!(SYSCTL->PLLSTAT & 0x1)) {
  }
  //
  // Enable use of the PLL.
  //
  rcc &= ~RCC_BYPASS_MSK;
#    if (CFG_RCC2_USERCC2)
  rcc2 &= ~RCC2_BYPASS2_MSK;
#    endif
#  endif
  //
  // Write the final RCC value.
  //
  SYSCTL->RCC = rcc;
  SYSCTL->RCC2 = rcc2;

  //
  // Delay for a little bit so that the system divider takes effect.
  //
  for (i = 0; i < 100; i++) rcc = SYSCTL->RCC;
  SystemCoreClockUpdate();

#endif
}
