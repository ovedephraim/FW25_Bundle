/**
* @file sysport.h
* @brief system portable functions
*
* @version 0.0.1
* @date 29.07.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>*/

#ifndef _SYSPORT_H
#define _SYSPORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#if   defined ( __CC_ARM ) /*------------------RealView Compiler -----------------*/
/* ARM armcc specific functions */

#if (__ARMCC_VERSION < 400677)
  #error "Please use ARM Compiler Toolchain V4.0.677 or later!"
#endif


/**
 * @fn uint32_t __disableInterrupts(void)
 * @brief Get Priority Mask and disable interrupts
 *
 * This function returns the current state of the priority mask bit from the Priority Mask Register.
 *
 *
 * @author Eli Schneider
 * @return Priority Mask value
 * @date 16.04.2012
 */
static __INLINE uint32_t __disableInterrupts(void)
{
  register uint32_t __regPriMask         __ASM("primask");
  __ASM("cpsid i");
  return(__regPriMask);
}


/**
 * @fn void __restoreInterrupts(uint32_t priMask)
 * @brief Set Priority Mask
 *
 * This function assigns the given value to the Priority Mask Register.
 *
 * @author Eli Schneider
 * @param [in]    priMask  Priority Mask
 * @date 16.04.2012
 */
static __INLINE void __restoreInterrupts(uint32_t priMask)
{
  register uint32_t __regPriMask         __ASM("primask");
  __regPriMask = (priMask);
}

/**
 * @fn uint32_t __enableInterrupts(void)
 * @brief Get Priority Mask and enable interrupts
 *
 * This function returns the current state of the priority mask bit from the Priority Mask Register.
 *
 *
 * @author Eli Schneider
 * @return Priority Mask value
 * @date 16.04.2012
 */
static __INLINE uint32_t __enableInterrupts(void)
 {
   register uint32_t __regPriMask		  __ASM("primask");
   __ASM("cpsie i");
   return(__regPriMask);
 }


#elif defined ( __ICCARM__ ) /*------------------ ICC Compiler -------------------*/
/* IAR iccarm specific functions */

#include <cmsis_iar.h>

#elif defined ( __GNUC__ ) /*------------------ GNU Compiler ---------------------*/
/* GNU gcc specific functions */


/**
 * @fn uint32_t __disableInterrupts(void)
 * @brief Get Priority Mask and disable interrupts
 *
 * This function returns the current state of the priority mask bit from the Priority Mask Register.
 *
 *
 * @author Eli Schneider
 * @return Priority Mask value
 * @date 16.04.2012
 */
static inline uint32_t __disableInterrupts(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  __asm volatile ("cpsid i");
  return(result);
}


/**
 * @fn void __restoreInterrupts(uint32_t priMask)
 * @brief Set Priority Mask
 *
 * This function assigns the given value to the Priority Mask Register.
 *
 * @author Eli Schneider
 * @param [in]    priMask  Priority Mask
 * @date 16.04.2012
 */
static inline void __restoreInterrupts(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) );
}

/**
 * @fn uint32_t __enableInterrupts(void))
 * @brief Get Priority Mask and enable interrupts
 *
 * This function returns the current state of the priority mask bit from the Priority Mask Register.
 *
 *
 * @author Eli Schneider
 * @return Priority Mask value
 * @date 16.04.2012
 */
static inline uint32_t __enableInterrupts(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  __asm volatile ("cpsie i");
  return(result);
}

#endif

extern uint8_t isrLevel;


/**
 * @fn int inIsr(void)
 * @brief Test if execution is in ISR
 *
 * @author Eli Schneider
 * @return 0=Not in ISR, 1= In ISR
 * @date 16.04.2012
 */
static inline int inIsr(void)
{
	register uint32_t key;

	key=__disableInterrupts();
	if (isrLevel==0)
	{
		__restoreInterrupts(key);
		return 0;
	}
	else
	{
		__restoreInterrupts(key);
		return 1;
	}
}

#define ENTER_ISR()	isrLevel++ /**< A prolog of ISR code */
#define EXIT_ISR()	isrLevel-- /**< An epilog of ISR code */

#ifdef __cplusplus
}
#endif


#endif
