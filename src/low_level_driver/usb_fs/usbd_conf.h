/**
  ******************************************************************************
  * @file           : usbd_conf.h
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_conf file.
  ******************************************************************************
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "mcu_include.h"
#include "usbd_def.h"


/*---------- -----------*/
#define USBD_MAX_NUM_INTERFACES     1
/*---------- -----------*/
#define USBD_MAX_NUM_CONFIGURATION  1
/*---------- -----------*/
#define USBD_MAX_STR_DESC_SIZ       512
/*---------- -----------*/
#define USBD_SUPPORT_USER_STRING    0
/*---------- -----------*/
#define USBD_DEBUG_LEVEL            0
/*---------- -----------*/
#define USBD_SELF_POWERED           1
/****************************************/
/* #define for FS and HS identification */
#define DEVICE_FS       0

/* Memory management macros */
#define USBD_malloc               (uint32_t *)USBD_static_malloc
#define USBD_free                 USBD_static_free
#define USBD_memset               /* Not used */
#define USBD_memcpy               /* Not used */

#define USBD_Delay   HAL_Delay

/* For footprint reasons and since only one allocation is handled in the HID class
   driver, the malloc/free is changed into a static allocation method */
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#if (USBD_DEBUG_LEVEL > 0)
#define  USBD_UsrLog(...)   printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1)
#define  USBD_ErrLog(...)   printf("ERROR: ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2)
#define  USBD_DbgLog(...)   printf("DEBUG : ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_DbgLog(...)
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
