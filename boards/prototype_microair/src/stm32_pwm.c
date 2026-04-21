/****************************************************************************
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_pwm.h"
#include "prototype.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_PWM 1

#ifndef CONFIG_PWM
#  undef HAVE_PWM
#endif

#if !defined(CONFIG_STM32H7_TIM1) && !defined(CONFIG_STM32H7_TIM3) && \
    !defined(CONFIG_STM32H7_TIM4) && !defined(CONFIG_STM32H7_TIM15)
#  undef HAVE_PWM
#endif

#if !defined(CONFIG_STM32H7_TIM1_PWM) && !defined(CONFIG_STM32H7_TIM3_PWM) && \
    !defined(CONFIG_STM32H7_TIM4_PWM) && !defined(CONFIG_STM32H7_TIM15_PWM)
#  undef HAVE_PWM
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef HAVE_PWM
static int stm32_pwm_register_timer(const char *devpath, int timer)
{
  struct pwm_lowerhalf_s *pwm;
  int ret;

  pwm = stm32_pwminitialize(timer);
  if (!pwm)
    {
      tmrerr("ERROR: Failed to get TIM%d PWM lower half\n", timer);
      return -ENODEV;
    }

  ret = pwm_register(devpath, pwm);
  if (ret < 0)
    {
      tmrerr("ERROR: pwm_register(%s) failed: %d\n", devpath, ret);
      return ret;
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

int stm32_pwm_setup(void)
{
#ifdef HAVE_PWM
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Register motor PWM timer groups. */

#if defined(CONFIG_STM32H7_TIM1) && defined(CONFIG_STM32H7_TIM1_PWM)
      ret = stm32_pwm_register_timer("/dev/pwm0", PWM_TIM1);
      if (ret < 0)
        {
          return ret;
        }
#endif

#if defined(CONFIG_STM32H7_TIM3) && defined(CONFIG_STM32H7_TIM3_PWM)
      ret = stm32_pwm_register_timer("/dev/pwm1", PWM_TIM3);
      if (ret < 0)
        {
          return ret;
        }
#endif

#if defined(CONFIG_STM32H7_TIM4) && defined(CONFIG_STM32H7_TIM4_PWM)
      ret = stm32_pwm_register_timer("/dev/pwm2", PWM_TIM4);
      if (ret < 0)
        {
          return ret;
        }
#endif

#if defined(CONFIG_STM32H7_TIM15) && defined(CONFIG_STM32H7_TIM15_PWM)
      ret = stm32_pwm_register_timer("/dev/pwm3", PWM_TIM15);
      if (ret < 0)
        {
          return ret;
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
#else
  return -ENODEV;
#endif
}
