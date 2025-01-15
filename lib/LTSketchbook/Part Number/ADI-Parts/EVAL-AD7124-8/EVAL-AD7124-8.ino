/***************************************************************************//**
 *   @file   EVAL-AD7124-8.ino
 *   @brief  Exerciser program for ad2174 no-OS driver
 *   @author Mthoren
********************************************************************************
 * Copyright 2017(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "UserInterface.h"
#include "QuikEval_EEPROM.h"
#include <platform_drivers.h>
extern "C" {
#include <ad7124.h>
//#include <ad7124_regs.h> // un-comment to use default values from ad7124_regs.c
};

/******************************************************************************/
/*********************** AD7124 initial configuration *************************/
/******************************************************************************/

struct ad7124_st_reg my_ad7124_regs[AD7124_REG_NO] =
{
  {0x00, 0x00,   1, 2}, /* AD7124_Status */
  {0x01, 0x0000, 2, 1}, /* AD7124_ADC_Control */
  {0x02, 0x0000, 3, 2}, /* AD7124_Data */
  {0x03, 0x0000, 3, 1}, /* AD7124_IOCon1 */
  {0x04, 0x0000, 2, 1}, /* AD7124_IOCon2 */
  {0x05, 0x02,   1, 2}, /* AD7124_ID */
  {0x06, 0x0000, 3, 2}, /* AD7124_Error */
  {0x07, 0x0044, 3, 1}, /* AD7124_Error_En */
  {0x08, 0x00,   1, 2}, /* AD7124_Mclk_Count */
  {0x09, 0x8013, 2, 1}, /* AD7124_Channel_0 */
  {0x0A, 0x0001, 2, 1}, /* AD7124_Channel_1 */
  {0x0B, 0x0001, 2, 1}, /* AD7124_Channel_2 */
  {0x0C, 0x0001, 2, 1}, /* AD7124_Channel_3 */
  {0x0D, 0x0001, 2, 1}, /* AD7124_Channel_4 */
  {0x0E, 0x0001, 2, 1}, /* AD7124_Channel_5 */
  {0x0F, 0x0001, 2, 1}, /* AD7124_Channel_6 */
  {0x10, 0x0001, 2, 1}, /* AD7124_Channel_7 */
  {0x11, 0x0001, 2, 1}, /* AD7124_Channel_8 */
  {0x12, 0x0001, 2, 1}, /* AD7124_Channel_9 */
  {0x13, 0x0001, 2, 1}, /* AD7124_Channel_10 */
  {0x14, 0x0001, 2, 1}, /* AD7124_Channel_11 */
  {0x15, 0x0001, 2, 1}, /* AD7124_Channel_12 */
  {0x16, 0x0001, 2, 1}, /* AD7124_Channel_13 */
  {0x17, 0x0001, 2, 1}, /* AD7124_Channel_14 */
  {0x18, 0x0001, 2, 1}, /* AD7124_Channel_15 */
  {0x19, 0x0860, 2, 1}, /* AD7124_Config_0 */
  {0x1A, 0x0860, 2, 1}, /* AD7124_Config_1 */
  {0x1B, 0x0860, 2, 1}, /* AD7124_Config_2 */
  {0x1C, 0x0860, 2, 1}, /* AD7124_Config_3 */
  {0x1D, 0x0860, 2, 1}, /* AD7124_Config_4 */
  {0x1E, 0x0860, 2, 1}, /* AD7124_Config_5 */
  {0x1F, 0x0860, 2, 1}, /* AD7124_Config_6 */
  {0x20, 0x0860, 2, 1}, /* AD7124_Config_7 */
  {0x21, 0x060180, 3, 1}, /* AD7124_Filter_0 */
  {0x22, 0x060180, 3, 1}, /* AD7124_Filter_1 */
  {0x23, 0x060180, 3, 1}, /* AD7124_Filter_2 */
  {0x24, 0x060180, 3, 1}, /* AD7124_Filter_3 */
  {0x25, 0x060180, 3, 1}, /* AD7124_Filter_4 */
  {0x26, 0x060180, 3, 1}, /* AD7124_Filter_5 */
  {0x27, 0x060180, 3, 1}, /* AD7124_Filter_6 */
  {0x28, 0x060180, 3, 1}, /* AD7124_Filter_7 */
  {0x29, 0x800000, 3, 1}, /* AD7124_Offset_0 */
  {0x2A, 0x800000, 3, 1}, /* AD7124_Offset_1 */
  {0x2B, 0x800000, 3, 1}, /* AD7124_Offset_2 */
  {0x2C, 0x800000, 3, 1}, /* AD7124_Offset_3 */
  {0x2D, 0x800000, 3, 1}, /* AD7124_Offset_4 */
  {0x2E, 0x800000, 3, 1}, /* AD7124_Offset_5 */
  {0x2F, 0x800000, 3, 1}, /* AD7124_Offset_6 */
  {0x30, 0x800000, 3, 1}, /* AD7124_Offset_7 */
  {0x31, 0x500000, 3, 1}, /* AD7124_Gain_0 */
  {0x32, 0x500000, 3, 1}, /* AD7124_Gain_1 */
  {0x33, 0x500000, 3, 1}, /* AD7124_Gain_2 */
  {0x34, 0x500000, 3, 1}, /* AD7124_Gain_3 */
  {0x35, 0x500000, 3, 1}, /* AD7124_Gain_4 */
  {0x36, 0x500000, 3, 1}, /* AD7124_Gain_5 */
  {0x37, 0x500000, 3, 1}, /* AD7124_Gain_6 */
  {0x38, 0x500000, 3, 1}, /* AD7124_Gain_7 */
};



/******************************************************************************/
/************************** Variables Declarations ****************************/
/******************************************************************************/


spi_init_param spi_params =
{
  GENERIC_SPI, // Type
  QUIKEVAL_CS, // ID
  1000000, // Max speed, Hz
  3, // Mode
  QUIKEVAL_CS, // CS ID
};

ad7124_init_param init_params =
{
  spi_params,     // SPI parameters
  my_ad7124_regs, // declared in ad7124_regs.c
  64000,     // spi_rdy_poll_cnt: # tries before timeout
};


struct ad7124_dev my_ad7124;                    /* A new driver instance */
struct ad7124_dev *ad7124_handler = &my_ad7124; /* A driver handle to pass around */

int regNr;                                      /* Variable to iterate through registers */
long timeout = 100;                             /* Number of tries before a function times out */
long ret = 0;                                   /* Return value */
long sample;                                    /* Stores raw value read from the ADC */


void setup()
{
  char demo_name[] = "AD7124-8";

  int32_t x;

  Serial.begin(115200);

  // Give the serial port a chance to initialize
  // Without this we get some garbage on the console
  delay(100);

  if (false) // Set to true to dump initial values to screen
  {
    Serial.println(F("Print inital values..."));
    for (regNr = AD7124_Status; (regNr < AD7124_REG_NO) && !(ret < 0); regNr++)
    {
      //ret = ad7124_read_register(ad7124_handler, &ad7124_regs[regNr]);
      Serial.print("Register #: ");
      Serial.print(regNr);
      Serial.print(", Value: ");
      x = my_ad7124_regs[regNr].value;
      Serial.println(x);
    }
  }

  Serial.println(F("Attempting to initialize..."));
  delay(50);

  /* Initialize AD7124 device. */
  ret = ad7124_setup(&ad7124_handler, init_params);
  if (ret < 0)
  {
    /* AD7124 initialization failed, check the value of ret! */
    Serial.println(F("AD7124-8 not found! :("));
    Serial.print(F("Return value: "));
    Serial.println(ret);
  }
  else
  {
    /* AD7124 initialization OK */
    Serial.println(F("AD7124-8 Initialized!! :)"));
  }

  /* Read all registers */
  for (regNr = AD7124_Status; (regNr < AD7124_REG_NO) && !(ret < 0); regNr++)
  {
    ret = ad7124_read_register(ad7124_handler, &my_ad7124_regs[regNr]);
    Serial.print("Register #: ");
    Serial.print(regNr);
    Serial.print(", Value: ");
    x = my_ad7124_regs[regNr].value;
    Serial.println(x);
  }

  /* Read data from the ADC */
  ret = ad7124_wait_for_conv_ready(ad7124_handler, timeout);
  if (ret < 0)
  {
    /* Something went wrong, check the value of ret! */
    Serial.print("error waiting for conv ready: ");
    Serial.println(ret);
  }
  ret = ad7124_read_data(ad7124_handler, &sample);
  if (ret < 0)
  {
    /* Something went wrong, check the value of ret! */
    Serial.print("error reading ADC data: ");
    Serial.println(ret);
  }
  else
  {
    Serial.print("ADC data: ");
    Serial.println(sample);
  }
}

void loop()
{
  while (1)
  {
    delay(1000);
    Serial.println("Looping...");
    ret = ad7124_read_data(ad7124_handler, &sample);
    Serial.print("ADC data: ");
    Serial.println(sample);
  }
}
