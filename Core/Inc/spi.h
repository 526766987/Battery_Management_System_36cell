/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */
#define CFGR 0
#define CFGRB 4
#define CELL 1
#define AUX 2
#define STAT 3
/*inline static uint8_t transfer(uint8_t data) {
    SPDR = data;

    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    return SPDR;
 }*/
uint8_t SPI1_ReadWriteByte(uint8_t TxData);
typedef struct
{
  uint8_t tx_data[6];  //!< Stores data to be transmitted 
  uint8_t rx_data[8];  //!< Stores received data 
  uint8_t rx_pec_match; //!< If a PEC error was detected during most recent read cmd
} ic_register;
typedef struct
{
  uint16_t c_codes[18]; //!< Cell Voltage Codes
  uint8_t pec_match[6]; //!< If a PEC error was detected during most recent read cmd
} cv;
typedef struct
{
  uint16_t a_codes[9]; //!< Aux Voltage Codes
  uint8_t pec_match[4]; //!< If a PEC error was detected during most recent read cmd
} ax;
typedef struct
{
  uint16_t stat_codes[4]; //!< Status codes
	uint8_t flags[3]; //!< Byte array that contains the uv/ov flag data
	uint8_t oc_cntr[1]; //!< Oscillator check counter activated by ADC Conversions or the DIAGN Commond
	uint8_t va_ovhi[1]; //!< Analog Rail Overvoltage
	uint8_t va_uvlo[1]; //!< Analog Rail Undervoltage
	uint8_t vd_ovhi[1]; //!< Digital Rail Overvoltage
	uint8_t vd_uvlo[1]; //!< Digital Rail Undervoltage
	uint8_t a_otp_ed[1]; //!< ADC OTP Trim Error Detection
	uint8_t a_otp_med[1]; //!< ADC OTP Trim Multiple Error Detection
	uint8_t otp_ed[1]; //!< Other OTP Trim Error Detection
	uint8_t otp_med[1]; //!< Other OTP Trim Multiple Error Detection
	uint8_t redfail[1]; //!< Digital Redundancy Failure Detection
	uint8_t compchk[1]; //!< ADC Current Compensation Logic Error Detection
	uint8_t sleep[1]; //!< Sleep State Detection
	uint8_t tmodechk[1]; //!< Test Mode Detection
	uint8_t muxfail[1]; //!< Multiplexer Self Test Result
	uint8_t thsd[1]; //!< Thermal Shutdown Status
	uint8_t cpchk[1]; //!< Charge Pump Check
	uint8_t oscchk[1]; //!< Oscillator Check
	uint16_t adol[2]; //!< 16-bit ADC Overlap Measurement Value for Cell 7
	uint8_t pec_match[3]; //!< If a PEC error was detected during most recent read cmd
} st;
typedef struct
{
  uint16_t pec_count; //!< Overall PEC error count
  uint16_t cfgr_pec;  //!< Configuration register data PEC error count
  uint16_t cell_pec[6]; //!< Cell voltage register data PEC error count
  uint16_t aux_pec[4];  //!< Aux register data PEC error count
  uint16_t stat_pec[2]; //!< Status register data PEC error count
} pec_counter;
typedef struct
{
  uint8_t cell_channels; //!< Number of Cell channels
  uint8_t stat_channels; //!< Number of Stat channels
  uint8_t aux_channels;  //!< Number of Aux channels
  uint8_t num_cv_reg;    //!< Number of Cell voltage register
  uint8_t num_gpio_reg;  //!< Number of Aux register
  uint8_t num_stat_reg;  //!< Number of  Status register
} register_cfg;

typedef struct
{
  ic_register config;
  ic_register configb;
  cv  cells;
  ax  aux;
  st  stat;
  ic_register com;
  ic_register pwm;
  ic_register pwmb;
  ic_register sctrl;
  ic_register sctrlb;
  uint8_t sid[6];
  bool isospi_reverse;
  pec_counter crc_count;
  register_cfg ic_reg;
  long system_open_wire;
	/* Custom Variables */
	// Time
	float nowTime;
	float lastSampleTime;
	float timeInterval;
	// 6811 Measurement State
	int offline;
	int firstTime;
	float sc;
	float sc_Past;
	float temp;
	float distrubTemp[6];
	uint8_t flag[3];
	uint8_t thsd;
	float cellVoltage[12];
	// discharge strategy
	int dischargeHLock[12];
	int dischargeLLock[12];
	// dvdt
	float *dvdt;
	int dvdtpos;
	int dvdtReady;
	int dvdtLock;
	// current
	float current;
	float currentPast;
	// SOC
	float SOC;
	float Cmax;
	float SOCPast;
	// SOH
	float SOH;
	int SOH_state;
	float SOH_Q;
	float SOH_foundSOC;
	// SOP
	float SOP;
} cell_asic;

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
