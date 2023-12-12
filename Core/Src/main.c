/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file     : main.c
 * @brief    : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *      opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "stdlib.h"
#include <math.h>
// Function Flag
bool tim3Int = false;
int canRcvDataInt = -1;
int canRcvConfInt = -1;
// 6815 CS
uint8_t subID = 0x01;
uint8_t board = 0x00;
GPIO_TypeDef *cs_GPIO[3] = {GPIOG, GPIOD, GPIOB};
uint16_t cs_PIN[3] = {GPIO_PIN_10,GPIO_PIN_1,GPIO_PIN_8};
GPIO_TypeDef *multiplexer_GPIO[3] = {GPIOD, GPIOD, GPIOD}; // A1 A0 EN
uint16_t multiplexer_PIN[3] = {GPIO_PIN_5, GPIO_PIN_3, GPIO_PIN_4};

// 6815 Configuration Setting
bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool PS[2] = {0, 0};
bool CVMIN[2] = {0, 0};
bool MCAL = 0;
bool COMM_BK = 0;
bool FLAG_D[8] = {0,0,0,0,0,0,0,0};
bool SOAKON = 0;
bool OWRNG = 0;
bool OWA[3] = {0,0,0};
bool OWC[3] = {0,0,0};
bool GPO[7] = {true,true,true,true,true,true,true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5,6,7
uint16_t UV_THRESHOLD = 0; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(0V)
uint16_t OV_THRESHOLD = 38500; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(3.85V)
bool DTMEN = 0;
bool DTRNG = 0;
bool DCTO[6] = {0,0,1,0,0,0}; //!< Discharge time value // Dcto 0,1,2,3 // Programed for 4 min
bool DCC[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
// discharge
int dischargeLockLimit = 5;
// dvdt
int timeZones = 500;
float Vsta = 1; //mV/h
int dvdtLockLimit = 5;
// SOC
float OCV_List[51] = {2.524,3.0265,3.137,3.1945,3.211,3.214,3.2165,3.2275,3.239,3.25,3.26,3.2675,3.275,3.2825,3.2875,3.292,3.296,3.2975,3.298,3.2985,3.2985,3.2995,3.2995,3.2995,3.3,3.3005,3.301,3.301,3.302,3.303,3.305,3.3095,3.3215,3.332,3.336,3.3365,3.3365,3.336,3.3365,3.337,3.3375,3.3375,3.3375,3.3375,3.338,3.338,3.3385,3.3385,3.339,3.34,3.3795};
float SOC_List[51] = {0,0.02,0.04,0.06,0.08,0.1,0.12,0.14,0.16,0.18,0.2,0.22,0.24,0.26,0.28,0.3,0.32,0.34,0.36,0.38,0.4,0.42,0.44,0.46,0.48,0.5,0.52,0.54,0.56,0.58,0.6,0.62,0.64,0.66,0.68,0.7,0.72,0.74,0.76,0.78,0.8,0.82,0.84,0.86,0.88,0.9,0.92,0.94,0.96,0.98,1};
const float C0 = 138.5; //Ah
float plainThsd[2] = {3.15,3.35}; //V
// SOP
float SOP_ListX[7] = {-30, -20, -10, 0, 10, 25, 45};
float SOP_ListY[5] = {0,0.2,0.5,0.8,1};
float SOP_chargeList[7][5] = {{0,0,0,0,0},
            {11,11,11,11,0},
            {45,45,45,45,0},
            {620,620,445,315,0},
            {855,855,715,520,0},
            {960,960,905,650,0},
            {945,945,955,905,0}};
float SOP_dischargeList[7][5] = {{0,300,365,435,435},
         {0,535,580,605,605},
         {0,635,665,685,685},
         {0,705,730,745,745},
         {0,740,765,780,780},
         {0,785,805,815,815},
         {0,800,820,830,830}};
// current
/*
   //Forced to use Hall by GPIO1(6815)
   float baseVoltage = 3.307; //V
   float RM = 30.77; //Ohm
   int turns = 5;
   int currentGPIOPin = 1; //GPIO1 (1-7)
 */
float smallCurrentLimit = 0.01f * C0;
// CAN
#define CAN_BUF_SIZE 32
uint8_t can1_tx_buff[CAN_BUF_SIZE]={0};
uint8_t can1_rx_buff[CAN_BUF_SIZE]={0};
uint32_t StdId = 0;
union dataTrans {
  uint8_t single[4];
  float dataF;
};
CAN_HandleTypeDef *activeHcan;
uint32_t punishDelay = 1;
// CRC Table
const uint16_t crc15Table[256]={0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  // precomputed CRC15 Table
        0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
        0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
        0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
        0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
        0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
        0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
        0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
        0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
        0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
        0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
        0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
        0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
        0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
        0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
        0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
        0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
        0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
        0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
        0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
        0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
        0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
        0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095};
const uint16_t crc10Table[256]={0x0, 0x8f, 0x11e, 0x191, 0x23c, 0x2b3, 0x322, 0x3ad, 0xf7, 0x78, 0x1e9, 0x166, 0x2cb, 0x244, 0x3d5, 0x35a,
        0x1ee, 0x161, 0xf0, 0x7f, 0x3d2, 0x35d, 0x2cc, 0x243, 0x119, 0x196, 0x7, 0x88, 0x325, 0x3aa, 0x23b, 0x2b4,
        0x3dc, 0x353, 0x2c2, 0x24d, 0x1e0, 0x16f, 0xfe, 0x71, 0x32b, 0x3a4, 0x235, 0x2ba, 0x117, 0x198, 0x9, 0x86,
        0x232, 0x2bd, 0x32c, 0x3a3, 0xe, 0x81, 0x110, 0x19f, 0x2c5, 0x24a, 0x3db, 0x354, 0xf9, 0x76, 0x1e7, 0x168,
        0x337, 0x3b8, 0x229, 0x2a6, 0x10b, 0x184, 0x15, 0x9a, 0x3c0, 0x34f, 0x2de, 0x251, 0x1fc, 0x173, 0xe2, 0x6d,
        0x2d9, 0x256, 0x3c7, 0x348, 0xe5, 0x6a, 0x1fb, 0x174, 0x22e, 0x2a1, 0x330, 0x3bf, 0x12, 0x9d, 0x10c, 0x183,
        0xeb, 0x64, 0x1f5, 0x17a, 0x2d7, 0x258, 0x3c9, 0x346, 0x1c, 0x93, 0x102, 0x18d, 0x220, 0x2af, 0x33e, 0x3b1,
        0x105, 0x18a, 0x1b, 0x94, 0x339, 0x3b6, 0x227, 0x2a8, 0x1f2, 0x17d, 0xec, 0x63, 0x3ce, 0x341, 0x2d0, 0x25f,
        0x2e1, 0x26e, 0x3ff, 0x370, 0xdd, 0x52, 0x1c3, 0x14c, 0x216, 0x299, 0x308, 0x387, 0x2a, 0xa5, 0x134, 0x1bb,
        0x30f, 0x380, 0x211, 0x29e, 0x133, 0x1bc, 0x2d, 0xa2, 0x3f8, 0x377, 0x2e6, 0x269, 0x1c4, 0x14b, 0xda, 0x55,
        0x13d, 0x1b2, 0x23, 0xac, 0x301, 0x38e, 0x21f, 0x290, 0x1ca, 0x145, 0xd4, 0x5b, 0x3f6, 0x379, 0x2e8, 0x267,
        0xd3, 0x5c, 0x1cd, 0x142, 0x2ef, 0x260, 0x3f1, 0x37e, 0x24, 0xab, 0x13a, 0x1b5, 0x218, 0x297, 0x306, 0x389,
        0x1d6, 0x159, 0xc8, 0x47, 0x3ea, 0x365, 0x2f4, 0x27b, 0x121, 0x1ae, 0x3f, 0xb0, 0x31d, 0x392, 0x203, 0x28c,
        0x38, 0xb7, 0x126, 0x1a9, 0x204, 0x28b, 0x31a, 0x395, 0xcf, 0x40, 0x1d1, 0x15e, 0x2f3, 0x27c, 0x3ed, 0x362,
        0x20a, 0x285, 0x314, 0x39b, 0x36, 0xb9, 0x128, 0x1a7, 0x2fd, 0x272, 0x3e3, 0x36c, 0xc1, 0x4e, 0x1df, 0x150,
        0x3e4, 0x36b, 0x2fa, 0x275, 0x1d8, 0x157, 0xc6, 0x49, 0x313, 0x39c, 0x20d, 0x282, 0x12f, 0x1a0, 0x31, 0xbe};

//--------------------------------------- cell_asic -----------------------------------------------------#
cell_asic BMS_IC[3];
//----------------------- Set Configuration Data Into BMS_IC for ADBMS6815 ------------------------------#
/* Helper function to set CFGR variable */
void ADBMS6815_set_cfgr(uint8_t nIC, // Current IC
      cell_asic *ic, // A two dimensional array that stores the data
      // CFGAR0
      bool refon, // The REFON bit
      bool adcopt, // The ADCOPT bit
      bool ps[2], // The PS bits
      bool cvmin[2], // The CVMIN bits
      bool mcal, // The MCAL bit
      bool commBk, // The COMM_BK bit
      // CFGAR1
      bool flagD[8], // The FLAG_D bits
      // CFGAR2
      bool soakon, // The SOAKON bit
      bool owrng, // The OWRNG bit
      bool owa[3], // The OWA bits
      bool owc[3], // The OWC bits
      // CFGAR3
      bool gpo[7], // The GPO bits
      // CFGAR4 & CFGAR5 - READ ONLY BIT: gpi rev dtype
      // CFGBR0 & CFGBR1 & CFGBR2
      uint16_t uv, // The UV value - trans to bool vuv[12]
      uint16_t ov, // The OV value - trans to bool vov[12]
      // CFGBR3
      bool dtmen, // The DTMEN bit
      bool dtrng, // The DTRNG bit
      bool dcto[6], // The DCTO bits
      // CFGBR4 & CFGBR5 - READ ONLY BIT: muteSt
      bool dcc[12] // The DCC bits
      // RSVD
      /*
         bool ar3b7Rsvd, // A_R3_Bit7
         bool ar4b7Rsvd, // A_R4_Bit7
         bool br5b4Rsvd, // B_R5_Bit4
         bool br5b5Rsvd, // B_R5_Bit5
         bool br5b6Rsvd, // B_R5_Bit6
       */
      );
/* Helper function to set the REFON bit */
void ADBMS6815_set_cfgr_refon(uint8_t nIC, cell_asic *ic, bool refon);
/* Helper function to set the ADCOPT bit*/
void ADBMS6815_set_cfgr_adcopt(uint8_t nIC, cell_asic *ic, bool adcopt);
/* Helper function to set the PS bits*/
void ADBMS6815_set_cfgr_ps(uint8_t nIC, cell_asic *ic, bool ps[]);
/* Helper function to set the CVMIN bits*/
void ADBMS6815_set_cfgr_cvmin(uint8_t nIC, cell_asic *ic, bool cvmin[]);
/* Helper function to set the MCAL bit*/
void ADBMS6815_set_cfgr_mcal(uint8_t nIC, cell_asic *ic, bool mcal);
/* Helper function to set the COMM_BK bit*/
void ADBMS6815_set_cfgr_commBk(uint8_t nIC, cell_asic *ic, bool commBk);
/* Helper function to set the FLAG_D bits*/
void ADBMS6815_set_cfgr_flagD(uint8_t nIC, cell_asic *ic, bool flagD[]);
/* Helper function to set the SOAKON bit*/
void ADBMS6815_set_cfgr_soakon(uint8_t nIC, cell_asic *ic, bool soakon);
/* Helper function to set the OWRNG bit*/
void ADBMS6815_set_cfgr_owrng(uint8_t nIC, cell_asic *ic, bool owrng);
/* Helper function to set the OWA bits*/
void ADBMS6815_set_cfgr_owa(uint8_t nIC, cell_asic *ic, bool owa[]);
/* Helper function to set the OWC bits*/
void ADBMS6815_set_cfgr_owc(uint8_t nIC, cell_asic *ic, bool owc[]);
/* Helper function to set the GPO bits*/
void ADBMS6815_set_cfgr_gpo(uint8_t nIC, cell_asic *ic, bool gpo[]);
/* Helper function to set VUV value in CFG B register */
void ADBMS6815_set_cfgr_vuv(uint8_t nIC, cell_asic *ic,uint16_t uv);
/* Helper function to set VOV value in CFG B register */
void ADBMS6815_set_cfgr_vov(uint8_t nIC, cell_asic *ic,uint16_t ov);
/* Helper function to set the DTMEN bit*/
void ADBMS6815_set_cfgr_dtmen(uint8_t nIC, cell_asic *ic, bool dtmen);
/* Helper function to set the DTRNG bit*/
void ADBMS6815_set_cfgr_dtrng(uint8_t nIC, cell_asic *ic, bool dtrng);
/* Helper function to set the DCTO bits*/
void ADBMS6815_set_cfgr_dcto(uint8_t nIC, cell_asic *ic, bool dcto[]);
/* Helper function to control discharge */
void ADBMS6815_set_cfgr_dis(uint8_t nIC, cell_asic *ic,bool dcc[]);
/* Helper function to set the RSVD(A_R3_Bit7) bit*/
void ADBMS6815_set_cfgr_ar3b7Rsvd(uint8_t nIC, cell_asic *ic, bool rsvd);
/* Helper function to set the RSVD(A_R4_Bit7) bit*/
void ADBMS6815_set_cfgr_ar4b7Rsvd(uint8_t nIC, cell_asic *ic, bool rsvd);
/* Helper function to set the RSVD(B_R5_Bit4) bit*/
void ADBMS6815_set_cfgr_br5b4Rsvd(uint8_t nIC, cell_asic *ic, bool rsvd);
/* Helper function to set the RSVD(B_R5_Bit5) bit*/
void ADBMS6815_set_cfgr_br5b5Rsvd(uint8_t nIC, cell_asic *ic, bool rsvd);
/* Helper function to set the RSVD(B_R5_Bit6) bit*/
void ADBMS6815_set_cfgr_br5b6Rsvd(uint8_t nIC, cell_asic *ic, bool rsvd);
/* Helper function to reset PEC counters */
void ADBMS6815_reset_crc_count(uint8_t total_ic, cell_asic *ic);
/* Initialize the Register limits */
void ADBMS6815_init_reg_limits(uint8_t total_ic, cell_asic *ic);
//--------------------------------------------- SPI -----------------------------------------------------#
/* SPI write*/
void spi_write_array(uint8_t len, uint8_t data[]);
/* SPI read */
uint8_t spi_read_byte(void);
/* SPI write and read */
void spi_write_read(uint8_t tx_Data[], uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len);
/* PEC 15-bit calculator */
uint16_t pec15_calc(uint8_t len, uint8_t *data);
/* PEC 10-bit calculator */
uint16_t pec10_calc(uint8_t len, uint8_t *data, bool bIsRxCmd);
/* Generic function to write ADBMS6815 commands. Function calculates PEC for tx_cmd data. */
void cmd_68(uint8_t tx_cmd[2]);
/* Generic function to write ADBMS6815 commands and read data. Function calculated PEC for tx_cmd data */
int8_t read_68(uint8_t tx_cmd[2], uint8_t *rx_data);
/* Generic function to write ADBMS6815 commands with following data. Function calculated PEC for tx_cmd data */
void write_68(uint8_t tx_cmd[2], uint8_t data[]);
/* Helper function to check PEC counters */
void ADBMS6815_check_pec(uint8_t current_ic, uint8_t reg, cell_asic *ic);
//------------------------------------- ADBMS6815 Funciton ----------------------------------------------#
/* ADBMS6815 Wakeup sleep*/
void wakeup_sleep(void);
/* ADBMS6815 poll ADC status */
uint32_t ADBMS6815_pladc(void);
/* ADBMS6815 write configuration register group A*/
void ADBMS6815_wrcfga(uint8_t current_ic, cell_asic ic[]);
/* ADBMS6815 write configuration register group B */
void ADBMS6815_wrcfgb(uint8_t current_ic, cell_asic ic[]);
/* ADBMS6815 write configuration register group A & B */
void ADBMS6815_wrcfg(uint8_t current_ic, cell_asic ic[]);
/* ADBMS6815 read configuration register group A */
int8_t ADBMS6815_rdcfga(uint8_t current_ic, cell_asic ic[]);
/* ADBMS6815 read configuration register group B */
int8_t ADBMS6815_rdcfgb(uint8_t current_ic, cell_asic ic[]);
/* ADBMS6815 read configuration register group A & B */
int8_t ADBMS6815_rdcfg(uint8_t current_ic, cell_asic ic[]);
/* ADBMS6815 start cell voltage ADC conversion and poll status*/
void ADBMS6815_adcv( uint8_t MD, uint8_t DCP, uint8_t CH);
/*
   Reads and parses the ADBMS6815 cell voltage registers.
   The function is used to read the parsed Cell voltages codes of the ADBMS6815.
   This function will send the requested read commands parse the data
   and store the cell voltages in c_codes variable.
 */
uint8_t ADBMS6815_rdcv(uint8_t reg, uint8_t current_ic, cell_asic *ic);
/* Writes the command and reads the raw cell voltage register data */
void ADBMS6815_rdcv_reg(uint8_t reg, uint8_t *data);
/* Writes the command and reads all cell voltage registers data */
void ADBMS6815_rdcvall(uint8_t *data);
/* Helper function that parses voltage measurement registers */
int8_t parse_cells(uint8_t cell_reg, uint8_t cell_data[], uint16_t *cell_codes, uint8_t *ic_pec);
/* Helper function that parses voltage measurement registers */
int8_t parse_allcells(uint8_t cell_data[], uint16_t *cell_codes, uint8_t *ic_pec);
/* Start GPIOs ADC conversion and poll status */
void ADBMS6815_adax(uint8_t MD, uint8_t CHG);
/*
   The function is used to read the parsed GPIO codes of the ADBMS6815.
   This function will send the requested read commands parse the data
   and store the gpio voltages in a_codes variable.
 */
int8_t ADBMS6815_rdaux(uint8_t reg, uint8_t current_ic, cell_asic *ic);
/*
   The function reads a single GPIO voltage register and stores the read data
   in the *data point as a byte array. This function is rarely used outside of
   the ADBMS6815_rdaux() command.
 */
void ADBMS6815_rdaux_reg(uint8_t reg, uint8_t *data);
/* ADBMS6815 start group ADC conversion and poll status */
void ADBMS6815_adstat(uint8_t MD, uint8_t CHST);
/*
   Reads and parses the ADBMS6815 stat registers.
   The function is used to read the parsed Stat codes of the ADBMS6815.
   This function will send the requested read commands parse the data
   and store the gpio voltages in stat_codes variable.
 */
int8_t ADBMS6815_rdstat(uint8_t reg, uint8_t current_ic, cell_asic *ic);
/*
   The function reads a single stat register and stores the read data
   in the *data point as a byte array. This function is rarely used outside of
   the ADBMS6815_rdstat() command.
 */
void ADBMS6815_rdstat_reg(uint8_t reg, uint8_t *data);
/* Read All Aux/Status Registers */
int8_t ADBMS6815_rdasall(uint8_t current_ic,cell_asic *ic);
/* Writes the command and reads all Aux/Status registers data */
void ADBMS6815_rdasall_reg(uint8_t *data);
/* Helper function to set discharge bit in CFG register */
void ADBMS6815_set_discharge(int Cell, uint8_t current_ic, cell_asic *ic);
/* Helper function to clear discharge bit in CFG register */
void ADBMS6815_clear_discharge(int Cell, uint8_t current_ic, cell_asic *ic);
/* ADBMS6815 mute discharge*/
void ADBMS6815_mute(void);
/* ADBMS6815 unmute discharge*/
void ADBMS6815_unmute(void);
/* MCU Restart */
void mcu_restart(void);
/* Mulitiplexer Active */
void mulitiplexer_switch(void);
/* Helper funciton to reinitialize CAN */
void CANReInit(CAN_HandleTypeDef *hcan, uint16_t delay);
/* Helper funciton to clear CAN TX buff*/
void cleartx(void);
//timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//board
void first_service(void);
void basic_service(void);
void writeConfOnly(void);
void UploadMeasureData(void);
void UploadConfigData(void);
/*Test Function Refer to BMS_CAN20220701*/
void UploadCVData(void);


/* Helper function to set CFGR variable */
void ADBMS6815_set_cfgr(uint8_t nIC, // Current IC
      cell_asic *ic, // A two dimensional array that stores the data
      // CFGAR0
      bool refon, // The REFON bit
      bool adcopt, // The ADCOPT bit
      bool ps[2], // The PS bits
      bool cvmin[2], // The CVMIN bits
      bool mcal, // The MCAL bit
      bool commBk, // The COMM_BK bit
      // CFGAR1
      bool flagD[8], // The FLAG_D bits
      // CFGAR2
      bool soakon, // The SOAKON bit
      bool owrng, // The OWRNG bit
      bool owa[3], // The OWA bits
      bool owc[3], // The OWC bits
      // CFGAR3
      bool gpo[7], // The GPO bits
      // CFGAR4 & CFGAR5 - READ ONLY BIT: gpi rev dtype
      // CFGBR0 & CFGBR1 & CFGBR2
      uint16_t uv, // The UV value - trans to bool vuv[12]
      uint16_t ov, // The OV value - trans to bool vov[12]
      // CFGBR3
      bool dtmen, // The DTMEN bit
      bool dtrng, // The DTRNG bit
      bool dcto[6], // The DCTO bits
      // CFGBR4 & CFGBR5 - READ ONLY BIT: muteSt
      bool dcc[12] // The DCC bits
      // RSVD
      /*
         bool ar3b7Rsvd, // A_R3_Bit7
         bool ar4b7Rsvd, // A_R4_Bit7
         bool br5b4Rsvd, // B_R5_Bit4
         bool br5b5Rsvd, // B_R5_Bit5
         bool br5b6Rsvd, // B_R5_Bit6
       */
      )
{
  // CFGAR0
  ADBMS6815_set_cfgr_refon(nIC,ic,refon);
  ADBMS6815_set_cfgr_adcopt(nIC,ic,adcopt);
  ADBMS6815_set_cfgr_ps(nIC,ic,ps);
  ADBMS6815_set_cfgr_cvmin(nIC,ic,cvmin);
  ADBMS6815_set_cfgr_mcal(nIC,ic,mcal);
  ADBMS6815_set_cfgr_commBk(nIC,ic,commBk);
  // CFGAR1
  ADBMS6815_set_cfgr_flagD(nIC,ic,flagD);
  // CFGAR2
  ADBMS6815_set_cfgr_soakon(nIC,ic,soakon);
  ADBMS6815_set_cfgr_owrng(nIC,ic,owrng);
  ADBMS6815_set_cfgr_owa(nIC,ic,owa);
  ADBMS6815_set_cfgr_owc(nIC,ic,owc);
  // CFGAR3
  ADBMS6815_set_cfgr_gpo(nIC,ic,gpo);
  // CFGAR4 & CFGAR5 - READ ONLY BIT: gpi rev dtype
  // CFGBR0 & CFGBR1 & CFGBR2
  ADBMS6815_set_cfgr_vuv(nIC,ic,uv);
  ADBMS6815_set_cfgr_vov(nIC,ic,ov);
  // CFGBR3
  ADBMS6815_set_cfgr_dtmen(nIC,ic,dtmen);
  ADBMS6815_set_cfgr_dtrng(nIC,ic,dtrng);
  ADBMS6815_set_cfgr_dcto(nIC,ic,dcto);
  // CFGBR4 & CFGBR5 - READ ONLY BIT: muteSt
  ADBMS6815_set_cfgr_dis(nIC,ic,dcc);
  // RSVD
  // ADBMS6815_set_cfgr_ar3b7Rsvd(nIC,ic,ar3b7Rsvd);
  // ADBMS6815_set_cfgr_ar4b7Rsvd(nIC,ic,ar4b7Rsvd);
  // ADBMS6815_set_cfgr_br5b4Rsvd(nIC,ic,br5b4Rsvd);
  // ADBMS6815_set_cfgr_br5b5Rsvd(nIC,ic,br5b5Rsvd);
  // ADBMS6815_set_cfgr_br5b6Rsvd(nIC,ic,br5b6Rsvd);
}

/* Helper function to set the REFON bit */
void ADBMS6815_set_cfgr_refon(uint8_t nIC, cell_asic *ic, bool refon)
{
  if (refon) ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|0x80;
  else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&0x7F;
}

/* Helper function to set the ADCOPT bit*/
void ADBMS6815_set_cfgr_adcopt(uint8_t nIC, cell_asic *ic, bool adcopt)
{
  if (adcopt) ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|0x40;
  else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&0xBF;
}

/* Helper function to set the PS bits*/
void ADBMS6815_set_cfgr_ps(uint8_t nIC, cell_asic *ic, bool ps[])
{
  for (int i = 0; i < 2; i++)
  {
    if (ps[i]) ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|(0x01<<(i+4));
    else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&(~(0x01<<(i+4)));
  }
}

/* Helper function to set the CVMIN bits*/
void ADBMS6815_set_cfgr_cvmin(uint8_t nIC, cell_asic *ic, bool cvmin[])
{
  for (int i = 0; i < 2; i++)
  {
    if (cvmin[i]) ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|(0x01<<(i+2));
    else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&(~(0x01<<(i+2)));
  }
}

/* Helper function to set the MCAL bit*/
void ADBMS6815_set_cfgr_mcal(uint8_t nIC, cell_asic *ic, bool mcal)
{
  if (mcal) ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|0x02;
  else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&0xFD;
}

/* Helper function to set the COMM_BK bit*/
void ADBMS6815_set_cfgr_commBk(uint8_t nIC, cell_asic *ic, bool commBk)
{
  if (commBk) ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]|0x01;
  else ic[nIC].config.tx_data[0] = ic[nIC].config.tx_data[0]&0xFE;
}

/* Helper function to set the FLAG_D bits*/
void ADBMS6815_set_cfgr_flagD(uint8_t nIC, cell_asic *ic, bool flagD[])
{
  for (int i = 0; i < 8; i++)
  {
    if (flagD[i]) ic[nIC].config.tx_data[1] = ic[nIC].config.tx_data[1]|(0x01<<(i));
    else ic[nIC].config.tx_data[1] = ic[nIC].config.tx_data[1]&(~(0x01<<(i)));
  }
}

/* Helper function to set the SOAKON bit*/
void ADBMS6815_set_cfgr_soakon(uint8_t nIC, cell_asic *ic, bool soakon)
{
  if (soakon) ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]|0x80;
  else ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]&0x7F;
}

/* Helper function to set the OWRNG bit*/
void ADBMS6815_set_cfgr_owrng(uint8_t nIC, cell_asic *ic, bool owrng)
{
  if (owrng) ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]|0x40;
  else ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]&0xBF;
}

/* Helper function to set the OWA bits*/
void ADBMS6815_set_cfgr_owa(uint8_t nIC, cell_asic *ic, bool owa[])
{
  for (int i = 0; i < 3; i++)
  {
    if (owa[i]) ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]|(0x01<<(i+3));
    else ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]&(~(0x01<<(i+3)));
  }
}

/* Helper function to set the OWC bits*/
void ADBMS6815_set_cfgr_owc(uint8_t nIC, cell_asic *ic, bool owc[])
{
  for (int i = 0; i < 3; i++)
  {
    if (owc[i]) ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]|(0x01<<(i));
    else ic[nIC].config.tx_data[2] = ic[nIC].config.tx_data[2]&(~(0x01<<(i)));
  }
}

/* Helper function to set the GPO bits*/
void ADBMS6815_set_cfgr_gpo(uint8_t nIC, cell_asic *ic, bool gpo[])
{
  for (int i = 0; i < 7; i++)
  {
    if (gpo[i]) ic[nIC].config.tx_data[3] = ic[nIC].config.tx_data[3]|(0x01<<(i));
    else ic[nIC].config.tx_data[3] = ic[nIC].config.tx_data[3]&(~(0x01<<(i)));
  }
}

/* Helper Function to set VUV value in CFG B register */
void ADBMS6815_set_cfgr_vuv(uint8_t nIC, cell_asic *ic,uint16_t uv)
{
  // Comparison voltage = VUV[11:0]*16*100uV
  uint16_t tmp = (uv/16);
  ic[nIC].configb.tx_data[0] = 0x00FF & tmp;
  ic[nIC].configb.tx_data[1] = ic[nIC].configb.tx_data[1]&0xF0;
  ic[nIC].configb.tx_data[1] = ic[nIC].configb.tx_data[1]|((0x0F00 & tmp)>>8);
}

/* Helper Function to set VOV value in CFG B register */
void ADBMS6815_set_cfgr_vov(uint8_t nIC, cell_asic *ic, uint16_t ov)
{
  // Comparison voltage = VOV[11:0]*16*100uV
  uint16_t tmp = (ov/16);
  ic[nIC].configb.tx_data[1] = ic[nIC].configb.tx_data[1]&0x0F;
  ic[nIC].configb.tx_data[1] = ic[nIC].configb.tx_data[1]|((0x000F & tmp)<<4);
  ic[nIC].configb.tx_data[2] = 0x00FF & (tmp>>4);
}

/* Helper function to set the DTMEN bit*/
void ADBMS6815_set_cfgr_dtmen(uint8_t nIC, cell_asic *ic, bool dtmen)
{
  if (dtmen) ic[nIC].configb.tx_data[3] = ic[nIC].configb.tx_data[3]|0x80;
  else ic[nIC].configb.tx_data[3] = ic[nIC].configb.tx_data[3]&0x7F;
}

/* Helper function to set the DTRNG bit*/
void ADBMS6815_set_cfgr_dtrng(uint8_t nIC, cell_asic *ic, bool dtrng)
{
  if (dtrng) ic[nIC].configb.tx_data[3] = ic[nIC].configb.tx_data[3]|0x40;
  else ic[nIC].configb.tx_data[3] = ic[nIC].configb.tx_data[3]&0xBF;
}

/* Helper function to set the DCTO bits*/
void ADBMS6815_set_cfgr_dcto(uint8_t nIC, cell_asic *ic, bool dcto[])
{
  for (int i = 0; i < 6; i++)
  {
    if (dcto[i]) ic[nIC].configb.tx_data[3] = ic[nIC].configb.tx_data[3]|(0x01<<(i));
    else ic[nIC].configb.tx_data[3] = ic[nIC].configb.tx_data[3]&(~(0x01<<(i)));
  }
}

/* Helper function to control discharge */
void ADBMS6815_set_cfgr_dis(uint8_t nIC, cell_asic *ic,bool dcc[])
{
  for (int i =0; i<8; i++)
  {
    if (dcc[i]) ic[nIC].configb.tx_data[4] = ic[nIC].configb.tx_data[4]|(0x01<<i);
    else ic[nIC].configb.tx_data[4] = ic[nIC].configb.tx_data[4]& (~(0x01<<i));
  }
  for (int i =0; i<4; i++)
  {
    if (dcc[i+8]) ic[nIC].configb.tx_data[5] = ic[nIC].configb.tx_data[5]|(0x01<<i);
    else ic[nIC].configb.tx_data[5] = ic[nIC].configb.tx_data[5]&(~(0x01<<i));
  }
}

/* Helper function to set the RSVD(A_R3_Bit7) bit*/
void ADBMS6815_set_cfgr_ar3b7Rsvd(uint8_t nIC, cell_asic *ic, bool rsvd)
{
  if (rsvd) ic[nIC].config.tx_data[3] = ic[nIC].config.tx_data[3]|0x80;
  else ic[nIC].config.tx_data[3] = ic[nIC].config.tx_data[3]&0x7F;
}

/* Helper function to set the RSVD(A_R4_Bit7) bit*/
void ADBMS6815_set_cfgr_ar4b7Rsvd(uint8_t nIC, cell_asic *ic, bool rsvd)
{
  if (rsvd) ic[nIC].config.tx_data[4] = ic[nIC].config.tx_data[4]|0x80;
  else ic[nIC].config.tx_data[4] = ic[nIC].config.tx_data[4]&0x7F;
}

/* Helper function to set the RSVD(B_R5_Bit4) bit*/
void ADBMS6815_set_cfgr_br5b4Rsvd(uint8_t nIC, cell_asic *ic, bool rsvd)
{
  if (rsvd) ic[nIC].configb.tx_data[5] = ic[nIC].configb.tx_data[5]|0x40;
  else ic[nIC].configb.tx_data[5] = ic[nIC].configb.tx_data[5]&0x7F;
}

/* Helper function to set the RSVD(B_R5_Bit5) bit*/
void ADBMS6815_set_cfgr_br5b5Rsvd(uint8_t nIC, cell_asic *ic, bool rsvd)
{
  if (rsvd) ic[nIC].configb.tx_data[5] = ic[nIC].configb.tx_data[5]|0x20;
  else ic[nIC].configb.tx_data[5] = ic[nIC].configb.tx_data[5]&0xDF;
}

/* Helper function to set the RSVD(B_R5_Bit6) bit*/
void ADBMS6815_set_cfgr_br5b6Rsvd(uint8_t nIC, cell_asic *ic, bool rsvd)
{
  if (rsvd) ic[nIC].configb.tx_data[5] = ic[nIC].configb.tx_data[5]|0x10;
  else ic[nIC].configb.tx_data[5] = ic[nIC].configb.tx_data[5]&0xEF;
}

/* Helper Function to reset PEC counters */
void ADBMS6815_reset_crc_count(uint8_t total_ic, cell_asic *ic)
{
  for (int current_ic = 0; current_ic < total_ic; current_ic++)
  {
    ic[current_ic].crc_count.pec_count = 0;
    ic[current_ic].crc_count.cfgr_pec = 0;
    for (int i=0; i<6; i++)
    {
      ic[current_ic].crc_count.cell_pec[i]=0;
    }
    for (int i=0; i<4; i++)
    {
      ic[current_ic].crc_count.aux_pec[i]=0;
    }
    for (int i=0; i<2; i++)
    {
      ic[current_ic].crc_count.stat_pec[i]=0;
    }
  }
}

/* Initialize the Register limits */
void ADBMS6815_init_reg_limits(uint8_t total_ic, cell_asic *ic)
{
  for (uint8_t cic=0; cic<total_ic; cic++)
  {
    ic[cic].ic_reg.cell_channels=12;
    ic[cic].ic_reg.stat_channels=4;
    ic[cic].ic_reg.aux_channels=6;
    ic[cic].ic_reg.num_cv_reg=4; // amount of Cell VOltage Register Groups
    ic[cic].ic_reg.num_gpio_reg=3; // amount of Auxiliary Register Groups
    ic[cic].ic_reg.num_stat_reg=3; // amount of Status Register Groups
  }
}

/* SPI write*/
void spi_write_array(uint8_t len, uint8_t data[])
{
  for (uint8_t i = 0; i < len; i++)
  {
    //HAL_SPI_TransmitReceive(&hspi1,&data[i],&receive,1,100);
    SPI1_ReadWriteByte(data[i]);
  }
}

/* SPI read */
uint8_t spi_read_byte()
{
  uint8_t data;
  uint8_t tem_data = 0xFF;
  //HAL_SPI_TransmitReceive(&hspi1,&tem_data,&data,1,100);
  data = SPI1_ReadWriteByte(tem_data);
  return(data);
}

/* SPI write and read */
void spi_write_read(uint8_t tx_Data[], uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len)
{
  uint8_t data = 0xFF;
  for (uint8_t i = 0; i < tx_len; i++)
  {
    //HAL_SPI_TransmitReceive(&hspi1,&tx_Data[i],&receive,1,100);
    SPI1_ReadWriteByte(tx_Data[i]);
  }
  for (uint8_t i = 0; i < rx_len; i++)
  {
    //HAL_SPI_TransmitReceive(&hspi1,&data,&rx_data[i],1,100);
    rx_data[i] = SPI1_ReadWriteByte(data);
  }
}

/* PEC 15-bit calculator  */
uint16_t pec15_calc(uint8_t len, uint8_t *data)
{
  uint16_t remainder,addr;
  remainder = 16;//initialize the PEC
  for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
  {
    addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
    remainder = (remainder<<8)^crc15Table[addr];
  }
  return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/* PEC 10-bit calculator */
uint16_t pec10_calc(uint8_t len, uint8_t *data, bool bIsRxCmd)
{
  uint16_t remainder,addr;
  remainder = 16;//initialize the PEC
  for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
  {
    addr = ((remainder>>2)^data[i])&0xff;//calculate PEC table address
    remainder = (remainder<<8)^crc10Table[addr];
  }

  if (bIsRxCmd == true)
  {
    remainder ^= (uint16_t)(((uint16_t)data[len] & (uint8_t)0xFC) << 2);
  }
  uint8_t nBitIndex;
  uint16_t nPolynomial = 0x8F;
  for(nBitIndex = 6; nBitIndex > 0; --nBitIndex)
  {
    if ((remainder & 0x200) > 0)
    {
      remainder = (uint16_t)(remainder << 1);
      remainder = (uint16_t)(remainder ^ nPolynomial);
    }
    else
    {
      remainder = (uint16_t)(remainder << 1);
    }
  }

  remainder = remainder&0x03ff;
  return(remainder);
}


/* Generic function to write ADBMS6815 commands. Function calculates PEC for tx_cmd data. */
void cmd_68(uint8_t tx_cmd[2])  //The command to be transmitted
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  //uint8_t md_bits;

  cmd[0] = tx_cmd[0];
  cmd[1] =  tx_cmd[1];
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_RESET);
  spi_write_array(4,cmd);
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_SET);
}

/* Generic function to write ADBMS6815 commands and read data. Function calculated PEC for tx_cmd data */
int8_t read_68(uint8_t tx_cmd[2], // The command to be transmitted
         uint8_t *rx_data // Data to be read
         )
{
  const uint8_t BYTES_IN_REG = 8;
  uint8_t cmd[4];
  uint8_t data[256];
  int8_t pec_error = 0;
  uint16_t cmd_pec;
  uint16_t data_pec;
  uint16_t received_pec;

  cmd[0] = tx_cmd[0];
  cmd[1] = tx_cmd[1];
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_RESET);
  spi_write_read(cmd, 4, data, BYTES_IN_REG); //Transmits the command and reads the configuration data of all ICs on the daisy chain into rx_data[] array
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_SET);


  for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
  {
    rx_data[current_byte] = data[current_byte];
  }
  received_pec = (rx_data[6]<<8) + rx_data[7];
  data_pec = pec10_calc(6, &rx_data[0], true);
  if ((received_pec&0x03ff) != data_pec)
  {
    pec_error = -1;
  }

  return(pec_error);
}

/* Generic function to write ADBMS6815 commands with following data. Function calculates PEC for tx_cmd data.*/
void write_68(uint8_t tx_cmd[2], uint8_t data[])
{
  const uint8_t BYTES_IN_REG = 6;
  const uint8_t CMD_LEN = 4+8;
  uint8_t *cmd;
  uint16_t data_pec;
  uint16_t cmd_pec;
  uint8_t cmd_index;

  cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));
  cmd[0] = tx_cmd[0];
  cmd[1] = tx_cmd[1];
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  cmd_index = 4;
  for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
  {
    cmd[cmd_index] = data[current_byte];
    cmd_index = cmd_index + 1;
  }
  data_pec = (uint16_t)pec10_calc(BYTES_IN_REG, &data[0], false); // Calculating the PEC for each ICs configuration register data
  cmd[cmd_index] = (uint8_t)(data_pec >> 8);
  cmd[cmd_index + 1] = (uint8_t)data_pec;
  cmd_index = cmd_index + 2;


  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_RESET);
  spi_write_array(CMD_LEN, cmd);
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_SET);

  free(cmd);
}

/* Helper function to check PEC counters */
void ADBMS6815_check_pec(uint8_t current_ic,  //Index of current IC
       uint8_t reg, //Type of Register
       cell_asic *ic //A two dimensional array that stores the data
       )
{
  switch (reg)
  {
  case CFGR:
    ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].config.rx_pec_match;
    ic[current_ic].crc_count.cfgr_pec = ic[current_ic].crc_count.cfgr_pec + ic[current_ic].config.rx_pec_match;
    break;
  case CFGRB:
    ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].configb.rx_pec_match;
    ic[current_ic].crc_count.cfgr_pec = ic[current_ic].crc_count.cfgr_pec + ic[current_ic].configb.rx_pec_match;
    break;
  case CELL:
    for (int i=0; i<ic[0].ic_reg.num_cv_reg; i++)
    {
      ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].cells.pec_match[i];
      ic[current_ic].crc_count.cell_pec[i] = ic[current_ic].crc_count.cell_pec[i] + ic[current_ic].cells.pec_match[i];
    }
    break;
  case AUX:
    for (int i=0; i<ic[0].ic_reg.num_gpio_reg; i++)
    {
      ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + (ic[current_ic].aux.pec_match[i]);
      ic[current_ic].crc_count.aux_pec[i] = ic[current_ic].crc_count.aux_pec[i] + (ic[current_ic].aux.pec_match[i]);
    }
    break;
  case STAT:
    for (int i=0; i<ic[0].ic_reg.num_stat_reg-1; i++)
    {
      ic[current_ic].crc_count.pec_count = ic[current_ic].crc_count.pec_count + ic[current_ic].stat.pec_match[i];
      ic[current_ic].crc_count.stat_pec[i] = ic[current_ic].crc_count.stat_pec[i] + ic[current_ic].stat.pec_match[i];
    }
    break;
  default:
    break;
  }
}


/* ADBMS6815 Wakeup sleep*/
void wakeup_sleep()
{
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_RESET);
  delay_us(500);
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_SET);
  delay_us(500);
}

/* ADBMS6815 poll ADC status */
uint32_t ADBMS6815_pladc()
{
  uint32_t counter = 0;
  uint8_t finished = 0;
  uint8_t current_time = 1;
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x07;
  cmd[1] = 0x14;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_RESET);
  spi_write_array(4,cmd);
  while ((counter<200000)&&(finished == 0))
  {
    current_time = spi_read_byte();
    if (current_time>0)
    {
      finished = 1;
    }
    else
    {
      counter = counter + 10;
    }
  }
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_SET);
  return(counter);
}

/* ADBMS6815 write configuration register group A*/
void ADBMS6815_wrcfga(uint8_t current_ic, //The index of current IC being written to
          cell_asic ic[] // A two dimensional array of the configuration data that will be written
          )
{
  uint8_t cmd[2] = {0x00, 0x01};
  uint8_t write_buffer[256];
  uint8_t write_count = 0;

  for (uint8_t data = 0; data<6; data++)
  {
    write_buffer[write_count] = ic[current_ic].config.tx_data[data];
    write_count++;
  }

  write_68(cmd, write_buffer);
}

/* ADBMS6815 write configuration register group B */
void ADBMS6815_wrcfgb(uint8_t current_ic, //The index of current IC being written to
          cell_asic ic[] // A two dimensional array of the configuration data that will be written
          )
{
  uint8_t cmd[2] = {0x00, 0x24};
  uint8_t write_buffer[256];
  uint8_t write_count = 0;

  for (uint8_t data = 0; data<6; data++)
  {
    write_buffer[write_count] = ic[current_ic].configb.tx_data[data];
    write_count++;
  }
  write_68(cmd, write_buffer);
}

/* ADBMS6815 write configuration register group A & B */
void ADBMS6815_wrcfg(uint8_t current_ic, //The index of current IC being written to
         cell_asic ic[] // A two dimensional array of the configuration data that will be written
         )
{
  ADBMS6815_wrcfga(current_ic, ic);
  ADBMS6815_wrcfgb(current_ic, ic);
}

/* ADBMS6815 read configuration register group A */
int8_t ADBMS6815_rdcfga(uint8_t current_ic, //Index of current IC
      cell_asic ic[] // A two dimensional array that the function stores the read configuration data.
      )
{
  uint8_t cmd[2]= {0x00, 0x02};
  uint8_t read_buffer[256];
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t calc_pec;

  pec_error = read_68(cmd, read_buffer);

  for (int byte=0; byte<8; byte++)
  {
    ic[current_ic].config.rx_data[byte] = read_buffer[byte];
  }
  calc_pec = pec10_calc(6,&read_buffer[0],true);
  data_pec = read_buffer[7] | (read_buffer[6]<<8);
  if (calc_pec != (data_pec&0x3ff))
  {
    ic[current_ic].config.rx_pec_match = 1;
  }
  else ic[current_ic].config.rx_pec_match = 0;

  ADBMS6815_check_pec(current_ic,CFGR,ic);
  return(pec_error);
}

/* ADBMS6815 read configuration register group B */
int8_t ADBMS6815_rdcfgb(uint8_t current_ic, //Index of current IC
      cell_asic ic[] // A two dimensional array that the function stores the read configuration data.
      )
{
  uint8_t cmd[2]= {0x00, 0x26};
  uint8_t read_buffer[256];
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t calc_pec;

  pec_error = read_68(cmd, read_buffer);

  for (int byte=0; byte<8; byte++)
  {
    ic[current_ic].configb.rx_data[byte] = read_buffer[byte];
  }
  calc_pec = pec10_calc(6,&read_buffer[0],true);
  data_pec = read_buffer[7] | (read_buffer[6]<<8);
  if (calc_pec != (data_pec&0x3ff))
  {
    ic[current_ic].configb.rx_pec_match = 1;
  }
  else ic[current_ic].configb.rx_pec_match = 0;

  ADBMS6815_check_pec(current_ic,CFGRB,ic);
  return(pec_error);
}

/* ADBMS6815 read configuration register group A & B */
int8_t ADBMS6815_rdcfg(uint8_t current_ic, //Index of current IC
           cell_asic ic[] // A two dimensional array that the function stores the read configuration data.
           )
{
  int8_t pec_errorA = ADBMS6815_rdcfga(current_ic, ic);
  int8_t pec_errorB = ADBMS6815_rdcfgb(current_ic, ic);
  return(pec_errorA+pec_errorB);
}

/* ADBMS6815 start cell voltage ADC conversion and poll status*/
void ADBMS6815_adcv( uint8_t MD, //ADC Mode
         uint8_t DCP, //Discharge Permit
         uint8_t CH //Cell Channels to be measured
         )
{
  uint8_t cmd[2];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + 0x60 + (DCP<<4) + CH;

  cmd_68(cmd);
}

/*
   Reads and parses the ADBMS6815 cell voltage registers.
   The function is used to read the parsed Cell voltages codes of the ADBMS6815.
   This function will send the requested read commands parse the data
   and store the cell voltages in c_codes variable.
 */
uint8_t ADBMS6815_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
           uint8_t current_ic, // The index of current IC
           cell_asic *ic // Array of the parsed cell codes
           )
{
  int8_t pec_error = 0;

  if (reg == 0)
  {
		uint8_t *cell_data;
		cell_data = (uint8_t *) malloc((6*4+2)*sizeof(uint8_t));
    ADBMS6815_rdcvall(cell_data);
		pec_error = pec_error + parse_allcells(cell_data,
                  &ic[current_ic].cells.c_codes[0],
                  &ic[current_ic].cells.pec_match[0]);
		free(cell_data);
  }
  else
  {
		uint8_t *cell_data;
		cell_data = (uint8_t *) malloc((NUM_RX_BYT)*sizeof(uint8_t));
    ADBMS6815_rdcv_reg(reg, cell_data);
    pec_error = pec_error + parse_cells(reg, cell_data,
                &ic[current_ic].cells.c_codes[0],
                &ic[current_ic].cells.pec_match[0]);
		free(cell_data);
  }
  ADBMS6815_check_pec(current_ic,CELL,ic);

  return(pec_error);
}

/* Writes the command and reads the raw cell voltage register data */
void ADBMS6815_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
      uint8_t *data //An array of the unparsed cell codes
      )
{
  const uint8_t REG_LEN = 8; //Number of bytes in each ICs register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;

  if (reg == 1) //1: RDCVA
  {
    cmd[1] = 0x04;
    cmd[0] = 0x00;
  }
  else if (reg == 2) //2: RDCVB
  {
    cmd[1] = 0x06;
    cmd[0] = 0x00;
  }
  else if (reg == 3) //3: RDCVC
  {
    cmd[1] = 0x08;
    cmd[0] = 0x00;
  }
  else if (reg == 4) //4: RDCVD
  {
    cmd[1] = 0x0A;
    cmd[0] = 0x00;
  }

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_RESET);
  spi_write_read(cmd,4,data,REG_LEN);
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_SET);
}

/* Writes the command and reads all cell voltage registers data */
void ADBMS6815_rdcvall(uint8_t *data //An array of the unparsed cell codes
           )
{
  const uint8_t REG_LEN = 26; //Number of bytes in each ICs register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[1] = 0x38;
  cmd[0] = 0x00;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_RESET);
  spi_write_read(cmd,4,data,REG_LEN);
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_SET);
}

/* Helper function that parses voltage measurement registers */
int8_t parse_cells(uint8_t cell_reg, // Type of register
       uint8_t cell_data[], // Unparsed data
       uint16_t *cell_codes, // Parsed data
       uint8_t *ic_pec // PEC error
       )
{
  const uint8_t BYT_IN_REG = 6;
  const uint8_t CELL_IN_REG = 3;
  int8_t pec_error = 0;
  uint16_t parsed_cell;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t data_counter = 0; //data counter


  for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++) // This loop parses the read back data into the register codes, it
  { // loops once for each of the 3 codes in the register

    parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);//Each code is received as two bytes and is combined to
    // create the parsed code
    cell_codes[current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;

    data_counter = data_counter + 2; //Because the codes are two bytes, the data counter
    //must increment by two for each parsed code
  }
  received_pec = (cell_data[data_counter] << 8) | cell_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
  //after the 6 cell voltage data bytes
  data_pec = pec10_calc(BYT_IN_REG, &cell_data[0],true);

  if ((received_pec&0x3ff) != data_pec)
  {
    pec_error = 1; //The pec_error variable is simply set negative if any PEC errors
    ic_pec[cell_reg-1]=1;
  }
  else
  {
    ic_pec[cell_reg-1]=0;
  }
  data_counter=data_counter+2;

  return(pec_error);
}

/* Helper function that parses voltage measurement registers */
int8_t parse_allcells(uint8_t cell_data[], // Unparsed data
       uint16_t *cell_codes, // Parsed data
       uint8_t *ic_pec // PEC error
       )
{
  const uint8_t BYT_IN_REG = 24;
  const uint8_t CELL_IN_REG = 12;
  int8_t pec_error = 0;
  uint16_t parsed_cell;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t data_counter = 0; //data counter


  for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++) // This loop parses the read back data into the register codes, it
  { // loops once for each of the 3 codes in the register

    parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);//Each code is received as two bytes and is combined to
    // create the parsed code
    cell_codes[current_cell] = parsed_cell;

    data_counter = data_counter + 2; //Because the codes are two bytes, the data counter
    //must increment by two for each parsed code
  }
  received_pec = (cell_data[data_counter] << 8) | cell_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
  //after the 6 cell voltage data bytes
  data_pec = pec10_calc(BYT_IN_REG, &cell_data[0],true);
	
  if ((received_pec&0x3ff) != data_pec)
  {
    pec_error = 1; //The pec_error variable is simply set negative if any PEC errors
    ic_pec[0]=1;
    ic_pec[1]=1;
    ic_pec[2]=1;
    ic_pec[3]=1;
  }
  else
  {
    ic_pec[0]=0;
    ic_pec[1]=0;
    ic_pec[2]=0;
    ic_pec[3]=0;
  }
  data_counter=data_counter+2;
	

  return(pec_error);
}

/* Start GPIOs ADC conversion and poll status */
void ADBMS6815_adax(uint8_t MD, //ADC Mode
        uint8_t CHG //GPIO Channels to be measured
        )
{
  uint8_t cmd[4];
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + CHG;

  cmd_68(cmd);
}

/*
   The function is used to read the parsed GPIO codes of the ADBMS6815.
   This function will send the requested read commands parse the data
   and store the gpio voltages in a_codes variable.
 */
int8_t ADBMS6815_rdaux(uint8_t reg, //Determines which GPIO voltage register is read back.
           uint8_t current_ic,//The index of IC in the system
           cell_asic *ic//A two dimensional array of the gpio voltage codes.
           )
{
  uint8_t *data;
  int8_t pec_error = 0;
  data = (uint8_t *) malloc(NUM_RX_BYT*sizeof(uint8_t));

  if (reg == 0)
  {
    for (uint8_t gpio_reg = 1; gpio_reg<ic[0].ic_reg.num_gpio_reg+1; gpio_reg++) //Executes once for each of the LTC681x aux voltage registers
    {
      ADBMS6815_rdaux_reg(gpio_reg, data); //Reads the raw auxiliary register data into the data[] array
      pec_error = parse_cells(gpio_reg, data,
            &ic[current_ic].aux.a_codes[0],
            &ic[current_ic].aux.pec_match[0]);
    }
  }
  else
  {
    ADBMS6815_rdaux_reg(reg, data);
    pec_error = parse_cells(reg, data,
          &ic[current_ic].aux.a_codes[0],
          &ic[current_ic].aux.pec_match[0]);
  }
  ADBMS6815_check_pec(current_ic,AUX,ic);
  free(data);

  return (pec_error);
}

/*
   The function reads a single GPIO voltage register and stores the read data
   in the *data point as a byte array. This function is rarely used outside of
   the ADBMS6815_rdaux() command.
 */
void ADBMS6815_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
       uint8_t *data //Array of the unparsed auxiliary codes
       )
{
  const uint8_t REG_LEN = 8; // Number of bytes in the register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;

  if (reg == 1) //Read back auxiliary group A
  {
    cmd[1] = 0x0C;
    cmd[0] = 0x00;
  }
  else if (reg == 2) //Read back auxiliary group B
  {
    cmd[1] = 0x0E;
    cmd[0] = 0x00;
  }
  else if (reg == 3) //Read back auxiliary group C
  {
    cmd[1] = 0x0D;
    cmd[0] = 0x00;
  }

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_RESET);
  spi_write_read(cmd,4,data,REG_LEN);
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_SET);
}

/* ADBMS6815 start group ADC conversion and poll status */

void ADBMS6815_adstat(uint8_t MD, uint8_t CHST)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x05;//md_bits + 0x04;
  cmd[1] = 0x68;//md_bits + 0x68 + CHST ;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_RESET);
  spi_write_array(4,cmd);
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_SET);
}

/*
   Reads and parses the ADBMS6815 stat registers.
   The function is used to read the parsed Stat codes of the ADBMS6815.
   This function will send the requested read commands parse the data
   and store the gpio voltages in stat_codes variable.
 */
int8_t ADBMS6815_rdstat(uint8_t reg, //Determines which Stat  register is read back.
      uint8_t current_ic,//The index of current IC
      cell_asic *ic //A two dimensional array of the stat codes.
      )
{
  const uint8_t BYT_IN_REG = 6;
  const uint8_t STAT_IN_REG = 3;
  uint8_t *data;
  uint8_t data_counter = 0;
  int8_t pec_error = 0;
  uint16_t parsed_stat;
  uint16_t received_pec;
  uint16_t data_pec;

  data = (uint8_t *) malloc((12)*sizeof(uint8_t));

  if (reg == 0)
  {
    for (uint8_t stat_reg = 1; stat_reg< ic[0].ic_reg.num_stat_reg+1; stat_reg++) //Executes once for each of the LTC681x stat voltage registers
    {
      data_counter = 0;
      ADBMS6815_rdstat_reg(stat_reg, data); //Reads the raw status register data into the data[] array
      if (stat_reg ==1)
      {
        for (uint8_t current_stat = 0; current_stat< STAT_IN_REG; current_stat++) // This loop parses the read back data into Status registers,
        { // it loops once for each of the 3 stat codes in the register
          parsed_stat = data[data_counter] + (data[data_counter+1]<<8); //Each stat codes is received as two bytes and is combined to create the parsed status code
          ic[current_ic].stat.stat_codes[current_stat] = parsed_stat;
          data_counter=data_counter+2; //Because stat codes are two bytes the data counter
        }
      }
      else if (stat_reg == 2)
      {
        parsed_stat = data[data_counter] + (data[data_counter+1]<<8); //Each stat is received as two bytes and is combined to create the parsed status code
        data_counter = data_counter +2;
        ic[current_ic].stat.stat_codes[3] = parsed_stat;
        ic[current_ic].stat.flags[0] = data[data_counter++];
        ic[current_ic].stat.flags[1] = data[data_counter++];
        ic[current_ic].stat.flags[2] = data[data_counter++];
        ic[current_ic].stat.oc_cntr[0] = data[data_counter++];
      }
      else if (stat_reg == 3)
      {
        ic[current_ic].stat.va_ovhi[0] = (data[data_counter] & 0x80)>>7;
        ic[current_ic].stat.va_uvlo[0] = (data[data_counter] & 0x40)>>6;
        ic[current_ic].stat.vd_ovhi[0] = (data[data_counter] & 0x20)>>5;
        ic[current_ic].stat.vd_uvlo[0] = (data[data_counter] & 0x10)>>4;
        ic[current_ic].stat.a_otp_ed[0] = (data[data_counter] & 0x08)>>3;
        ic[current_ic].stat.a_otp_med[0] = (data[data_counter] & 0x04)>>2;
        ic[current_ic].stat.otp_ed[0] = (data[data_counter] & 0x02)>>1;
        ic[current_ic].stat.otp_med[0] = (data[data_counter++] & 0x01);
        ic[current_ic].stat.redfail[0] = (data[data_counter] & 0x80)>>7;
        ic[current_ic].stat.compchk[0] = (data[data_counter] & 0x40)>>6;
        ic[current_ic].stat.sleep[0] = (data[data_counter] & 0x20)>>5;
        ic[current_ic].stat.tmodechk[0] = (data[data_counter] & 0x10)>>4;
        ic[current_ic].stat.muxfail[0] = (data[data_counter] & 0x08)>>3;
        ic[current_ic].stat.thsd[0] = (data[data_counter] & 0x04)>>2;
        ic[current_ic].stat.cpchk[0] = (data[data_counter] & 0x02)>>1;
        ic[current_ic].stat.oscchk[0] = (data[data_counter++] & 0x01);
        parsed_stat = data[data_counter] + (data[data_counter+1]<<8); //Each stat codes is received as two bytes and is combined to create the parsed status code
        ic[current_ic].stat.adol[0] = parsed_stat;
        data_counter=data_counter+2; //Because stat codes are two bytes the data counter
        parsed_stat = data[data_counter] + (data[data_counter+1]<<8); //Each stat codes is received as two bytes and is combined to create the parsed status code
        ic[current_ic].stat.adol[1] = parsed_stat;
        data_counter=data_counter+2; //Because stat codes are two bytes the data counter
      }

      received_pec = (data[data_counter]<<8)+ data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 status data bytes
      data_pec = pec10_calc(BYT_IN_REG, &data[0], true);
      if ((received_pec&0x3ff) != data_pec)
      {
        pec_error = -1; //The pec_error variable is simply set negative if any PEC errors
        ic[current_ic].stat.pec_match[stat_reg-1]=1; //are detected in the received serial data
      }
      else
      {
        ic[current_ic].stat.pec_match[stat_reg-1]=0;
      }
      data_counter=data_counter+2; //Because the transmitted PEC code is 2 bytes long the data_counter
      //must be incremented by 2 bytes to point to the next ICs status data

    }
  }
  else
  {
    ADBMS6815_rdstat_reg(reg, data);
    if (reg ==1)
    {
      for (uint8_t current_stat = 0; current_stat< STAT_IN_REG; current_stat++) // This loop parses the read back data into Status voltages, it
      { // loops once for each of the 3 stat codes in the register
        parsed_stat = data[data_counter] + (data[data_counter+1]<<8); //Each stat codes is received as two bytes and is combined to
        // create the parsed stat code
        ic[current_ic].stat.stat_codes[current_stat] = parsed_stat;
        data_counter=data_counter+2; //Because stat codes are two bytes the data counter
        //must increment by two for each parsed stat code
      }
    }
    else if (reg == 2)
    {
      parsed_stat = data[data_counter++] + (data[data_counter++]<<8); //Each stat codes is received as two bytes and is combined to
      ic[current_ic].stat.stat_codes[3] = parsed_stat;
      ic[current_ic].stat.flags[0] = data[data_counter++];
      ic[current_ic].stat.flags[1] = data[data_counter++];
      ic[current_ic].stat.flags[2] = data[data_counter++];
      ic[current_ic].stat.oc_cntr[0] = data[data_counter++];
    }
    else if (reg == 3)
    {
      ic[current_ic].stat.va_ovhi[0] = (data[data_counter] & 0x80)>>7;
      ic[current_ic].stat.va_uvlo[0] = (data[data_counter] & 0x40)>>6;
      ic[current_ic].stat.vd_ovhi[0] = (data[data_counter] & 0x20)>>5;
      ic[current_ic].stat.vd_uvlo[0] = (data[data_counter] & 0x10)>>4;
      ic[current_ic].stat.a_otp_ed[0] = (data[data_counter] & 0x08)>>3;
      ic[current_ic].stat.a_otp_med[0] = (data[data_counter] & 0x04)>>2;
      ic[current_ic].stat.otp_ed[0] = (data[data_counter] & 0x02)>>1;
      ic[current_ic].stat.otp_med[0] = (data[data_counter++] & 0x01);
      ic[current_ic].stat.redfail[0] = (data[data_counter] & 0x80)>>7;
      ic[current_ic].stat.compchk[0] = (data[data_counter] & 0x40)>>6;
      ic[current_ic].stat.sleep[0] = (data[data_counter] & 0x20)>>5;
      ic[current_ic].stat.tmodechk[0] = (data[data_counter] & 0x10)>>4;
      ic[current_ic].stat.muxfail[0] = (data[data_counter] & 0x08)>>3;
      ic[current_ic].stat.thsd[0] = (data[data_counter] & 0x04)>>2;
      ic[current_ic].stat.cpchk[0] = (data[data_counter] & 0x02)>>1;
      ic[current_ic].stat.oscchk[0] = (data[data_counter++] & 0x01);
      parsed_stat = data[data_counter] + (data[data_counter+1]<<8); //Each stat codes is received as two bytes and is combined to create the parsed status code
      ic[current_ic].stat.adol[0] = parsed_stat;
      data_counter=data_counter+2; //Because stat codes are two bytes the data counter
      parsed_stat = data[data_counter] + (data[data_counter+1]<<8); //Each stat codes is received as two bytes and is combined to create the parsed status code
      ic[current_ic].stat.adol[1] = parsed_stat;
      data_counter=data_counter+2; //Because stat codes are two bytes the data counter
    }

    received_pec = (data[data_counter]<<8)+ data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
    //after the 6 status data bytes
    data_pec = pec10_calc(BYT_IN_REG, &data[0], true);
    if ((received_pec&0x3ff) != data_pec)
    {
      pec_error = -1; //The pec_error variable is simply set negative if any PEC errors
      ic[current_ic].stat.pec_match[reg-1]=1;
    }
    data_counter=data_counter+2;

  }
  ADBMS6815_check_pec(current_ic,STAT,ic);
  free(data);
  return (pec_error);
}

/*
   The function reads a single stat register and stores the read data
   in the *data point as a byte array. This function is rarely used outside of
   the ADBMS6815_rdstat() command.
 */
void ADBMS6815_rdstat_reg(uint8_t reg, uint8_t *data)
{
  const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;

  if (reg == 1) //Read back status group A
  {
    cmd[1] = 0x10;
    cmd[0] = 0x00;
  }
  else if (reg == 2) //Read back status group B
  {
    cmd[1] = 0x12;
    cmd[0] = 0x00;
  }
  else if (reg == 3) //Read back status group C
  {
    cmd[1] = 0x13;
    cmd[0] = 0x00;
  }

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_RESET);
  spi_write_read(cmd,4,data,REG_LEN);
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_SET);
}

/* Read All Aux/Status Registers */
int8_t ADBMS6815_rdasall(uint8_t current_ic,//The index of current IC
       cell_asic *ic //A two dimensional array of the stat codes.
       )
{
  uint8_t *data;
  uint8_t data_counter = 0;
  int8_t pec_error = 0;
	uint16_t parsed_gpio;
  uint16_t parsed_stat;
  uint16_t received_pec;
  uint16_t data_pec;

  data = (uint8_t *) malloc((6*6+2)*sizeof(uint8_t));
  ADBMS6815_rdasall_reg(data);

  for (uint8_t gpio_reg = 1; gpio_reg<ic[0].ic_reg.num_gpio_reg+1; gpio_reg++) //Executes once for each of the LTC681x aux voltage registers
  {
    pec_error = parse_cells(gpio_reg, data,
          &ic[current_ic].aux.a_codes[0],
          &ic[current_ic].aux.pec_match[0]);
  }

  for (uint8_t current_gpio = 0; current_gpio<9; current_gpio++) // This loop parses the read back data into the register codes, it
  { // loops once for each of the 3 codes in the register
    parsed_gpio = data[data_counter] + (data[data_counter + 1] << 8);//Each code is received as two bytes and is combined to
    // create the parsed code
    ic[current_ic].aux.a_codes[current_gpio] = parsed_gpio;
    data_counter = data_counter + 2; //Because the codes are two bytes, the data counter
    //must increment by two for each parsed code
  }

  for (uint8_t stat_reg = 1; stat_reg< ic[0].ic_reg.num_stat_reg+1; stat_reg++) //Executes once for each of the LTC681x stat voltage registers
  {
    if (stat_reg == 1)
    {
      for (uint8_t current_stat = 0; current_stat< 3; current_stat++) // This loop parses the read back data into Status registers,
      { // it loops once for each of the 3 stat codes in the register
        parsed_stat = data[data_counter] + (data[data_counter+1]<<8); //Each stat codes is received as two bytes and is combined to create the parsed status code
        ic[current_ic].stat.stat_codes[current_stat] = parsed_stat;
        data_counter=data_counter+2; //Because stat codes are two bytes the data counter
      }
    }
    else if (stat_reg == 2)
    {
      parsed_stat = data[data_counter] + (data[data_counter+1]<<8); //Each stat is received as two bytes and is combined to create the parsed status code
      data_counter = data_counter +2;
      ic[current_ic].stat.stat_codes[3] = parsed_stat;
      ic[current_ic].stat.flags[0] = data[data_counter++];
      ic[current_ic].stat.flags[1] = data[data_counter++];
      ic[current_ic].stat.flags[2] = data[data_counter++];
      ic[current_ic].stat.oc_cntr[0] = data[data_counter++];
    }
    else if (stat_reg == 3)
    {
      ic[current_ic].stat.va_ovhi[0] = (data[data_counter] & 0x80)>>7;
      ic[current_ic].stat.va_uvlo[0] = (data[data_counter] & 0x40)>>6;
      ic[current_ic].stat.vd_ovhi[0] = (data[data_counter] & 0x20)>>5;
      ic[current_ic].stat.vd_uvlo[0] = (data[data_counter] & 0x10)>>4;
      ic[current_ic].stat.a_otp_ed[0] = (data[data_counter] & 0x08)>>3;
      ic[current_ic].stat.a_otp_med[0] = (data[data_counter] & 0x04)>>2;
      ic[current_ic].stat.otp_ed[0] = (data[data_counter] & 0x02)>>1;
      ic[current_ic].stat.otp_med[0] = (data[data_counter++] & 0x01);
      ic[current_ic].stat.redfail[0] = (data[data_counter] & 0x80)>>7;
      ic[current_ic].stat.compchk[0] = (data[data_counter] & 0x40)>>6;
      ic[current_ic].stat.sleep[0] = (data[data_counter] & 0x20)>>5;
      ic[current_ic].stat.tmodechk[0] = (data[data_counter] & 0x10)>>4;
      ic[current_ic].stat.muxfail[0] = (data[data_counter] & 0x08)>>3;
      ic[current_ic].stat.thsd[0] = (data[data_counter] & 0x04)>>2;
      ic[current_ic].stat.cpchk[0] = (data[data_counter] & 0x02)>>1;
      ic[current_ic].stat.oscchk[0] = (data[data_counter++] & 0x01);
      parsed_stat = data[data_counter] + (data[data_counter+1]<<8); //Each stat codes is received as two bytes and is combined to create the parsed status code
      ic[current_ic].stat.adol[0] = parsed_stat;
      data_counter=data_counter+2; //Because stat codes are two bytes the data counter
      parsed_stat = data[data_counter] + (data[data_counter+1]<<8); //Each stat codes is received as two bytes and is combined to create the parsed status code
      ic[current_ic].stat.adol[1] = parsed_stat;
      data_counter=data_counter+2; //Because stat codes are two bytes the data counter
    }
  }
  received_pec = (data[data_counter]<<8)+ data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
  //after the 6 status data bytes
  data_pec = pec10_calc(6*6, &data[0], true);
  if ((received_pec&0x3ff) != data_pec)
  {
    pec_error = -1; //The pec_error variable is simply set negative if any PEC errors
    ic[current_ic].aux.pec_match[0]=1;
    ic[current_ic].aux.pec_match[1]=1;
    ic[current_ic].aux.pec_match[2]=1;
    ic[current_ic].stat.pec_match[0]=1; //are detected in the received serial data
    ic[current_ic].stat.pec_match[1]=1;
    ic[current_ic].stat.pec_match[2]=1;
  }
  else
  {
    ic[current_ic].aux.pec_match[0]=0;
    ic[current_ic].aux.pec_match[1]=0;
    ic[current_ic].aux.pec_match[2]=0;
    ic[current_ic].stat.pec_match[0]=0;
    ic[current_ic].stat.pec_match[1]=0;
    ic[current_ic].stat.pec_match[2]=0;
  }
  data_counter=data_counter+2; //Because the transmitted PEC code is 2 bytes long the data_counter
  //must be incremented by 2 bytes to point to the next ICs status data

  ADBMS6815_check_pec(current_ic,AUX,ic);
  ADBMS6815_check_pec(current_ic,STAT,ic);
  free(data);
  return (pec_error);
}

/* Writes the command and reads all Aux/Status registers data */
void ADBMS6815_rdasall_reg(uint8_t *data //An array of the unparsed cell codes
         )
{
  const uint8_t REG_LEN = 36+2; //Number of bytes in each ICs register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[1] = 0x3C;
  cmd[0] = 0x00;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_RESET);
  spi_write_read(cmd,4,data,REG_LEN);
  HAL_GPIO_WritePin(cs_GPIO[board],cs_PIN[board],GPIO_PIN_SET);
}


/* Helper function to set discharge bit in CFG register */
void ADBMS6815_set_discharge(int Cell, //The cell to be discharged
           uint8_t current_ic, //Index of current IC
           cell_asic *ic //A two dimensional array that will store the data
           )
{
  if ((Cell<8)&& (Cell>=0) )
  {
    ic[current_ic].configb.tx_data[4] = ic[current_ic].configb.tx_data[4] | (1<<(Cell));
  }
  else if (Cell < 12)
  {
    ic[current_ic].configb.tx_data[5] = ic[current_ic].configb.tx_data[5] | (1<<(Cell-8));
  }
}

/* Helper function to clear discharge bit in CFG register */
void ADBMS6815_clear_discharge(int Cell, //The cell to be discharged
             uint8_t current_ic, //Index of current IC
             cell_asic *ic //A two dimensional array that will store the data
             )
{
  if ((Cell<8)&& (Cell>=0) )
  {
    ic[current_ic].configb.tx_data[4] = ic[current_ic].configb.tx_data[4] & (~(1<<(Cell)));
  }
  else if (Cell < 12)
  {
    ic[current_ic].configb.tx_data[5] = ic[current_ic].configb.tx_data[5] & (~(1<<(Cell-8)));
  }
}

/* ADBMS6815 mute discharge*/
void ADBMS6815_mute(void)
{
  uint8_t cmd[2];

  cmd[0] = 0x00;
  cmd[1] = 0x28;

  cmd_68(cmd);
}

/* ADBMS6815 unmute discharge*/
void ADBMS6815_unmute(void)
{
  uint8_t cmd[2];

  cmd[0] = 0x00;
  cmd[1] = 0x29;

  cmd_68(cmd);
}

/* MCU Restart */
void mcu_restart(void)
{
  __set_FAULTMASK(1);
  HAL_NVIC_SystemReset();
}

/* Multiplexer Active */
void mulitiplexer_switch(void)
{
	if (board == 0)
	{
		HAL_GPIO_WritePin(multiplexer_GPIO[0], multiplexer_PIN[0], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(multiplexer_GPIO[1], multiplexer_PIN[1], GPIO_PIN_RESET);
	}
	else if(board == 1)
	{
		HAL_GPIO_WritePin(multiplexer_GPIO[0], multiplexer_PIN[0], GPIO_PIN_SET);
		HAL_GPIO_WritePin(multiplexer_GPIO[1], multiplexer_PIN[1], GPIO_PIN_RESET);
	}
	else //if(board == 2)
	{
		HAL_GPIO_WritePin(multiplexer_GPIO[0], multiplexer_PIN[0], GPIO_PIN_SET);
		HAL_GPIO_WritePin(multiplexer_GPIO[1], multiplexer_PIN[1], GPIO_PIN_SET);
	}
	delay_us(100);
}


/* Helper funciton to reinitialize CAN */
void CANReInit(CAN_HandleTypeDef *hcan, uint16_t delay)
{
  if (delay > 512)
  {
    mcu_restart();
    return;
  }
  HAL_Delay(1000*delay);
  HAL_CAN_ResetError(hcan);
  MX_CAN1_Init();
  CAN1_Config();
}

/* Helper funciton to clear CAN TX buff*/
void cleartx(void)
{
  int i;
  for(i = 0; i < CAN_BUF_SIZE; i++)
  {
    can1_tx_buff[i] = 0;
  }
}

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//--------------------------------- Tim2 ---------------------------------------#
void TIM2_IRQHandler(void)
{
  // 15s dog feeder
  HAL_TIM_IRQHandler(&htim2);
  //HAL_IWDG_Refresh(&hiwdg);
}

//--------------------------------- Tim4 ---------------------------------------#
// System clock onboard
// float tim4cnt = htim4.Instance->CNT;
// nowTime = 1.0f*TimerCnt*50000*65536/84000000+1.0f*tim4cnt*50000/84000000; //s
volatile uint32_t TimerCnt;
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
  TimerCnt++;
}

//--------------------------------- Tim3 ---------------------------------------#
// 50ms*20 for basic service
int tim3round = 0;
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
  tim3round++;
  if (tim3round < 20) //20*50=1000ms
  {
    return;
  }
  tim3round = 0;
  tim3Int = true;
}

//------------------------- CAN received Call Back -----------------------------#
float currentRcv = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
	uint16_t canRcvState = HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, can1_rx_buff);
	
	if (((activeHcan == &hcan2)&(CanHandle->Instance == hcan1.Instance))|
			((activeHcan == &hcan1)&(CanHandle->Instance == hcan2.Instance)))
	{
		return;
	}
	if (canRcvState != HAL_OK)
	{
		Error_Handler();
	}
  uint8_t masterFlag = (RxHeader.StdId & 0x0400)/0x0400;
  uint8_t dataType = (RxHeader.StdId & 0x0200)/0x0200;
  uint8_t subBoardID = (RxHeader.StdId & 0x01F0)/0x0010;
  uint8_t b6815ID = (RxHeader.StdId & 0x000C)/0x0004;
  //uint8_t dataID = (RxHeader.StdId & 0x0003)/0x0001;
  /* Get Current and select mode */
  if ((masterFlag == 0) & ((subBoardID == 0)|(subBoardID == subID)))
  {
    if (dataType == 0)
    {
      // if (dataID==0)
			if (b6815ID == 0)
      {
        for (uint8_t i=0; i<3; i++)
        {
          BMS_IC[i].configb.tx_data[0] = can1_rx_buff[0];
					BMS_IC[i].configb.tx_data[1] = can1_rx_buff[1];
					BMS_IC[i].configb.tx_data[2] = can1_rx_buff[2];
					BMS_IC[i].configb.tx_data[4] = can1_rx_buff[3];
					BMS_IC[i].configb.tx_data[5] = (((BMS_IC[i].configb.tx_data[5])&(0xF0)) |	((can1_rx_buff[4]>>4)&(0x0F)));
        }
      }
      else// if (b6815ID == 1/2/3)
      {
				BMS_IC[b6815ID-1].configb.tx_data[0] = can1_rx_buff[0];
				BMS_IC[b6815ID-1].configb.tx_data[1] = can1_rx_buff[1];
				BMS_IC[b6815ID-1].configb.tx_data[2] = can1_rx_buff[2];
				BMS_IC[b6815ID-1].configb.tx_data[4] = can1_rx_buff[3];
				BMS_IC[b6815ID-1].configb.tx_data[5] = (((BMS_IC[b6815ID-1].configb.tx_data[5])&(0xF0)) |	((can1_rx_buff[4]>>4)&(0x0F)));
      }
      canRcvConfInt = b6815ID;
    }
    else// if (dataType == 0)
    {
      // if (dataID==0)
      // testdata 0x000 0x9C 63 00 00 00 00 00 00 -> 10A
      uint16_t currentdata = (can1_rx_buff[1]<<8) | can1_rx_buff[0];
      currentRcv = 0.02f*currentdata-500;
      canRcvDataInt = b6815ID;
    }
  }
	return;
}

//--------------------------- CAN Error Call Back ------------------------------#
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	
  uint32_t errorcode = hcan->ErrorCode;
  if ((errorcode & (HAL_CAN_ERROR_BOF)) != 0U)
  {
    CANReInit(hcan, punishDelay);
    punishDelay = punishDelay *2;
		if (hcan == &hcan1)
	{
		activeHcan = &hcan2;
	}
	else
	{
		activeHcan = &hcan1;
	}
		
    return;
  }
  else if ((errorcode &
      (HAL_CAN_ERROR_TIMEOUT | HAL_CAN_ERROR_NOT_INITIALIZED | HAL_CAN_ERROR_NOT_READY |
       HAL_CAN_ERROR_NOT_STARTED | HAL_CAN_ERROR_PARAM |
       HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1)) != 0U)
  {
    CANReInit(hcan, 0);
		if (hcan == &hcan1)
	{
		activeHcan = &hcan2;
	}
	else
	{
		activeHcan = &hcan1;
	}
    return;
  }
	/*
  else if ((punishDelay > 512) &&
     ((errorcode & (HAL_CAN_ERROR_EWG | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_BOF)) == 0U))
     {
     punishDelay = 1;
     }*/
}

//--------------------------- First Time Measure -------------------------------#
void first_service(void)
{
  /* Online confirmed*/
  wakeup_sleep();
  ADBMS6815_adstat(2,0);
  ADBMS6815_pladc();
  int8_t error = ADBMS6815_rdstat(2,board,BMS_IC);
  if (BMS_IC[board].stat.flags[0] == 0xFF)
  {
    BMS_IC[board].offline = 0;
    return;
  }
  else
  {
    BMS_IC[board].offline = 1;
  }

  /* Measure Start */
  // Time
  float tim4cnt = htim4.Instance->CNT;
  BMS_IC[board].nowTime = 1.0f*TimerCnt*50000*65536/84000000+1.0f*tim4cnt*50000/84000000;
  // Write Config to 6815
  wakeup_sleep();
  ADBMS6815_wrcfg(board,BMS_IC);
  error = ADBMS6815_rdcfg(board,BMS_IC);
  // State Measure
  wakeup_sleep();
  ADBMS6815_adstat(2,0);
  ADBMS6815_pladc();
  error = ADBMS6815_rdstat(0,board,BMS_IC);
  BMS_IC[board].sc = 1.0*BMS_IC[board].stat.stat_codes[0]*0.0001*20; //total voltage
  // Init SOC
  int sheetSize = sizeof(OCV_List)/sizeof(OCV_List[0]);
  if (BMS_IC[board].sc < OCV_List[0])
  {
    BMS_IC[board].SOC = 0.00;
  }
  else if (BMS_IC[board].sc > OCV_List[sheetSize-1])
  {
    BMS_IC[board].SOC = 1.00;
  }
  else
  {
    for(int i = 1; i < sheetSize; i++)
    {
      if (BMS_IC[board].sc <= OCV_List[i])
      {
        BMS_IC[board].SOC = 1.0f*(SOC_List[i]-SOC_List[i-1])/(OCV_List[i]-OCV_List[i-1])*(BMS_IC[board].sc-OCV_List[i-1])+SOC_List[i-1];
        break;
      }
    }
  }
  /*
     //Forced to use Hall by GPIO1(6815)
     wakeup_sleep();
     ADBMS6815_adax(2,0);
     ADBMS6815_pladc();
     error = ADBMS6815_rdaux(0,board,BMS_IC);
     float GPIOVoltage = 1.0*BMS_IC[board].aux.a_codes[currentGPIOPin]*0.0001;
     BMS_IC[board].current = 1.0f*(1.0f*GPIOVoltage-baseVoltage)/RM*2000/turns;
   */
  // Past state init
  BMS_IC[board].lastSampleTime = BMS_IC[board].nowTime;
  BMS_IC[board].sc_Past = BMS_IC[board].sc;
  BMS_IC[board].currentPast = BMS_IC[board].current;
  BMS_IC[board].SOCPast = BMS_IC[board].SOC;

  /* First Measure Finish */
  BMS_IC[board].firstTime = 0;
}

//----------------------------- BASIC_SERVICE ----------------------------------#
void basic_service(void)
{
  /* Online confirmed*/
  wakeup_sleep();
  ADBMS6815_adstat(2,0);
  ADBMS6815_pladc();
  int8_t error = ADBMS6815_rdstat(2,board,BMS_IC);
  if (BMS_IC[board].stat.flags[0] == 0xFF)
  {
    BMS_IC[board].offline = 0;
    return;
  }
  else
  {
    BMS_IC[board].offline = 1;
  }

  /* Measure Start */
  // Time
  float tim4cnt = htim4.Instance->CNT;
  BMS_IC[board].nowTime = 1.0f*TimerCnt*50000*65536/84000000+1.0f*tim4cnt*50000/84000000; //s
  BMS_IC[board].timeInterval = BMS_IC[board].nowTime-BMS_IC[board].lastSampleTime;
  // Write Config to 6815
  ADBMS6815_wrcfg(board,BMS_IC);
  error = ADBMS6815_rdcfg(board,BMS_IC);
	
  // State Measure
	
	ADBMS6815_mute();
  ADBMS6815_adstat(2,0);
	ADBMS6815_pladc();
  ADBMS6815_adax(2,0);
	ADBMS6815_pladc();
	ADBMS6815_adcv(2,0,0);
	ADBMS6815_pladc();
	ADBMS6815_unmute();
	
	error = ADBMS6815_rdasall(board,BMS_IC);
  //error = ADBMS6815_rdstat(0,board,BMS_IC);
	//error = ADBMS6815_rdaux(0,board,BMS_IC);
	error = ADBMS6815_rdcv(0,board,BMS_IC);
	/*
	for(uint16_t i = 1; i <= 4; i++)
	{
		error = ADBMS6815_rdcv(i,board,BMS_IC);
	}
	*/
	
	BMS_IC[board].sc = 1.0*BMS_IC[board].stat.stat_codes[0]*0.0001*20; //total voltage
  BMS_IC[board].temp = 1.0*BMS_IC[board].stat.stat_codes[1]*0.0001/0.0075-260;
  BMS_IC[board].flag[0] = BMS_IC[board].stat.flags[0];
  BMS_IC[board].flag[1] = BMS_IC[board].stat.flags[1];
  BMS_IC[board].flag[2] = BMS_IC[board].stat.flags[2];
  BMS_IC[board].thsd = BMS_IC[board].stat.thsd[0];
	BMS_IC[board].distrubTemp[0] = 1.0f/(log(BMS_IC[board].aux.a_codes[1]*0.0001f/(3.0f-BMS_IC[board].aux.a_codes[1]*0.0001f))/3435.0f+1.0f/(25.00f+273.15f))-273.15f;
	BMS_IC[board].distrubTemp[1] = 1.0f/(log(BMS_IC[board].aux.a_codes[2]*0.0001f/(3.0f-BMS_IC[board].aux.a_codes[2]*0.0001f))/34350.f+1.0f/(25.00f+273.15f))-273.15f;
	BMS_IC[board].distrubTemp[2] = 1.0f/(log(BMS_IC[board].aux.a_codes[3]*0.0001f/(3.0f-BMS_IC[board].aux.a_codes[3]*0.0001f))/3435.0f+1.0f/(25.00f+273.15f))-273.15f;
	BMS_IC[board].distrubTemp[3] = 1.0f/(log(BMS_IC[board].aux.a_codes[4]*0.0001f/(3.0f-BMS_IC[board].aux.a_codes[4]*0.0001f))/3435.0f+1.0f/(25.00f+273.15f))-273.15f;
	BMS_IC[board].distrubTemp[4] = 1.0f/(log(BMS_IC[board].aux.a_codes[5]*0.0001f/(3.0f-BMS_IC[board].aux.a_codes[5]*0.0001f))/3435.0f+1.0f/(25.00f+273.15f))-273.15f;
	BMS_IC[board].distrubTemp[5] = 1.0f/(log(BMS_IC[board].aux.a_codes[6]*0.0001f/(3.0f-BMS_IC[board].aux.a_codes[6]*0.0001f))/3435.0f+1.0f/(25.00f+273.15f))-273.15f;
	for(uint16_t i = 0; i < 12; i++)
	{
    BMS_IC[board].cellVoltage[i] = BMS_IC[board].cells.c_codes[i]*0.0001;
  }
	
  /*
     //Forced to use Hall by GPIO1(6815)
     wakeup_sleep();
     ADBMS6815_adax(2,0);
     ADBMS6815_pladc();
     error = ADBMS6815_rdaux(0,board,BMS_IC);
     float GPIOVoltage = 1.0*BMS_IC[board].aux.a_codes[currentGPIOPin]*0.0001;
     BMS_IC[board].current = 1.0f*(1.0f*GPIOVoltage-baseVoltage)/RM*2000/turns;
   */

  /* discharge strategy with Voltage THRESHOLD */
  for (int flag_number = 0; flag_number < 3; flag_number++)
  {
    for (uint8_t current_pos = 0; current_pos < 8; current_pos++)
    {
      uint8_t cover = 1 << current_pos;
      uint8_t coverpos = cover & BMS_IC[board].stat.flags[flag_number];
      if (coverpos == cover)
      {
        int num = current_pos+flag_number*8;
        num = num/2; //0-11
        if ((current_pos%2) == 0) //L
        {
          BMS_IC[board].dischargeLLock[num]++;
          if (BMS_IC[board].dischargeLLock[num] >= dischargeLockLimit)
          {
            ADBMS6815_clear_discharge(num, board, BMS_IC);
          }
        }
        else //H
        {
          BMS_IC[board].dischargeHLock[num]++;
          if (BMS_IC[board].dischargeHLock[num] >= dischargeLockLimit)
          {
            ADBMS6815_set_discharge(num, board, BMS_IC);
          }
        }
      }
      else
      {
        int num = current_pos+flag_number*8;
        num = num/2; //0-11
        if ((current_pos%2) == 0) //L
        {
          BMS_IC[board].dischargeLLock[num] = 0;
        }
        else //H
        {
          BMS_IC[board].dischargeHLock[num] = 0;
        }
      }
      for (int num = 0; num < 12; num++)
      {
        if ((BMS_IC[board].dischargeLLock[num] < dischargeLockLimit) && (BMS_IC[board].dischargeHLock[num] < dischargeLockLimit))
        {
          // recover init
          if (num<8)
					{
						if((BMS_IC[board].configb.rx_data[4]&(0x01<<num))==(0x01<<num))
						{
							ADBMS6815_set_discharge(num, board, BMS_IC);
						}
						else
						{
							ADBMS6815_clear_discharge(num, board, BMS_IC);
						}
					}
					else
					{
						if((BMS_IC[board].configb.rx_data[5]&(0x01<<(num-8)))==(0x01<<(num-8)))
						{
							ADBMS6815_set_discharge(num, board, BMS_IC);
						}
						else
						{
							ADBMS6815_clear_discharge(num, board, BMS_IC);
						}
					}
        }
      }
    }
  }
  wakeup_sleep();
  ADBMS6815_wrcfg(board,BMS_IC);
  error = ADBMS6815_rdcfg(board,BMS_IC);

  /* Charge state & Q */
  int charge = 0;
  if (BMS_IC[board].current < 0)
  {
    charge = 1;
  }
  float deltaQ = (-1.0f)*(BMS_IC[board].current+BMS_IC[board].currentPast)/2*(BMS_IC[board].timeInterval/60/60);
  BMS_IC[board].SOH_Q = BMS_IC[board].SOH_Q + deltaQ*12;

  /* dvdt */
  BMS_IC[board].dvdt[BMS_IC[board].dvdtpos] = 1.0f*(BMS_IC[board].sc-BMS_IC[board].sc_Past)/(BMS_IC[board].timeInterval/60/60);
  BMS_IC[board].dvdtpos++;
  if (BMS_IC[board].dvdtpos == timeZones)
  {
    BMS_IC[board].dvdtpos = 0;
    BMS_IC[board].dvdtReady = 1;
  }
  float dvdtAvg = 0;
  for (int i = 0; i < timeZones; i++)
  {
    dvdtAvg = dvdtAvg + BMS_IC[board].dvdt[i];
  }
  dvdtAvg = dvdtAvg/timeZones;
  if (fabsf(dvdtAvg) < Vsta)
  {
    BMS_IC[board].dvdtLock++;
  }
  else
  {
    BMS_IC[board].dvdtLock = 0;
  }

  /* SOC */
  if (((BMS_IC[board].sc <= plainThsd[0])||(BMS_IC[board].sc >= plainThsd[1])) && (fabsf(BMS_IC[board].current) < smallCurrentLimit) && (BMS_IC[board].dvdtReady == 1) && (BMS_IC[board].dvdtLock >= dvdtLockLimit))
  {
    // Query OCV-SOC
    int sheetSize = sizeof(OCV_List)/sizeof(OCV_List[0]);
    if (BMS_IC[board].sc < OCV_List[0])
    {
      BMS_IC[board].SOC = 0.00;
    }
    else if (BMS_IC[board].sc > OCV_List[sheetSize-1])
    {
      BMS_IC[board].SOC = 1.00;
    }
    else
    {
      for(int j = 1; j < sheetSize; j++)
      {
        if (BMS_IC[board].sc <= OCV_List[j])
        {
          BMS_IC[board].SOC = 1.0f*(SOC_List[j]-SOC_List[j-1])/(OCV_List[j]-OCV_List[j-1])*(BMS_IC[board].sc-OCV_List[j-1])+SOC_List[j-1];
          break;
        }
      }
    }
    /* SOH */
    if (BMS_IC[board].SOH_state == 0)
    {
      if (BMS_IC[board].sc <= plainThsd[0])
      {
        BMS_IC[board].SOH_state = 2;
      }
      else
      {
        BMS_IC[board].SOH_state = 1;
      }
      BMS_IC[board].SOH_foundSOC = BMS_IC[board].SOC;
      BMS_IC[board].SOH_Q = 0;
    }
    else if ((BMS_IC[board].SOH_state == 1) && (BMS_IC[board].sc <= plainThsd[0]))
    {
      BMS_IC[board].Cmax = fabsf(BMS_IC[board].SOH_Q/(BMS_IC[board].SOC-BMS_IC[board].SOH_foundSOC));
      BMS_IC[board].SOH = BMS_IC[board].Cmax/C0;
      BMS_IC[board].SOH_foundSOC = BMS_IC[board].SOC;
      BMS_IC[board].SOH_state = 2;
      BMS_IC[board].SOH_Q = 0;
    }
    else if ((BMS_IC[board].SOH_state == 2) && (BMS_IC[board].sc >= plainThsd[1]))
    {
      BMS_IC[board].Cmax = fabsf(BMS_IC[board].SOH_Q/(BMS_IC[board].SOC-BMS_IC[board].SOH_foundSOC));
      BMS_IC[board].SOH = BMS_IC[board].Cmax/C0;
      BMS_IC[board].SOH_foundSOC = BMS_IC[board].SOC;
      BMS_IC[board].SOH_state = 1;
      BMS_IC[board].SOH_Q = 0;
    }
  }
  else
  {
    // Ah integration
    BMS_IC[board].SOC = BMS_IC[board].SOCPast + (-1.0f)*deltaQ/BMS_IC[board].Cmax;
    BMS_IC[board].SOCPast = BMS_IC[board].SOC;
  }

  /* SOP */
  if ((BMS_IC[board].temp < SOP_ListX[0]) || (BMS_IC[board].temp > SOP_ListX[sizeof(SOP_ListX)/sizeof(SOP_ListX[0])-1]))
  {
    BMS_IC[board].SOP = 0;
  }
  else
  {
    int lenListX = sizeof(SOP_ListX)/sizeof(SOP_ListX[0]);
    int lenListY = sizeof(SOP_ListY)/sizeof(SOP_ListY[0]);
    for (int i = 2; i<lenListX; i++)
    {
      if (BMS_IC[board].temp <= SOP_ListX[i])
      {
        for (int j = 2; j < lenListY; j++)
        {
          if (BMS_IC[board].SOC <= SOP_ListY[j])
          {
            float a = (BMS_IC[board].temp-SOP_ListX[i-1])/(SOP_ListX[i]-SOP_ListX[i-1]);
            float b = (BMS_IC[board].SOC-SOP_ListY[j-1])/(SOP_ListY[j]-SOP_ListY[j-1]);
            if (charge == 1)
            {
              BMS_IC[board].SOP = a*b*SOP_chargeList[i-1][j-1]+a*(1-b)*SOP_chargeList[i][j-1]+(1-a)*b*SOP_chargeList[i-1][j]+(1-a)*(1-b)*SOP_chargeList[i][j];
            }
            else
            {
              BMS_IC[board].SOP = a*b*SOP_dischargeList[i-1][j-1]+a*(1-b)*SOP_dischargeList[i][j-1]+(1-a)*b*SOP_dischargeList[i-1][j]+(1-a)*(1-b)*SOP_dischargeList[i][j];
            }
            break;
          }
        }
        break;
      }
    }
  }

  /* Past state update */
  BMS_IC[board].lastSampleTime = BMS_IC[board].nowTime;
  BMS_IC[board].currentPast = BMS_IC[board].current;
  BMS_IC[board].sc_Past = BMS_IC[board].sc;
  BMS_IC[board].SOCPast = BMS_IC[board].SOC;
	
	/* Upload measure data*/
	UploadMeasureData();
	UploadCVData();

  /* Sample used time */
  tim4cnt = htim4.Instance->CNT;
  float sampleUsedTime = BMS_IC[board].nowTime - 1.0f*TimerCnt*50000*65536/84000000+1.0f*tim4cnt*50000/84000000; //s
}

void writeConfOnly(void)
{
  /* Online confirmed*/
  wakeup_sleep();
  ADBMS6815_adstat(2,0);
  ADBMS6815_pladc();
  int8_t error = ADBMS6815_rdstat(2,board,BMS_IC);
  if (BMS_IC[board].stat.flags[0] == 0xFF)
  {
    BMS_IC[board].offline = 0;
    return;
  }
  else
  {
    BMS_IC[board].offline = 1;
  }
	// Write Config to 6815
  wakeup_sleep();
  ADBMS6815_wrcfg(board,BMS_IC);
  error = ADBMS6815_rdcfg(board,BMS_IC);
	
	UploadConfigData();
	
			/*
      uint16_t uvRcv = (((can1_rx_buff[1]<<8) | can1_rx_buff[0])&(0x0FFF))*16;
      uint16_t ovRcv = (((can1_rx_buff[2]<<4) | (can1_rx_buff[1]>>4))&(0x0FFF))*16;
      bool DCCRcv[12];
      for (uint8_t i=0; i<8; i++)
      {
				
        DCCRcv[i] = !((can1_rx_buff[3] & (0x01<<(7-i))) == 0);
      }
      for (uint8_t i=0; i<4; i++)
      {
        DCCRcv[i+8] = !((can1_rx_buff[4] & (0x01<<(7-i))) == 0);
      }
      if (b6815ID == 0)
      {
        for (uint8_t i=0; i<3; i++)
        {
          ADBMS6815_set_cfgr_vuv(i,BMS_IC,uvRcv);
          ADBMS6815_set_cfgr_vov(i,BMS_IC,ovRcv);
          ADBMS6815_set_cfgr_dis(i,BMS_IC,DCCRcv);
        }
      }
      else// if (b6815ID == 1/2/3)
      {
        ADBMS6815_set_cfgr_vuv(b6815ID-1,BMS_IC,uvRcv);
        ADBMS6815_set_cfgr_vov(b6815ID-1,BMS_IC,ovRcv);
        ADBMS6815_set_cfgr_dis(b6815ID-1,BMS_IC,DCCRcv);
      }
      canRcvConfInt = b6815ID;*/
}

void UploadMeasureData()
{
	uint16_t masterFlag = 1;
  uint16_t dataType = 1;
  uint16_t subBoardID = subID;
  uint16_t b6815ID = board+1;
  uint16_t dataID = 1;
	
  float data;
  uint16_t dataTrans;
  // 1
  cleartx();
  StdId = (masterFlag<<10)+(dataType<<9)+(subBoardID<<4)+(b6815ID<<2)+(dataID);
  data = BMS_IC[board].sc/0.004f; //4mV
  dataTrans = (uint16_t) data;
  can1_tx_buff[0] = dataTrans;
  can1_tx_buff[1] = (dataTrans>>8);
  data = BMS_IC[board].SOC/0.005f; //0.5%
  dataTrans = (uint16_t) data;
  can1_tx_buff[2] = dataTrans;
  can1_tx_buff[3] = (dataTrans>>8);
  data = BMS_IC[board].Cmax/0.005f; //0.005Ah
  dataTrans = (uint16_t) data;
  can1_tx_buff[4] = dataTrans;
  can1_tx_buff[5] = (dataTrans>>8);
  data = BMS_IC[board].SOP/0.0001f; //1W
  dataTrans = (uint16_t) data;
  can1_tx_buff[6] = dataTrans;
  can1_tx_buff[7] = (dataTrans>>8);
	if (activeHcan == &hcan1)
	{
		int state = CAN1_Send_Msg(StdId,can1_tx_buff,8);
	}
	else
	{
		int state = CAN2_Send_Msg(StdId,can1_tx_buff,8);
	}
	dataID++;
  // 2
  cleartx();
  StdId = (masterFlag<<10)+(dataType<<9)+(subBoardID<<4)+(b6815ID<<2)+(dataID);
  can1_tx_buff[0] = (BMS_IC[board].flag[0]<<8)+BMS_IC[board].flag[1];
  can1_tx_buff[1] = (BMS_IC[board].flag[2]<<8)+BMS_IC[board].thsd;
  //data = (BMS_IC[board].temp+40)/1; //maxtemp 1`C
	data = (BMS_IC[board].distrubTemp[4]+40.0f)/1.0f;
  dataTrans = (uint16_t) data;
  can1_tx_buff[2] = dataTrans;
  can1_tx_buff[3] = (dataTrans>>8);
  //data = (BMS_IC[board].temp+40)/1; //mintemp 1`C
	data = (BMS_IC[board].distrubTemp[4]+40.0f)/1.0f;
  dataTrans = (uint16_t) data;
  can1_tx_buff[4] = dataTrans;
  can1_tx_buff[5] = (dataTrans>>8);
	if (activeHcan == &hcan1)
	{
		int state = CAN1_Send_Msg(StdId,can1_tx_buff,8);
	}
	else
	{
		int state = CAN2_Send_Msg(StdId,can1_tx_buff,8);
	}
}

void UploadConfigData()
{	
	uint16_t masterFlag = 1;
  uint16_t dataType = 0;
  uint16_t subBoardID = subID;
  uint16_t b6815ID = board+1;
  uint16_t dataID = 0;
	
	cleartx();
  StdId = (masterFlag<<10)+(dataType<<9)+(subBoardID<<4)+(b6815ID<<2)+(dataID);
	can1_tx_buff[0] = BMS_IC[board].configb.rx_data[0];
	can1_tx_buff[1] = BMS_IC[board].configb.rx_data[1];
	can1_tx_buff[2] = BMS_IC[board].configb.rx_data[2];
	can1_tx_buff[3] = BMS_IC[board].configb.rx_data[4];
	can1_tx_buff[4] = BMS_IC[board].configb.rx_data[5]<<4;
	if (activeHcan == &hcan1)
	{
		uint8_t state = CAN1_Send_Msg(StdId,can1_tx_buff,8);
	}
	else
	{
		uint8_t state = CAN2_Send_Msg(StdId,can1_tx_buff,8);
	}
}

/*Test Function Refer to BMS_CAN20220701*/
void UploadCVData(void)
{
	uint16_t masterFlag = 1;
  uint16_t dataType = 1;
  uint16_t subBoardID = subID+1;
  uint16_t b6815ID = board+1;
  uint16_t dataID = 1;

  // 1
  for(uint8_t i = 0; i < 3; i++)
  {
    cleartx();
    StdId = (masterFlag<<10)+(dataType<<9)+(subBoardID<<4)+(b6815ID<<2)+(dataID);
    for(uint8_t j = 0; j < 4; j++)
    {
      can1_tx_buff[j*2] = 0x0F*BMS_IC[board].cells.c_codes[i*4+j];
      can1_tx_buff[j*2+1] = BMS_IC[board].cells.c_codes[i*4+j]>>8;
    }
    if (activeHcan == &hcan1)
  	{
  		int state = CAN1_Send_Msg(StdId,can1_tx_buff,8);
  	}
  	else
  	{
  		int state = CAN2_Send_Msg(StdId,can1_tx_buff,8);
  	}
    dataID++;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM7_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  // Converts individual cell data into 12-cell pack data
  int sheetSize = sizeof(OCV_List)/sizeof(OCV_List[0]);
  for (int i = 1; i < sheetSize; i++)
  {
    OCV_List[i] = OCV_List[i]*12;
  }
  plainThsd[0] = plainThsd[0] * 12;
  plainThsd[1] = plainThsd[1] * 12;
  for (int i = 1; i < sizeof(SOP_ListX)/sizeof(SOP_ListX[0]); i++)
  {
    for (int j = 1; j < sizeof(SOP_ListY)/sizeof(SOP_ListY[0]); j++)
    {
      SOP_chargeList[i][j] = SOP_chargeList[i][j]*12;
      SOP_dischargeList[i][j] = SOP_dischargeList[i][j]*12;
    }
  }

  //LTC6815_init_cfg
  for (int c_ic = 0; c_ic < 3; c_ic++)
  {
    for (int i = 0; i < 6; i++)
    {
      BMS_IC[c_ic].config.tx_data[i] = 0;
    }
    BMS_IC[c_ic].nowTime = 0;
    BMS_IC[c_ic].lastSampleTime = 0;
    BMS_IC[c_ic].timeInterval = 0;
    BMS_IC[c_ic].offline = 0;
    BMS_IC[c_ic].firstTime = 1;
    for (int j = 0; j < 12; j++)
    {
      BMS_IC[c_ic].dischargeHLock[j] = 0;
      BMS_IC[c_ic].dischargeLLock[j] = 0;
    }
    BMS_IC[c_ic].dvdt = (float *)malloc(timeZones*sizeof(float));
    BMS_IC[c_ic].dvdtpos = 0;
    BMS_IC[c_ic].dvdtReady = 0;
    BMS_IC[c_ic].dvdtLock = 0;
    BMS_IC[c_ic].current = 0;
    BMS_IC[c_ic].currentPast = 0;
    BMS_IC[c_ic].SOC = 0;
    BMS_IC[c_ic].Cmax = C0;
    BMS_IC[c_ic].SOCPast = 0;
    BMS_IC[c_ic].SOH = 1;
    BMS_IC[c_ic].SOH_state = 0;
    BMS_IC[c_ic].SOH_Q = 0;
    BMS_IC[c_ic].SOH_foundSOC = 0;
    BMS_IC[c_ic].SOP = 0;
  }
  ADBMS6815_set_cfgr(0,BMS_IC,REFON,ADCOPT,PS,CVMIN,MCAL,COMM_BK,FLAG_D,SOAKON,OWRNG,OWA,OWC,GPO,
         UV_THRESHOLD,OV_THRESHOLD,DTMEN,DTRNG,DCTO,DCC);
  ADBMS6815_set_cfgr(1,BMS_IC,REFON,ADCOPT,PS,CVMIN,MCAL,COMM_BK,FLAG_D,SOAKON,OWRNG,OWA,OWC,GPO,
         UV_THRESHOLD,OV_THRESHOLD,DTMEN,DTRNG,DCTO,DCC);
  ADBMS6815_set_cfgr(2,BMS_IC,REFON,ADCOPT,PS,CVMIN,MCAL,COMM_BK,FLAG_D,SOAKON,OWRNG,OWA,OWC,GPO,
         UV_THRESHOLD,OV_THRESHOLD,DTMEN,DTRNG,DCTO,DCC);
  ADBMS6815_reset_crc_count(3,BMS_IC);
  ADBMS6815_init_reg_limits(3,BMS_IC);
	
	// Start Multiplexer
	HAL_GPIO_WritePin(multiplexer_GPIO[2], multiplexer_PIN[2], GPIO_PIN_SET);
	delay_us(100);

  // Start Timer 2 & Timer 4 & CAN
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim2);
  TimerCnt = 0;
	activeHcan = &hcan2;
  CAN1_Config();
	CAN2_Config();

  //Start Timer 3
  //HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (canRcvConfInt == 0)
		{
			for (board = 0; board < 3; board++)
      {
				mulitiplexer_switch();
        writeConfOnly();
      }
			canRcvConfInt = -1;
		}
		else if (canRcvConfInt > 0)
		{
			board = canRcvConfInt-1;
			mulitiplexer_switch();
			writeConfOnly();
			canRcvConfInt = -1;
		}

    if (canRcvDataInt == 0)
    {
      for (board = 0; board < 3; board++)
      {
				mulitiplexer_switch();
        BMS_IC[board].current = currentRcv;
        if (BMS_IC[board].firstTime == 0)
        {
          basic_service();
        }
        else
        {
          first_service();
        }
      }
      canRcvDataInt = -1;
    }
    else if (canRcvDataInt > 0)
    {
      board = canRcvDataInt-1;
			mulitiplexer_switch();
      BMS_IC[board].current = currentRcv;
      if (BMS_IC[board].firstTime == 0)
      {
        basic_service();
      }
      else
      {
        first_service();
      }
      canRcvDataInt = -1;
    }

		/*
    if (tim3Int)
    {
      tim3Int = false;
      // mode select
      for (board = 0; board < 3; board++)
      {
				mulitiplexer_switch();
        BMS_IC[board].current = 0;
        if (BMS_IC[board].firstTime == 0)
        {
          basic_service();
        }
        else
        {
          first_service();
        }
      }
    }
		*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
