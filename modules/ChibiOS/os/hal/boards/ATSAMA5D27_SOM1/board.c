/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"

#define _PIOA                       ((Pio*)0xFC038000U)
/*
 * SAMA PIO CFGR masks.
 */
#define SAMA_PIO_FUNC_GPIO          0U
#define SAMA_PIO_FUNC_PERIPH_A      1U
#define SAMA_PIO_FUNC_PERIPH_B      2U
#define SAMA_PIO_FUNC_PERIPH_C      3U
#define SAMA_PIO_FUNC_PERIPH_D      4U
#define SAMA_PIO_FUNC_PERIPH_E      5U
#define SAMA_PIO_FUNC_PERIPH_F      6U
#define SAMA_PIO_FUNC_PERIPH_G      7U
#define SAMA_PIO_FUNC(n)            (n)
#define SAMA_PIO_DIR_INPUT          (0U)
#define SAMA_PIO_DIR_OUTPUT         (1U << 8U)
#define SAMA_PIO_PUEN               (1U << 9U)
#define SAMA_PIO_PDEN               (1U << 10U)
#define SAMA_PIO_IFEN               (1U << 12U)
#define SAMA_PIO_IFSCEN             (1U << 13U)
#define SAMA_PIO_OPD                (1U << 14U)
#define SAMA_PIO_SCHMITT            (1U << 15U)
#define SAMA_PIO_DRVSTR_LO          (0U << 16U)
#define SAMA_PIO_DRVSTR_ME          (2U << 16U)
#define SAMA_PIO_DRVSTR_HI          (3U << 16U)

#define SAMA_PIO_LOW                0U
#define SAMA_PIO_HIGH               1U
/*
 * SAMA PIO default SIOSR, MSKR and CFGR values.
 */
#define SAMA_DEFAULT_SIOSR          0x00000000U
#define SAMA_DEFAULT_SIONR          0xFFFFFFFFU
#define SAMA_DEFAULT_MSKR           0xFFFFFFFFU
#define SAMA_DEFAULT_CFGR           SAMA_PIO_FUNC_GPIO | SAMA_PIO_PUEN |    \
                                    SAMA_PIO_DIR_INPUT | SAMA_PIO_SCHMITT

/*
 * This macro converts a pin identifier to a bitmask.
 */
#define SAMA_PIN_N(n)               (1U << n)

/**
 * @brief   SIU/SIUL register initializer type.
 */
typedef struct {
  int32_t pio_id;
  uint32_t pio_msk;
  uint32_t pio_cfg;
  uint32_t pio_ods;
} sama_pio_init_t;

/*
 * @brief   Initial setup of all defined pads.
 * @note    All pads are secured when SAMA_HAL_IS_SECURE is set as @p TRUE.
 * @note    The list is terminated by a {-1, 0, 0, 0}
 */
static const sama_pio_init_t sama_inits[] = {
  /* RGB Led Red and Led Blue */
  {SAMA_PIOA,
   SAMA_PIN_N(PIOA_LED_BLUE) | SAMA_PIN_N(PIOA_LED_RED),
   SAMA_PIO_FUNC_GPIO | SAMA_PIO_DIR_OUTPUT | SAMA_PIO_DRVSTR_HI,
   SAMA_PIO_LOW},
  /* RGB Led Green */
  {SAMA_PIOB,
   SAMA_PIN_N(PIOB_LED_GREEN),
   SAMA_PIO_FUNC_GPIO | SAMA_PIO_DIR_OUTPUT | SAMA_PIO_DRVSTR_HI,
   SAMA_PIO_LOW},
  /* User Button */
  {SAMA_PIOA,
   SAMA_PIN_N(PIOA_USER_PB),
   SAMA_PIO_FUNC_GPIO | SAMA_PIO_DIR_INPUT | SAMA_PIO_PUEN | SAMA_PIO_SCHMITT,
   SAMA_PIO_LOW},
  /* QSPI1 */
  {SAMA_PIOB,
   SAMA_PIN_N(PIOB_QSPI1_SCK) | SAMA_PIN_N(PIOB_QSPI1_CS) |
   SAMA_PIN_N(PIOB_QSPI1_IO0) | SAMA_PIN_N(PIOB_QSPI1_IO1) |
   SAMA_PIN_N(PIOB_QSPI1_IO2) | SAMA_PIN_N(PIOB_QSPI1_IO3),
   SAMA_PIO_FUNC_PERIPH_D | SAMA_PIO_PUEN,
   SAMA_PIO_HIGH},
  /* UART1 */
  {SAMA_PIOD,
   SAMA_PIN_N(PIOD_URXD1) | SAMA_PIN_N(PIOD_UTXD1),
   SAMA_PIO_FUNC_PERIPH_A,
   SAMA_PIO_HIGH},
  /* UART2 */
  {SAMA_PIOD,
   SAMA_PIN_N(PIOD_URXD2) | SAMA_PIN_N(PIOD_UTXD2),
   SAMA_PIO_FUNC_PERIPH_A,
   SAMA_PIO_HIGH},
  /* UART4 */
  {SAMA_PIOB,
   SAMA_PIN_N(PIOB_URXD4) | SAMA_PIN_N(PIOB_UTXD4),
   SAMA_PIO_FUNC_PERIPH_A,
   SAMA_PIO_HIGH},
  /* FLEXSPI4 */
  {SAMA_PIOC,
   SAMA_PIN_N(PIOC_SPI_FLEXCOM4_IO0) |
   SAMA_PIN_N(PIOC_SPI_FLEXCOM4_IO1) |
   SAMA_PIN_N(PIOC_SPI_FLEXCOM4_IO2) |
   SAMA_PIN_N(PIOC_SPI_FLEXCOM4_IO3),
   SAMA_PIO_FUNC_PERIPH_B,
   SAMA_PIO_HIGH},
  {SAMA_PIOD,
   SAMA_PIN_N(PIOD_SPI_FLEXCOM4_IO4),
   SAMA_PIO_FUNC_PERIPH_B,
   SAMA_PIO_HIGH},
  /* IRQ9 */
  {SAMA_PIOC,
   SAMA_PIN_N(PIOC_IRQ9),
   SAMA_PIO_FUNC_GPIO | SAMA_PIO_DIR_INPUT | SAMA_PIO_PDEN,
   SAMA_PIO_LOW},
  /* PIOD_IRQ1 Touch */
  {SAMA_PIOD,
   SAMA_PIN_N(PIOD_IRQ1),
   SAMA_PIO_FUNC_GPIO | SAMA_PIO_DIR_INPUT | SAMA_PIO_PUEN | SAMA_PIO_SCHMITT,
   SAMA_PIO_HIGH},
  /* LCDC */
  {SAMA_PIOB,
   SAMA_PIN_N(PIOB_LCDDAT0) | SAMA_PIN_N(PIOB_LCDDAT1) |
   SAMA_PIN_N(PIOB_LCDDAT2) | SAMA_PIN_N(PIOB_LCDDAT3) |
   SAMA_PIN_N(PIOB_LCDDAT4) | SAMA_PIN_N(PIOB_LCDDAT5) |
   SAMA_PIN_N(PIOB_LCDDAT6) | SAMA_PIN_N(PIOB_LCDDAT7) |
   SAMA_PIN_N(PIOB_LCDDAT8) | SAMA_PIN_N(PIOB_LCDDAT9) |
   SAMA_PIN_N(PIOB_LCDDAT10) | SAMA_PIN_N(PIOB_LCDDAT11) |
   SAMA_PIN_N(PIOB_LCDDAT12) | SAMA_PIN_N(PIOB_LCDDAT13) |
   SAMA_PIN_N(PIOB_LCDDAT14) | SAMA_PIN_N(PIOB_LCDDAT15) |
   SAMA_PIN_N(PIOB_LCDDAT16) | SAMA_PIN_N(PIOB_LCDDAT17) |
   SAMA_PIN_N(PIOB_LCDDAT18) | SAMA_PIN_N(PIOB_LCDDAT19) |
   SAMA_PIN_N(PIOB_LCDDAT20),
   SAMA_PIO_FUNC_PERIPH_A,
   SAMA_PIO_HIGH},
  {SAMA_PIOC,
   SAMA_PIN_N(PIOC_LCDDAT21) | SAMA_PIN_N(PIOC_LCDDAT22) |
   SAMA_PIN_N(PIOC_LCDDAT23) | SAMA_PIN_N(PIOC_LCDPWM) |
   SAMA_PIN_N(PIOC_LCDDISP) | SAMA_PIN_N(PIOC_LCDVSYNC) |
   SAMA_PIN_N(PIOC_LCDHSYNC) | SAMA_PIN_N(PIOC_LCDPCK) |
   SAMA_PIN_N(PIOC_LCDDEN),
   SAMA_PIO_FUNC_PERIPH_A,
   SAMA_PIO_HIGH},
  /* MMC Slot0 pads -J12 SD card */
  {SAMA_PIOA,
   SAMA_PIN_N(PIOA_SDMMC0_CK) | SAMA_PIN_N(PIOA_SDMMC0_CMD) |
   SAMA_PIN_N(PIOA_SDMMC0_DAT0) | SAMA_PIN_N(PIOA_SDMMC0_DAT1) |
   SAMA_PIN_N(PIOA_SDMMC0_DAT2) | SAMA_PIN_N(PIOA_SDMMC0_DAT3) |
   SAMA_PIN_N(PIOA_SDMMC0_DAT4) | SAMA_PIN_N(PIOA_SDMMC0_DAT5) |
   SAMA_PIN_N(PIOA_SDMMC0_DAT6) | SAMA_PIN_N(PIOA_SDMMC0_DAT7) |
   SAMA_PIN_N(PIOA_SDMMC0_WP) | SAMA_PIN_N(PIOA_SDMMC0_CD) |
   SAMA_PIN_N(PIOA_SDMMC0_VDDSEL),
   SAMA_PIO_FUNC_PERIPH_A | SAMA_PIO_DRVSTR_HI,
   SAMA_PIO_HIGH},
   /* MMC Slot1 pads -J14 MicroSD card */
  {SAMA_PIOA,
   SAMA_PIN_N(PIOA_SDMMC1_CK) | SAMA_PIN_N(PIOA_SDMMC1_CMD) |
   SAMA_PIN_N(PIOA_SDMMC1_DAT0) | SAMA_PIN_N(PIOA_SDMMC1_DAT1) |
   SAMA_PIN_N(PIOA_SDMMC1_DAT2) | SAMA_PIN_N(PIOA_SDMMC1_DAT3) |
   SAMA_PIN_N(PIOA_SDMMC1_CD),
   SAMA_PIO_FUNC_PERIPH_E | SAMA_PIO_DRVSTR_HI,
   SAMA_PIO_HIGH},
  /* list terminated*/
  {-1, 0, 0, 0}
};

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void) {

  sama_clock_init();
  /* Configures ETH's pins */
#if SAMA_HAL_IS_SECURE
  palSetGroupMode(PIOD, PAL_PORT_BIT(PIOD_ETH_GTXCK) | PAL_PORT_BIT(PIOD_ETH_GTXEN) |
                  PAL_PORT_BIT(PIOD_ETH_GRXDV) | PAL_PORT_BIT(PIOD_ETH_GRXER) |
                  PAL_PORT_BIT(PIOD_ETH_GRX0) | PAL_PORT_BIT(PIOD_ETH_GRX1) |
                  PAL_PORT_BIT(PIOD_ETH_GTX0) | PAL_PORT_BIT(PIOD_ETH_GTX1) |
                  PAL_PORT_BIT(PIOD_ETH_GMDC) | PAL_PORT_BIT(PIOD_ETH_GMDIO),
                  0U, PAL_SAMA_FUNC_PERIPH_D  |  PAL_MODE_SECURE);
#else
  palSetGroupMode(PIOD, PAL_PORT_BIT(PIOD_ETH_GTXCK) | PAL_PORT_BIT(PIOD_ETH_GTXEN) |
                  PAL_PORT_BIT(PIOD_ETH_GRXDV) | PAL_PORT_BIT(PIOD_ETH_GRXER) |
                  PAL_PORT_BIT(PIOD_ETH_GRX0) | PAL_PORT_BIT(PIOD_ETH_GRX1) |
                  PAL_PORT_BIT(PIOD_ETH_GTX0) | PAL_PORT_BIT(PIOD_ETH_GTX1) |
                  PAL_PORT_BIT(PIOD_ETH_GMDC) | PAL_PORT_BIT(PIOD_ETH_GMDIO),
                  0U, PAL_SAMA_FUNC_PERIPH_D);
#endif
}

/**
 * @brief   Board-specific initialization code.
 */
void boardInit(void) {
  unsigned i;

#if SAMA_HAL_IS_SECURE
  /* Disabling PMC write protection. */
  pmcDisableWP();

  /* Enabling port clock. */
  pmcEnablePIO();

  /* Enabling write protection.  */
  pmcEnableWP();
#endif /* SAMA_HAL_IS_SECURE */

  /* Configuring all PIO A pads with default configuration.  */
#if SAMA_HAS_PIOA
#if SAMA_HAL_IS_SECURE
  _PIOA->PIO_PIO_[SAMA_PIOA].S_PIO_SIOSR = SAMA_DEFAULT_SIOSR;
  _PIOA->PIO_PIO_[SAMA_PIOA].S_PIO_SIONR = SAMA_DEFAULT_SIONR;
#endif /* SAMA_HAL_IS_SECURE */
  _PIOA->PIO_PIO_[SAMA_PIOA].S_PIO_MSKR = SAMA_DEFAULT_MSKR;
  _PIOA->PIO_PIO_[SAMA_PIOA].S_PIO_CFGR = SAMA_DEFAULT_CFGR;
#endif /* SAMA_HAS_PIOA */

  /* Configuring all PIO B pads with default configuration.  */
#if SAMA_HAS_PIOB
#if SAMA_HAL_IS_SECURE
  _PIOA->PIO_PIO_[SAMA_PIOB].S_PIO_SIOSR = SAMA_DEFAULT_SIOSR;
  _PIOA->PIO_PIO_[SAMA_PIOB].S_PIO_SIONR = SAMA_DEFAULT_SIONR;
#endif /* SAMA_HAL_IS_SECURE */
  _PIOA->PIO_PIO_[SAMA_PIOB].S_PIO_MSKR = SAMA_DEFAULT_MSKR;
  _PIOA->PIO_PIO_[SAMA_PIOB].S_PIO_CFGR = SAMA_DEFAULT_CFGR;
#endif /* SAMA_HAS_PIOB */

  /* Configuring all PIO C pads with default configuration.  */
#if SAMA_HAS_PIOC
#if SAMA_HAL_IS_SECURE
  _PIOA->PIO_PIO_[SAMA_PIOC].S_PIO_SIOSR = SAMA_DEFAULT_SIOSR;
  _PIOA->PIO_PIO_[SAMA_PIOC].S_PIO_SIONR = SAMA_DEFAULT_SIONR;
#endif /* SAMA_HAL_IS_SECURE */
  _PIOA->PIO_PIO_[SAMA_PIOC].S_PIO_MSKR = SAMA_DEFAULT_MSKR;
  _PIOA->PIO_PIO_[SAMA_PIOC].S_PIO_CFGR = SAMA_DEFAULT_CFGR;
#endif /* SAMA_HAS_PIOC */

  /* Configuring all PIO D pads with default configuration.  */
#if SAMA_HAS_PIOD
#if SAMA_HAL_IS_SECURE
  _PIOA->PIO_PIO_[SAMA_PIOD].S_PIO_SIOSR = SAMA_DEFAULT_SIOSR;
  _PIOA->PIO_PIO_[SAMA_PIOD].S_PIO_SIONR = SAMA_DEFAULT_SIONR;
#endif /* SAMA_HAL_IS_SECURE */
  _PIOA->PIO_PIO_[SAMA_PIOD].S_PIO_MSKR = SAMA_DEFAULT_MSKR;
  _PIOA->PIO_PIO_[SAMA_PIOD].S_PIO_CFGR = SAMA_DEFAULT_CFGR;
#endif /* SAMA_HAS_PIOD */

  /* Initialize PIO registers for defined pads.*/
  i = 0;
  while (sama_inits[i].pio_id != -1) {
#if SAMA_HAL_IS_SECURE
    _PIOA->PIO_PIO_[sama_inits[i].pio_id].S_PIO_SIOSR = sama_inits[i].pio_msk;
    _PIOA->PIO_PIO_[sama_inits[i].pio_id].S_PIO_MSKR = sama_inits[i].pio_msk;
    _PIOA->PIO_PIO_[sama_inits[i].pio_id].S_PIO_CFGR = sama_inits[i].pio_cfg;
    if(sama_inits[i].pio_ods == SAMA_PIO_HIGH) {
      _PIOA->PIO_PIO_[sama_inits[i].pio_id].S_PIO_SODR = sama_inits[i].pio_msk;
    }
    else {
      _PIOA->PIO_PIO_[sama_inits[i].pio_id].S_PIO_CODR = sama_inits[i].pio_msk;
    }
#else
    _PIOA->PIO_IO_GROUP[sama_inits[i].pio_id].PIO_MSKR = sama_inits[i].pio_msk;
    _PIOA->PIO_IO_GROUP[sama_inits[i].pio_id].PIO_CFGR = sama_inits[i].pio_cfg;
    if(sama_inits[i].pio_ods == SAMA_PIO_HIGH) {
      _PIOA->PIO_IO_GROUP[sama_inits[i].pio_id].PIO_SODR = sama_inits[i].pio_msk;
    }
    else {
      _PIOA->PIO_IO_GROUP[sama_inits[i].pio_id].PIO_CODR = sama_inits[i].pio_msk;
    }
#endif /* SAMA_HAL_IS_SECURE */
    i++;
  }
}
