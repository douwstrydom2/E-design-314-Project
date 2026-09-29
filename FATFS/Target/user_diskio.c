/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "user_diskio_spi.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static volatile DSTATUS Stat = STA_NOINIT;

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status     (BYTE pdrv);
DRESULT USER_read       (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write    (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif
#if _USE_IOCTL == 1
  DRESULT USER_ioctl    (BYTE pdrv, BYTE cmd, void *buff);
#endif

Diskio_drvTypeDef USER_Driver =
{
    USER_initialize,
    USER_status,
    USER_read,
#if _USE_WRITE
    USER_write,
#endif
#if _USE_IOCTL == 1
    USER_ioctl,
#endif
};

/* ---------------------------------------------------------------------------
   Thin wrappers — all real work is done in user_diskio_spi.c
   --------------------------------------------------------------------------- */

/**
  * @brief  Initialises a drive.
  * @param  pdrv  Physical drive number (0..)
  * @retval DSTATUS
  */
DSTATUS USER_initialize(BYTE pdrv)
{
  /* USER CODE BEGIN INIT */
    return USER_SPI_initialize(pdrv);
  /* USER CODE END INIT */
}

/**
  * @brief  Gets disk status.
  * @param  pdrv  Physical drive number (0..)
  * @retval DSTATUS
  */
DSTATUS USER_status(BYTE pdrv)
{
  /* USER CODE BEGIN STATUS */
    return USER_SPI_status(pdrv);
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads sector(s).
  * @param  pdrv    Physical drive number (0..)
  * @param  buff    Data buffer to store read data
  * @param  sector  Sector address (LBA)
  * @param  count   Number of sectors to read (1..128)
  * @retval DRESULT
  */
DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  /* USER CODE BEGIN READ */
    return USER_SPI_read(pdrv, buff, sector, count);return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes sector(s).
  * @param  pdrv    Physical drive number (0..)
  * @param  buff    Data to be written
  * @param  sector  Sector address (LBA)
  * @param  count   Number of sectors to write (1..128)
  * @retval DRESULT
  */
#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  /* USER CODE BEGIN WRITE */
    return USER_SPI_write(pdrv, buff, sector, count);
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation.
  * @param  pdrv  Physical drive number (0..)
  * @param  cmd   Control code
  * @param  buff  Buffer to send/receive control data
  * @retval DRESULT
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  /* USER CODE BEGIN IOCTL */
    return USER_SPI_ioctl(pdrv, cmd, buff);
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */
