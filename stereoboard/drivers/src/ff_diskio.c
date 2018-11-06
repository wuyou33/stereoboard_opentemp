/**
  ******************************************************************************
  * @file    diskio.c 
  * @brief   FatFs low level disk I/O module.
/* Includes ------------------------------------------------------------------*/
#include "diskio.h"
#include "image_spi_sd.h"

extern SD_CARD_INFO SD_CardInfo;

/**
  * @brief  Gets Disk Status 
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
  switch (pdrv)
	{
		case 0 :
			return RES_OK;
		case 1 :
			return RES_OK;
		case 2 :
			return RES_OK;
		default:
			return STA_NOINIT;
	}
}

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
  int Status;
	switch(pdrv)
	{
    case 0:
      Status = SD_Init();
      if(Status == 0)
      {
        return RES_OK;
      }
      else
      {
        return STA_NOINIT;
      }
    case 1:
      return RES_OK;
    case 2:
		  return RES_OK;
    case 3:
		  return RES_OK;
    default:
		  return STA_NOINIT;
	}
}

/**
  * @brief  Reads Sector(s) 
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	        /* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
  int Status;
	if(!count)
	{
    return RES_PARERR;
	}
	switch(pdrv)
	{
    case 0:
      if(count == 1)
      {
        Status = SD_ReadSingleBlock(sector, buff);
        if(Status == 0)
        {
          return RES_OK;
        }
        else
        {
          return RES_ERROR;
        }
		  }
		  else
		  {
        Status = SD_ReadMultiBlock(sector, buff, count);
        if(Status == 0)
        {
          return RES_OK;
        }
        else
        {
          return RES_ERROR;
        }
		  }
    case 1:
      if(count == 1)
      {
        return RES_OK;
      }
      else
      {
        return RES_OK;
      }
    default:
		  return RES_ERROR;
	}
}

/**
  * @brief  Writes Sector(s)  
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT disk_write (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count        	/* Number of sectors to write */
)
{
  int Status;
	if(!count)
	{
    return RES_PARERR;
	}
	switch(pdrv)
	{
    case 0:
      if(count == 1)
		  {
        Status = SD_WriteSingleBlock(sector, (uint8_t*)(&buff[0]));
        if(Status == 0)
        {
          return RES_OK;
        }
        else
        {
          return RES_ERROR;
        }
		  }                                                
		  else
      {
        Status = SD_WriteMultiBlock(sector, (uint8_t*)(&buff[0]), count);
        if(Status == 0)
        {
          return RES_OK;
        }
        else
        {
          return RES_ERROR;
        }
      }
    case 1:
      if(count == 1)
      {
        return RES_OK;
      }                                                
      else
      {
        return RES_OK;
      }
    default:
      return RES_ERROR;
	}
}

/**
  * @brief  I/O control operation  
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
  if (pdrv == 0)
	{
    SD_GetCardInfo(&SD_CardInfo);
    switch(cmd)
    {
      case CTRL_SYNC:
        return RES_OK;
      case GET_SECTOR_COUNT:
        *(DWORD*)buff = SD_CardInfo.Capacity/SD_CardInfo.BlockSize;
        return RES_OK;
      case GET_BLOCK_SIZE:
        *(WORD*)buff = SD_CardInfo.BlockSize;
        return RES_OK;
      case CTRL_POWER:
        break;
      case CTRL_LOCK:
        break;
      case CTRL_EJECT:
        break;
      case MMC_GET_TYPE:
        break;
      case MMC_GET_CSD:
        break;
      case MMC_GET_CID:
        break;
      case MMC_GET_OCR:
        break;
      case MMC_GET_SDSTAT:
        break;
    } 
  }
  else if(pdrv == 1)
  {
    switch (cmd) 
    {
      case CTRL_SYNC:
        return RES_OK;
      case GET_SECTOR_COUNT:
        return RES_OK;
      case GET_SECTOR_SIZE:
        return RES_OK;
      case GET_BLOCK_SIZE:
        return RES_OK;
      case CTRL_POWER:
        break;
      case CTRL_LOCK:
        break;
      case CTRL_EJECT:
        break;
      case MMC_GET_TYPE:
        break;
      case MMC_GET_CSD:
        break;
      case MMC_GET_CID:
        break;
      case MMC_GET_OCR:
        break;
      case MMC_GET_SDSTAT:
        break;	
    }
	}
	else
  {
    return RES_PARERR;
	}
	return RES_PARERR;
}

/**
  * @brief  Gets Time, to be defined 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime (void)
{
  return 0;
}