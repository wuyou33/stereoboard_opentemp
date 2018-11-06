/* File Info
  This file provides spi driver for SD card operation on STM32F4 chips.

  It also includes an image cature function for saving images to SD card.      
------------------------------------------------------------------------------*/ 
#include "image_spi_sd.h"

/* To use image_save_sd function, following files should be included */
#include "ff.h"  
#include "diskio.h"
#include "encoding/jpeg.h"
#include <stdio.h>
/* -------------------------------------------------------------- */
SD_CARD_INFO SD_CardInfo;

/* Examples for using this function in main.c
  1. #include "image_spi_sd.h"
  2. spi_sd_init();    
  3. image_save_sd(&current_image_pair);

  This function can save images to SD card with incremental file names like i00001.jpg ...
  And it will start saving as i00001.jpg when rebooting the board.   
------------------------------------------------------------------------------*/ 
void image_save_sd(struct image_t *img)
{
  static char mount_ok = 0;
  static int image_file_num = 0;
  char save_name[10];
  if (image_file_num < 9999)
  {
    image_file_num++;
    sprintf(save_name, "i%05d.jpg", image_file_num);
    FATFS FatFs;
    FIL MyFile;
    struct image_t img_jpeg;
    unsigned int numWrite;
    img_jpeg.type = IMAGE_JPEG;
    img_jpeg.w = img->w;
    img_jpeg.h = img->h;
    img_jpeg.buf_size = 2 * img_jpeg.w * img_jpeg.h;
    uint8_t temp_buffer[img_jpeg.buf_size];
    img_jpeg.buf = temp_buffer;
    jpeg_encode_image(img, &img_jpeg, 99, true);
    if (mount_ok == 1)
    {
      if(f_open(&MyFile,save_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
      {
        f_write(&MyFile, img_jpeg.buf, img_jpeg.buf_size, &numWrite);
        f_close(&MyFile);
      }
    }else
    {
      if (f_mount(&FatFs, "", 1) == FR_OK)
      {
        mount_ok = 1;
        if(f_open(&MyFile,save_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
        {
          f_write(&MyFile, img_jpeg.buf, img_jpeg.buf_size, &numWrite);
          f_close(&MyFile);
        }
      }
    }
  }
}

int image_spi_read_write(uint8_t data)
{  
  // Send byte 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI2, data);

  // Read byte  
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI2);
}

void spi_sd_init(void)
{		
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  // SCK--GPIOB^13; MISO--GPIOB^14; MOSI--GPIOB^15 //
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2); 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // CSN--GPIOB^12 //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  spi_sd_disable(); 
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_Init(SPI2, &SPI_InitStructure);	
  SPI_Cmd(SPI2, ENABLE);
}

int SD_Init(void)
{
  uint16_t counter; 
  uint8_t response;	
  uint8_t r[6] = {0};
  /* Send dummy byte 0xFF, 10 times with CS high
    Rise CS and MOSI for 80 clocks cycles */
  for(counter=0; counter<=9; counter++)
  {
      image_spi_read_write(DUMMY_BYTE);
  }	

  /* Start send CMD0 to put SD in SPI mode
    and return 0x01 means it is in IDLE state */
  for(counter=0; counter<0xFFF; counter++)
  {
      response = SD_send_command(CMD0, 0, 0x95);
      if(response == 0x01)
      {
          counter = 0;
          break;
      }
  }    
  if(counter == 0xFFF)
  {
    return 1;// Timeout return
  }

  /* Get the card type, version */
  response = SD_send_command_hold(CMD8, 0x1AA, 0x87);
  /* response=0x05 means V1.0 */
  if(response == 0x05)
  {
    SD_CardInfo.CardType = CARDTYPE_SDV1;
    /* End of CMD8, chip disable and dummy byte */
    spi_sd_disable();
    image_spi_read_write(DUMMY_BYTE);
    /* SD1.0/MMC start initialize */
    /* Send CMD55+ACMD41, No-response is a MMC card, otherwise is a SD1.0 card */
    for(counter=0; counter<0xFFF; counter++)
    {
      response = SD_send_command(CMD55, 0, 0);			/* should return 0x01 */
      if(response != 0x01)
      {
        return response;
      }
      response = SD_send_command(ACMD41, 0, 0);			/* should return 0x00 */
      if(response == 0x00)
      {
        counter = 0;
        break;
      }
    }
    /* MMC card initialize start */
    if(counter == 0xFFF)
    {
      for(counter=0; counter<0xFFF; counter++)
      {
        response = SD_send_command(CMD1, 0, 0);		/* should be return 0x00 */
        if(response == 0x00)
        {
          counter = 0;
          break;
        }
      }
      /* Timeout return */
      if(counter == 0xFFF)
      {
        return 2;
      }	

      SD_CardInfo.CardType = CARDTYPE_MMC;
    }
                
    /* CRC disable */
    response = SD_send_command(CMD59, 0, 0x01);
    if(response != 0x00)
    {
      return response;		/* response error, return */
    }
    
    /* Set the block size */
    response = SD_send_command(CMD16, SD_BLOCKSIZE, 0xFF);
    if(response != 0x00)
    {
      return response;		/* response error, return r1 */
    }
  }
  /* response=0x01 -> V2.x, read OCR register, check version */
  else if(response == 0x01)
  {
    /* 4Bytes returned after CMD8 sent	*/
    r[0] = image_spi_read_write(DUMMY_BYTE);				/* should be 0x00 */
    r[1] = image_spi_read_write(DUMMY_BYTE);				/* should be 0x00 */
    r[2] = image_spi_read_write(DUMMY_BYTE);				/* should be 0x01 */
    r[3] = image_spi_read_write(DUMMY_BYTE);				/* should be 0xAA */

    /* End of CMD8, chip disable and dummy byte */ 
    spi_sd_disable();
    image_spi_read_write(DUMMY_BYTE);

    /* Check voltage range be 2.7-3.6V	*/
    if(r[2]==0x01 && r[3]==0xAA)
    {
      for(counter=0; counter<0xFFF; counter++)
      {
        response = SD_send_command(CMD55, 0, 0);			/* should be return 0x01 */
        if(response!=0x01)
        {
          return response;
        }				

        response = SD_send_command(ACMD41, 0x40000000, 0);	/* should be return 0x00 */
        if(response == 0x00)
        {
          counter = 0;
          break;
        }
      }
      /* Timeout return */
      if(counter == 0xFFF)
      {
        return 3;
      }

      /* Read OCR by CMD58 */
      response = SD_send_command_hold(CMD58, 0, 0);
      if(response!=0x00)
      {
        return response;		/* response error, return r1 */
      }
      r[0] = image_spi_read_write(DUMMY_BYTE);					
      r[1] = image_spi_read_write(DUMMY_BYTE);					
      r[2] = image_spi_read_write(DUMMY_BYTE);					
      r[3] = image_spi_read_write(DUMMY_BYTE);					

      /* End of CMD58, chip disable and dummy byte */
      spi_sd_disable();
      image_spi_read_write(DUMMY_BYTE);

      /* OCR -> CCS(bit30)  1: SDV2HC	 0: SDV2 */
      if(r[0] & 0x40)
      {
        SD_CardInfo.CardType = CARDTYPE_SDV2HC;
      }
      else
      {
        SD_CardInfo.CardType = CARDTYPE_SDV2;
      }
    }	
  }
  return 0;
}

int SD_GetCardInfo(SD_CARD_INFO* SD_CardInfo)
{
  uint8_t response;
  uint8_t CSD_Tab[16];
  uint8_t CID_Tab[16];

  /* Send CMD9, Read CSD */
  response = SD_send_command(CMD9, 0, 0xFF);
  if(response != 0x00)
  {
    return response;
  }

  if(SD_read_buffer(CSD_Tab, 16, RELEASE))
  {
	return 1;
  }

  /* Send CMD10, Read CID */
  response = SD_send_command(CMD10, 0, 0xFF);
  if(response != 0x00)
  {
    return response;
  }

  if(SD_read_buffer(CID_Tab, 16, RELEASE))
  {
	return 2;
  }  

  /* Byte 0 */
  SD_CardInfo->CSD.CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
  SD_CardInfo->CSD.SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
  SD_CardInfo->CSD.Reserved1 = CSD_Tab[0] & 0x03;
  /* Byte 1 */
  SD_CardInfo->CSD.TAAC = CSD_Tab[1] ;
  /* Byte 2 */
  SD_CardInfo->CSD.NSAC = CSD_Tab[2];
  /* Byte 3 */
  SD_CardInfo->CSD.MaxBusClkFrec = CSD_Tab[3];
  /* Byte 4 */
  SD_CardInfo->CSD.CardComdClasses = CSD_Tab[4] << 4;
  /* Byte 5 */
  SD_CardInfo->CSD.CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
  SD_CardInfo->CSD.RdBlockLen = CSD_Tab[5] & 0x0F;
  /* Byte 6 */
  SD_CardInfo->CSD.PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;
  SD_CardInfo->CSD.WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
  SD_CardInfo->CSD.RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
  SD_CardInfo->CSD.DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
  SD_CardInfo->CSD.Reserved2 = 0; /* Reserved */
  SD_CardInfo->CSD.DeviceSize = (CSD_Tab[6] & 0x03) << 10;
  /* Byte 7 */
  SD_CardInfo->CSD.DeviceSize |= (CSD_Tab[7]) << 2;
  /* Byte 8 */
  SD_CardInfo->CSD.DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;
  SD_CardInfo->CSD.MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
  SD_CardInfo->CSD.MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);
  /* Byte 9 */
  SD_CardInfo->CSD.MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
  SD_CardInfo->CSD.MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
  SD_CardInfo->CSD.DeviceSizeMul = (CSD_Tab[9] & 0x03) << 1;
  /* Byte 10 */
  SD_CardInfo->CSD.DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;
  SD_CardInfo->CSD.EraseGrSize = (CSD_Tab[10] & 0x7C) >> 2;
  SD_CardInfo->CSD.EraseGrMul = (CSD_Tab[10] & 0x03) << 3;
  /* Byte 11 */
  SD_CardInfo->CSD.EraseGrMul |= (CSD_Tab[11] & 0xE0) >> 5;
  SD_CardInfo->CSD.WrProtectGrSize = (CSD_Tab[11] & 0x1F);
  /* Byte 12 */
  SD_CardInfo->CSD.WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
  SD_CardInfo->CSD.ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
  SD_CardInfo->CSD.WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
  SD_CardInfo->CSD.MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;
  /* Byte 13 */
  SD_CardInfo->CSD.MaxWrBlockLen |= (CSD_Tab[13] & 0xc0) >> 6;
  SD_CardInfo->CSD.WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
  SD_CardInfo->CSD.Reserved3 = 0;
  SD_CardInfo->CSD.ContentProtectAppli = (CSD_Tab[13] & 0x01);
  /* Byte 14 */
  SD_CardInfo->CSD.FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
  SD_CardInfo->CSD.CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
  SD_CardInfo->CSD.PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
  SD_CardInfo->CSD.TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
  SD_CardInfo->CSD.FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
  SD_CardInfo->CSD.ECC = (CSD_Tab[14] & 0x03);
  /* Byte 15 */
  SD_CardInfo->CSD.CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;
  SD_CardInfo->CSD.Reserved4 = 1;

  if(SD_CardInfo->CardType == CARDTYPE_SDV2HC)
  {
	 /* Byte 7 */
	 SD_CardInfo->CSD.DeviceSize = (u16)(CSD_Tab[8]) *256;
	 /* Byte 8 */
	 SD_CardInfo->CSD.DeviceSize += CSD_Tab[9] ;
  }

  SD_CardInfo->Capacity = SD_CardInfo->CSD.DeviceSize * SD_BLOCKSIZE * 1024;
  SD_CardInfo->BlockSize = SD_BLOCKSIZE;

  /* Byte 0 */
  SD_CardInfo->CID.ManufacturerID = CID_Tab[0];
  /* Byte 1 */
  SD_CardInfo->CID.OEM_AppliID = CID_Tab[1] << 8;
  /* Byte 2 */
  SD_CardInfo->CID.OEM_AppliID |= CID_Tab[2];
  /* Byte 3 */
  SD_CardInfo->CID.ProdName1 = CID_Tab[3] << 24;
  /* Byte 4 */
  SD_CardInfo->CID.ProdName1 |= CID_Tab[4] << 16;
  /* Byte 5 */
  SD_CardInfo->CID.ProdName1 |= CID_Tab[5] << 8;
  /* Byte 6 */
  SD_CardInfo->CID.ProdName1 |= CID_Tab[6];
  /* Byte 7 */
  SD_CardInfo->CID.ProdName2 = CID_Tab[7];
  /* Byte 8 */
  SD_CardInfo->CID.ProdRev = CID_Tab[8];
  /* Byte 9 */
  SD_CardInfo->CID.ProdSN = CID_Tab[9] << 24;
  /* Byte 10 */
  SD_CardInfo->CID.ProdSN |= CID_Tab[10] << 16;
  /* Byte 11 */
  SD_CardInfo->CID.ProdSN |= CID_Tab[11] << 8;
  /* Byte 12 */
  SD_CardInfo->CID.ProdSN |= CID_Tab[12];
  /* Byte 13 */
  SD_CardInfo->CID.Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
  /* Byte 14 */
  SD_CardInfo->CID.ManufactDate = (CID_Tab[13] & 0x0F) << 8;
  /* Byte 15 */
  SD_CardInfo->CID.ManufactDate |= CID_Tab[14];
  /* Byte 16 */
  SD_CardInfo->CID.CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
  SD_CardInfo->CID.Reserved2 = 1;
  return 0;  
}

int SD_read_buffer(uint8_t *buff, uint16_t len, uint8_t release)
{
  uint8_t r1;
  register uint16_t retry;

  /* Card enable, Prepare to read	*/
  spi_sd_enable();

  /* Wait start-token 0xFE */
  for(retry=0; retry<2000; retry++)
  {
	 r1 = image_spi_read_write(DUMMY_BYTE);
	 if(r1 == 0xFE)
	 {
		 retry = 0;
		 break;
	 }
  }

  /* Timeout return	*/
  if(retry == 2000)
  {
	 spi_sd_disable();
	 return 1;
  }

  /* Start reading */
  for(retry=0; retry<len; retry++)
  {
     *(buff+retry) = image_spi_read_write(DUMMY_BYTE);
  }

  /* 2bytes dummy CRC */
  image_spi_read_write(DUMMY_BYTE);
  image_spi_read_write(DUMMY_BYTE);

  /* chip disable and dummy byte */ 
  if(release)
  {
	 spi_sd_disable();
	 image_spi_read_write(DUMMY_BYTE);
  }
  return 0;
}

int SD_ReadSingleBlock(uint32_t sector, uint8_t *buffer)
{
  uint8_t r1;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(SD_CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	 sector = sector<<9;
  }
	
  /* Send CMD17 : Read single block command */
  r1 = SD_send_command(CMD17, sector, 0);
	
  if(r1 != 0x00)
  {
	 return 1;
  }
	
  /* Start read and return the result */
  r1 = SD_read_buffer(buffer, SD_BLOCKSIZE, RELEASE);

  /* Send stop data transmit command - CMD12 */
  SD_send_command(CMD12, 0, 0);
  return r1;
}

int SD_ReadMultiBlock(uint32_t sector, uint8_t *buffer, uint32_t NbrOfSector)
{
  uint8_t r1;
  register uint32_t i;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(SD_CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	 sector = sector<<9;
  }

  /* Send CMD18 : Read multi block command */
  r1 = SD_send_command(CMD18, sector, 0);
  if(r1 != 0x00)
  {
     return 1;
  }

  /* Start read	*/
  for(i=0; i<NbrOfSector; i++)
  {
     if(SD_read_buffer(buffer+i*SD_BLOCKSIZE, SD_BLOCKSIZE, HOLD))
     {
		 /* Send stop data transmit command - CMD12	*/
		 SD_send_command(CMD12, 0, 0);
		 /* chip disable and dummy byte */
		 spi_sd_disable();
		 return 2;
     }
  }
	
  /* Send stop data transmit command - CMD12 */
  SD_send_command(CMD12, 0, 0);

  /* chip disable and dummy byte */
  spi_sd_disable();
  image_spi_read_write(DUMMY_BYTE);	
  return 0;
}

int SD_WriteSingleBlock(uint32_t sector, uc8 *buffer)
{
  uint8_t r1;
  register uint16_t i;
  uint32_t retry;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(SD_CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	 sector = sector<<9;
  }
	
  /* Send CMD24 : Write single block command */
  r1 = SD_send_command(CMD24, sector, 0);
	
  if(r1 != 0x00)
  {
	 return 1;
  }

  /* Card enable, Prepare to write */
  spi_sd_enable();
  image_spi_read_write(DUMMY_BYTE);
  image_spi_read_write(DUMMY_BYTE);
  image_spi_read_write(DUMMY_BYTE);
  /* Start data write token: 0xFE */
  image_spi_read_write(0xFE);
	
  /* Start single block write the data buffer */
  for(i=0; i<SD_BLOCKSIZE; i++)
  {
    image_spi_read_write(*buffer++);
  }

  /* 2Bytes dummy CRC */
  image_spi_read_write(DUMMY_BYTE);
  image_spi_read_write(DUMMY_BYTE);
	
  /* MSD card accept the data */
  r1 = image_spi_read_write(DUMMY_BYTE);
  if((r1&0x1F) != 0x05)
  {
    spi_sd_disable();
    return 2;
  }
	
  /* Wait all the data programm finished */
  retry = 0;
  while(image_spi_read_write(DUMMY_BYTE) == 0x00)
  {	
	 /* Timeout return */
	 if(retry++ == 0x40000)
	 {
	    spi_sd_disable();
	    return 3;
	 }
  }

  /* chip disable and dummy byte */ 
  spi_sd_disable();
  image_spi_read_write(DUMMY_BYTE);	
  return 0;
}

int SD_WriteMultiBlock(uint32_t sector, uc8 *buffer, uint32_t NbrOfSector)
{
  uint8_t r1;
  register uint16_t i;
  register uint32_t n;
  uint32_t retry;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(SD_CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	  sector = sector<<9;
  }

  /* Send command ACMD23 berfore multi write if is not a MMC card */
  if(SD_CardInfo.CardType != CARDTYPE_MMC)
  {
	  SD_send_command(ACMD23, NbrOfSector, 0x00);
  }
	
  /* Send CMD25 : Write nulti block command	*/
  r1 = SD_send_command(CMD25, sector, 0);
	
  if(r1 != 0x00)
  {
	  return 1;
  }

  /* Card enable, Prepare to write */
  spi_sd_enable();
  image_spi_read_write(DUMMY_BYTE);
  //image_spi_read_write(DUMMY_BYTE);
  //image_spi_read_write(DUMMY_BYTE);

  for(n=0; n<NbrOfSector; n++)
  {	
	 /* Start multi block write token: 0xFC */
	 image_spi_read_write(0xFC);

	 for(i=0; i<SD_BLOCKSIZE; i++)
	 {
		image_spi_read_write(*buffer++);
	 }	

	 /* 2Bytes dummy CRC */
	 image_spi_read_write(DUMMY_BYTE);
	 image_spi_read_write(DUMMY_BYTE);

	 /* MSD card accept the data */
	 r1 = image_spi_read_write(DUMMY_BYTE);
	 if((r1&0x1F) != 0x05)
	 {
	    spi_sd_disable();
	    return 2;
	 }

	 /* Wait all the data programm finished	*/
	 retry = 0;
	 while(image_spi_read_write(DUMMY_BYTE) != 0xFF)
	 {	
		/* Timeout return */
		if(retry++ == 0x40000)
		{
		   spi_sd_disable();
		   return 3;
		}
	 }
  }

  /* Send end of transmit token: 0xFD */
  r1 = image_spi_read_write(0xFD);
  if(r1 == 0x00)
  {
	 return 4;
  }

  /* Wait all the data programm finished */
  retry = 0;
  while(image_spi_read_write(DUMMY_BYTE) != 0xFF)
  {	
	 /* Timeout return */
	 if(retry++ == 0x40000)
	 {
	     spi_sd_disable();
	     return 5;
	 }
  }

  /* chip disable and dummy byte */
  spi_sd_disable();
  image_spi_read_write(DUMMY_BYTE);
  return 0;
}

int SD_send_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t response;
  uint8_t retry;

  /* Dummy byte and chip enable */
  image_spi_read_write(DUMMY_BYTE);
  spi_sd_enable();

  /* Command, argument and crc */
  image_spi_read_write(cmd | 0x40);
  image_spi_read_write(arg >> 24);
  image_spi_read_write(arg >> 16);
  image_spi_read_write(arg >> 8);
  image_spi_read_write(arg);
  image_spi_read_write(crc);
  
  /* Wait response, quit till timeout */
  for(retry=0; retry<200; retry++)
  {
	 response = image_spi_read_write(DUMMY_BYTE);
	 if(response != 0xFF)
	 {
		 break;
	 }
  }

  /* Chip disable and dummy byte */ 
  spi_sd_disable();
  image_spi_read_write(DUMMY_BYTE);
  return response;
}	

int SD_send_command_hold(uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t r1;
  uint8_t retry;

  /* Dummy byte and chip enable */
  image_spi_read_write(DUMMY_BYTE);
  spi_sd_enable();

  /* Command, argument and crc */
  image_spi_read_write(cmd | 0x40);
  image_spi_read_write(arg >> 24);
  image_spi_read_write(arg >> 16);
  image_spi_read_write(arg >> 8);
  image_spi_read_write(arg);
  image_spi_read_write(crc);
  
  /* Wait response, quit till timeout */
  for(retry=0; retry<200; retry++)
  {
	 r1 = image_spi_read_write(DUMMY_BYTE);
	 if(r1 != 0xFF)
	 {
		 break;
	 }
  }
  return r1;
}
