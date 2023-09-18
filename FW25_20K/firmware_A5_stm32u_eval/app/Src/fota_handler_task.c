/**************************************************************************//**
* @file cmd_task.c
* @brief command interpreter task
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/


//#include "../../app/Inc/cmd_handler_task.h"
#include "../../app/Inc/fota_handler_task.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "_sem.h"
#include "_mem.h"
#include "msg_type.h"
#include "membuf.h"
#include "A4_proto/a4opc.h"
#include "A4_proto/a4err.h"
#include "A4_proto/a4frame.h"
#include "A4_proto/Parser.h"
#include "packetbuf.h"
#include "stub.h"
#include "auxcmd.h"

#include "../../app/Inc/main.h"
#include "../../app/Inc/sys_errno.h"
#include "../../app/Inc/ver.h"

#define A4_TX_BUFFER_SIZE	264
#define N_CMD_TX_BUFFERS	2

uint8_t UartReadyTx  = RESET;

#define Y_HEADER 0x67

extern IWDG_HandleTypeDef hiwdg;
extern QueueHandle_t samplerq;
static PACKETBUF_HDR * handlefotaCmdPacket(MEMBUF_POOL *pool, PACKETBUF_HDR * p, SemaphoreHandle_t sync_sm);

fota_sm _fota_sm;
extern UART_HandleTypeDef huart5;

int16_t crc_received,crc_calculated;

#define FLASH_START_ADDR		  0x08000000
#define FLASH_BACKUP_START_ADDR   0x08020000
#define FLASH_RUNTIME_START_ADDR  0x08100000
#define FLASH_USER_END_ADDR       0x081dfff
#define FLASH_RUNTIME_STORE       0x08170000
#define FLASH_A5_SIG_SWITCH       0x081e0000
#define FLASH_BACKUP_A5_SIG_ADR   0x081e2000
#define FLASH_RUNTIME_A5_SIG_ADR  0x081e4000
#define FLASH_RUNTIME_A5_END_ADR  0x081f0000

uint8_t runtime_a5_unique[16] =     {0x23,0x45,0xa6,0x59,0xa4,0xd2,0xe6,0x7a,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
uint8_t backup_a5_unique[16] =      {0x32,0x54,0x6a,0x95,0x4a,0x2d,0x6e,0xa7,0x80,0x90,0xa0,0xb0,0xc0,0xd0,0xe0,0xf0};
uint8_t runtime_a5_write_data[16] = {0x45,0x18,0xd1,0xa5,0x5a,0xc8,0xe2,0xdb,0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf1};

uint8_t runtime_a5_string[16] =        "Application # 1 ";
uint8_t backup_a5_string[16] =         "Application # 2 ";
uint8_t runtime_a5_write_string[16] =  "Application # m ";

/*Variable used for Erase procedure*/
//static FLASH_EraseInitTypeDef EraseInitStruct;
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);

flash_file_param _flash_file_param;

uint8_t UartReadyRx  = RESET;


/**
 * @brief   Receives data from UART.
 * @param   *data: Array to save the received data.
 * @param   length:  Size of the data.
 * @return  status: Report about the success of the receiving.
 */
uint32_t uart_receive(uint8_t *data, uint16_t length)
{
   uint32_t status;

   status = UART_ERROR;

   if (HAL_OK == HAL_UART_Receive_IT(&huart5, data, length))
   {
	 status = UART_OK;
   }

    return status;
}

void write_signature(uint8_t signature)
{
	uint32_t address = 0;
	uint32_t index = 0;
	uint32_t address_stop;
	uint8_t  a,data[100];

	 /* Disable instruction cache prior to internal cacheable memory update */
	  if (HAL_ICACHE_Disable() != HAL_OK)
	  {
		 // Error_Handler();
		  Error_Handler((uint8_t *)__FILE__, __LINE__);
	  }

	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_NS_START_ADDR and FLASH_USER_NS_END_ADDR) ***********/

	  switch(signature)
	  {
	  	  case backup_a5_program:
                                  for(a = 0;a < 16;a++)
                                	  data[a] =  _flash_file_param.f_magic_val0[a];
                                  for(a = 16;a < 32;a++)
                                	  data[a] =  _flash_file_param.f_file0[a - 16];
                                  data[32] =  _flash_file_param.f_size0         & 0xff;
                                  data[33] = (_flash_file_param.f_size0 >> 8)   & 0xff;
                                  data[34] = (_flash_file_param.f_size0 >> 16)  & 0xff;
                                  data[35] = (_flash_file_param.f_size0 >> 24)  & 0xff;

                                  data[36] =  _flash_file_param.f_blocks0          & 0xff;
                                  data[37] = (_flash_file_param.f_blocks0 >> 8)    & 0xff;
                                  data[38] = (_flash_file_param.f_blocks0 >> 16)   & 0xff;
                                  data[39] = (_flash_file_param.f_blocks0 >> 24)   & 0xff;

                                  data[40] =  _flash_file_param.f_crc0       & 0xff;
                                  data[41] = (_flash_file_param.f_crc0 >> 8) & 0xff;

								  address = FLASH_BACKUP_A5_SIG_ADR;
								  address_stop = address + 48;
	  		  	  	  	  	  	  break;
	  	  case runtime_a5_program:
	  		  	  	  	  	  	  for(a = 0;a < 16;a++)
	  		  	  	  	  	  		  data[a] =  _flash_file_param.f_magic_val1[a];
								  for(a = 16;a < 32;a++)
									  data[a] =  _flash_file_param.f_file1[a - 16];
								  data[32] =  _flash_file_param.f_size1         & 0xff;
								  data[33] = (_flash_file_param.f_size1 >> 8)   & 0xff;
								  data[34] = (_flash_file_param.f_size1 >> 16)  & 0xff;
								  data[35] = (_flash_file_param.f_size1 >> 24)  & 0xff;

								  data[36] =  _flash_file_param.f_blocks1          & 0xff;
								  data[37] = (_flash_file_param.f_blocks1 >> 8)    & 0xff;
								  data[38] = (_flash_file_param.f_blocks1 >> 16)   & 0xff;
								  data[39] = (_flash_file_param.f_blocks1 >> 24)   & 0xff;

								  data[40] =  _flash_file_param.f_crc1       & 0xff;
								  data[41] = (_flash_file_param.f_crc1 >> 8) & 0xff;

								  address = FLASH_RUNTIME_A5_SIG_ADR;
								  address_stop = address + 48;
	  		  		  	  	  	  break;
	  	  case runtime_a5_write:
	  		  	  	  	  	  	  for(a = 0;a < 16;a++)
	  			  		  	  	  	  data[a] =  _flash_file_param.f_magic_val2[a];

	  							  for(a = 16;a < 32;a++)
	  									data[a] =  _flash_file_param.f_file2[a - 16];

	  							  data[32] =  _flash_file_param.f_size2         & 0xff;
								  data[33] = (_flash_file_param.f_size2 >> 8)   & 0xff;
								  data[34] = (_flash_file_param.f_size2 >> 16)  & 0xff;
								  data[35] = (_flash_file_param.f_size2 >> 24)  & 0xff;

								  data[36] =  _flash_file_param.f_blocks2          & 0xff;
								  data[37] = (_flash_file_param.f_blocks2 >> 8)    & 0xff;
								  data[38] = (_flash_file_param.f_blocks2 >> 16)   & 0xff;
								  data[39] = (_flash_file_param.f_blocks2 >> 24)   & 0xff;

								  data[40] =  _flash_file_param.f_crc2       & 0xff;
								  data[41] = (_flash_file_param.f_crc2 >> 8) & 0xff;

	  		  	  	  	  	  	  address = FLASH_RUNTIME_A5_SIG_ADR;
	  		  	  	  	  	  	  address_stop = address + 16;
	  		  		  	  	  	  break;
//	  	  case A5_signature:
//	  		  	  	  	  	  	  address = FLASH_A5_SIG_SWITCH;
//	  		  		  		  	  break;
	  	  default:
	  		  	  	  	  	  	  HAL_FLASH_Lock();
	  		  	  	  	  	  	  return;
	  }

//	  for(address_init = 0;address_init < 1024;address_init++)
//		  aRxBuffer[3 + address_init] = address_init & 0xff;
      index = 0;
//	  address_init = Address;
	  while (address < address_stop)
	  {
	   // if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address, ((uint32_t)FlashWord)) == HAL_OK)
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, ((uint32_t) &data[index])) == HAL_OK)
	  //  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, ((uint32_t) &data[0])) == HAL_OK)
	    {
	      address += 16;  /* increment to the next Flash word */
	      index += 16;
	    }
	    else
	    {
	      /* Error occurred while writing data in Flash memory.
	         User can add here some code to deal with this error */
	      while (1)
	      {
	        /* Make LED_RED ON to indicate error in Write operation */
	      }
	    }
	  }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();

	  /* Re-enable instruction cache */
	  if (HAL_ICACHE_Enable() != HAL_OK)
	  {
	   // Error_Handler();
	    Error_Handler((uint8_t *)__FILE__, __LINE__);
	  }
	  HAL_Delay(2);
}


void write_A5_signature(uint8_t signature)
{
	uint32_t address = 0;
	uint32_t address_stop;
	uint8_t  a,data[20];

	 /* Disable instruction cache prior to internal cacheable memory update */
	  if (HAL_ICACHE_Disable() != HAL_OK)
	  {
		 // Error_Handler();
		  Error_Handler((uint8_t *)__FILE__, __LINE__);
	  }

	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_NS_START_ADDR and FLASH_USER_NS_END_ADDR) ***********/

	  switch(signature)
	  {
	  	  case backup_a5_program:
                                  for(a = 0;a < 16;a++)
                                	  data[a] =  backup_a5_unique[a];

								  address = FLASH_A5_SIG_SWITCH;
								  address_stop = address + 16;
	  		  	  	  	  	  	  break;
	  	  case runtime_a5_program:
	  		  	  	  	  	  	  for(a = 0;a < 16;a++)
									  data[a] =  runtime_a5_unique[a];

								  address = FLASH_A5_SIG_SWITCH;
								  address_stop = address + 16;
	  		  		  	  	  	  break;
	  	  case runtime_a5_write:
	  		  	  	  	  	  	  for(a = 0;a < 16;a++)
	  		                          data[a] = runtime_a5_write_data[a];

	  		  	  	  	  	  	  address = FLASH_A5_SIG_SWITCH;
	  		  	  	  	  	  	  address_stop = address + 16;
	  		  		  	  	  	  break;
	  	  default:
	  		  	  	  	  	  	  HAL_FLASH_Lock();
	  		  	  	  	  	  	  return;
	  }


	  while (address < address_stop)
	  {
	   // if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address, ((uint32_t)FlashWord)) == HAL_OK)
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, ((uint32_t) &data[0])) == HAL_OK)
	  //  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, ((uint32_t) &data[0])) == HAL_OK)
	    {
	      address += 16;  /* increment to the next Flash word */
	    }
	    else
	    {
	      /* Error occurred while writing data in Flash memory.
	         User can add here some code to deal with this error */
	      while (1)
	      {
	        /* Make LED_RED ON to indicate error in Write operation */
	      }
	    }
	  }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();

	  /* Re-enable instruction cache */
	  if (HAL_ICACHE_Enable() != HAL_OK)
	  {
	   // Error_Handler();
	    Error_Handler((uint8_t *)__FILE__, __LINE__);
	  }
	  HAL_Delay(2);
}


/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  return FLASH_BANK_1;
}


void Error_Handlerb(uint8_t *file, uint32_t line)
{
	char dbug[200]={0};

	sprintf(dbug,"[FATAL] file %s on line %d \r\n",
			file,(int)line);

	/* send to debug aux channel */
//	aux_sendToAux(dbug,strlen(dbug),0,1,DBG_AUX);

	/* wait for print out */
//	vTaskDelay(1000);

	/* desable interrupts */

	__disable_irq();
	while (1)
	{
	}
}

void erasesignature(uint8_t sel)
{
	uint32_t FirstPage = 0, NbOfPages = 0,BankNumber = 0;
	uint32_t PageError = 0;

	/*Variable used for Erase procedure*/
	static FLASH_EraseInitTypeDef EraseInitStruct;


	/* Disable instruction cache prior to internal cacheable memory update */
	if (HAL_ICACHE_Disable() != HAL_OK)
	{
		//Error_Handler();
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	   (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	 /* Get the only page to erase */
	switch(sel)
	{
		case backup_a5_program:
								FirstPage = GetPage(FLASH_BACKUP_A5_SIG_ADR);
								BankNumber = FLASH_BANK_2;
								break;
		case runtime_a5_program:
								FirstPage = GetPage(FLASH_RUNTIME_A5_SIG_ADR);
								BankNumber = FLASH_BANK_2;
								break;
//		case runtime_a5_write:
//								FirstPage = GetPage(FLASH_RUNTIME_A5_SIG_ADR);
//								BankNumber = GetBank(FLASH_RUNTIME_A5_SIG_ADR);
//								break;
		case A5_signature:
								FirstPage = GetPage(FLASH_A5_SIG_SWITCH);
								BankNumber = FLASH_BANK_2;
								break;
	}

	/* Get the number of pages to erase from 1st page */
	NbOfPages = 1;// GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;


    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page        = FirstPage;
    EraseInitStruct.NbPages     = NbOfPages;
    EraseInitStruct.Banks       = BankNumber;


	// Erase page  &PageError
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		  /*
			Error occurred while page erase.
			User can add here some code to deal with this error.
			PageError will contain the faulty page and then to know the code error on this page,
			user can call function 'HAL_FLASH_GetError()'
		  */
		  /* Infinite loop */
		  while (1)
		  {
			/* Turn on LED3 */
			//	if(a5_poly == 0)
			//	BSP_LED_On(LED3);
		  }
	}

    /* Unlock the Flash to enable the flash control register access *************/
	 HAL_FLASH_Lock();

	if (HAL_ICACHE_Enable() != HAL_OK)
	{
		//Error_Handler();
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}
}


void fota_handler_Task(void *para)
{
	QueueHandle_t q=(QueueHandle_t)para;
	SemaphoreHandle_t sm=NULL;
	MSG_HDR msg;
	//PACKETBUF_HDR *resp = NULL;
	MEMBUF_POOL txBufPool;
	void *p = NULL;

	/* create synchronization semaphore */
	sm=(SemaphoreHandle_t)_SEM_create(1,NULL);

	/* Create  buffers pool */
	if(NULL == (p=_MEM_alloc((A4_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_TX_BUFFERS)))
	{
		Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	initMemBufPool(&txBufPool,p,(A4_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR)+MEM_BUF_HEADER_SIZE)*N_CMD_TX_BUFFERS, A4_TX_BUFFER_SIZE+sizeof(PACKETBUF_HDR),N_CMD_TX_BUFFERS);
	for (;;)
		if (xQueueReceive(q, &msg, portMAX_DELAY))
		{
			if (msg.hdr.bit.type==MSG_TYPE_CMD)//commands handling
			{

				if (A4ValidFrame(PACKETBUF_DATA(msg.buf), ((PACKETBUF_HDR *)msg.buf)->dlen))
				{
					/* Execute command */
					handlefotaCmdPacket(&txBufPool, (PACKETBUF_HDR *) msg.buf, sm);

					/* free command buffer */
					if(retMemBuf(msg.buf))
					{
						Error_Handler((uint8_t *)__FILE__, __LINE__);
					}
				}
				else
				{
					retMemBuf(msg.buf);
				}
			}
			else
			if (msg.hdr.bit.type==MSG_TYPE_NOTIFY)
			{
				if (msg.hdr.bit.source==MSG_SRC_DSP){

				}
				else
					retMemBuf(msg.buf);
			}
			else
			{
				retMemBuf(msg.buf);
			}
		}
}

int startFotaHndlTask(QueueHandle_t *fota, TaskHandle_t *cmdT, const char * const pcName, uint16_t usStackDepth, UBaseType_t prio)
{
	QueueHandle_t q;
	TaskHandle_t t;

	/* create queue */
	if ((q = xQueueCreate(24,sizeof(MSG_HDR))) == NULL)
	{
		if (fota)
			*fota=NULL;
		if (cmdT)
			*cmdT=NULL;
		return pdFAIL;
	}

	if (pdFAIL == xTaskCreate(fota_handler_Task, pcName, usStackDepth, q, prio, &t))
	{
		vQueueDelete(q);
		if (fota)
			*fota=NULL;
		if (cmdT)
			*cmdT=NULL;
		return pdFAIL;
	}
	vQueueAddToRegistry( q, pcName);
	if (fota)
		*fota=q;
	if (cmdT)
		*cmdT=t;
	return pdPASS;
}/* end of startCmdTask */


static PACKETBUF_HDR * handlefotaCmdPacket(MEMBUF_POOL *pool, PACKETBUF_HDR * p, SemaphoreHandle_t sync_sm)
{
	PACKETBUF_HDR * b=NULL;
	uint32_t cmd_len = 0;
	char *pData = NULL;

	pData = (char*)PACKETBUF_DATA(p);
	cmd_len = p->dlen;

	if(cmd_len > 0)
	{
		pData[cmd_len]='\0';

		#ifdef DEBUG_COMMAND
		PrintLogBuffer(pData, cmd_len);
		#endif

		/* Handle commands and build output string */
		PARSER_ParseCommand((char *)pData, &b, pool);
	}
	return b;
}/* End of handleAuxCmdPacket */


void uart_transmit_ch(uint8_t data)
{
	uint8_t a[2];

	a[0] = data;

    HAL_UART_Transmit_IT(&huart5, (uint8_t *)a, 1);
}

uint32_t Transmit(uint8_t k)
{
	 uint32_t result = 0;

	 UartReadyTx = false;
	 uart_transmit_ch(k);
	 while(UartReadyTx == false) {};

	 return result;
}


/**
 * @brief   Calculates the CRC-16 for the input package.
 * @param   *data:  Array of the data which we want to calculate.
 * @param   length: Size of the data, either 128 or 1024 bytes.
 * @return  status: The calculated CRC.
 */
static uint16_t calc_crc(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0u;
    while (length)
    {
        length--;
        crc = crc ^ ((uint16_t)*data++ << 8u);
        for (uint8_t i = 0u; i < 8u; i++)
        {
            if (crc & 0x8000u)
            {
                crc = (crc << 1u) ^ 0x1021u;
            }
            else
            {
                crc = crc << 1u;
            }
        }
    }
    return crc;
}

uint32_t checkyheader(uint8_t * buffer)
{
	uint32_t result = 0;
	int16_t crc_received,crc_calculated;
	uint32_t size,checksum;


	if(buffer[0] != Y_HEADER)
	{
		//	transmit_notification(&message_err[0]);
		result = 0xff;
		return result;
	}

	if((buffer[1] != 'A') || (buffer[2] != 'p') || (buffer[3] != 'p') ||
	   (buffer[4] != 'l') || (buffer[5] != 'i') || (buffer[6] != 'c') ||
       (buffer[7] != 'a') || (buffer[8] != 't') || (buffer[9] != 'i') ||
	   (buffer[10] != 'o') || (buffer[11] != 'n'))
	{
	//	transmit_notification(&message_err[0]);
		result = 0xff;
		return result;
	}

	size = buffer[12] +
		  (buffer[13] * 0x100) +
		  (buffer[14] * 0x10000) +
		  (buffer[15] * 0x1000000);

	checksum = buffer[16] + (buffer[17] * 0x100);

//	_flash_file_param.f_crc2 = _flash_file_param.f_crc2;
//	_flash_file_param.f_blocks2 = size;


	crc_received = ((uint16_t)buffer[19] << 8u) | ((uint16_t)buffer[18]);

	/* We calculate it too. */
    crc_calculated = calc_crc(&buffer[1],17);

    if(crc_received != crc_calculated)
    {
      // transmit_notification(&crc_bad[0]);
      HAL_Delay(1000);
	  result = 0xff;
	}

	return result;
}


uint32_t checkmainchunk(uint8_t * buffer)
{
	uint32_t result = 0;
//	int16_t crc_received,crc_calculated;

	uint8_t abc[DATA_CHUNK];

	memcpy(&abc[0],&buffer[0],DATA_CHUNK);

	if(buffer[0] != X_STX)
	{
	//  Transmit(0x16);
	  result = 0xff;
	  return result;
	}

	if((buffer[1] + buffer[2]) != 0xff)
	{
	//  Transmit(0x17);
	  result = 0xff;
	  return result;
	}

	crc_received = buffer[DATA_CHUNK - 1] + (buffer[DATA_CHUNK - 2] * 0x100);

	/* We calculate it too. */
	crc_calculated = calc_crc(&buffer[3],DATA_CHUNK - 5);

	//abc[12] = 0X33;

	if(crc_received != crc_calculated)
	{
	  //	transmit_notification(&crc_bad[0]);
	//  Transmit(0x18);
	  result = 0xff;
	  return result;
	}
	 return result;
}

int sendTofotaCmdInterp(uint8_t * buffer)
//int sendTofotaCmdInterp(QueueHandle_t q, void *packet, uint16_t hdr, uint32_t timeout)
{
//	MSG_HDR msg;
	uint32_t result;
//	uint8_t abc[DATA_CHUNK];
	static uint8_t cnt = 0;
//	static uint8_t index = 0;

	if(_fota_sm.f_fota_ena == 1)
	{
	//	memcpy(&abc[0],&buffer[0],DATA_CHUNK);
	//	switch(&packet[0])
		switch(buffer[0])
		{
			case Y_HEADER:
							_fota_sm.fota = f_header;
							break;
			case X_STX:
							_fota_sm.fota = f_x_stx;
							break;
			case X_EOT:
				            _fota_sm.fota = f_x_eot;
							break;
			default:
							_fota_sm.fota = f_error;
							break;
		}

		switch(_fota_sm.fota)
		{
			case f_idle:
						 break;
			case f_fota:
						 break;
			case f_header:
						//	memcpy(&abc[0],&buffer[0],50);
							result = checkyheader(&buffer[0]);
						//	result = 0;
							if(result == 0)
							{
								// Erase top of flash.
								//flasherase(0);


								_fota_sm.fota = f_x_stx;
								Transmit(X_ACK);
								HAL_Delay(200);
							}
							else
							{
								Transmit(X_NAK);
								HAL_Delay(200);
							}
							_fota_sm.f_timer = 0;
						    break;
			case f_x_stx:
						//	memcpy(&abc[0],&buffer[0],DATA_CHUNK);

				            if(++cnt % 30 == 1)
				            {
								/* Refresh IWDG: reload counter */
								if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
								{
								  /* Refresh Error */
								  Error_Handler((uint8_t *)__FILE__, __LINE__);
								}
				            }

							result = checkmainchunk(&buffer[0]);
						//	result = 0;
							if(result == 0)
							{
								//index = buffer[0] - 1;
								// change later destination of flash addr.
								//flash_transfer(FLASH_RUNTIME_STORE,index);
								//  flash_write(index);
					         	///		flash_read(FLASH_BACKUP_START_ADDR + index * WRITE_BLOCK,&compare_block[0],WRITE_BLOCK);

								_fota_sm.fota = f_x_stx;
								Transmit(X_ACK);
								HAL_Delay(200);
							}
							else
							{
								_fota_sm.fota = f_idle;
								Transmit(X_NAK);
								 HAL_Delay(200);
							}
							_fota_sm.f_timer = 0;
						    break;
			case f_x_eot:
							Transmit(X_ACK);
							HAL_Delay(200);
							_fota_sm.f_fota_ena = 0;
							_fota_sm.fota = f_idle;

//							erasesignature(A5_signature);
//							write_A5_signature(runtime_a5_write);
//							erasesignature(runtime_a5_program);
//							write_signature(runtime_a5_program);
//							_JumpToProgram(FLASH_BACKUP_START_ADDR);

						    break;
			case f_error:
				            Transmit(X_NAK);
				            HAL_Delay(200);
//							Transmit(X_NAK);
//							_fota_sm.f_fota_ena = 0;
//							_fota_sm.fota = f_idle;
						    break;

		}

	}

//	if (q==NULL)
//		return pdFAIL;
//	msg.hdr.all = hdr;
//	msg.data=0;
//	msg.buf=packet;
//	return xQueueSend(q,&msg,timeout);
}/* End of sendToAuxCmdInterp */


