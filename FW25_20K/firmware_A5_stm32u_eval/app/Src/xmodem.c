/**
 * @file    xmodem.c
 * @author  Ferenc Nemeth
 * @date    21 Dec 2018
 * @brief   This module is the implementation of the Xmodem protocol.
 *
 *          Copyright (c) 2018 Ferenc Nemeth - https://github.com/ferenc-nemeth
 */

#include "main.h"
#include "xmodem.h"

extern UART_HandleTypeDef huart5;


#define uint8_t unsigned char
#define uint16_t unsigned short
//#define uint32_t unsigned int

#define UART_OK		0
#define UART_ERROR	0xff
#define UART_TIMEOUT		100
#define WRITE_BLOCK			1024

uint8_t UartReadyTx  = RESET;
uint8_t UartReadyRx  = RESET;

extern uint8_t response_ok[];
extern uint8_t response_bad[];
extern uint8_t y_modem[];

extern IWDG_HandleTypeDef hiwdg;

/* Buffer used for transmission */
//uint8_t aTxBuffer[60] = {" "};
uint8_t compare_block[1040];

/* Buffer used for reception */
//uint8_t aRxBuffer[1040],compare_block[1040];
//uint8_t UART_CH2 = 0;
//uint8_t a5_poly = 1;

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
uint32_t timeout = 0;

flash_file_param _flash_file_param;

/* Global variables. */
//static uint8_t xmodem_packet_number = 1u;         /**< Packet number counter. */
//static uint32_t xmodem_actual_flash_address = 0u; /**< Address where we have to write. */
//static uint8_t x_first_packet_received = false;   /**< First packet or not. */


static uint16_t xmodem_calc_crc(uint8_t *data, uint16_t length);
static uint16_t xmodem_calc_crc_partial(uint8_t *data, uint16_t length,uint32_t crc_init);
//static xmodem_status xmodem_handle_packet(uint8_t size);

extern uint8_t aDbgRxBuffer[],compare_block[];  // aRxBuffer[],
extern uint8_t timeout_note[],crc_bad[],message_err[];
extern uint8_t end_of_tx[];



extern uint32_t xmodem_on;

uint16_t cs = 0;



void flash_read (uint32_t StartPageAddress, uint8_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{
		*RxBuf = *(__IO uint32_t *)StartPageAddress;
		StartPageAddress += 1;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
}

void flash_transfer(uint32_t origin,uint32_t numerator)
{
	uint32_t address = 0;
	uint32_t index = 0;
	uint32_t address_init;


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
	  address = origin + (numerator * WRITE_BLOCK);

//	  for(address_init = 0;address_init < 1024;address_init++)
//		  aRxBuffer[3 + address_init] = address_init & 0xff;

	  address_init = address;
	  while (address < (address_init + WRITE_BLOCK)) //FLASH_USER_NS_END_ADDR)
	  {
	   // if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address, ((uint32_t)FlashWord)) == HAL_OK)
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, ((uint32_t) &aDbgRxBuffer[index])) == HAL_OK)
	    {
	      address = address + 16;  /* increment to the next Flash word */
	      index   = index + 16;
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

void flash_write(uint32_t numerator)
{
	uint32_t Address = 0;
	uint32_t Index = 0;
	uint32_t address_init;


	 /* Disable instruction cache prior to internal cacheable memory update */
	  if (HAL_ICACHE_Disable() != HAL_OK)
	  {
		//  Error_Handler();
		  Error_Handler((uint8_t *)__FILE__, __LINE__);
	  }

	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_NS_START_ADDR and FLASH_USER_NS_END_ADDR) ***********/
	  Address = FLASH_BACKUP_START_ADDR + (numerator * 1024);

//	  for(address_init = 0;address_init < 1024;address_init++)
//		  aRxBuffer[3 + address_init] = address_init & 0xff;

	  address_init = Address;
	  while (Address < (address_init + 1024)) //FLASH_USER_NS_END_ADDR)
	  {
	   // if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address, ((uint32_t)FlashWord)) == HAL_OK)
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address, ((uint32_t) &aDbgRxBuffer[3 + Index])) == HAL_OK)
	    {
	      Address = Address + 16;  /* increment to the next Flash word */
	      Index   = Index + 16;
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


void uart_transmit_ch(uint8_t data)
{
    //aTxBuffer[0] = data;
	uint8_t a[2];

	a[0] = data;

 //   HAL_UART_Transmit_IT(&huart5, (uint8_t *)aTxBuffer, 1);
    HAL_UART_Transmit_IT(&huart5, (uint8_t *)a, 1);

}

uint32_t Transmit(uint8_t k)
{
	 uint32_t result = X_OK;

	 UartReadyTx = false;
	 uart_transmit_ch(k);
	 while(UartReadyTx == false) {};

	 return result;
}

uint32_t Transmit_result(uint8_t k)
{
	 uint32_t result = X_OK;

	 UartReadyTx = false;
	 uart_transmit_ch(k);
	 while(UartReadyTx == false) {};

	 return result;
}

void transmit_notification(uint8_t * command)
{
	uint8_t a,b;
    b = strlen(command);

	for(a = 0;a < b ;a++)
		Transmit_result(command[a]);
}

void trasmit_answer(uint8_t * command,uint32_t index)
{
	uint8_t a,b;
    b = strlen(command);

    command[22] = (index % 10) + 0x30;
    command[21] = ((index / 10) % 10) + 0x30;
    command[20] = ((index / 100) % 10) + 0x30;
    command[19] = ((index / 1000) % 10) + 0x30;

	for(a = 0;a < b ;a++)
		Transmit_result(command[a]);
}


void flasherasesection(uint32_t start,uint32_t stop)
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

	/* Get the 1st page to erase */
	FirstPage = GetPage(start);

	/* Get the number of pages to erase from 1st page */
    NbOfPages = GetPage(stop) - FirstPage + 1;

	/* Get the bank */
    if(start < FLASH_RUNTIME_START_ADDR)
    	BankNumber = FLASH_BANK_1;
    else
	    BankNumber = FLASH_BANK_2;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;
	EraseInitStruct.Banks       = BankNumber;


	// Erase First block  &PageError
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




void flasherase(uint8_t full)
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

	if(full == 1)
	{
		  /* Erase the user Flash area
		  (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

		  /* Get the 1st page to erase */
		  FirstPage = GetPage(FLASH_BACKUP_START_ADDR);

		  /* Get the number of pages to erase from 1st page */
		  NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;

		  /* Get the bank */
		  BankNumber = GetBank(FLASH_BACKUP_START_ADDR);

		  /* Fill EraseInit structure*/
		  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		  EraseInitStruct.Page        = FirstPage;
		  EraseInitStruct.NbPages     = NbOfPages;
		  EraseInitStruct.Banks       = BankNumber;


		  // Erase First block  &PageError
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
	}


	  /* Get the bank */
	  BankNumber = FLASH_BANK_2;//GetBank(FLASH_RUNTIME_START_ADDR);

	  /* Get the 1st page to erase */
	  FirstPage = GetPage(FLASH_RUNTIME_START_ADDR);

	  /* Get the number of pages to erase from 1st page */
	  NbOfPages = GetPage(FLASH_RUNTIME_A5_END_ADR) - FirstPage + 1;


	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.Page        = FirstPage;
	  EraseInitStruct.NbPages     = NbOfPages;
	  EraseInitStruct.Banks       = BankNumber;

	  // Erase Second block   &PageError
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
//			if(a5_poly == 0)
//			  BSP_LED_On(LED3);
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



uint32_t checkstring(void)
{
	uint32_t result = X_OK;
	int16_t crc_received,crc_calculated;

	    //uint8_t abc[1029];

	    //memcpy(&abc[0],&aDbgRxBuffer[0],1029);

		if(aDbgRxBuffer[0] != X_STX)
		{
		  result = X_ERROR;
		  return result;
		}

		if((aDbgRxBuffer[1] + aDbgRxBuffer[2]) != 0xff)
		{
		  result = X_ERROR;
		  return result;
		}

		crc_received = aDbgRxBuffer[X_PACKET_CRC_LOW_INDEX] + (aDbgRxBuffer[X_PACKET_CRC_HIGH_INDEX] * 0x100);

		/* We calculate it too. */
	    crc_calculated = xmodem_calc_crc(&aDbgRxBuffer[X_PACKET_START_DATA],1024);

	    //abc[12] = 0X33;

	    if(crc_received != crc_calculated)
	    {
	      //	transmit_notification(&crc_bad[0]);
		  result = X_ERROR;
		  return result;
		}
	 return result;
}


uint16_t checksum_adder(void)
{
	uint16_t a;

	for(a = 0;a < 1024;a++)
	{
		cs += compare_block[a];
	}

	return cs;
}


uint32_t compare_array(void)
{
	uint32_t result = 0;
	uint16_t a;

	for(a = 0;a < 1024;a++)
	{
		if(aDbgRxBuffer[a + 3] != compare_block[a])
		{
			result = 1;
			a = 1026;
		}
	}
	return result;
}

void LongDelay(void)
{
	volatile uint32_t  a,b,c = 0;

	for(a = 0;a < 100;a++)
		for(b = 0;b < 100;b++)
		{
           c++;
		}
}


uint32_t read_flash_check_crc(uint32_t start , uint16_t blocks)
{
//	uint32_t result = 0;
	uint16_t size;
	uint32_t crc_result = 0,crc_temp = 0;

	uint32_t address;


	for(size = 0;size < blocks;size++)
	{
		address = FLASH_BACKUP_START_ADDR + size * WRITE_BLOCK;
		flash_read(address,&compare_block[0],WRITE_BLOCK);
		crc_result = xmodem_calc_crc_partial(&compare_block[0],WRITE_BLOCK,crc_temp);
		crc_temp = crc_result;
	}

	return crc_temp;
}



uint32_t get_A5_signature(void)
{
	uint32_t result = X_ERROR;
	uint8_t RxBuffer[30];
	uint16_t a;

	// Read runtime_a5_presence signature.
	flash_read(FLASH_A5_SIG_SWITCH,&RxBuffer[0],30);

	for(a = 0;a < 16;a++)
	{
		if(RxBuffer[a] == runtime_a5_write_data[a])
		{
		    // Do something later
			result = 1;
			return result;
		}
	}

	for(a = 0;a < 16;a++)
	{
		if(RxBuffer[a] == backup_a5_unique[a])
		{
			// Do something later
			result = 2;
		    return result;
		}
	}

	// First check unique code.
	for(a = 0;a < 16;a++)
	{
		if(RxBuffer[a] == runtime_a5_unique[a])
		{
			// Do something later
			result = 3;
			return result;
		}
	}


	return result;
}


uint32_t a5_program_presence(uint8_t src)
{
	uint32_t result = X_OK;
	uint8_t RxBuffer[1024];
	uint16_t a,b;
	uint32_t runtime_adr = 0;
	uint32_t length = 0;
	uint16_t chunk;//,remainder;
	uint32_t cs = 0;
	uint32_t cs_tmp = 0;

	// Read runtime_a5_presence validation bytes.
	switch(src)
	{
	    case backup_a5_program:
	    						flash_read(FLASH_BACKUP_A5_SIG_ADR,&RxBuffer[0],50);

	    						// Check unique code.
								for(a = 0;a < 16;a++)
								{
									if(RxBuffer[a] != backup_a5_unique[a])
									{
									   result = X_ERROR;
									   return result;
									}
								}
	                            break;
	    case runtime_a5_program:
	    						flash_read(FLASH_RUNTIME_A5_SIG_ADR,&RxBuffer[0],50);

	    						// Check unique code.
								for(a = 0;a < 16;a++)
								{
									if(RxBuffer[a] != runtime_a5_unique[a])
									{
									   result = X_ERROR;
									   return result;
									}
								}
	   	                        break;
	}


	// Read size a5 program.
	runtime_adr  = RxBuffer[16] +
				  (RxBuffer[17]  * 0x100) +
				  (RxBuffer[18] * 0x10000) +
				  (RxBuffer[19] * 0x1000000);

    // Read length of runtime_a5 program.
	length     =  RxBuffer[20] +
				  (RxBuffer[21] * 0x100) +
				  (RxBuffer[22] * 0x10000) +
				  (RxBuffer[23] * 0x1000000);

    // Read length of runtime_a5 program.
	cs          = RxBuffer[24] + RxBuffer[25] * 0x100;

	// Compare program data agains cs.
	chunk = length / 1024;
	if(chunk != 0)
		chunk +=1;
	//remainder = length - stub * 1024;

	for(a = 0;a < chunk;a++)
	{
	   flash_read(runtime_adr,&RxBuffer[0],1024);
	   for(b = 0;b < 1024;b++)
		   cs_tmp += RxBuffer[b];
	   runtime_adr += a * 1024;
	}


//	// add remainder
//	if(remainder != 0)
//	{
//	   flash_read(runtime_adr,&RxBuffer[0],remainder);
//	   for(b = 0;b < remainder;b++)
//		   cs_tmp += RxBuffer[b];
//	}

	if(cs_tmp != cs)
		result = X_ERROR;

	return result;
}

//FLASH_BACKUP_A5_SIG_ADR   0x081e0000
//FLASH_RUNTIMEP_A5_SIG_ADR
uint32_t checkyheader(void)
{
	uint32_t result = X_OK;
	int16_t crc_received,crc_calculated;
	uint32_t size,checksum;

	if((aDbgRxBuffer[1] != 'A') || (aDbgRxBuffer[2] != 'p') || (aDbgRxBuffer[3] != 'p') ||
	   (aDbgRxBuffer[4] != 'l') || (aDbgRxBuffer[5] != 'i') || (aDbgRxBuffer[6] != 'c') ||
       (aDbgRxBuffer[7] != 'a') || (aDbgRxBuffer[8] != 't') || (aDbgRxBuffer[9] != 'i') ||
	   (aDbgRxBuffer[10] != 'o') || (aDbgRxBuffer[11] != 'n'))
	{
	//	transmit_notification(&message_err[0]);
		result = X_ERROR;
		return result;
	}

	size = aDbgRxBuffer[12] +
		  (aDbgRxBuffer[13] * 0x100) +
		  (aDbgRxBuffer[14] * 0x10000) +
		  (aDbgRxBuffer[15] * 0x1000000);

	checksum = aDbgRxBuffer[16] + (aDbgRxBuffer[17] * 0x100);

	_flash_file_param.f_crc2 = _flash_file_param.f_crc2;
	_flash_file_param.f_blocks2 = size;


	crc_received = ((uint16_t)aDbgRxBuffer[18] << 8u) | ((uint16_t)aDbgRxBuffer[19]);

	/* We calculate it too. */
    crc_calculated = xmodem_calc_crc(&aDbgRxBuffer[0],20);
#if 0
    if(crc_received != crc_calculated)
    {
      transmit_notification(&crc_bad[0]);
      HAL_Delay(1000);
    //  Transmit(X_NAK);
    //  HAL_Delay(1000);
	  result = X_ERROR;
	 // return result;
	}
#endif
	return result;
}


void ack_mimic(void)
{
	  uint32_t timeout = 0;;

	  uart_receive(&aDbgRxBuffer[0], 1029);
	  while(1)
	  {
		if(UartReadyRx == true)
		{
			Transmit(X_ACK);
			UartReadyRx = false;
			timeout = 0;
		}
		else
		{
			if(++timeout > 200000)
			{
				UartReadyRx = true;
				timeout = 0;

				/* Refresh IWDG: reload counter */
				if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
				{
				  /* Refresh Error */
				  Error_Handler((uint8_t *)__FILE__, __LINE__);
				}
			}
		}
	  }
}


/**
 * @brief   This function is the base of the Xmodem protocol.
 *          When we receive a header from UART, it decides what action it shall take.
 * @param   void
 * @return  void
 */

uint32_t get_ymodem(void)
{
  uint32_t result = X_ERROR;
  uint8_t header;
  uint8_t y_first_header_received = false;
  uint32_t end_of_handshake = false;
  uint32_t index = 0;
//  uint32_t timeout = 0;


  // trasmit_answer(&y_modem[0],0);
  // For FOTA command.
  Transmit(X_ACK);

  	/* Loop until there isn't any error (or until we jump to the user application). */
  UartReadyRx = false;

  while(y_first_header_received == false)
  {
	  if(aDbgRxBuffer[0] == Y_HEADER)
	  {
		 LongDelay();
		// Check validity of package.
		result = checkyheader();
		result = X_OK;
		if(result == X_OK)
		{
			y_first_header_received = true;
			xmodem_on = true;
		}
		else
		{
			HAL_Delay(100);
			Transmit(X_NAK);
		}
	  }
  }
  //	  ack_mimic();

  // Erase top of flash.
  flasherase(0);

  aDbgRxBuffer[0] = 0x11;
  // Ack for Y_HEADER.
  Transmit(X_ACK);
  uart_receive(&aDbgRxBuffer[0], 1029);
  UartReadyRx = false;
  //  trasmit_answer(&response_ok[0],0);

  /* The header can be: SOH, STX, EOT and CAN. */
  while(end_of_handshake == false)
  {
	timeout = 0;
	while(UartReadyRx == false)
	{
		//HAL_Delay(20);
		//	 if(++timeout > 200000)
		//	 {
		//		 timeout = 0;
		//	 }
	}

	result = checkstring();
	if(result == X_OK)
	{
		header = aDbgRxBuffer[0];
	}

	/* Refresh IWDG: reload counter */
	if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
	{
	  /* Refresh Error */
	  Error_Handler((uint8_t *)__FILE__, __LINE__);
	}

	switch(header)
	{
		  /* 128 or 1024 bytes of data. */
		  /* we use only 1K package */
		  case X_SOH:
					  Transmit(X_NAK);
					  break;
		  case X_STX:
				    // change later destination of flash addr.
			        flash_transfer(FLASH_RUNTIME_STORE,index);
					//  flash_write(index);
		    ///		flash_read(FLASH_BACKUP_START_ADDR + index * WRITE_BLOCK,&compare_block[0],WRITE_BLOCK);
					//  Calculate CS.
					//  cs += checksum_adder();
					  index++;
					//  result = compare_array();
					//	if(result != X_OK)
					//  {
					//		 trasmit_answer(&response_bad[0],index - 1);
					//	     eturn X_ERROR;
					//	}
					//	else
					//	{
					//		trasmit_answer(&response_ok[0],index - 1);
					//	}

					  // Force new RX data.
					  aDbgRxBuffer[0] = 0x11;
					  UartReadyRx = false;
					  Transmit(X_ACK);
					  break;
		  case X_EOT:
					//  Transmit(X_ACK);


					//  transmit_notification(&end_of_tx[0]);

					//  for(a = 0;a < 16;a++)
					//	  _flash_file_param.f_magic_val2[a] = runtime_a5_write_data[a];
					//  memcpy(&_flash_file_param.f_file2[0],"Application #2",strlen("Application #2"));
					//  _flash_file_param.f_size2  = WRITE_BLOCK * (index - 1);
					//  _flash_file_param.f_blocks2  = index - 1;
					//  _flash_file_param.f_crc2  = cs;
					//  _flash_file_param.f_crc2 = _flash_file_param.f_crc2;
					//  _flash_file_param.f_blocks2 = size;
					  if(_flash_file_param.f_blocks2 != (index - 1))
					  {
						  result = X_ERROR;
						  return result;
					  }

					  if(_flash_file_param.f_crc2 != cs)
					  {
						  result = X_ERROR;
						  return result;
					  }

					  erasesignature(A5_signature);
					  write_A5_signature(runtime_a5_write);
					  erasesignature(runtime_a5_program);
					  write_signature(runtime_a5_program);
					  _JumpToProgram(FLASH_BACKUP_START_ADDR);
					//  write_A5_signature(runtime_a5_program);
					//  update_signature_switch(runtime_a5_write);

					  result = X_OK;
					  end_of_handshake = true;
					  break;
		  case X_CAN:
					  break;
		  default:
					  break;
	 }
}
  return result;
}


/**
 * @brief   Calculates the CRC-16 for the input package.
 * @param   *data:  Array of the data which we want to calculate.
 * @param   length: Size of the data, either 128 or 1024 bytes.
 * @return  status: The calculated CRC.
 */
static uint16_t xmodem_calc_crc(uint8_t *data, uint16_t length)
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

/**
 * @brief   Calculates the CRC-16 for the input package.
 * @param   *data:  Array of the data which we want to calculate.
 * @param   length: Size of the data, either 128 or 1024 bytes.
 * @return  status: The calculated CRC.
 */
static uint16_t xmodem_calc_crc_partial(uint8_t *data, uint16_t length,uint32_t crc_init)
{
    uint16_t crc = crc_init;
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


