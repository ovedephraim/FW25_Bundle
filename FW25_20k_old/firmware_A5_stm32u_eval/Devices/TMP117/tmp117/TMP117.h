
#ifndef _TMP117_H_
#define _TMP117_H_


#include "stdint.h"
#include "stdbool.h"

#include "TMP117_reg.h"
#include "RpcFifo.h"

#define TMP117_OK                       0
#define TMP117_ERROR                   -1

#define TMP117_WHOAMI 0x0117      ///< Correct 2-byte ID register value response

#define TMP117_I2C_ADD1               0x48<<1 // Skin temp @0x48
#define TMP117_I2C_ADD2               0x49<<1 // Control temp @0x49
#define TMP117_I2C_ADD3               0x4A<<1 // Ambient temp @0x4A

#define DEVICE_ID_VALUE   0x0117			// Value found in the device ID register on reset (page 24 Table 3 of datasheet)
#define TMP117_RESOLUTION 0.0078125f	// Resolution of the device, found on (page 1 of datasheet)
#define CONTINUOUS_CONVERSION_MODE 0b00 // Continuous Conversion Mode
#define ONE_SHOT_MODE 0b11				// One Shot Conversion Mode
#define SHUTDOWN_MODE 0b01				// Shutdown Conversion Mode

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

typedef int32_t (*TMP117_Init_Func)(void);
typedef int32_t (*TMP117_DeInit_Func)(void);
typedef int32_t (*TMP117_GetTick_Func)(void);
typedef int32_t (*TMP117_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*TMP117_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);


typedef struct
{
  TMP117_Init_Func          Init;
  TMP117_DeInit_Func        DeInit;
  uint8_t                    Address;
  TMP117_WriteReg_Func      WriteReg;
  TMP117_ReadReg_Func       ReadReg;
  TMP117_GetTick_Func       GetTick;
} TMP117_IO_t;

typedef struct
{
  TMP117_IO_t        IO;
  stmdev_ctx_t        Ctx;
  uint8_t             is_initialized;
  fifo_t             *fifo;
} TMP117_Object_t;

// Configuration register found on page 25 Figure 26 and Table 6
typedef union {
	struct
	{
		uint8_t EMPTY : 1;			// Empty bit in register
		uint8_t TMP_SOFT_RESET : 1; // Software reset bit
		uint8_t DR_ALERT : 1;		// ALERT pin select bit
		uint8_t POL : 1;			// ALERT pin polarity bit
		uint8_t T_NA : 1;			// Therm/alert mode select
		uint8_t AVG : 2;			// Conversion averaging modes
		uint8_t CONV : 3;			// Conversion cycle bit
		uint8_t MOD : 2;			// Set conversion mode
		uint8_t EEPROM_BUSY : 1;	// EEPROM busy flag
		uint8_t DATA_READY : 1;		// Data ready flag
		uint8_t LOW_ALERT : 1;		// Low Alert flag
		uint8_t HIGH_ALERT : 1;		// High Alert flag
	} CONFIGURATION_FIELDS;
	uint16_t CONFIGURATION_COMBINED;
} CONFIGURATION_REG;

// Device ID Register used for checking if the device ID is the same as declared
// This register is found on Page 30 of the datasheet in Table 15 and Figure 34
typedef union {
	struct
	{
		uint16_t DID : 12; // Indicates the device ID
		uint8_t REV : 4;   // Indicates the revision number
	} DEVICE_ID_FIELDS;
	uint16_t DEVICE_ID_COMBINED;
} DEVICE_ID_REG;


int32_t TMP117_RegisterBusIO(TMP117_Object_t *pObj, TMP117_IO_t *pIO);

int
TMP117_readTemp(TMP117_Object_t *pobj, int16_t * pval);
int
TMP117_readId(TMP117_Object_t *pobj, int16_t * pval);

int
TMP117_readTempC(TMP117_Object_t *pobj, float * pval);

#if 0
void TMP117_softReset();													// Performs a software reset on the Configuration Register Field bits
float TMP117_getTemperatureOffset();										// Reads the temperature offset
void TMP117_setTemperatureOffset(float offset);							// Writes to the temperature offset
float TMP117_getLowLimit();												// Returns the low limit register
void TMP117_setLowLimit(float lowLimit);									// Sets the low limit temperature for the low limit register
float TMP117_getHighLimit();												// Returns the high limit register
void TMP117_setHighLimit(float highLimit);									// Sets the low limit temperature for the low limit register
uint16_t TMP117_getConfigurationRegister();								// Get Configuration Register
uint8_t TMP117_getHighLowAlert();											// Reads in Alert mode for high and low alert flags
bool TMP117_getHighAlert();												// Reads in Alert mode for a high alert flag
bool TMP117_getLowAlert();													// Reads in Alert mode for a low alert flag
void TMP117_setAlertFunctionMode(uint8_t setAlertMode);					// Set alert or therm mode
uint8_t TMP117_getAlertFunctionMode();										// Check to see if in alert or therm mode
uint8_t TMP117_getConversionMode();										// Checks to see the Conversion Mode the device is currently in
void TMP117_setContinuousConversionMode();									// Sets the Conversion Mode of the Device to be Continuous
void TMP117_setOneShotMode();												// Sets the Conversion Mode of the Device to be One Shot
void TMP117_setShutdownMode();												// Sets the Conversion Mode of the Device to be Shutdown
void TMP117_setConversionAverageMode(uint8_t convMode);					// Sets the conversion averaging mode
uint8_t TMP117_getConversionAverageMode();									// Returns the Conversion Averaging Mode
void TMP117_setConversionCycleBit(uint8_t convTime);						// Sets the conversion cycle time bit
uint8_t TMP117_getConversionCycleBit();									// Returns the conversion cycle time bit value
bool TMP117_dataReady();													// Checks to see if there is data ready from the device
#endif

#endif
