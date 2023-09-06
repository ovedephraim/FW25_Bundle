/**
  ******************************************************************************
  * @file           : sys.h
  * @brief          : system header file
  ******************************************************************************
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MISC_H_
#define MISC_H_



void Enable_Power(void);
void Enable_Heater(void);
void Disable_Heater(void);
void InitializeHeaterState(void);
void HeaterState(void);
void InitializeChargeState(void);
void ChargeState(void);
void ReadTemperatures(void);
void SomeDelay(void);
void LedState(void);
void Pid(void);
void TestMax3000x(void);
void PiezoTest(void);
void NotifyRephael(uint8_t message);
void SetLowPassFilter(uint8_t sel);
void SCLNRG(uint8_t sel);
void CAPSENSE(uint8_t sel);
void BatterySuperviser(void);
//void Touchkey(void);
//void Touch_Sense_test(void);
//void FindValidTemperatureIC(void);
//void DACOutput(uint8_t num);


#endif  /* MISC_H */


