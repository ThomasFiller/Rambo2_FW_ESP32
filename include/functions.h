#include "globalconsts.h"

void SetDac(unsigned char ucDacAddress ,typUnsignedWord uiDacValue, uint8_t u8I2cChannel);
void ReadAdc();
void ReadSettings();
void Communication();
void SetLaserDriver();
//extern uint8_t ReadFromIcHtp(uint8_t u8Address, uint8_t u8Cmd, uint8_t u8Len);
extern void SetLaserMode(uint8_t u8Laser, uint8_t u8LedMode);
extern void SwitchToI2cChan(uint8_t ucChannel);
extern void SetLaserDriver300mA(unsigned char ucDriverAddress ,typUnsignedWord uiDriverValue);
//extern void InternalTrigger_Task(void * pvParameters);
extern void CycleTimer_Task(void * pvParameters);
extern void InitExternTrigger(uint8_t u8TriggerFunction);