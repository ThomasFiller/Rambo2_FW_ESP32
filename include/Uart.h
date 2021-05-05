
extern void UartReceiveOrTransmit(void);
void ReplyData(unsigned char ucCmd);//DataDirection == 1
void SetParameter(unsigned char ucCmd, unsigned char *pucData, unsigned char ucLenOfFrame); //DataDirection==0
void UartSendErrorMessage(unsigned char ucDataDirection, unsigned char ucErrorCode);
void UartTransmitString(unsigned char *pucSendString);
void WriteCalibrationData();
void ReadCalibrationData();