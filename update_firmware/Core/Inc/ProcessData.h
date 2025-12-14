#ifndef __PROCESSDATA_H
#define __PROCESSDATA_H

#ifdef __cplusplus
extern "C" {
#endif\

#include "stm32g4xx_hal.h"

#define StartMainFirmwarePage 0 //Page 1 Blank 2
#define StartAddressOfBlank2 0x08040000

#define SizeOfBuffer 4096 // 2KB page
#define MaxLineDisplay 25
#define MaxLengthLine 43 
#define MinLengthLine 11
#define NumberOfCheckSumBits 2
#define RecordTypeData '0'
#define RecordTypeEndOfFile '1'
#define RecordTypeExtendedLinearAddress '4'
#define BitOfRecordType 8
#define BitOfStartCode 0
#define StartBitOfByteCount 1
#define StartBitOfAddress 3
#define StopBitOfAddress 7
#define StartBitOfData 9
#define StartBitOfHighAddress 9


enum State {
	error = 0,
	success = 1,
	completed = 2
};

enum EndOfFile {
	notEOF = 0,
	EOF = 1
};

typedef enum ERROR_CODE
{
	NoError = 0, \
	ErrorStartBit = 1, \
	ErrorHex = 2, \
	ErrorCheckSum = 3, \
	ErrorByteCount = 4, \
	ErrorLength = 5, \
	ErrorAddress = 6, \
}ErrorCode;

typedef enum WRITE_PAGE_ERROR_CODE
{
	WritePageNoError = 0, \
	WritePageError = 1, \
	UnindentifiedError = 2
}WritePageErrorCode;

void convertCharsToHex(unsigned char *uchar_array, int uchar_array_length, uint8_t *uint8_array);
void convertHexPairs(uint8_t *input_array, uint8_t *output_array, int input_array_length);
int ProcessData(char* respond);
void deleteBuffer(char* buf);
void WriteBuffer(uint32_t size,uint32_t page);
void ClearBufferAndResetWriteVariable();

#ifdef __cplusplus
}
#endif


#endif /* __PROCESSDATA_H */