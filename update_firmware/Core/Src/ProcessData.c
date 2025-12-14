#include "ProcessData.h"
#include "FlashG4.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"

volatile uint8_t currentPage = 0;
volatile char fLineTmp[50];
unsigned char buffer[SizeOfBuffer];
volatile int currentBufferIndex = 0;
uint8_t hexArray[SizeOfBuffer];
uint8_t hexPair[SizeOfBuffer/2];
uint8_t RxLength = 0;
uint32_t pageAddress = StartAddressOfBlank2;

uint8_t ConvertToDec(uint8_t HexNumber)
{
    if (HexNumber >= '0' && HexNumber <= '9')
    {
        HexNumber = HexNumber - '0';
    }
    else if (HexNumber >= 'A' && HexNumber <= 'F')
    {
        HexNumber = HexNumber - 'A' + 10;
    }
    else
    {
        HexNumber = HexNumber - 'a' + 10;
    }
    return HexNumber;
}

bool CheckStartBit(uint8_t fLine[])
{
    bool result = false;
    if (fLine[BitOfStartCode] == ':')
    {
        result = true;
    }
    else
    {
        /* Do nothing */
    }
    return result;
}

bool CheckHex(uint8_t fLine[], uint8_t RxLength)
{
    bool result = true;
    uint32_t index = 0;
    uint8_t length = RxLength;
    for (index = 1; index < length; index++)
    {
        if (ConvertToDec(fLine[index]) < 0 || ConvertToDec(fLine[index]) > 15)
        {
            result = false;
        }
        else
        {
            /* Do nothing */
        }
    }
    return result;
}

bool CheckSum(uint8_t fLine[], uint8_t RxLength)
{
    bool result = false;
    uint8_t index = 0;
    uint8_t length = RxLength;
    uint8_t value = 0;
    for (index = 1; index < length; index += 2)
    {
        if (index + 1 < length)
        {
            char cHexPair[3] = {fLine[index], fLine[index + 1], '\0'};
            value = value + strtol(cHexPair, NULL, 16);
        }
    }
    if (value % 256 == 0)
    {
        result = true;
    }
    else
    {
        /* Do nothing */
    }
    return result;
}

bool CheckLength(uint8_t RxLength)
{
	bool result = false;
	if(RxLength < MinLengthLine || RxLength > MaxLengthLine)
	{
		result = false;
	}
	else
	{
		result = true;
	}
	return result;
}

bool CheckByteCount(uint8_t fLine[], uint8_t RxLength)
{
	bool result = false;
	uint8_t byteCountArrayACSII[2] = {fLine[StartBitOfByteCount], fLine[StartBitOfByteCount + 1]};
	uint8_t byteCountArrayHex[2];
	uint8_t byteCountValue[1];
	convertCharsToHex(byteCountArrayACSII, 2, byteCountArrayHex);
  convertHexPairs(byteCountArrayHex, byteCountValue, 1);
	if(((RxLength - MinLengthLine) / 2) == byteCountValue[0])
	{
		result = true;
	}
	else
	{
		// Do nothing
	}
	return result;
}

void CountRxLength(uint8_t fLine[]) {
	  RxLength = 0;
		while (fLine[RxLength] != '\n')
		{
			RxLength++ ;
		}
		RxLength = RxLength - 1;
}

ErrorCode CheckLineError(uint8_t fLine[])
{
    volatile ErrorCode errorCode = NoError;
	  CountRxLength(fLine);
    if (CheckStartBit(fLine) == false)
    {
        errorCode = ErrorStartBit;
    }
		else if (CheckLength(RxLength) == false)
		{
				errorCode = ErrorLength;
		}
    else if (CheckHex(fLine, RxLength) == false)
    {
        errorCode = ErrorHex;
    }
    else if (CheckSum(fLine, RxLength) == false)
    {
        errorCode = ErrorCheckSum;
    }
		else if (CheckByteCount(fLine, RxLength) == false)
		{
				errorCode = ErrorByteCount;
		}
    else
    {
        errorCode = NoError;
    }
    return errorCode;
}

void convertCharsToHex(unsigned char *uchar_array, int uchar_array_length, uint8_t *uint8_array) {
    const uint8_t char_to_hex_map[256] = {
        ['0'] = 0x00, ['1'] = 0x01, ['2'] = 0x02, ['3'] = 0x03, ['4'] = 0x04, ['5'] = 0x05, ['6'] = 0x06, ['7'] = 0x07,
        ['8'] = 0x08, ['9'] = 0x09, ['A'] = 0x0A, ['B'] = 0x0B, ['C'] = 0x0C, ['D'] = 0x0D, ['E'] = 0x0E, ['F'] = 0x0F,
        ['a'] = 0x0A, ['b'] = 0x0B, ['c'] = 0x0C, ['d'] = 0x0D, ['e'] = 0x0E, ['f'] = 0x0F,
    };

    for (int i = 0; i < uchar_array_length; ++i) {
        uint8_array[i] = char_to_hex_map[uchar_array[i]];
    }
}

void convertHexPairs(uint8_t *input_array, uint8_t *output_array, int input_array_length) {
    for (int i = 0; i < input_array_length; i += 2) {
        output_array[i / 2] = (input_array[i] << 4) | (input_array[i + 1] & 0x0F);
    }
}

void ClearBuffer()
{
	for(int i=0;i<1024;i++)buffer[i]=255;
}

uint8_t isEOF(uint8_t fLine[])
{
//	uint8_t endCommand[] = {0x3A, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x31, 0x46, 0x46, 0x0A, 0x0D}; // equal to ":00000001FF\r\n"
//	if (fLine == endCommand)
//	{
//		return EOF;
//	}
//	uint8_t indexLine = 0;
//	while(fLine[indexLine] != 0x0D)
//	{
//		if(fLine[indexLine] != endCommand[indexLine])
//		{
//			return notEOF;
//		}
//		indexLine++;
//	}
//	return EOF
	volatile uint8_t result = EOF;
	if (fLine[BitOfRecordType] != RecordTypeEndOfFile)
	{
		result = notEOF;
	}
	else
	{
		result = EOF;
	}
	return result;
}

void deleteBuffer(char* buf)
{
	int len = strlen(buf);
	for(int i = 0; i < len; i++)
	{
		buf[i] = 0;
	}
}

void WriteBuffer(uint32_t size,uint32_t pageAddress)
{
	convertCharsToHex(buffer, SizeOfBuffer, hexArray);
  convertHexPairs(hexArray, hexPair, SizeOfBuffer);
	Flash_Write_Data(pageAddress , (uint64_t *)hexPair, SizeOfBuffer/2);
}

int ProcessData(char* respond)
{
	int result = error;
	int index = 0;
	ErrorCode lineErrorCode = NoError;
	lineErrorCode = CheckLineError((uint8_t*)respond);
	volatile uint8_t flagEOF = notEOF;
	if(lineErrorCode != NoError)
	{
		ClearBuffer();
		currentBufferIndex = 0;
		deleteBuffer(respond);
		return result;
	}
	flagEOF = isEOF((uint8_t*)respond);	
	if(respond[BitOfRecordType] != RecordTypeData && respond[BitOfRecordType] != RecordTypeEndOfFile)
	{
		return success;
	}
	if(flagEOF != EOF)
	{
		for(index = StartBitOfData; index < MaxLengthLine - NumberOfCheckSumBits ; index++)
		{
			buffer[currentBufferIndex] = respond[index];
			currentBufferIndex++;
			if(currentBufferIndex == SizeOfBuffer)
			{		
				WriteBuffer(SizeOfBuffer,pageAddress);
				pageAddress = pageAddress + 0x800;
				ClearBuffer();
				currentPage++;
				currentBufferIndex = 0;				
			}
		}
	}
	else if(flagEOF == EOF)
	{
		WriteBuffer(SizeOfBuffer,pageAddress);
		ClearBuffer();
		currentBufferIndex = 0;
		currentPage = 0;
		pageAddress = StartAddressOfBlank2;
		return completed;
	}
	deleteBuffer(respond);
	result = success;
	return result;
}

void ClearBufferAndResetWriteVariable()
{
	ClearBuffer();
	currentBufferIndex = 0;
	currentPage = 0;
	pageAddress = StartAddressOfBlank2;
	
}
