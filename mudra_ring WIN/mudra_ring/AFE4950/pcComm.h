/*
 * pcComm.h
 *
 *  Created on: Oct 10, 2019
 *      Author: a0230206
 */

#ifndef PCCOMM_H_
#define PCCOMM_H_

#include "driverlib.h"
#include "USB_config/descriptors.h"
#include "USB_API/USB_HID_API/UsbHid.h"
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"
#include "EVM_Hardware.h"

enum pcCommand
{
    UNKNOWN,
    CDC_QUERY,
    RESET_AFE,
    RESET_UC,
    REG_WRITE,
    REG_READ,
    START_CAPTURE,
    STOP_CAPTURE,
    GET_REG_VALS,
    COMM_SELECT,
    EXTRA_REG_CAPTURE,
    FIRMWARE_VER,
    DEV_QUERY,
    BSL_ENTRY,
    WRITE_INFO,
    READ_INFO
};

struct usbDataStruct
{
    uint8_t RX_BUFF[512];
    uint16_t NUM_BYTES_REC;
    uint8_t TX_BUFF[2048];
    bool DataReceieved;
};

enum _captureStatus
{
    datasendComplete,
    dataReadyTosend
};

struct captureDetails
{
    uint16_t numBytesToSend;
    enum _captureStatus captureStatus;
};

struct _usbStruct
{
    uint8_t  afeRegAddress;
    uint32_t afeRegData;
    uint32_t numFiFoRdy;
    uint16_t txBuffPointer;
    bool finiteCapture;
    struct captureDetails captureDetail;
    struct usbDataStruct usbData[2];
    enum pcCommand receievedCommand;
};

struct _usbStruct usbStruct;

enum _extraRegCaptureStatus
{
    enabled,
    disabled
};

struct _extraRegCapture
{
    uint8_t startStop;
    uint8_t afeRegAddress[8];
    uint32_t afeRegData[8];
    uint8_t numRegsToCap;
    enum _extraRegCaptureStatus extraRegCaptureStatus;
};
struct _extraRegCapture extraRegCapture;

struct _nibbleSplit
{
    uint8_t N0:4;
    uint8_t N1:4;
};

union _splitToNibbles
{
    struct _nibbleSplit splittedData[4];
    uint32_t DataToSplit;
};

void hex2Ascii(char* result, uint32_t hexVal, uint8_t numBytes);
void ascii2Hex(char* result, char *str, uint8_t numBytes);
void processPcCommands(void);
void decodeCommand(uint8_t *commandBuff);
uint8_t compareStr(uint8_t *str1, uint8_t *str2, uint8_t len);
void copy(uint8_t *source,uint8_t *des,uint8_t numBytes);
void copyRegDataInTxBuf1(uint8_t dataIdentifier);
void capture(void);
void gotoBslLocation(void);

#endif /* PCCOMM_H_ */
