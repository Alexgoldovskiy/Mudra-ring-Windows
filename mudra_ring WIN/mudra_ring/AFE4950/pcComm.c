/*
 * pcComm.c
 *
 *  Created on: Oct 10, 2019
 *      Author: a0230206
 */


#include "pcComm.h"

extern enum CAP_STATs volatile CAP_STAT;
extern enum SELECTED_COMMs SELECTED_COMM;
extern enum SPI_WRITE_MODES     SPI_WRITE_MODE;
extern enum SPI_READ_MODES      SPI_READ_MODE;

union _splitToNibbles splitToNibbles;

void hex2Ascii(char* result, uint32_t hexVal, uint8_t numBytes)
{
    static uint8_t strArr[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    uint8_t numNibble=numBytes*2;
    uint8_t counter=0;
    splitToNibbles.DataToSplit=hexVal;
    while(numBytes!=0)
    {
        *(result+numNibble-1) = strArr[splitToNibbles.splittedData[counter].N0];
        *(result+numNibble-2) = strArr[splitToNibbles.splittedData[counter].N1];
        numBytes--;
        numNibble-=2;
        counter++;
    }
}

void ascii2Hex(char* result, char *str, uint8_t numChars)
{
    volatile uint8_t counter=0;
    volatile uint8_t charToSub=0;
    volatile uint8_t numBytes=numChars>>1;

    union _resultedData
    {
        volatile uint32_t result32;
        volatile char resultArr[4];
    }resultedData;

    resultedData.result32=0;
    for(counter=0;counter<numChars;counter++)
    {
        charToSub='0';
        if(*str>0x39)
            charToSub='7';

        resultedData.result32=resultedData.result32<<4;
        resultedData.result32 += (*str)-charToSub;
        str++;
    }

    for(counter=0;counter<numBytes;counter++)
    {
        *(result+counter)=resultedData.resultArr[counter];
    }
}

void copy(uint8_t *source,uint8_t *des,uint8_t numBytes)
{
    volatile uint8_t bytesToCopy=numBytes;

    while(bytesToCopy!=0)
    {
        *(des+bytesToCopy-1) =  *(source+bytesToCopy-1);
        bytesToCopy--;
    }
}

void processPcCommands(void)
{
    uint16_t regCounter=0;

    usbStruct.receievedCommand=UNKNOWN;
    if(usbStruct.usbData[0].DataReceieved)
    {
        usbStruct.usbData[0].DataReceieved=false;
        decodeCommand(usbStruct.usbData[0].RX_BUFF);

        switch(usbStruct.receievedCommand)
        {
            case CDC_QUERY:     USBCDC_sendDataAndWaitTillDone((uint8_t*)"REG_EDIT\n",9,CDC0_INTFNUM,0);
                 break;

            case RESET_AFE:     resetAFE();
                                USBCDC_sendDataAndWaitTillDone((uint8_t*)"SUCCESS\n",8,CDC0_INTFNUM,0);
                 break;

            case RESET_UC:      USBCDC_sendDataAndWaitTillDone((uint8_t*)"SUCCESS\n",8,CDC0_INTFNUM,0);
                                __disable_interrupt(); // disable interrupts
                                PMMCTL0 = PMMPW + PMMSWBOR + (PMMCTL0 & 0x0003);
                 break;


            case COMM_SELECT:
                                switch(usbStruct.usbData[0].RX_BUFF[11])
                                {
                                case 'I':   SELECTED_COMM=I2C;
                                            USBCDC_sendDataAndWaitTillDone((uint8_t*)"SUCCESS\n",8,CDC0_INTFNUM,0);
                                    break;
                                case 'S':   SELECTED_COMM=SPI;
                                            USBCDC_sendDataAndWaitTillDone((uint8_t*)"SUCCESS\n",8,CDC0_INTFNUM,0);
                                    break;
                                default:
                                            USBCDC_sendDataAndWaitTillDone((uint8_t*)"FAILED. UNKNOWN COMMUNICATION INTERFACE CHOSEN\n",47,CDC0_INTFNUM,0);
                                    break;
                                }
                                SELECT_COMM();

                 break;


            case REG_WRITE:     ascii2Hex((char *)&usbStruct.afeRegAddress,(char *)&usbStruct.usbData[0].RX_BUFF[7],2);
                                ascii2Hex((char *)&usbStruct.afeRegData,(char *)&usbStruct.usbData[0].RX_BUFF[9],6);

                                if(usbStruct.usbData[0].RX_BUFF[15]=='1')
                                    SPI_WRITE_MODE = WRITE_AFE1;

                                writeToAFE(usbStruct.afeRegAddress,usbStruct.afeRegData);
                                USBCDC_sendDataAndWaitTillDone((uint8_t*)"SUCCESS\n",8,CDC0_INTFNUM,0);

                 break;
            case REG_READ:      ascii2Hex((char *)&usbStruct.afeRegAddress,(char *)&(usbStruct.usbData[0].RX_BUFF[7]),2);

                                if(usbStruct.usbData[0].RX_BUFF[9]=='1')
                                    SPI_READ_MODE = READ_AFE1;

                                readFromAFE(usbStruct.afeRegAddress,(uint8_t *)&usbStruct.afeRegData);
                                copy((uint8_t*)"SUCCESS\n",&usbStruct.usbData[0].TX_BUFF[0],8);
                                hex2Ascii((char*)&usbStruct.usbData[0].TX_BUFF[8],usbStruct.afeRegData,3);
                                usbStruct.usbData[0].TX_BUFF[14]='\n';
                                USBCDC_sendDataAndWaitTillDone(&usbStruct.usbData[0].TX_BUFF[0],15,CDC0_INTFNUM,0);
                 break;

            case GET_REG_VALS:
                break;

            case FIRMWARE_VER: USBCDC_sendDataAndWaitTillDone((uint8_t*)"V3.0.0\n",7,CDC0_INTFNUM,0);
                break;

            case DEV_QUERY:USBCDC_sendDataAndWaitTillDone((uint8_t*)"AFE4950\n",8,CDC0_INTFNUM,0);
                break;

            case BSL_ENTRY:     USBCDC_sendDataAndWaitTillDone((uint8_t*)"SUCCESS\n",8,CDC0_INTFNUM,0);
                                gotoBslLocation();
                break;

            case WRITE_INFO:    writeInformationData((uint8_t *)&usbStruct.usbData[0].RX_BUFF[8]);
                                USBCDC_sendDataAndWaitTillDone((uint8_t*)"SUCCESS\n",8,CDC0_INTFNUM,0);
                break;

            case READ_INFO:     ascii2Hex((char*)&usbStruct.usbData[0].NUM_BYTES_REC,(char *)&usbStruct.usbData[0].RX_BUFF[8], 4); //One byte. Number of data to be read
                                readInformationData((uint8_t*)&usbStruct.usbData[0].TX_BUFF[0],usbStruct.usbData[0].NUM_BYTES_REC);
                                USBCDC_sendDataAndWaitTillDone((uint8_t*)&usbStruct.usbData[0].TX_BUFF[0],usbStruct.usbData[0].NUM_BYTES_REC*2,CDC0_INTFNUM,0);
                break;

            default:            USBCDC_sendDataAndWaitTillDone((uint8_t*)"FAILED. UNKNOWN COMMAND\n",24,CDC0_INTFNUM,0);
                break;
        }
    }


    if(usbStruct.usbData[1].DataReceieved)
    {
        usbStruct.usbData[1].DataReceieved=false;
        decodeCommand(usbStruct.usbData[1].RX_BUFF);

        switch(usbStruct.receievedCommand)
        {
            case CDC_QUERY:USBCDC_sendDataAndWaitTillDone((uint8_t*)"CAPTURE\n",8,CDC1_INTFNUM,0);
                break;

            case START_CAPTURE: ascii2Hex((char *)&usbStruct.finiteCapture,(char *)&usbStruct.usbData[1].RX_BUFF[11],2);
                                ascii2Hex((char *)&usbStruct.numFiFoRdy,(char *)&usbStruct.usbData[1].RX_BUFF[13],8);
                                usbStruct.captureDetail.captureStatus=datasendComplete;
                                enableCapture();

                break;

            case STOP_CAPTURE:  disableCapture();
                break;

            case EXTRA_REG_CAPTURE: ascii2Hex((char *)&extraRegCapture.startStop,(char *)&usbStruct.usbData[1].RX_BUFF[10],2);
                if(extraRegCapture.startStop==1)
                {
                    ascii2Hex((char *)&extraRegCapture.numRegsToCap,(char *)&usbStruct.usbData[1].RX_BUFF[12],2);
                    for(regCounter=0;regCounter<extraRegCapture.numRegsToCap;regCounter++)
                    {
                        ascii2Hex((char *)&extraRegCapture.afeRegAddress[regCounter],(char *)&usbStruct.usbData[1].RX_BUFF[14+(2*regCounter)],2);
                    }
                    extraRegCapture.extraRegCaptureStatus=enabled;
                }
                else
                {
                    extraRegCapture.extraRegCaptureStatus=disabled;
                }
                break;
        }
    }

    if(usbStruct.captureDetail.captureStatus==dataReadyTosend)
    {
        USBCDC_sendDataAndWaitTillDone(&(usbStruct.usbData[1].TX_BUFF[0]),usbStruct.captureDetail.numBytesToSend,CDC1_INTFNUM,0);
        usbStruct.captureDetail.captureStatus=datasendComplete;

        if(extraRegCapture.extraRegCaptureStatus==enabled)
        {
            usbStruct.txBuffPointer = 0;
            usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=0;
            usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=0;
            usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=0;
            usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=1;

            for(regCounter=0;regCounter<extraRegCapture.numRegsToCap;regCounter++)
            {
                SPI_READ_MODE=READ_AFE1;
                readFromAFE(extraRegCapture.afeRegAddress[regCounter],(uint8_t *)&usbStruct.afeRegData);
                copyRegDataInTxBuf1(7);
            }
            usbStruct.captureDetail.numBytesToSend=usbStruct.txBuffPointer;
            USBCDC_sendDataAndWaitTillDone(&(usbStruct.usbData[1].TX_BUFF[0]),usbStruct.captureDetail.numBytesToSend,CDC1_INTFNUM,0);
        }

        if(CAP_STAT==RUNNING)/*Capture not completed*/
        {
            DATA_RDY_PORT_FLAG=0;
            GPIO_enableInterrupt(ADC_RDY_PORT,ADC_RDY_PIN);
        }
    }
}

void decodeCommand(uint8_t *commandBuff)
{
    if(compareStr(commandBuff,(uint8_t*)"C:CDC?",6))
        usbStruct.receievedCommand=CDC_QUERY;
    if(compareStr(commandBuff,(uint8_t*)"C:RST_AFE",9))
            usbStruct.receievedCommand=RESET_AFE;
    if(compareStr(commandBuff,(uint8_t*)"C:RST_UC",8))
            usbStruct.receievedCommand=RESET_UC;
    if(compareStr(commandBuff,(uint8_t*)"C:REG_W",7))
            usbStruct.receievedCommand=REG_WRITE;
    if(compareStr(commandBuff,(uint8_t*)"C:REG_R",7))
            usbStruct.receievedCommand=REG_READ;
    if(compareStr(commandBuff,(uint8_t*)"C:CAP_START",11))
            usbStruct.receievedCommand=START_CAPTURE;
    if(compareStr(commandBuff,(uint8_t*)"C:CAP_STOP",10))
            usbStruct.receievedCommand=STOP_CAPTURE;
    if(compareStr(commandBuff,(uint8_t*)"C:REG_VALS",10))
            usbStruct.receievedCommand=GET_REG_VALS;
    if(compareStr(commandBuff,(uint8_t*)"C:COMM_SEL",10))
            usbStruct.receievedCommand=COMM_SELECT;
    if(compareStr(commandBuff,(uint8_t*)"C:XTRA_REG",10))
            usbStruct.receievedCommand=EXTRA_REG_CAPTURE;
    if(compareStr(commandBuff,(uint8_t*)"FIRMWARE_VER?",13))
            usbStruct.receievedCommand=FIRMWARE_VER;
    if(compareStr(commandBuff,(uint8_t*)"DEVICE?",7))
            usbStruct.receievedCommand=DEV_QUERY;
    if(compareStr(commandBuff,(uint8_t*)"ENTER_BSL",9))
                usbStruct.receievedCommand=BSL_ENTRY;
    if(compareStr(commandBuff,(uint8_t*)"C:W_INFO",8))
            usbStruct.receievedCommand=WRITE_INFO;
    if(compareStr(commandBuff,(uint8_t*)"C:R_INFO",8))
            usbStruct.receievedCommand=READ_INFO;
}

uint8_t compareStr(uint8_t *str1, uint8_t *str2, uint8_t len)
{
    uint8_t compareStatus=1;
    while(len!=0)
    {
        if(*(str1+len-1) != *(str2+len-1))
        {
            compareStatus=0;
            break;
        }

        len--;
    }
    return(compareStatus);
}

void copyRegDataInTxBuf1(uint8_t dataIdentifier)
{
    usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=usbStruct.afeRegData        & 0xFF;
    usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=(usbStruct.afeRegData>>8)   & 0xFF;
    usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=(usbStruct.afeRegData>>16)  & 0xFF;
    usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=dataIdentifier;
}

void capture(void)
{
    volatile bool captureNow=false;
    volatile uint16_t numWordsToRead[1]={0};

    if(usbStruct.finiteCapture==false)
    {
        captureNow=true;
    }
    else
    {
        if(usbStruct.numFiFoRdy!=0)
        {
            usbStruct.numFiFoRdy--;
            captureNow=true;
        }
        else
        {
            disableCapture();
        }
    }

    if(captureNow)
    {
        usbStruct.txBuffPointer = 0;
        usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=0;
        usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=0;
        usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=0;
        usbStruct.usbData[1].TX_BUFF[usbStruct.txBuffPointer++]=2;

        /*Read pointer difference from AFE1*/
        SPI_READ_MODE=READ_AFE1;
        readFromAFE(0x6D,(uint8_t *)&usbStruct.afeRegData);
        copyRegDataInTxBuf1(3);
        numWordsToRead[0]=(usbStruct.afeRegData+1)&0xFF;


        SPI_READ_MODE=READ_AFE1;
        while(numWordsToRead[0]!=0)
        {
            readFromAFE(0xFF,(uint8_t *)&usbStruct.afeRegData);
            copyRegDataInTxBuf1(5);
            numWordsToRead[0]--;
        }

        usbStruct.captureDetail.numBytesToSend=usbStruct.txBuffPointer;
        usbStruct.captureDetail.captureStatus=dataReadyTosend;
    }
}



void gotoBslLocation(void)
{
    __delay_cycles(18000000);
    USB_reset();
    USB_disconnect();
    __disable_interrupt(); // disable interrupts
    ((void ( * )())0x1000)(); // jump to BSL
}
