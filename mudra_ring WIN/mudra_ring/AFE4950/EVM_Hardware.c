/*
 * EVM_Hardware.c
 *
 *  Created on: Oct 11, 2019
 *      Author: a0230206
 */

#include "EVM_Hardware.h"

enum CAP_STATs volatile CAP_STAT        = STOPPED;
enum SELECTED_COMMs SELECTED_COMM       = SPI;
enum RW_MODES RW_MODE                   = WRITE_MODE_AFE1;
float AFE_SUP_V                         = 0;
enum SPI_WRITE_MODES  SPI_WRITE_MODE    = WRITE_AFE1;
enum SPI_READ_MODES SPI_READ_MODE       = READ_AFE1;


union _informationData
{
    uint8_t informationWriteData[128];
    uint8_t informationReadData[128];
}informationData;

void initEvmHardware(void)
{
	extraRegCapture.extraRegCaptureStatus=disabled;
	PMM_setVCore(PMMCOREV_3);
    USBHAL_initPorts();
    USB_setup(TRUE,TRUE);
	initGPIO();
	initClock();
    initTimer();
    SELECT_COMM();
    __enable_interrupt();
    resetAFE();
}

void initGPIO(void)
{
    GPIO_setAsOutputPin(I2C_SPI_SEL_PORT  ,I2C_SPI_SEL_PIN );
    GPIO_setAsOutputPin(LDO_ENABLE_PORT  ,LDO_ENABLE_PIN );
    GPIO_setAsInputPin  (ADC_RDY_PORT     ,ADC_RDY_PIN     );
    GPIO_setAsOutputPin (AFE_RESET_PORT   ,AFE_RESET_PIN   );
    GPIO_setAsOutputPin (LED_READ_PORT    ,LED_READ_PIN    );
    GPIO_setAsOutputPin (LED_WRITE_PORT   ,LED_WRITE_PIN   );

    GPIO_setOutputLowOnPin(I2C_SPI_SEL_PORT  ,I2C_SPI_SEL_PIN );//SPI Selected while powering up.
    GPIO_setOutputLowOnPin(LDO_ENABLE_PORT  ,LDO_ENABLE_PIN );  //LDO Enable mode.
    GPIO_setOutputLowOnPin(AFE_RESET_PORT    ,AFE_RESET_PIN   );//AFE Powered Down while powering up.
    GPIO_setOutputLowOnPin(LED_READ_PORT    ,LED_READ_PIN    );//LED OFF
    GPIO_setOutputLowOnPin(LED_WRITE_PORT   ,LED_WRITE_PIN   );//LED OFF
}

void initClock(void)
{
    UCS_initClockSignal(UCS_ACLK, UCS_XT2CLK_SELECT,     UCS_CLOCK_DIVIDER_1);
    UCS_initClockSignal(UCS_MCLK, UCS_XT2CLK_SELECT,     UCS_CLOCK_DIVIDER_1);
    UCS_initClockSignal(UCS_SMCLK, UCS_XT2CLK_SELECT,    UCS_CLOCK_DIVIDER_1);
}


void resetAFE(void)
{
    disableCapture();
    GPIO_setOutputLowOnPin(AFE_RESET_PORT,AFE_RESET_PIN);
    __delay_cycles(1200);//50uSec
    GPIO_setOutputHighOnPin(AFE_RESET_PORT,AFE_RESET_PIN);


    /*Reset register array*/
    afeRegs.AFE1_REG0.AFE_REG=0;

    RW_MODE  = WRITE_MODE_AFE1;
    CAP_STAT = STOPPED;
}


void initTimer (void)
{
    Timer_A_initUpModeParam Timer_A_params = {0};
    Timer_A_params.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    Timer_A_params.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    Timer_A_params.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    Timer_A_params.captureCompareInterruptEnable_CCR0_CCIE =TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_params.timerClear = TIMER_A_DO_CLEAR;
    Timer_A_params.startTimer = true;

    Timer_A_clearTimerInterrupt(TIMER_A0_BASE);
    Timer_A_params.timerPeriod = _50MS;
    Timer_A_initUpMode(TIMER_A0_BASE, &Timer_A_params);

}


void initI2C(void)
{
   GPIO_setAsPeripheralModuleFunctionOutputPin(I2C_SCL_PORT,I2C_SCL_PIN);//SCL
   GPIO_setAsPeripheralModuleFunctionOutputPin(I2C_SDA_PORT,I2C_SDA_PIN);//SDA
   USCI_B_I2C_initMasterParam I2C_INTERFACE;
   I2C_INTERFACE.selectClockSource = USCI_B_I2C_CLOCKSOURCE_SMCLK;
   I2C_INTERFACE.i2cClk =FSYSTEM;
   I2C_INTERFACE.dataRate = USCI_B_I2C_SET_DATA_RATE_400KBPS;
   USCI_B_I2C_initMaster(I2C_BASE,&I2C_INTERFACE);
   USCI_B_I2C_enable(I2C_BASE);

   GPIO_setAsOutputPin(SPI_SENZ_PORT,SPI_SENZ_PIN);
   GPIO_setOutputLowOnPin(SPI_SENZ_PORT,SPI_SENZ_PIN);//I2C SELECTED
   GPIO_setOutputHighOnPin(I2C_SPI_SEL_PORT,I2C_SPI_SEL_PIN);//I2C SELECTED
}
void initSPI(void)
{
    GPIO_setAsOutputPin(SPI_SENZ_PORT,SPI_SENZ_PIN);
    GPIO_setAsPeripheralModuleFunctionInputPin(SPI_SDOUT_PORT,SPI_SDOUT_PIN);
    GPIO_setAsPeripheralModuleFunctionOutputPin(SPI_SCLK_PORT,SPI_SCLK_PIN);
    GPIO_setAsPeripheralModuleFunctionOutputPin(SPI_SDIN_PORT,SPI_SDIN_PIN);

    USCI_A_SPI_initMasterParam SPI_Config;
    SPI_Config.selectClockSource=USCI_A_SPI_CLOCKSOURCE_SMCLK;
    SPI_Config.clockSourceFrequency=FSYSTEM;
    SPI_Config.desiredSpiClock=1000000;
    SPI_Config.msbFirst=USCI_A_SPI_MSB_FIRST;
    SPI_Config.clockPhase=USCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    SPI_Config.clockPolarity=USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    USCI_A_SPI_initMaster(SPI_BASE,&SPI_Config);
    USCI_A_SPI_enable(SPI_BASE);


    GPIO_setOutputLowOnPin(I2C_SPI_SEL_PORT,I2C_SPI_SEL_PIN);//SPI SELECTED
}

void SELECT_COMM(void)
{
    switch(SELECTED_COMM)
    {
        case I2C:   initI2C();
                    readAFE     =&readI2C;
                    writeAFE    =&writeI2C;
            break;
        case SPI:   initSPI();
                    readAFE     =&readSPI;
                    writeAFE    =&writeSPI;
            break;
    }
}


void readI2C(uint8_t regAddress, uint8_t*data)
{
    unsigned char c=0;
    uint8_t i2CData[4];
    USCI_B_I2C_setSlaveAddress(I2C_BASE,DEV_I2C_ADDRESS);
    USCI_B_I2C_setMode(I2C_BASE,USCI_B_I2C_TRANSMIT_MODE);
    while((USCI_B_I2C_isBusBusy(I2C_BASE)));
    USCI_B_I2C_masterSendSingleByte(I2C_BASE,regAddress);
    while (USCI_B_I2C_masterIsStartSent(I2C_BASE));
    USCI_B_I2C_setMode(I2C_BASE,USCI_B_I2C_RECEIVE_MODE);
    USCI_B_I2C_masterReceiveMultiByteStart(I2C_BASE);

    i2CData[0]=0;
    for(c=1;c<3;c++)
        i2CData[c]   = USCI_B_I2C_masterReceiveSingle(I2C_BASE);

    USCI_B_I2C_masterReceiveMultiByteStop(I2C_BASE);
    i2CData[c]=USCI_B_I2C_masterReceiveMultiByteEndPoll(I2C_BASE);
    USCI_B_I2C_setMode(I2C_BASE,USCI_B_I2C_TRANSMIT_MODE);

    *data=i2CData[3]; *(data+1)=i2CData[2]; *(data+2)=i2CData[1]; *(data+3)=i2CData[0];

}

void readSPI(uint8_t regAddress, uint8_t*data)
{
    uint8_t spiData[4]={0,0,0,0};

    if(SPI_READ_MODE == READ_AFE1)
        GPIO_setOutputLowOnPin(SPI_SENZ_PORT,SPI_SENZ_PIN);


    while(USCI_A_SPI_isBusy(SPI_BASE)==USCI_A_SPI_BUSY);

    USCI_A_SPI_transmitData(SPI_BASE,regAddress);
    while(USCI_A_SPI_isBusy(SPI_BASE)==USCI_A_SPI_BUSY);
    spiData[0]=USCI_A_SPI_receiveData(SPI_BASE);        //Dummy Read

    USCI_A_SPI_transmitData(SPI_BASE,0x00);
    while(USCI_A_SPI_isBusy(SPI_BASE)==USCI_A_SPI_BUSY);
    spiData[1]=USCI_A_SPI_receiveData(SPI_BASE);

    USCI_A_SPI_transmitData(SPI_BASE,0x00);
    while(USCI_A_SPI_isBusy(SPI_BASE)==USCI_A_SPI_BUSY);
    spiData[2]=USCI_A_SPI_receiveData(SPI_BASE);

    USCI_A_SPI_transmitData(SPI_BASE,0x00);
    while(USCI_A_SPI_isBusy(SPI_BASE)==USCI_A_SPI_BUSY);
    spiData[3]=USCI_A_SPI_receiveData(SPI_BASE);

    while(USCI_A_SPI_isBusy(SPI_BASE)==USCI_A_SPI_BUSY);

    GPIO_setOutputHighOnPin(SPI_SENZ_PORT,SPI_SENZ_PIN);

    *data=spiData[3]; *(data+1)=spiData[2]; *(data+2)=spiData[1]; *(data+3)=spiData[0];
}

void writeI2C(uint8_t regAddress, uint32_t data)
{

    USCI_B_I2C_setSlaveAddress(I2C_BASE,DEV_I2C_ADDRESS);
    USCI_B_I2C_setMode(I2C_BASE,USCI_B_I2C_TRANSMIT_MODE);
    while(USCI_B_I2C_isBusBusy(I2C_BASE));
    USCI_B_I2C_masterSendMultiByteStart(I2C_BASE,regAddress);
    while (USCI_B_I2C_masterIsStartSent(I2C_BASE));

    USCI_B_I2C_masterSendMultiByteNext(I2C_BASE,(data>>16) &0xFF);
    USCI_B_I2C_masterSendMultiByteNext(I2C_BASE,(data>>8)  &0xFF);
    USCI_B_I2C_masterSendMultiByteNext(I2C_BASE,(data)     &0xFF);

    USCI_B_I2C_masterSendMultiByteStop(I2C_BASE);
    while (USCI_B_I2C_masterIsStopSent(I2C_BASE));
}

void writeSPI(uint8_t regAddress, uint32_t data)
{
    if(SPI_WRITE_MODE == WRITE_AFE1)
        GPIO_setOutputLowOnPin(SPI_SENZ_PORT,SPI_SENZ_PIN);

    while(USCI_A_SPI_isBusy(SPI_BASE)==USCI_A_SPI_BUSY);
    USCI_A_SPI_transmitData(SPI_BASE,regAddress);
    while(USCI_A_SPI_isBusy(SPI_BASE)==USCI_A_SPI_BUSY);
    USCI_A_SPI_transmitData(SPI_BASE,(data>>16) &0xFF);
    while(USCI_A_SPI_isBusy(SPI_BASE)==USCI_A_SPI_BUSY);
    USCI_A_SPI_transmitData(SPI_BASE,(data>>8)  &0xFF);
    while(USCI_A_SPI_isBusy(SPI_BASE)==USCI_A_SPI_BUSY);
    USCI_A_SPI_transmitData(SPI_BASE,(data)     &0xFF);
    while(USCI_A_SPI_isBusy(SPI_BASE)==USCI_A_SPI_BUSY);
    GPIO_setOutputHighOnPin(SPI_SENZ_PORT,SPI_SENZ_PIN);
}

/*readWriteValue = 1 for read*/
/*readWriteValue = 0 for write*/
void switchReadWriteMode(uint8_t readWriteValue)
{
    enum SPI_WRITE_MODES  SPI_WRITE_MODE_COPY = SPI_WRITE_MODE;

    if(afeRegs.AFE1_REG0.afeRegisterBits.AFE_BIT0 == (readWriteValue^1))
    {
        afeRegs.AFE1_REG0.afeRegisterBits.AFE_BIT0=readWriteValue;
        SPI_WRITE_MODE=WRITE_AFE1;
        writeAFE(0x00,afeRegs.AFE1_REG0.AFE_REG);
        SPI_WRITE_MODE = SPI_WRITE_MODE_COPY;
    }

}
void writeToAFE(uint8_t regAddress, uint32_t data)
{
    trunOnLed(WRITE_TO_DEVICE);
    switchReadWriteMode(0);

    /*Software Reset*/
    if(regAddress==0)
    {
        if((data & 0x08)==0x08)
        {
            disableCapture();
            /*Reset register array*/
            afeRegs.AFE1_REG0.AFE_REG=0;
            RW_MODE  = WRITE_MODE_AFE1;
            CAP_STAT = STOPPED;
        }
        else
        {
            afeRegs.AFE1_REG0.AFE_REG = data;
        }

        SPI_WRITE_MODE=WRITE_AFE1;
        writeAFE(regAddress,data);
    }
    else
    {
        writeAFE(regAddress,data);
    }


}

void readFromAFE(uint8_t regAddress, uint8_t* data)
{
    trunOnLed(READ_FROM_DEVICE);
    switchReadWriteMode(1);

    if(regAddress!=0)
    {
        readAFE(regAddress,data);
    }
    else
    {
        *(data+3)=0;
        if(SPI_READ_MODE==READ_AFE1)
        {
            *(data+2)=(afeRegs.AFE1_REG0.AFE_REG>>16)&0xFF;
            *(data+1)=(afeRegs.AFE1_REG0.AFE_REG>>8)&0xFF;
            *(data+0)=(afeRegs.AFE1_REG0.AFE_REG)&0xFF;
        }
    }
}

void trunOnLed(enum LEDs selectedLED)
{
    /*
     * Time = 50mS* counter value
     * count= 5 = 5*50e-3mSec = 250mSec
     */
    switch(selectedLED)
    {
        case BOARD_UP: timerCounter.BOARD_UP_LED=10;
                break;
        case WRITE_TO_DEVICE: timerCounter.WRITE_TO_DEVICE_LED=5;
                break;
        case READ_FROM_DEVICE: timerCounter.READ_FROM_DEVICE_LED=5;
                break;
        case ERRORS: timerCounter.ERROR_LED=5;
                break;
        default: timerCounter.ERROR_LED=5;
                break;
    }

}

void enableCapture(void)
{
    if(CAP_STAT==STOPPED)
    {
        /*Stop toggling LEDs*/
        GPIO_setAsInputPin (LED_READ_PORT    ,LED_READ_PIN    );
        GPIO_setAsInputPin (LED_WRITE_PORT   ,LED_WRITE_PIN   );

        SPI_WRITE_MODE  = WRITE_AFE1;

        writeToAFE(0x01,0x00);//Switch to Page 0
        //writeToAFE(0x1D,0x000000); //Stop timing engine
        writeToAFE(0x00,0x02);//Reset FIFO
        DATA_RDY_PORT_FLAG=0;
        writeToAFE(0x00,0x40);//Reset FIFO
        //writeToAFE(0x1D,0xC00000); //Start timing engine
        CAP_STAT= RUNNING;
        GPIO_enableInterrupt(ADC_RDY_PORT,ADC_RDY_PIN);
    }
}
void disableCapture(void)
{
    if(CAP_STAT==RUNNING)
    {
        CAP_STAT=STOP;

        /*Blindly stop interrupt on FIFO ready pin*/
        if(CAP_STAT==STOP)
        {
           GPIO_disableInterrupt(ADC_RDY_PORT,ADC_RDY_PIN);
           DATA_RDY_PORT_FLAG=0;
           CAP_STAT = STOPPED;
        }

        while(CAP_STAT!=STOPPED);

        GPIO_setAsOutputPin (LED_READ_PORT    ,LED_READ_PIN    );
        GPIO_setAsOutputPin (LED_WRITE_PORT   ,LED_WRITE_PIN   );
    }
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{

    if(timerCounter.BOARD_UP_LED!=0)
    {
        timerCounter.BOARD_UP_LED--;
    }


    if(timerCounter.WRITE_TO_DEVICE_LED!=0)
    {
        timerCounter.WRITE_TO_DEVICE_LED--;
        GPIO_setOutputHighOnPin(LED_WRITE_PORT,LED_WRITE_PIN);
        if(timerCounter.WRITE_TO_DEVICE_LED==0)
            GPIO_setOutputLowOnPin(LED_WRITE_PORT,LED_WRITE_PIN);
    }
    if(timerCounter.READ_FROM_DEVICE_LED!=0)
    {
        timerCounter.READ_FROM_DEVICE_LED--;
        GPIO_setOutputHighOnPin(LED_READ_PORT,LED_READ_PIN);
        if(timerCounter.READ_FROM_DEVICE_LED==0)
            GPIO_setOutputLowOnPin(LED_READ_PORT,LED_READ_PIN);
    }
    if(timerCounter.ERROR_LED!=0)
    {
        timerCounter.ERROR_LED--;
    }
}



#pragma vector=DATA_RDY_VECTOR
__interrupt
void PORT1_INTERRUPT (void)
{
    if(DATA_RDY_PORT_FLAG & DATA_RDY_INT_MASK)
    {
        GPIO_disableInterrupt(ADC_RDY_PORT,ADC_RDY_PIN);/*No more triggers before completing the interrupt should re-trigger the interrupt*/
        capture();
    }
    DATA_RDY_PORT_FLAG=0;
}



void readInformationData(uint8_t* transmitBuffer, uint16_t numData)
{
    uint16_t counter=0;
    copy((uint8_t*)INFO_ADDRESS, (uint8_t*)informationData.informationReadData,numData);
    for(counter=0; counter<= numData; counter++)
    {
        hex2Ascii((char*)(transmitBuffer+(2*counter)), informationData.informationReadData[counter], 1);
    }
}
void writeInformationData(uint8_t* receieveBuffer)
{
    uint16_t counter=0;
    uint16_t numData;

    ascii2Hex((char*)&numData,(char*)receieveBuffer, 4); //One byte. Number of data to be written
    for(counter=0; counter<= numData; counter++)
    {
        ascii2Hex((char*)&informationData.informationWriteData[counter], (char*)(receieveBuffer+4+(2*counter)), 2); //One byte. Data to be written
    }

    FlashCtl_unlockInfoA();
    FlashCtl_eraseSegment((uint8_t*)INFO_ADDRESS);
    FlashCtl_write8((uint8_t*)informationData.informationWriteData, (uint8_t*)INFO_ADDRESS, numData);
    FlashCtl_lockInfoA();
}
