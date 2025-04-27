/*
 * EVM_Hardware.h
 *
 *  Created on: Oct 11, 2019
 *      Author: a0230206
 */

#ifndef EVM_HARDWARE_H_ 
#define EVM_HARDWARE_H_

#include "driverlib.h"
#include "pcComm.h" 
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"
#include "USB_API/USB_MSC_API/UsbMscScsi.h"
#include "USB_API/USB_MSC_API/UsbMsc.h"
#include "USB_API/USB_MSC_API/UsbMscStateMachine.h"
#include "USB_API/USB_HID_API/UsbHid.h"
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"
#include "hal.h"


#define FSYSTEM             24000000
#define XTAL_FREQ           24000000
#define INFO_ADDRESS        0x1980
#define EVMPINCONFIG
#define I2C_BASE            USCI_B1_BASE
#define SPI_BASE            USCI_A0_BASE
#define DEV_I2C_ADDRESS     0x5B


#define I2C_SPI_SEL_PORT    GPIO_PORT_P1
#define LDO_ENABLE_PORT     GPIO_PORT_P2
#define I2C_SCL_PORT        GPIO_PORT_P4
#define I2C_SDA_PORT        GPIO_PORT_P4
#define ADC_RDY_PORT        GPIO_PORT_P2
#define AFE_RESET_PORT      GPIO_PORT_P1
#define SPI_SCLK_PORT       GPIO_PORT_P2
#define SPI_SDOUT_PORT      GPIO_PORT_P3
#define SPI_SDIN_PORT       GPIO_PORT_P3
#define SPI_SENZ_PORT       GPIO_PORT_P3
#define LED_READ_PORT       GPIO_PORT_P5
#define LED_WRITE_PORT      GPIO_PORT_P5

#define I2C_SPI_SEL_PIN     GPIO_PIN2
#define LDO_ENABLE_PIN      GPIO_PIN2
#define I2C_SCL_PIN         GPIO_PIN2
#define I2C_SDA_PIN         GPIO_PIN1
#define ADC_RDY_PIN         GPIO_PIN3
#define AFE_RESET_PIN       GPIO_PIN1
#define SPI_SCLK_PIN        GPIO_PIN7
#define SPI_SDOUT_PIN       GPIO_PIN4
#define SPI_SDIN_PIN        GPIO_PIN3
#define SPI_SENZ_PIN        GPIO_PIN2
#define LED_READ_PIN        GPIO_PIN1
#define LED_WRITE_PIN       GPIO_PIN0



#define DATA_RDY_INT_MASK   0X08
#define DATA_RDY_PORT_FLAG  P2IFG
#define DATA_RDY_VECTOR     PORT2_VECTOR

#define _50MS               50*(FSYSTEM/64000)

enum LEDs
{
    BOARD_UP,
    WRITE_TO_DEVICE,
    READ_FROM_DEVICE,
    ERRORS
};

enum CAP_STATs{STOP,STOPPED,RUNNING};
enum SELECTED_COMMs{I2C,SPI};


struct timerCounters
{
    volatile uint16_t BOARD_UP_LED;
    volatile uint16_t WRITE_TO_DEVICE_LED;
    volatile uint16_t READ_FROM_DEVICE_LED;
    volatile uint16_t ERROR_LED;
};



struct _afeRegisterBits
{
    uint32_t AFE_BIT0:1;
    uint32_t AFE_BIT1:1;
    uint32_t AFE_BIT2:1;
    uint32_t AFE_BIT3:1;
    uint32_t AFE_BIT4:1;
    uint32_t AFE_BIT5:1;
    uint32_t AFE_BIT6:1;
    uint32_t AFE_BIT7:1;
    uint32_t AFE_BIT8:1;
    uint32_t AFE_BIT9:1;
    uint32_t AFE_BIT10:1;
    uint32_t AFE_BIT11:1;
    uint32_t AFE_BIT12:1;
    uint32_t AFE_BIT13:1;
    uint32_t AFE_BIT14:1;
    uint32_t AFE_BIT15:1;
    uint32_t AFE_BIT16:1;
    uint32_t AFE_BIT17:1;
    uint32_t AFE_BIT18:1;
    uint32_t AFE_BIT19:1;
    uint32_t AFE_BIT20:1;
    uint32_t AFE_BIT21:1;
    uint32_t AFE_BIT22:1;
    uint32_t AFE_BIT23:1;
};

union _afeRegister
{
    struct _afeRegisterBits afeRegisterBits;
    uint32_t AFE_REG;
};

struct _afeRegs
{
    union _afeRegister   AFE1_REG0;
};

enum RW_MODES
{
    WRITE_MODE_AFE1,
    READ_MODE_AFE1
};

enum SPI_WRITE_MODES
{
    WRITE_AFE1
};

enum SPI_READ_MODES
{
    READ_AFE1
};

struct timerCounters timerCounter;
struct _afeRegs afeRegs;

void initEvmHardware(void);
void initGPIO(void);
void initClock(void);
void resetAFE(void);
void initTimer(void);
void trunOnLed(enum LEDs selectedLED);
void initI2C(void);
void initSPI(void);
void SELECT_COMM(void);
void switchReadWriteMode(uint8_t readWriteValue);


void readI2C(uint8_t regAddress, uint8_t* data);
void readSPI(uint8_t regAddress, uint8_t*data);
void writeI2C(uint8_t regAddress, uint32_t data);
void writeSPI(uint8_t regAddress, uint32_t data);
void (*writeAFE)(uint8_t regAddress, uint32_t data);
void (*readAFE)(uint8_t regAddress, uint8_t*data);
void writeToAFE(uint8_t regAddress, uint32_t data);
void readFromAFE(uint8_t regAddress, uint8_t* data);
void enableCapture(void);
void disableCapture(void);
void readInformationData(uint8_t* transmitBuffer, uint16_t numData);
void writeInformationData(uint8_t* receieveBuffer);

#endif /* EVM_HARDWARE_H_ */
