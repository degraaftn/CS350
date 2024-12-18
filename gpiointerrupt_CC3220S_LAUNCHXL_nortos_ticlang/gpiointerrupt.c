/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Display definition */
#define DISPLAY(x) UART2_write(uart, &output, x, NULL);

/* Global Variable Declarations */
int16_t leftButtonFlag;                                             // Left button decreases the temperature
int16_t rightButtonFlag;                                            // Right button increases the temperature

int16_t currentTemp;
int16_t setTemp;
int16_t heat;                                                       // 0 when LED is off, 1 when LED is on
int seconds;

const unsigned long taskPeriod = 100;
const unsigned long readButtonPeriod = 200;
const unsigned long readTempPeriod = 500;
const unsigned long updateOutputPeriod = 1000;

/* State machines */
enum Button_States {Button_SMStart, Button_Read} Button_State;      // SM that checks the button flags every 200 ms
enum Temp_States {Temp_SMStart, Temp_Read} Temp_State;              // SM that reads the temperature every 500 ms
enum Heat_States {Heat_SMStart, Heat_Read} Heat_State;              // SM that toggles the LED if heat is on and
                                                                    // outputs the chosen variables every second
/*
 * ==== UART2 Driver code ====
 */
 //UART2 Global Variables
char output[64];
int bytesToSend;

UART2_Handle uart;

// initUart function
void initUart(void){
    UART2_Params uartParams;

    // Init the driver
    UART2_Params_init(&uartParams);
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    /* UART open failed */
    if(uart == NULL){
        while(1);
    }
}

/*
 * ==== I2C Driver Code ====
 */
 // I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char* id;
} sensors[3] = {
    {0x48, 0x0000, "11X"},
    {0x49, 0x0000, "116"},
    {0x41, 0x0001, "006"}
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];

I2C_Transaction i2cTransaction;
I2C_Handle i2c;

// initI2C function
void initI2C(void){
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    /* I2C Open Failed */
    if(i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while(1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses.

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;

    for(i = 0; i < 3; ++i){
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));

        if(I2C_transfer(i2c, &i2cTransaction)){
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if(found){
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress));
    } else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}

// readTemp function
int16_t readTemp(void){
    int j;
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;

    if(I2C_transfer(i2c, &i2cTransaction)){
        // Extract degrees C from the received data;
        // see TMP datasheet
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        // If MSB is set '1', then we have a 2's complement
        // negative value which needs to be sign extended
        if(rxBuffer[0] & 0x80){
            temperature |= 0xF000;
        }
    } else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }

    return temperature;
}

/*
 * ==== Timer Driver Code ====
 */
 // Timer Global Variables
Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;

// timerCallback function
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;
}

// initTimer function
void initTimer(void){
    Timer_Params params;

    // Init the driver
    Timer_init();

    //Configure the driver
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    // Failed to initalize timer
    if(timer0 == NULL){
        while(1);
    }

    // Failed to start timer
    if(Timer_start(timer0) == Timer_STATUS_ERROR){
        while(1);
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0 (Left).
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    // Left Button pressed
    leftButtonFlag += 1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1 (Right).
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    // Right Button pressed
    rightButtonFlag += 1;
}

/*
 * ======== GPIO Driver Code ========
 */
void initGPIO(void){
    /* Call driver init function */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }
}

/*
 * ======== setTemp_SM ========
 *
 * The Task Scheduler uses this SM to read the button flags and adjust the setTemp accordingly.
 */
int setTemp_SM(int state){
    switch(state){

    // SM starts in Button_SMStart state and automatically switches to Button_Read state
    case Button_SMStart:
        Button_State = Button_Read;
        break;

    case Button_Read:
        // Check for left button flags and decrease the setTemp if active
        if(leftButtonFlag > 0 && setTemp > 0){
            leftButtonFlag -= 1;
            setTemp -= 1;
        }
        // Check for right button flags and increase the setTemp if active
        if(rightButtonFlag > 0 && setTemp < 99){
            rightButtonFlag -= 1;
            setTemp += 1;
        }

        // Stay in Button_Read state
        Button_State = Button_Read;
        break;

    default:
        Button_State = Button_SMStart;
        break;
    }
    state = Button_State;
    return state;
}

/*
 * ======== readTemp_SM ========
 *
 * The Task Scheduler uses this SM to read the currentTemp.
 */
int readTemp_SM(int state){
    switch(state){

    // The SM starts in Temp_SMStart state and automatically switches to Temp_Read state.
    case Temp_SMStart:
        Temp_State = Temp_Read;
        break;

    // Reads the current temperature
    case Temp_Read:
        currentTemp = readTemp();
        // Stay in Temp_Read state
        Temp_State = Temp_Read;
        break;

    default:
        Temp_State = Temp_SMStart;
        break;
    }
    state = Temp_State;
    return state;
}

/*
 * ======== updateOutput_SM ========
 *
 * The Task Scheduler uses this SM to toggle the LED on if the currentTemp is less than the setTemp,
 * and off otherwise; and to output (currentTemp, setTemp, heat, seconds).
 */
int updateOutput_SM(int state){
    switch(state){

    // The SM starts in Heat_SMStart state and automatically switches to Heat_Read state.
    case Heat_SMStart:
        Heat_State = Heat_Read;
        break;

    case Heat_Read:
        // Turn on the LED if the currentTemp is less than the setTemp
        if(currentTemp < setTemp){
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            heat = 1;
        }
        // Turn off the LED if the currentTemp is greater or equal to the setTemp
        else {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            heat = 0;
        }

        // Stay in Heat_Read state
        Heat_State = Heat_Read;
        break;

    default:
        Heat_State = Heat_SMStart;
        break;
    }
    state = Heat_State;
    //DISPLAY(snprintf(output, 64, "<%02d, %02d, %d, %04d>\n\r", currentTemp, setTemp, heat, seconds));
    return state;
}

/* Task Scheduler Code */
// Each task uses a different state machine to achieve the required functionality

typedef struct task {
    int state;                                                      // State for the state machine
    unsigned long period;                                           // Period for each task
    unsigned long elapsedTime;                                      // Incrementing variable for keeping track of time
    int (*TickFunction)(int);                                       // Pointer for the state machine function of each task
} task;

task tasks[3] = {
                 // Task 1: Check the buttons and update setTemp
                 {
                  .state = Button_SMStart,
                  .period = readButtonPeriod,
                  .elapsedTime = readButtonPeriod,
                  .TickFunction = &setTemp_SM
                 },
                 // Task 2: Read the temperature
                 {
                  .state = Temp_SMStart,
                  .period = readTempPeriod,
                  .elapsedTime = readTempPeriod,
                  .TickFunction = &readTemp_SM
                 },
                 // Task 3: Update LED and output
                 {
                  .state = Heat_SMStart,
                  .period = updateOutputPeriod,
                  .elapsedTime = updateOutputPeriod,
                  .TickFunction = &updateOutput_SM
                 }
};

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    initGPIO();
    initUart();
    initI2C();
    initTimer();

    /* Global Variables */
    leftButtonFlag = 0;
    rightButtonFlag = 0;

    currentTemp = readTemp();
    setTemp = 21;
    heat = 0;
    seconds = 0;

    /* Init Tasks */
    tasks[0].state = Button_SMStart;
    tasks[0].period = readButtonPeriod;
    tasks[0].elapsedTime = readButtonPeriod;
    tasks[0].TickFunction(tasks[0].state);

    tasks[1].state = Temp_SMStart;
    tasks[1].period = readTempPeriod;
    tasks[1].elapsedTime = readTempPeriod;
    tasks[1].TickFunction(tasks[1].state);

    tasks[2].state= Heat_SMStart;
    tasks[2].period = updateOutputPeriod;
    tasks[2].elapsedTime = updateOutputPeriod;
    tasks[2].TickFunction(tasks[2].state);

    /* Main Loop */
    while(1){
        /* Task Scheduler code */
        unsigned char i;

        // Loop through the tasks
        for(i = 0; i < 3; i++){
            // If elapsedTime hits or passes the period, execute the Tick Function
            if(tasks[i].elapsedTime >= tasks[i].period){
                tasks[i].state = tasks[i].TickFunction(tasks[i].state);
                tasks[i].elapsedTime = 0;
            }

            // Increment elapsedTime by 100 ms
            tasks[i].elapsedTime += taskPeriod;
        }

        DISPLAY(snprintf(output, 64, "<%02d, %02d, %d, %04d>\n\r", currentTemp, setTemp, heat, seconds));

        while(!TimerFlag){}
        TimerFlag = 0;
        ++seconds;
    }
    return (NULL);
}
