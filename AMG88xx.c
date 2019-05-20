/*
 * File:   AMGxx.c
 * Author: tommy
 *
 * Created on 2 November 2018, 9:50 PM
 * 
 * A library to run the AMG8833 Thermal IR sensor array.
 * Designed to run on a PIC 18F24J50, however could be easily
 * adapted to other devices.
 * 
 * Datasheet:
 * https://cdn-learn.adafruit.com/assets/assets/000/043/261/original/Grid-EYE_SPECIFICATIONS%28Reference%29.pdf
 */


#include <xc.h>
#include "AMG88xx.h"

/*
 * Sends a byte to a specified address over I2C.
 */
void i2c_send_data(unsigned char address, unsigned char reg, unsigned char data) {
    SSP1CON2 = 0b00000001; //Begin start condition
    i2c_master_wait();
    SSP1BUF = address; //Put address in the buffer ready to send with WRITE bit.
    i2c_master_wait(); //Wait for data to send and ACK to be received
    SSP1BUF = reg;
    i2c_master_wait(); //Wait for register
    SSP1BUF = data;
    i2c_master_wait(); //Wait for the data to send
    SSP1CON2 = 0b00000100; //Stop condition
}

/*
 * Send a normal transmission without any data for some reason?
 */
void i2c_send_request(unsigned char address, unsigned char reg) {
    i2c_master_wait(); //Wait for any previous operations to finish
    SSP1CON2 = 0b00000001; //Begin start condition
    i2c_master_wait();
    SSP1BUF = address & 0b11111110; //Put address in the buffer ready to send with WRITE bit.
    i2c_master_wait(); //Wait for data to send and ACK to be received
    SSP1BUF = reg;
}

/*
 * Receive a byte over I2C from a specified device address.
 */
unsigned char i2c_receive_data(unsigned char address, unsigned char reg) {
    i2c_send_request(address, reg); //Request the register for some reason, maybe, how knows, WTF machine.
    i2c_master_wait(); //Wait previous operation
    SSP1CON2 = 0b00000001; //Restart condition
    
    //Send address again for some reason
    while(SSP1CON2 & 0b00000001);
    SSP1BUF = address;
    i2c_master_wait();
    
    RCEN = 1; //Set Data receive enable bit (Deprecated reference)
    while(!SSP1STAT & 0b00000001); //Wait for byte received
    
    
    SSP1CON2 = 0b00010000; //NACK (ACK from master) bit
    
    i2c_master_wait();
    SSP1CON2 = 0b00000100; //Stop bit
    
    unsigned char data = SSP1BUF; //Get the data
    
    return data;
}

/*
 * Waits for master I2C functions to finish.
 * Checks the SSP1STAT register until all the process bits
 * have cleared, and transmission has ended.
 */
void i2c_master_wait() {
    while((SSP1STAT & 0b00000100) || (SSP1CON2 & 0b00011111));
}

/*
 * Initialisation routine for the AMG88xx IR sensor array
 */
void AMGInit() {
    //Set normal mode
    i2c_send_data(AMG88xx_ADDR, AMG88xx_PCTL, AMG88xx_MODE);
    //Power on reset
    i2c_send_data(AMG88xx_ADDR, AMG88xx_RST, AMG88xx_RESET);
    //Set 1hz refresh
    i2c_send_data(AMG88xx_ADDR, AMG88xx_FPSC, AMG88xx_REFRESH);
    
    
}


