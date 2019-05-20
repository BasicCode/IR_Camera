/*
 * File:   main.c
 * Author: tommy
 *
 * Created on 17 October 2018
 * 
 * Uses a little 8x8 far infrared (thermal infrared) sensor and
 * displays the image on an LCD.
 * Written for the PIC 18F24J50.
 */



// PIC18F24J50 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer (Disabled - Controlled by SWDTEN bit)
#pragma config PLLDIV = 4       // PLL Prescaler Selection bits (divide by 4 - fasted stable clock tested)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset  (Enabled)
#pragma config XINST = OFF       // Extended Instruction Set (Disabled)
// CONFIG1H
#pragma config CPUDIV = OSC1    // CPU System Clock Postscaler (No CPU system clock divide
// CONFIG2L
#pragma config OSC = INTOSCPLL  // Oscillator (INTOSCPLL)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <math.h>
#include "ST7735.h"
#include "AMG88xx.h"

#define LED     RC4 //LED is actually unrelated.


//Some global constants
const unsigned int BG_COLOUR = 0x0000;
const unsigned int colour_list[] = {0xFFAA, 0xF0F0, 0x0F0F, 0xF900, 0x009F, 0x0990, 0xF00F};
const int num_colours = 7;
//Size of each pixel
const int box_size = 6;
//AMG88 I2C address
const unsigned char addr = 0xD3;
//AMG88xx I2C memory address of the first address
const unsigned char start_pixel_reg = 0x80;
//Cursor colour for the overlay display (value calculated later)
const unsigned int cursor_colour = 0xFFFF;

//IR 'camera' chip receive buffer
int ir_buffer[64];
int y_interpolated_pixels[64];
int x_interpolated_pixels[64];
int xy_interpolated_pixels[64];

//The maximum and minimum pixel location
typedef struct Pixel {
    int x;
    int y;
    int temperature;
};

//Some RAM space
struct Pixel max_pixel;
struct Pixel min_pixel;


/*
 * Initialise the SPI interface
 */
void initSPI() {
    //SET Port Input / Output latches
    TRISC2 = 0;
    TRISC1 = 0;
    TRISC0 = 0;
    TRISA0 = 0;
    TRISA1 = 0;
    
    //SPI initialisation
    SSP2CON1 = 0b00110000;
    
    //Change the default SPI2 pins to something else so that we can use
    //I2C as well, otherwise we run out of pins.
    //Perpipheral Pin Select modification routine defined in datasheet.
    EECON2 = 0x55;
    EECON2 = 0xAA;
    IOLOCK = 0; //Unlock the PPS module
    RPOR0 = 9; //SPI2 SDO on RP0
    RPOR1 = 10; //SPI2 SCK on RP1
    //Re-lock the PPS module
    IOLOCK = 1;
    
    
}

/*
 * Initialise the I2C interface
 */
void initI2C() {
    //Set Pin latches as INPUT as defined in datasheet
    //Does require external pull-UP resistors.
    TRISB5 = 1;
    TRISB4 = 1;
    
    
    //Using SSP1 for the I2C module
    SSP1CON1 = 0b00101000;
    SSP1CON2 = 0b00000000;
    
    //Set a slow-ish clock time
    SSP1ADD = (0x0C);
}

/*
 * Initialise the ADC used for battery monitoring
 */
void initADC() {
    //Initialise registers. ADC input on AN10.
    ADCON0 = 0b00101001;
    ADCON1 = 0b10000000;
    
    //Start a the conversion routine
    ADCON0 = ADCON0 | 0b00000010;
    
}

/*
 * Convert the IR sensor output to a string reporesentaton of
 * the temperature in degrees C.
 * 
 * Takes an empty buffer and the ADC data, and fills the buffer
 * with the temperature
 */
void convertDataToString(char buff[], int data) {
    /* 12-bit resolution equates to 0.25 degreees per number. I think
     * an accuracy of 0.5 degrees is plenty, so we will just look at 
     * the second digit for the 0.5 degree position. Then the next
     * three digits for values 0 to 9, and so on.
     * 
     * Degrees  512 256 128 64  32  16  8   4   2   1   0.5 0.25
     * Digit    12  11  10  9   8   7   6   5   4   3   2   1
     * 
     * Output is of the form
     * xxx.x\0
     */
    //Add the decimal place
    buff[3] = '.';
    
    
    //First decimal place is only either 0, or 5.
    if(data & 0b10) {
        buff[4] = '5';
    } else {
        buff[4] = '0';
    }
    
    //Place a negative value at the front if a negative number
    if(data < 0) {
        buff[0] = '-';
        data = -data;
    } else {
        buff[0] = ' ';
    }
    
    //Discard the first two positions, we don't need that resolution anymore.
    data = data >> 2;
    
    //Convert values 0 to 9 to ASCII
    int units = (data & 0b11111);
    //Put the number in range
    if(units > 9) {
        buff[1] = '1';
        while(units > 9)
            units -= 10;
    } else {
        buff[1] = ' ';
    }
    
    //Convert to ASCII digit
    buff[2] = units + 48; 
    
    //Count how many tens
    int tens = (int)(data / 10);
    //Bring the tens column in to range.
    if(tens > 9) {
        buff[0] = '1';
        while(tens > 9)
            tens -= 10;
    }
    
    //Convert the tens column to ASCII
    buff[1] = tens + 48;

    //Terminate the string
    buff[5] = '\0';
}

/*
 * Draw an overlay which shows the battery voltage, as well as a
 * cross-hair over the centre pixel and the temperature of that pixel.
 */
void drawOverlay(int x, int y, int data) {
    
    //Also print the battery level jsut for the moment
    //Get ADC result
    unsigned int ADC_reading = (ADRESH << 8) | ADRESL;
    float batt_voltage = ((float)0.6 * (float)1023.0) / (float)(1024 - ADC_reading);
    //Start a new conversion, ready for next time this loops around.
    ADCON0 = ADCON0 | 0b00000010;

    //Check for negative number and invert - if 12th bit is set then is negative
    if(data > 2047) {
        //back-fill the first few bits with 1s
        data = data | (0b11111000 << 8);
        data = (~data + 1) * -1;
        //Seriously, is there a better way to do this?
    }
    
    //Convert the IR data to a string representation of the temperature
    char temperature_buffer[6];
    convertDataToString(temperature_buffer, data);
    
    //Print the string to screen
    draw_string(2, 100, cursor_colour, 2, temperature_buffer);
    
    //Draw a crosshair over the hottest pixel on screen
    int h_pos = x * box_size * 2;
    int v_pos = y * box_size * 2;
    draw_h_line((h_pos)-box_size, (v_pos)+(box_size/2)-1, (h_pos)+(2*box_size), (v_pos)+(box_size/2)-1, cursor_colour);
    draw_h_line((h_pos)-box_size, (v_pos)+(box_size/2), (h_pos)+(2*box_size), (v_pos)+(box_size/2), cursor_colour);
    draw_v_line((h_pos)+(box_size/2)-1, (v_pos)-box_size, (h_pos)+(box_size/2)-1, (v_pos)+(2*box_size), cursor_colour);
    draw_v_line((h_pos)+(box_size/2), (v_pos)-box_size, (h_pos)+(box_size/2), (v_pos)+(2*box_size), cursor_colour);
}

/*
 * Convert the raw pixel temperature data to a nice pixel colour.
 * 
 * Returns a 16bit RGB colour unsigned int.
 */
unsigned int dataToColour(int data, int temperature_range) {
    //convert to RGB
    //get first 11 bits (positive temperature)
    unsigned int colour_temp = (data & 0x7FF);
    unsigned char red = 0;
    unsigned char green = 0;
    unsigned char blue = 0;

    if(data & 0x800)
        blue = colour_temp * 2;
    if(colour_temp < temperature_range)
        green = colour_temp*2;
    if(colour_temp >= temperature_range) {
        red = colour_temp*2;
        green = colour_temp*2;
    }
    if((colour_temp * 2) > 0xFF) {
        red = colour_temp;
        green = 0;
    }
    
    //Convert to RGB for the display
    return get_colour(red, green, blue);
}

/*
 * Gets data from the IR sensor and stores it to the buffer
 * Need to provide a an array as a buffer.
 */
void getIRData(int buffer[]) {
    

    //Some RAM space
    unsigned char dataL;
    unsigned char dataH;
    int data;
    
    for(int x = 0; x < 8; x++) {
        for(int y = 0; y < 8; y++) {
            
            //Get the current pixel from the IR sensor
            unsigned int this_pixel = ((x) * 16) + ((y) * 2);
            dataL = i2c_receive_data(addr, this_pixel + start_pixel_reg);
            dataH = i2c_receive_data(addr, this_pixel + start_pixel_reg + 1);
            data = (dataH << 8) | dataL;
            //Add pixel to the buffer for later
            //Also inverts the horizontal axis
            buffer[((7-x) * 8) + y] = data;
        }
    }
}

/*
 * Draws the IR data buffer, the horizontal interpolated buffer, and the
 * vertical interpolated puffer to the display.
 * |-----|
 * |M|H|M|...
 * |-----|
 * |V|D|V|...
 * |-----|
 * |M|H|M|...
 * |-----|
 * 
 * Also draws the overlay.
 */
void drawBuffersToDisplay(int main_buffer[], int horizontal_buffer[], int vertical_buffer[], int diagonal_buffer[]) {
    //Temperature range for colour calculation
    int temperature_range = (int)((max_pixel.temperature - min_pixel.temperature) / 1)+min_pixel.temperature;
    
    //Reset min/max temperature values
    max_pixel.temperature = 0;
    min_pixel.temperature = 0x7FF;
    
    //Draw each pixel to the screen
    for(int x = 0; x < 8; x++) {
        for(int y = 0; y < 8; y++) {
            int data = main_buffer[((int)(x)*8)+y];
            int h_pixel = horizontal_buffer[((int)(x)*8)+y];
            int v_pixel = vertical_buffer[((int)(x)*8)+y];
            int d_pixel = diagonal_buffer[((int)(x)*8)+y];

            //Draw the current pixel
            int h = x * 2;
            int v = y * 2;
            fill_rectangle((h*box_size), (v*box_size), (h*box_size) + box_size, (v*box_size) + box_size, dataToColour(data, temperature_range));
            //Draw the interpoalted pixels
            fill_rectangle(((h+1)*box_size), (v*box_size), ((h+1)*box_size) + box_size, (v*box_size) + box_size, dataToColour(h_pixel, temperature_range));
            fill_rectangle((h*box_size), ((v+1)*box_size), (h*box_size) + box_size, ((v+1)*box_size) + box_size, dataToColour(v_pixel, temperature_range));
            fill_rectangle(((h+1)*box_size), ((v+1)*box_size), ((h+1)*box_size) + box_size, ((v+1)*box_size) + box_size, dataToColour(d_pixel, temperature_range));

            //Detect the maximum and minimum temperatures
            if(data > max_pixel.temperature) {
                max_pixel.x = x;
                max_pixel.y = y;
                max_pixel.temperature = data;
            }
            if(data < min_pixel.temperature) {
                min_pixel.x = x;
                min_pixel.y = y;
                min_pixel.temperature = data;
            }
        }
    }
    //Measure temperature of the maximum temperature pixel
    drawOverlay(max_pixel.x, max_pixel.y, max_pixel.temperature);
}

/*
 * Calculate three interpolated pixel buffers. X, Y, and diagonal pixels
 */
void calculateInterpolatedPixels(int main_buffer[], int y_interpolated_buffer[], int x_interpolated_buffer[], int xy_interpolated_buffer[]) {

    //Calculate interpolated pixels
    for(int x = 0; x < 8; x++) {
        for(int y = 0; y < 8; y++) {
            //If it's not an edge case then calculate the Y average
            if(y>0)
                y_interpolated_buffer[(x * 8) + y] = (int)((main_buffer[(x * 8) + y-1] + main_buffer[(x * 8) + y]) / 2);
            //If it's not an edge case then calculate the X average
            if(x>0)
                x_interpolated_buffer[(x * 8) + y] = (int)((main_buffer[((x-1) * 8) + y] + main_buffer[(x * 8) + y]) / 2);
            if((x < 8) && (y < 8)) {
                xy_interpolated_buffer[(x * 8) + y] = (int)((main_buffer[(x * 8) + y] + main_buffer[((x+1) * 8) + y + 1]) / 2);
            }
        }
    }
}

void main(void) {
    
    //Change internal oscillator
    OSCTUNE = 0b01000000;
    OSCCON = 0b01110000; //Set to 48MHz mode
    
    //SET Port Input / Output latches
    TRISC2 = 0; //LED Output
    LED = 1; //Boot test LED
    
    //Begin SPI and I2C communciation
    initSPI();
    initI2C();
    //Battery monitoring
    initADC();
    
    //Boot up delay
    delay_ms(500);
    
    
    //LCD initialisation routine
    lcd_init();
    //Initialise the AMG88xx IR sensor
    AMGInit();
    //LED off
    LED = 0;
    //more delay why not
    delay_ms(500);
    //Blank out the LCD
    fill_rectangle(0, 0, 160, 160, BG_COLOUR);
    //Boot finished - LED ON
    LED = 1;
    
    
    //Main routine
    while(1) {
        //Get all data form the IR sensor to the buffer
        getIRData(ir_buffer);
        
        //Calcualte the interpoalted pixel arrays
        calculateInterpolatedPixels(ir_buffer, y_interpolated_pixels, x_interpolated_pixels, xy_interpolated_pixels);
        
        //Draw the buffers to the display
        drawBuffersToDisplay(ir_buffer, y_interpolated_pixels, x_interpolated_pixels, xy_interpolated_pixels);
 
    }
    
    while(1); //fail-safe halt point
    return;
}

