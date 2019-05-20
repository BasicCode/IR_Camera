#ifndef AMGXX_H
#define	AMGXX_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    //Some AMG88xx commands
    #define AMG88xx_ADDR    0xD2 //the I2C address by default
    //Some AMG Registers
    #define AMG88xx_PCTL    0x00
    #define AMG88xx_RST     0x01
    #define AMG88xx_FPSC    0x02
    #define AMG88xx_TTHL    0x0E //Thermistor
    //Some AMG commands
    #define AMG88xx_MODE    0x00 //Normal operating mode
    #define AMG88xx_REFRESH 0x00 //10Hz refresh rate
    #define AMG88xx_RESET   0x3F //Software reset
    
    void AMGInit();
    void i2c_send_data(unsigned char address, unsigned char reg, unsigned char data);
    void i2c_send_request(unsigned char address, unsigned char reg);
    unsigned char i2c_receive_data(unsigned char address, unsigned char reg);
    void i2c_master_wait();

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

