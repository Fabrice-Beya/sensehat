/*****************************************************************************
* | File      	:   DEV_Config.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*----------------
* |	This version:   V2.0
* | Date        :   2019-07-08
* | Info        :   Basic version
*
******************************************************************************/
#include "DEV_Config.h"
#include <unistd.h>
#include <fcntl.h>

char *UART_Device;
#if USE_DEV_LIB
int GPIO_Handle;
int SPI_Handle;
int I2C_Handle;
int UART_Handle;
pthread_t *p1;
UWORD pwm_dule=100; // 1040
void *BL_PWM(void *arg){
	UWORD i=0;
	for(i=0;;i++){
		if(i>64)i=0;
        //printf("%s\n", (char *)arg);
        lguSleep(0.001);
		if(i<(pwm_dule/16))lgGpioWrite(GPIO_Handle, 18, LG_HIGH);
		else lgGpioWrite(GPIO_Handle, 18, LG_LOW);	
    }
	
}
#else
    uint32_t fd,UART_fd;
#endif

/******************************************************************************
function:	Equipment Testing
parameter:
Info:   Only supports Jetson Nano and Raspberry Pi
******************************************************************************/
static int DEV_Equipment_Testing(void)
{
	int i;
	int fd;
	char value_str[20];
	fd = open("/etc/issue", O_RDONLY);
    printf("Current environment: ");
	while(1) {
		if (fd < 0) {
			return -1;
		}
		for(i=0;; i++) {
			if (read(fd, &value_str[i], 1) < 0) {
				return -1;
			}
			if(value_str[i] ==32) {
				printf("\r\n");
				break;
			}
			printf("%c",value_str[i]);
		}
		break;
	}

	if(i<5) {
		printf("Unrecognizable\r\n");
        return -1;
	} else {
		char RPI_System[10]   = {"Raspbian"};
		for(i=0; i<6; i++) {
			if(RPI_System[i] != value_str[i]) {
                #if USE_DEV_LIB    
                    return 'J';
                #endif
                return -1;
			}
		}
        return 'R';
	}
	return -1;
}


/******************************************************************************
function:	GPIO Function initialization and transfer
parameter:
Info:
******************************************************************************/
void DEV_GPIO_Mode(uint16_t Pin, uint16_t Mode)
{
    /*
        0:  INPT   
        1:  OUTP
    */
#ifdef USE_BCM2835_LIB  
    if(Mode == 0 || Mode == BCM2835_GPIO_FSEL_INPT){
        bcm2835_gpio_fsel(Pin, BCM2835_GPIO_FSEL_INPT);
    }else {
        bcm2835_gpio_fsel(Pin, BCM2835_GPIO_FSEL_OUTP);
    }
#elif USE_WIRINGPI_LIB
    if(Mode == 0 || Mode == INPUT){
        pinMode(Pin, INPUT);
        pullUpDnControl(Pin, PUD_UP);
    }else{ 
        pinMode(Pin, OUTPUT);
    }
#elif USE_DEV_LIB
   if(Mode == 0 || Mode == LG_SET_INPUT){
        lgGpioClaimInput(GPIO_Handle,LFLAGS,Pin);
        // printf("IN Pin = %d\r\n",Pin);
    }else{
        lgGpioClaimOutput(GPIO_Handle, LFLAGS, Pin, LG_LOW);
        // printf("OUT Pin = %d\r\n",Pin);
    }
#endif   
}

void DEV_Digital_Write(uint16_t Pin, uint8_t Value)
{
#ifdef USE_BCM2835_LIB
    bcm2835_gpio_write(Pin, Value);
    
#elif USE_WIRINGPI_LIB
    digitalWrite(Pin, Value);
    
#elif USE_DEV_LIB
    lgGpioWrite(GPIO_Handle, Pin, Value);
    
#endif
}

uint8_t DEV_Digital_Read(uint16_t Pin)
{
    uint8_t Read_value = 0;
#ifdef USE_BCM2835_LIB
    Read_value = bcm2835_gpio_lev(Pin);
    
#elif USE_WIRINGPI_LIB
    Read_value = digitalRead(Pin);
    
#elif USE_DEV_LIB
    Read_value = lgGpioRead(GPIO_Handle,Pin);
#endif
    return Read_value;
}


/**
 * delay x ms
**/
void DEV_Delay_ms(UDOUBLE xms)
{
#ifdef USE_BCM2835_LIB
    bcm2835_delay(xms);
#elif USE_WIRINGPI_LIB
    delay(xms);
#elif USE_DEV_LIB
    lguSleep(xms/1000.0);
#endif
}


void GPIO_Config(void)
{
    int Equipment = DEV_Equipment_Testing();
    if(Equipment=='R'){
        /************************
        Raspberry Pi GPIO
        ***********************/

        
    }else if(Equipment=='J'){
        #if USE_DEV_LIB
        /************************
        Jetson Nano GPIO
        ***********************/

        
        #endif
    }else{
        printf("Device read failed or unrecognized!!!\r\n");
        while(1);
    }

}

/******************************************************************************
function:	SPI Function initialization and transfer
parameter:
Info:
******************************************************************************/
void DEV_SPI_Init()
{
#if DEV_SPI
    #ifdef USE_BCM2835_LIB
        printf("BCM2835 SPI Device\r\n");  
        bcm2835_spi_begin();                                         //Start spi interface, set spi pin for the reuse function
        bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);     //High first transmission
        bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                  //spi mode 0
        bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);  //Frequency
        bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                     //set CE0
        bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);     //enable cs0
        
    #elif USE_WIRINGPI_LIB
        printf("WIRINGPI SPI Device\r\n");       
        //wiringPiSPISetup(0,9000000);
        wiringPiSPISetupMode(0, 9000000, 0);
        
    #elif USE_DEV_LIB
        printf("DEV SPI Device\r\n"); 
        SPI_Handle = lgSpiOpen(0, 0, 10000000, 0);
    #endif
#endif
}

void DEV_SPI_WriteByte(uint8_t Value)
{
#if DEV_SPI
    #ifdef USE_BCM2835_LIB
        bcm2835_spi_transfer(Value);
        
    #elif USE_WIRINGPI_LIB
        wiringPiSPIDataRW(0,&Value,1);
        
    #elif USE_DEV_LIB
        lgSpiWrite(SPI_Handle,(char*)&Value, 1);
    #endif
#endif
}

void DEV_SPI_Write_nByte(uint8_t *pData, uint32_t Len)
{
#if DEV_SPI
    #ifdef USE_BCM2835_LIB
        uint8_t rData[Len];
        bcm2835_spi_transfernb(pData,rData,Len);
        
    #elif USE_WIRINGPI_LIB
        wiringPiSPIDataRW(0, pData, Len);
        
    #elif USE_DEV_LIB
        lgSpiWrite(SPI_Handle,(char*) pData, Len);
        
    #endif
#endif
}
/******************************************************************************
function:	I2C Function initialization and transfer
parameter:
Info:
******************************************************************************/
void DEV_I2C_Init(uint8_t Add)
{
#if DEV_I2C
    #ifdef USE_BCM2835_LIB
        printf("BCM2835 I2C Device\r\n");  
        bcm2835_i2c_begin();
        bcm2835_i2c_setSlaveAddress(Add);
        
    #elif USE_WIRINGPI_LIB
        printf("WIRINGPI I2C Device\r\n");       
        fd = wiringPiI2CSetup(Add);
        
    #elif USE_DEV_LIB
        printf("DEV I2C Device\r\n"); 
        I2C_Handle = lgI2cOpen(1, Add, 0);
    #endif
#endif
}

void DEV_I2C_Set_SlaveAddress(uint8_t Add)
{
#if DEV_I2C
    #ifdef USE_BCM2835_LIB
        bcm2835_i2c_setSlaveAddress(Add);
        
    #elif USE_WIRINGPI_LIB 
        fd = wiringPiI2CSetup(Add);
        
    #elif USE_DEV_LIB
        I2C_Handle = lgI2cOpen(1, Add, 0);
    #endif
#endif
}

void I2C_Write_Byte(uint8_t Cmd, uint8_t value)
{
	int ref;
#if DEV_I2C
    #ifdef USE_BCM2835_LIB
        char wbuf[2]={Cmd, value};
        bcm2835_i2c_write(wbuf, 2);
    #elif USE_WIRINGPI_LIB
        //wiringPiI2CWrite(fd,Cmd);
        ref = wiringPiI2CWriteReg8(fd, (int)Cmd, (int)value);
        while(ref != 0) {
            ref = wiringPiI2CWriteReg8 (fd, (int)Cmd, (int)value);
            if(ref == 0)
                break;
        }
    #elif USE_DEV_LIB
        lgI2cWriteByteData(I2C_Handle,Cmd,value);

    #endif
#endif
}

void I2C_Write_Word(uint8_t Cmd, uint16_t value)
{
	int ref;
#if DEV_I2C
    #ifdef USE_BCM2835_LIB
        char wbuf[3]={Cmd, (value>>8),(value & 0xff)};
        bcm2835_i2c_write(wbuf, 3);
    #elif USE_WIRINGPI_LIB
        //wiringPiI2CWrite(fd,Cmd);
        // uint16_t wbuf=(value & 0xff) << 8 | (value>>8);
        ref = wiringPiI2CWriteReg16(fd, (int)Cmd, (int)value);
        while(ref != 0) {
            ref = wiringPiI2CWriteReg16 (fd, (int)Cmd, (int)value);
            if(ref == 0)
                break;
        }
    #elif USE_DEV_LIB
        lgI2cWriteWordData(I2C_Handle,Cmd,value);

    #endif
#endif
}

void I2C_Write_NByte(uint8_t *Cmd, uint16_t num)
{
#if DEV_I2C
    #ifdef USE_BCM2835_LIB
        bcm2835_i2c_write(Cmd, num);
    #elif USE_WIRINGPI_LIB
        printf("请使用USE_BCM2835_LIB或者USE_DEV_LIB\n");
        exit(0);
    #elif USE_DEV_LIB
        lgI2cWriteDevice(I2C_Handle, Cmd, num);
    #endif
#endif
}

int I2C_Read_Byte(uint8_t Cmd)
{
	int ref;
#if DEV_I2C
    #ifdef USE_BCM2835_LIB
        char rbuf[2]={0};
        bcm2835_i2c_read_register_rs(&Cmd, rbuf, 1);
        ref = rbuf[0];
        
    #elif USE_WIRINGPI_LIB
        ref = wiringPiI2CReadReg8 (fd, (int)Cmd);
        
    #elif USE_DEV_LIB
        ref = lgI2cReadByteData(I2C_Handle,Cmd);
    #endif
#endif
    return ref;
}

int I2C_Read_Word(uint8_t Cmd)
{
	int ref;
#if DEV_I2C
    #ifdef USE_BCM2835_LIB
        char rbuf[2] = {0};
        bcm2835_i2c_read_register_rs(&Cmd, rbuf, 2);
        ref = rbuf[1]<<8 | rbuf[0];
        
    #elif USE_WIRINGPI_LIB
        ref = wiringPiI2CReadReg16 (fd, (int)Cmd);
        
    #elif USE_DEV_LIB
        ref = lgI2cReadWordData(I2C_Handle,Cmd);;
    #endif
#endif
    return ref;
}

void I2C_Read_NByte(uint8_t Cmd, uint8_t *rbuf, uint8_t num)
{
#if DEV_I2C
    #ifdef USE_BCM2835_LIB
        bcm2835_i2c_read_register_rs(&Cmd, rbuf, num);
    #elif USE_WIRINGPI_LIB
        printf("请使用USE_BCM2835_LIB或者USE_DEV_LIB\n");
        exit(0);
    #elif USE_DEV_LIB
        lgI2cReadI2CBlockData(I2C_Handle, Cmd, rbuf, num);
    #endif
#endif
}

/******************************************************************************
function:	SPI Function initialization and transfer
parameter:
Info:
******************************************************************************/
void DEV_UART_Init(char *Device)
{
    UART_Device = Device;
#if DEV_UART
    #ifdef USE_BCM2835_LIB
        printf("BCM2835 Not Supported UART!!! \r\n");
        
    #elif USE_WIRINGPI_LIB
        if((UART_fd = serialOpen(UART_Device,115200)) < 0){
            printf("set uart failed  !!! \r\n");
            return ;
        }else {
            printf("set uart success  !!! \r\n");
        }
    #elif USE_DEV_LIB
        UART_Handle = lgSerialOpen(UART_Device,115200,0);

    #endif
    UART_Set_Baudrate(115200);
#endif
}

void UART_Write_Byte(uint8_t data)
{
#if DEV_UART
    #ifdef USE_BCM2835_LIB
        printf("BCM2835 Not Supported UART!!! \r\n");
        
    #elif USE_WIRINGPI_LIB
        serialPutchar(UART_fd, data);
        
    #elif USE_DEV_LIB
        lgSerialWrite(UART_Handle,&data,1);
    #endif
#endif
}

int UART_Read_Byte(void)
{
    int ref;
#if DEV_UART
    #ifdef USE_BCM2835_LIB
        printf("BCM2835 Not Supported UART!!! \r\n");
        
    #elif USE_WIRINGPI_LIB
        ref = serialGetchar(UART_fd);
        
    #elif USE_DEV_LIB
        lgSerialRead(UART_Handle,ref,1);
    #endif
#endif
    return ref;
}

void UART_Set_Baudrate(uint32_t Baudrate)
{
#if DEV_UART
    #ifdef USE_BCM2835_LIB
        printf("BCM2835 Not Supported UART!!! \r\n");
        
    #elif USE_WIRINGPI_LIB
        serialClose(UART_fd);
        if((UART_fd = serialOpen(UART_Device, Baudrate)) < 0){
            printf("set uart failed	!!! \r\n");
        }else{
            printf("set uart success  !!! \r\n");
        }
        
    #elif USE_DEV_LIB
        UART_Handle = lgSerialOpen(UART_Device,Baudrate,0);
    #endif
#endif
 
}

int UART_Write_nByte(uint8_t *pData, uint32_t Lan)
{
#if DEV_UART
    #ifdef USE_BCM2835_LIB
        printf("BCM2835 Not Supported UART!!! \r\n");
        
    #elif USE_WIRINGPI_LIB
        for(uint32_t i=0; i<Lan; i++){
            serialPutchar(UART_fd, *(pData+i));
        }
        
    #elif USE_DEV_LIB
        lgSerialWrite(UART_Handle,pData,Lan);
    #endif
#endif 
}
/******************************************************************************
function:	Module Initialize, the library and initialize the pins, SPI protocol
parameter:
Info:
******************************************************************************/
uint8_t DEV_ModuleInit(void)
{
 #ifdef USE_BCM2835_LIB
    if(!bcm2835_init()) {
        printf("bcm2835 init failed  !!! \r\n");
        return 1;
    } else {
        printf("bcm2835 init success !!! \r\n");
    }

#elif USE_WIRINGPI_LIB  
    //if(wiringPiSetup() < 0)//use wiringpi Pin number table  
    if(wiringPiSetupGpio() < 0) { //use BCM2835 Pin number table
        printf("set wiringPi lib failed	!!! \r\n");
        return 1;
    } else {
        printf("set wiringPi lib success  !!! \r\n");
    }

#elif USE_DEV_LIB
    printf("USE_DEV_LIB \r\n");
    char buffer[NUM_MAXBUF];
    FILE *fp;

    fp = popen("cat /proc/cpuinfo | grep 'Raspberry Pi 5'", "r");
    if (fp == NULL) {
        DEBUG("It is not possible to determine the model of the Raspberry PI\n");
        return -1;
    }

    if(fgets(buffer, sizeof(buffer), fp) != NULL)  
    {
        GPIO_Handle = lgGpiochipOpen(4);
        if (GPIO_Handle < 0)
        {
            DEBUG( "gpiochip4 Export Failed\n");
            return -1;
        }
    }
    else
    {
        GPIO_Handle = lgGpiochipOpen(0);
        if (GPIO_Handle < 0)
        {
            DEBUG( "gpiochip0 Export Failed\n");
            return -1;
        }
    }
#endif
    GPIO_Config();
    DEV_I2C_Init(0x68);

    
    return 0;
}

/******************************************************************************
function:	Module exits, closes SPI and BCM2835 library
parameter:
Info:
******************************************************************************/
void DEV_ModuleExit(void)
{
#ifdef USE_BCM2835_LIB
    #if DEV_I2C
        bcm2835_i2c_end();
    #endif
    #if DEV_SPI
        bcm2835_spi_end();
    #endif
    bcm2835_close();
    
#elif USE_WIRINGPI_LIB
    #if DEV_UART
        serialFlush(UART_fd);
        serialClose(UART_fd);
    #endif
    
#elif USE_DEV_LIB
    
#endif
}

