#include <iostream>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <string.h>
#include <wiringPi.h>

#include <linux/i2c.h>

#include "i2cBitBangingBus.h"
#include <memory>
#include <stdlib.h>
//#include "fw.h"
//#include "max32664b_fw.h" //versio b firmware
//#include "max32664C_Z_fw.h" //C version fw for Z version IC 
#include "max32664C_fw.h"

#define MAX32664_I2C_ADDR 0x55
#define RESET_PIN 0
#define MFIO_PIN 2
#define ERASE_DELAY 1500
#define PAGE_WRITE_DELAY 350
#define PAGE_1_START_INDEX 0x4C
#define PAGE_PAYLOAD_SIZE 8208
#define PAGE_2_START_INDEX 0x205C
#define PAGE_WRITE_SIZE 0x2010

using namespace std;

//global variables
int file_i2c;
char *filename = (char*)"/dev/i2c-1";

//global objects
i2cBitBangingBus bus(8,9,0,200);


//~ uint8_t array_fw[229904];

//peripheral setup function prototypes
int gpio_init(void);
int i2c_bus_init(void);

//max32664 function prototypes
int max32664_i2c_write_read(uint8_t* w_buff, uint16_t w_size, uint8_t* r_buff, uint16_t r_size, uint32_t delay_ms);
int max32664_enter_bootloader_mode(void);
int max32664_enter_application_mode(void);
void print_buffer(uint8_t* buff, uint32_t size);
int max32664_enter_bootloader_mode_cmd(void);
int max32664_enter_application_mode_cmd(void);
int max32664_read_fw_version(void);
int max32664_read_page_size(void);
int max32664_read_device_mode(void);
int max32664_get_id_and_mcu_type(void);
int max32664_set_number_of_pages(void);
int max32664_bootloader_set_init_vector(void);
int max32664_bootloader_set_aut_bytes(void);
int max32664_bootloader_erase_application(void);
int max32664_write_fw_page(uint32_t start_index);
int max32664_write_fw(void);

int main(){
	int err;
	FILE *file_ptr;
	
	cout<< "Hello PI"<<endl;
		
	//~ file_ptr = fopen("MAX32664C_HSP2_WHRM_AEC_SCD_WSPO2_C_30.7.0.msbl", "r" );
	//~ if(file_ptr == NULL){
		//~ cout<< "could not open hex file"<<endl;
		//~ return 0;
	//~ }
	
	//~ err = fread(array_fw,sizeof(uint8_t),229904, file_ptr);
	//~ cout<<"fread returned: "<<err<<endl;
	
	printf("number of pages %x\n",array_fw[0x44]);
	
	gpio_init();
	i2c_bus_init();
	
	max32664_enter_bootloader_mode(); 
	
    //while(1){
		max32664_enter_bootloader_mode_cmd();
		delay(100);
		max32664_read_device_mode();
		delay(100);
		max32664_read_device_mode();//max32664_get_id_and_mcu_type();
		delay(100);
		max32664_read_fw_version();
		delay(100);
		max32664_read_page_size();
		delay(100);
		
		max32664_set_number_of_pages();
		delay(10);
		max32664_bootloader_set_init_vector();
		delay(10);
		max32664_bootloader_set_aut_bytes();
		delay(10);
		max32664_bootloader_erase_application();
		
		delay(250);
		max32664_write_fw();
		//max32664_write_fw_page(PAGE_1_START_INDEX);
		//delay(2500);
		//max32664_write_fw_page(PAGE_2_START_INDEX);
		
		delay(100);
		max32664_enter_application_mode_cmd();
		delay(3000);
		max32664_read_device_mode();
	
	//}
		 
	return 0;
}

int gpio_init(void){
		wiringPiSetup();
		pinMode(RESET_PIN,OUTPUT);
		pinMode(MFIO_PIN,OUTPUT);
		
		digitalWrite(RESET_PIN,1);
		digitalWrite(MFIO_PIN,0);
		
		return 0;
}

int i2c_bus_init(void){
	//~ if( (file_i2c = open(filename,O_RDint max32664_get_id_and_mcu_type(void);WR)) < 0){
			//~ cout<< "error:: failed to open i2c bus"<<endl;
			//~ return -10;
	//~ }
	
	//~ int addr = MAX32664_I2C_ADDR;
	//~ if (ioctl(file_i2c, I2C_SLAVE, addr) < 0){
		//~ cout<<"Failed to acquire bus access and/or talk to slave"<<endl;
		//~ //ERROR HANDLING; you can check errno to see what went wrong
		//~ return -10;
	//~ }
	
	//~ cout<<"got I2C!!!"<<endl;
	
	
	//bus = make_shared<i2cBitBangingBus>(8,9,0);
	
	return 0;
}

int max32664_i2c_write_read(uint8_t* w_buff, uint16_t w_size, uint8_t* r_buff, uint16_t r_size, uint32_t delay_ms){
	int err;
	int max32664_get_id_and_mcu_type(void);
	//err = write(file_i2c,w_buff, w_size) ;
	//bus.i2c_smbus_write_i2c_block_data(MAX32664_I2C_ADDR, w_buff[0], w_size-1, w_buff+1);
	err = bus.i2c_smbus_write_i2c_block_data_no_command(MAX32664_I2C_ADDR, w_size, w_buff);
	if(err != 0){ //used to be w_size -- changed to 0
			cout<<"error:: in i2c_write "<<err<<endl;
			return -10;
	}
	int max32664_get_id_and_mcu_type(void);
	delay(delay_ms);
	
	//err = read(file_i2c, r_buff, r_size);
	err = bus.i2c_smbus_read_i2c_block_data_no_command(MAX32664_I2C_ADDR, r_size, r_buff);
	if(err != r_size){

			cout<<"error:: in i2c_read:: "<<err<<endl;
			return -10;
	}
	
	if(r_buff[0] == 0){
		return 0;
	}
	else{
		printf("error:: status not ZERO:: %x\n",r_buff[0]);
		//cout<<"error: status not ZERO:: "<<r_buff[0]<<endl;
		return -5;
	}
	
	return -5;
}


int max32664_enter_bootloader_mode(void){
	
	digitalWrite(RESET_PIN,0);
	digitalWrite(MFIO_PIN,0);
	delay(10);
	
	digitalWrite(RESET_PIN,1);
	
	delay(50);
	
	return 0; 
}
	
int max32664_enter_application_mode(void){
	
	digitalWrite(RESET_PIN,0);
	digitalWrite(MFIO_PIN,1);
	delay(10);
	
	digitalWrite(RESET_PIN,1);
	
	delay(1060);
	
	return 0;

}

void print_buffer(uint8_t* buff, uint32_t size){

	printf("BUFFER: ");
	for(int i=0; i<size; i++){
		printf("%x ",buff[i]);
	}
	printf("\n");
}

int max32664_enter_bootloader_mode_cmd(void){

	uint8_t write_buff[3] ={0x01,0x00,0x08};
	uint8_t read_buff[1];

	if(max32664_i2c_write_read( write_buff,sizeof(write_buff), read_buff, sizeof(read_buff), 60) == 0){
			printf("entered bootloader mode, maybe?\n");
			return 0;
	}
	else{
			printf("error: could not enter_bootloader_mode_cmd!\n");
			return -5;
	}
}

int max32664_enter_application_mode_cmd(void){

	uint8_t write_buff[3] ={0x01,0x00,0x00};
	uint8_t read_buff[1];

	if(max32664_i2c_write_read( write_buff,sizeof(write_buff), read_buff, sizeof(read_buff), 1060) == 0){
			printf("entered application mode, maybe?\n");
			return 0;
	}
	else{
			printf("error: could not enter_application_mode_cmd!\n");
			return -5;
	}
}


int max32664_read_fw_version(void){

	uint8_t write_buff[3] ={0x81,0x00,0x00};
	uint8_t read_buff[4];

	if(max32664_i2c_write_read( write_buff,sizeof(write_buff), read_buff, sizeof(read_buff), 4) == 0){
			printf("fw version: %d.%d.%d\n",read_buff[1],read_buff[2],read_buff[3]);
			return 0;

	}
	else{
			printf("error:: could not get fw version!\n");
			return -5;
	}
}

int max32664_read_page_size(void){

	uint8_t write_buff[2] = {0x81,0x01};
	uint8_t read_buff[3];

	if(max32664_i2c_write_read( write_buff, sizeof(write_buff), read_buff, sizeof(read_buff), 4) == 0){
			printf("page size: %d\n",(read_buff[1]<<8)|read_buff[2]);
			return 0;                               

	}
	else{
			printf("error:: could not get page size version!\n");
			return -5;

	}		
}

int max32664_read_device_mode(void){
	
	uint8_t write_buff[2] = {0x02,0x00};
	uint8_t read_buff[2];

	if(max32664_i2c_write_read(write_buff,2,read_buff,2,4) == 0){
			printf("MODE: %x\n",read_buff[1]);
			return 0;
	}
	else{
			printf("error:: could not get MODE\n");
			return -5;
	}
}

int max32664_get_id_and_mcu_type(void){
	uint8_t write_buff[2] = {0xFF,0x00};
	uint8_t read_buff[2];

	if(max32664_i2c_write_read(write_buff,sizeof(write_buff),read_buff,sizeof(read_buff),4) == 0){
			printf("ID and MCU type: %x\n",read_buff[1]);
			return 0;
	}
	else{
			printf("error:: could not get ID and MCU type\n");
			return -5;
	}
}

int max32664_set_number_of_pages(void){
	uint8_t write_buff[4] = {0x80,0x02,0x00,array_fw[0x44]};
	uint8_t read_buff[1];

	if(max32664_i2c_write_read(write_buff,sizeof(write_buff),read_buff,sizeof(read_buff),60) == 0){
			printf("WROTE NUMBER OF PAGES: %x\n",array_fw[0x44]);
			return 0;
	}
	else{
			printf("error:: could not write number of pages\n");
			return -5;
	}
}

int max32664_bootloader_set_init_vector(void){
	uint8_t write_buff[13];
	uint8_t read_buff[1];

	write_buff[0] = 0x80;
	write_buff[1] = 0x00;
	memcpy(&write_buff[2],&array_fw[0x28],11);
	//print_buffer(write_buff, sizeof(write_buff));
	if(max32664_i2c_write_read(write_buff,sizeof(write_buff), read_buff, sizeof(read_buff), 60) == 0){
		printf("SET INIT VECTOR\n");
		return 0;

	}
	else{
		printf("error:: could not set init vector\n");
		return -5;
	}
}


int max32664_bootloader_set_aut_bytes(void){
	uint8_t write_buff[18];
	uint8_t read_buff[1];

	write_buff[0] = 0x80;
	write_buff[1] = 0x01;
	memcpy(&write_buff[2],&array_fw[0x34],18);
	//print_buffer(write_buff, sizeof(write_buff));
	if(max32664_i2c_write_read(write_buff,sizeof(write_buff), read_buff, sizeof(read_buff), 60) == 0){
		printf("SET AUTHENTICATION BYTES\n");
		return 0;
	}
	else{
		printf("error:: could not set authentication bytes\n");
		return -5;
	}
}


int max32664_bootloader_erase_application(void){
	uint8_t write_buff[2] = {0x80, 0x03};
	uint8_t read_buff[1];
	
	if(max32664_i2c_write_read(write_buff,sizeof(write_buff),read_buff,sizeof(read_buff),ERASE_DELAY) == 0){
		printf("ERASE SUCCESFULL!\n");
		return 0;
	}
	else{
		printf("error:: could not erase, got error %x",read_buff[0]);
		return -5;
	}
}

int max32664_write_fw_page(uint32_t start_index){
	uint8_t write_buff[8210];
	uint8_t read_buff[1];
	
	write_buff[0] = 0x80;
	write_buff[1] = 0x04;
	memcpy( &write_buff[2], &array_fw[start_index], PAGE_PAYLOAD_SIZE);
	//print_buffer(write_buff, sizeof(write_buff));
	if(max32664_i2c_write_read(write_buff,sizeof(write_buff), read_buff,sizeof(read_buff), PAGE_WRITE_DELAY) == 0){
		printf("WROTE FW PAGE!");
		return 0;
	}
	else{
		printf("error:: could not write fw page:: start index %x ::error code %x\n",start_index,read_buff[0]);
		return -5;
	}

}
//int max32664_write_fw(void);

int max32664_write_fw(void){
	int write_index = PAGE_1_START_INDEX;
	int counter=0;
	
	for(int i=0; i<array_fw[0x44];i++){
		delay(500);
		if(max32664_write_fw_page(write_index)!=0){
			printf("ERROR:: in max32664_write_fw\n");
			return -5;
		}
		write_index+=PAGE_WRITE_SIZE;
		counter++;
	}
	
	printf("WROTE ALL %d FW PAGES\n",counter);
	return 0;
}
