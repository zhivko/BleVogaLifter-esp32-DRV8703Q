/*
 * FDC2212.cpp
 *
 *  Created on: 26 Nov 2017
 *      Author: klemen
 */

#include "FDC2212.h"

FDC2212* FDC2212::instance = 0;

FDC2212* FDC2212::getInstance() {
	if (!instance) {
		instance = new FDC2212();
	}
	return instance;
}

FDC2212::FDC2212() {
	_i2caddr = (FDC2214_I2C_ADDRESS << 1);
	initialized = false;
	capMin = ULONG_MAX;
	capMax = 0;
	isReading = false;
	shouldread = false;
	lastReadingTick = 0;
	lastReading = 0;
	dCap_dT = 0;
	angle = 0;
}

FDC2212::FDC2212(i2c_cmd_handle_t i2cHandle) {
	instance = FDC2212::getInstance();
	instance->_i2cHandle = i2cHandle;
}

/**************************************************************************/
/*!
 @brief  Setups the HW
 */
/**************************************************************************/
bool FDC2212::begin(void) {
	//Wire.begin();

    i2c_port_t i2c_master_port = I2C_NUM_1;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);

	uint8_t aRxBuffer[2];
	uint16_t REG_CHIP_MEM_ADDR = 0x7e;

    /*
	if (HAL_I2C_Mem_Read(&this->instance->_i2cHandle, this->instance->_i2caddr, REG_CHIP_MEM_ADDR, I2C_MEMADD_SIZE_8BIT, aRxBuffer, 2, 10000) != HAL_OK) {
		if (HAL_I2C_GetError(&this->instance->_i2cHandle) != HAL_I2C_ERROR_AF) {
			_Error_Handler(__FILE__, __LINE__);
		}
	}
    */
    esp_err_t err = this->i2c_example_master_read_slave(I2C_EXAMPLE_MASTER_NUM, REG_CHIP_MEM_ADDR, aRxBuffer, 2);
    if(err != ESP_OK)
    {
        Serial.println("Error in i2c read slave.");
    }


	uint16_t manufacturer = aRxBuffer[0] << 8;
	manufacturer |= aRxBuffer[1];   // returns: manufacturer = 0x5449;

	int devId = read16FDC(FDC2212_DEVICE_ID_REGADDR);
	if (devId != 0x3055) {
        Serial.println("Wrong i2c device.");
		//two valid device ids for FDC2214 0x3054 and 0x3055
		return false;
	}
    Serial.println("i2c device FDC2212 present.");

    Serial.println("FDC2212 loadsettings.");
	loadSettings();
    Serial.println("FDC2212 setgain.");
	setGain();
    Serial.println("Initialized.");
	initialized = true;

	return true;
}

/**************************************************************************/
/*!
 @brief  Initalizes FDC2214
 */
/**************************************************************************/
void FDC2212::loadSettings(void) {

	//reset device
	write16FDC(FDC2212_RESET_DEV, 0b1000000000000000);  //reset device

	//0b00  00 0001                      RESERVED
	//0b0   0 00 0001                    Normal current drive (auto scan is enabled)
	//0b0   0 0 00 0001                  INTB_DIS - Do NOT disable interrupt pin
	//0b0   0 0 0 00 0001                RESERVED
	//0b0   0 0 0 0 00 0001              Use internal oscilator
	//0b1   1 0 0 0 0 00 0001            RESERVED
	//0b0   0 1 0 0 0 0 00 0001          full current mode
	//0b1   1 0 1 0 0 0 0 00 0001        RESERVED
	//0b0   0 1 0 1 0 0 0 0 00 0001      device is active - no sleep
	//0b00 00 0 1 0 1 0 0 0 0 00 0001    Contineous reads on CH0 CONFIG.ACTIVE_CHAN

	//FDC2212_CONFIG_REGADDR              0x1A
	write16FDC(FDC2212_CONFIG_REGADDR, 0b0001010000000001);  //set config
	//write16FDC(FDC2212_CONFIG_REGADDR, 0x1E81);  //set config
//    write16FDC(FDC2214_CONFIG_REGADDR, 0x201);  //set config

	//settle count maximized, slow application
	write16FDC(FDC2212_SETTLECOUNT_CH0_REGADDR, 0x64);

	//rcount maximized for highest accuracy
	write16FDC(FDC2212_RCOUNT_CH0_REGADDR, 0xFFFF);

	//no offset
	write16FDC(FDC2212_OFFSET_CH0_REGADDR, 0x0000);

	//set clock divider
	//0b00 0000 0001                                    Set clock div to 1
	//0b00 [00 0000 0001]                               RESERVED
	//0b01 [10] [00 0000 0001]                          divide by 2
	//0b01 0b100000000001
	//write16FDC(FDC2212_CLOCK_DIVIDERS_CH0_REGADDR, 0x1001);
	write16FDC(FDC2212_CLOCK_DIVIDERS_CH0_REGADDR, 0b100000000001);

	//set drive register
	//write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0xF800);
	//write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0b0111100000000000);
	//write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0b0010100000000000);
	write16FDC(FDC2212_DRIVE_CH0_REGADDR, 0b0111100000000000);

	// 0b 										101 					Deglitch 10MHz, oscilation is ~4MHz by default
	// 0b 		 	 	 0001000001 101           RESERVED
	// 0b00 		00 0001000001 101        		Ch0, Ch1 Sequence unused only read on CH0 using CONFIG.ACTIVE_CHAN
	// 0b0    0 00 0001000001 101        		Continuous conversion on the single channel selected by CONFIG.ACTIVE_CHAN register field.

	write16FDC(FDC2212_MUX_CONFIG_REGADDR, 0b0000001000001101);  //set mux config for channels

	// set warnings
	// 0b1    											1										report data ready flag
	// 0b0000 								 0000 1										RESERVED
	// 0b1 									 1 0000 1										watchdog timeout error enable
	// 0b00000 				 00000 1 0000 1										RESERVED
	// 0b1 					 1 00000 1 0000 1										enable amplitude low warning
	// 0b1 				 1 1 00000 1 0000 1										enable amplitude high warning
	// 0b0 			 0 1 1 00000 1 0000 1										disable watchdog timeout error
	// 0b00 	00 0 1 1 00000 1 0000 1										RESERVED

	write16FDC(FDC2212_ERROR_CONFIG, 0b0001100000100001);  //set error config

}

///**************************************************************************/
///*!
//    @brief  Given a reading calculates the sensor frequency
//*/
///**************************************************************************/
//long long FDC2212::calculateFsensor(unsigned long reading){
////    Serial.println("reading: "+ String(reading));
//    //fsensor = (CH_FIN_SEL * fref * data) / 2 ^ 28
//    //should be mega hz so can truncate to long long
//    Serial.println("FDC reading: " + String(reading));
//    unsigned long long temp;
//    temp = 1 * 40000000 * reading;
//    temp = temp / (2^28);
////    Serial.println("frequency: " + String((long)temp));
//    return temp;
//}

///**************************************************************************/
///*!
//    @brief  Given sensor frequency calculates capacitance
//*/
///**************************************************************************/
//double FDC2212::calculateCapacitance(long long fsensor){
//    //differential configuration
//    //c sensor = 1                            - (Cboard + Cparacitic)
//    //             / (L * (2*pi * fsensor)^2)
//
//    double pi = 3.14159265359;
//    double L = 18; //uH
//    double Cboard = 33; //pf
//    double Cparacitic = 3; //pf
//
//    double temp = 2 * pi * fsensor;
//    temp = temp * temp;
//
//    temp = temp / 1000000; //uH
//    temp *= L;
//
////    Serial.println("capacitance: " + String(temp));
//    return temp;
//
//}

/**************************************************************************/
/*!
 @brief  Takes a reading and calculates capacitance from i
 */
/**************************************************************************/
uint32_t FDC2212::getReading() {
	uint32_t reading = 0;
	if (!this->isReading) {
		this->isReading = true;
		int timeout = 100;

		int status = read16FDC(FDC2212_STATUS_REGADDR);

		if (status & FDC2212_CH0_DRDY)
			asm("nop");
		if (status & FDC2212_CH0_AMPL_LOW)
			asm("nop");
		if (status & FDC2212_CH0_AMPL_HIGH)
			asm("nop");
		if (status & FDC2212_CH0_WCHD_TO)
			asm("nop");

		while (timeout && !(status & FDC2212_CH0_UNREADCONV)) {
			status = read16FDC(FDC2212_STATUS_REGADDR);
			timeout--;
		}
		if (timeout == 100) {
			//could be stale grab another
			//read the 28 bit result
			reading = read16FDC(FDC2212_DATA_CH0_REGADDR) << 16;
			reading |= read16FDC(FDC2212_DATA_LSB_CH0_REGADDR);
			while (timeout && !(status & FDC2212_CH0_UNREADCONV)) {
//        Serial.println("status: " + String(status));
				status = read16FDC(FDC2212_STATUS_REGADDR);
				timeout--;
			}
		}
		if (timeout) {
			//read the 28 bit result
			reading = read16FDC(FDC2212_DATA_CH0_REGADDR) << 16;
			reading |= read16FDC(FDC2212_DATA_LSB_CH0_REGADDR);

			if(lastReading==0)
			{
				lastReading = reading;
			}

			this->dCap_dT = ((float)reading - (float)this->lastReading)/((float)(millis()) - this->lastReadingTick);

			this->lastReadingTick = millis();
		} else {
			//error not reading
			//printUsb("error reading fdc");
			reading = 0;
		}

		if (reading < capMin)
			capMin = reading;
		if (reading > capMax)
			capMax = reading;
	}
	this->shouldread = false;
	this->isReading = false;
	return reading;
}

///**************************************************************************/
///*!
//    @brief  Takes a reading and calculates capacitance from it
//*/
///**************************************************************************/
//double FDC2212::readCapacitance() {
//    int timeout = 100;
//    unsigned long reading = 0;
//    long long fsensor = 0;
//    int status = read16FDC(FDC2214_STATUS_REGADDR);
//    while (timeout && !(status & FDC2214_CH0_UNREADCONV)) {
////        Serial.println("status: " + String(status));
//        status = read16FDC(FDC2214_STATUS_REGADDR);
//        timeout--;
//    }
//    if (timeout) {
//        //read the 28 bit result
//        reading = read16FDC(FDC2214_DATA_CH0_REGADDR) << 16;
//        reading |= read16FDC(FDC2214_DATA_LSB_CH0_REGADDR);
//        fsensor = calculateFsensor(reading);
//        return calculateCapacitance(fsensor);
//    } else {
//        //error not reading
//        Serial.println("error reading fdc");
//        return 0;
//    }
//}

/**************************************************************************/
/*!
 @brief  Scans various gain settings until the amplitude flag is cleared.
 WARNING: Changing the gain setting will generally have an impact on the
 reading.
 */
/**************************************************************************/
void FDC2212::setGain(void) {
	//todo
}
/**************************************************************************/
/*!
 @brief  I2C low level interfacing
 */
/**************************************************************************/

// Read 1 byte from the VL6180X at 'address'
uint8_t FDC2212::read8FDC(uint16_t address) {
	uint8_t data;
	uint8_t aRxBuffer[2];
    esp_err_t err = this->i2c_example_master_read_slave(I2C_EXAMPLE_MASTER_NUM, address, aRxBuffer, 2);
    if(err != ESP_OK)
    {
        Serial.println("Error in i2c read slave.");
    }

	data = aRxBuffer[0];

#if defined(I2C_DEBUG)
	Serial.print("\t$"); Serial.print(address, HEX); Serial.print(": 0x"); Serial.println(r, HEX);
#endif

	return data;
}

// Read 2 byte from the VL6180X at 'address'
uint16_t FDC2212::read16FDC(uint16_t address) {
	uint16_t data;
	/*
	 Wire.beginTransmission(_i2caddr);
	 //    Wire.write(address >> 8);
	 Wire.write(address);
	 Wire.endTransmission();

	 Wire.requestFrom(_i2caddr, (uint8_t) 2);
	 while (!Wire.available());
	 data = Wire.read();
	 data <<= 8;
	 while (!Wire.available());
	 data |= Wire.read();
	 return data;
	 */
	uint8_t aRxBuffer[2];
    esp_err_t err = this->i2c_example_master_read_slave(I2C_EXAMPLE_MASTER_NUM, address, aRxBuffer, 2);
    if(err != ESP_OK)
    {
        Serial.println("Error in i2c read slave.");
    }
	data = aRxBuffer[0] << 8;
	data |= aRxBuffer[1];

	return data;
}

// write 1 byte
void FDC2212::write8FDC(uint8_t data) {
	/*
	 Wire.beginTransmission(_i2caddr);
	 Wire.write(address >> 8);
	 Wire.write(address);
	 Wire.write(data);
	 Wire.endTransmission();
	 */

    esp_err_t err = this->i2c_example_master_write_slave(I2C_EXAMPLE_MASTER_NUM, &data, 2);
    if(err != ESP_OK)
    {
        Serial.println("Error in i2c read slave.");
    }

#if defined(I2C_DEBUG)
	Serial.print("\t$"); Serial.print(address, HEX); Serial.print(" = 0x"); Serial.println(data, HEX);
#endif
}

//// write 2 bytes
//void FDC2212::write16FDC(uint16_t address, uint16_t data) {
//    Wire.beginTransmission(_i2caddr);
////    Wire.write(address >> 8);
//    Wire.write(address & 0xFF);
//    Wire.write(data >> 8);
//    Wire.write(data);
//    Wire.endTransmission();
//}
void FDC2212::write16FDC(uint16_t address, uint16_t value) {
	uint8_t aTxBuffer[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    /*
	if (c(&this->instance->_i2cHandle, this->instance->_i2caddr, address, I2C_MEMADD_SIZE_8BIT, aTxBuffer, 2, 10000) != HAL_OK) {
		if (HAL_I2C_GetError(&this->instance->_i2cHandle) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
	}
    */



    esp_err_t err = this->i2c_example_master_write_slave(I2C_EXAMPLE_MASTER_NUM, aTxBuffer, 2);
    if(err != ESP_OK)
        Serial.println("Error in write16FDC.");
}

void FDC2212::shouldRead() {
	this->shouldread = true;
}




/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
esp_err_t FDC2212::i2c_example_master_read_slave(i2c_port_t i2c_num, uint16_t address, uint8_t* data_rd, size_t size)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_rd, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data_rd++, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
return ret;
}


/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t FDC2212::i2c_example_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

