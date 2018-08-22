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

esp_err_t ma_read_uint16t(uint8_t address, uint8_t *rbuf)
{ esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (FDC2214_I2C_ADDRESS<<1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (FDC2214_I2C_ADDRESS<<1) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, rbuf++, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, rbuf++, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret == ESP_FAIL) {
      printf("i2c Error read - readback\n");
     return ESP_FAIL;
  }
  return ret; 
}

/*
esp_err_t ma_write_uint8t(uint8_t address, uint8_t value)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (FDC2214_I2C_ADDRESS<<1) | WRITE_BIT , ACK_CHECK_EN);
  i2c_master_write_byte(cmd, address, I2C_MASTER_NACK);
  i2c_master_write_byte(cmd, value, I2C_MASTER_ACK);
  i2c_master_stop(cmd); 
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 20 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret == ESP_FAIL) {
     printf("ESP_I2C_WRITE ERROR : %d\n",ret);
    return ret;
  }
  return ESP_OK;
}
*/

esp_err_t ma_write_uint16t(uint8_t address, uint16_t value)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (FDC2214_I2C_ADDRESS<<1) | WRITE_BIT , ACK_CHECK_EN);
  i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value >> 8, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value << 8, ACK_CHECK_EN);
  i2c_master_stop(cmd); 
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret == ESP_FAIL) {
     printf("ESP_I2C_WRITE ERROR : %d\n",ret);
    return ret;
  }
  return ESP_OK;
}

FDC2212::FDC2212() {
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

/**************************************************************************/
/*!
 @brief  Setups the HW
 */
/**************************************************************************/
bool FDC2212::begin(void) {
	esp_err_t ret = i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
	if(ret == ESP_OK)
	{
	_i2c = i2cInit(1,21,22,100000);
	vTaskDelay(20 / portTICK_PERIOD_MS);
	if(_i2c)
	{
		/*
		uint16_t manufacturer = read16FDC(FDC2212_MANUFACT_REGADDR);
		Serial.printf("i2c manufacturer %x\n", manufacturer);
		if (manufacturer != 0x5449) {
			Serial.printf("Wrong i2c device manufacturer %x\n", manufacturer);
			//two valid device ids for FDC2214 0x3054 and 0x3055
			return false;
		}
		*/

		uint16_t deviceId = read16FDC(FDC2212_DEVICE_ID_REGADDR);
		Serial.printf("i2c device %x\n", deviceId);
		if (!(deviceId == 0x3054 || deviceId == 0x3055)) {
			Serial.printf("Wrong i2c device %x\n", deviceId);
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
	}
	return false;
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
IRAM_ATTR uint32_t FDC2212::getReading() {
	uint32_t reading = 0;
	if (!this->isReading) {
		this->isReading = true;
		int timeout = 100;

	    //blink(1);
		int status = read16FDC(FDC2212_STATUS_REGADDR);

		if (status & FDC2212_CH0_DRDY)
			Serial.println("FDC2212_CH0_DRDY");
		if (status & FDC2212_CH0_AMPL_LOW)
			Serial.println("FDC2212_CH0_AMPL_LOW");
		if (status & FDC2212_CH0_AMPL_HIGH)
			Serial.println("FDC2212_CH0_AMPL_HIGH");
		if (status & FDC2212_CH0_WCHD_TO)
			Serial.println("FDC2212_CH0_WCHD_TO");

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
			Serial.println("error reading fdc");
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

// Read 2 byte from the VL6180X at 'address'
uint16_t FDC2212::read16FDC(uint16_t address) {
	/*
	uint16_t Address;
	Address = FDC2214_I2C_ADDRESS << 1;
	Address |= OAR1_ADD0_Set;
    uint16_t data;
    Wire.beginTransmission(Address);
    Wire.write(address);
    Wire.endTransmission();

    Wire.requestFrom(Address, (uint8_t) 2);
    while (!Wire.available());
    data = Wire.read();
    data <<= 8;
    while (!Wire.available());
    data |= Wire.read();
    return data;
	*/

	uint8_t r[2];
	esp_err_t res = ma_read_uint16t(address, r);
	uint16_t result = (r[0]<<8 | r[1]);
	return result;
}

// write 2 bytes
void FDC2212::write16FDC(uint16_t address, uint16_t data) {
/*
	uint16_t Address = FDC2214_I2C_ADDRESS << 1;
	Address &= OAR1_ADD0_Reset;

    Wire.beginTransmission(Address);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    Wire.write(data >> 8);
    Wire.write(data);
    Wire.endTransmission();
*/
    ma_write_uint16t(address, data);


}


void FDC2212::shouldRead() {
	this->shouldread = true;
}
