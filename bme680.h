#pragma once

// Credit: Adafruit BME680 library

#include "i2c.h"
#include "bme680_defs.h"

#define BME680_ADDRESS 0x77

struct BME680: private I2C, bme68x_dev {
	using I2C::I2C;
	BME680();

	bool begin();
	float readTemperature();
	float readPressure();
	float readHumidity();
	uint32_t readGas();
	float readAltitude(float sea_level);

	bool setTemperatureOversampling(uint8_t os);
	bool setPressureOversampling(uint8_t os);
	bool setHumidityOversampling(uint8_t os);
	bool setIIRFilterSize(uint8_t fs);
	bool setGasHeater(uint16_t heaterTemp, uint16_t heaterTime);
	bool setODR(uint8_t odr);

	bool performReading();
	uint32_t endReading();

	private:
		float temperature;
		float pressure;
		float humidity;
		uint32_t gas_resistance;

		bme68x_conf gas_conf;
		bme68x_heatr_conf gas_heatr_conf;

		int8_t setConf(bme68x_conf &conf);
		int8_t getOpMode(uint8_t *op_mode);
		int8_t setOpMode(uint8_t op_mode);
		int8_t getRegs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
		int8_t setRegs(const uint8_t *reg_addrs, const uint8_t *reg_data, uint8_t len);
		void boundaryCheck(uint8_t &value, uint8_t max);
};
