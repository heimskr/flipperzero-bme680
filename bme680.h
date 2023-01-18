#pragma once

// Credit: Adafruit BME680 library

#include "i2c.h"
#include "bme680_defs.h"

#define BME680_ADDRESS 0x77

struct BME680: private I2C, bme68x_dev {
	float temperature = 0.f;
	float pressure = 0.f;
	float humidity = 0.f;
	uint32_t gasResistance = 0;

	using I2C::I2C;
	BME680();

	static constexpr int readingNotStarted = -1;
	static constexpr int readingComplete = 0;

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
	bool setGasHeater(uint16_t heater_temp, uint16_t heater_time);
	bool setODR(uint8_t odr);

	bool performReading();
	uint32_t beginReading();
	bool endReading();

	int8_t getData(uint8_t op_mode, bme68x_data *data, uint8_t &n_data);
	int8_t readFieldData(uint8_t index, bme68x_data &data);

	private:
		uint32_t measureStart = 0;
		uint16_t measurePeriod = 0;

		bme68x_conf gasConf;
		bme68x_heatr_conf gasHeaterConf;

		int8_t setConf(bme68x_conf &conf);
		int8_t getOpMode(uint8_t *op_mode);
		int8_t setOpMode(uint8_t op_mode);
		int8_t getRegs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
		int8_t setRegs(const uint8_t *reg_addrs, const uint8_t *reg_data, uint8_t len);
		void boundaryCheck(uint8_t &value, uint8_t max);
		uint32_t getMeasurementDuration(uint8_t op_mode);

		float calculateTemperature(uint32_t temp_adc);
		float calculatePressure(uint32_t pres_adc);
		float calculateHumidity(uint16_t hum_adc);
		float calculateGasResistanceLow(uint16_t gas_res_adc, uint8_t gas_range);
		float calculateGasResistanceHigh(uint16_t gas_res_adc, uint8_t gas_range);
		uint8_t calculateResHeat(uint16_t temp);
		uint8_t calculateGasWait(uint16_t dur);

		uint32_t millis() const;
		int remainingReadingMillis();
		int8_t readAllFieldData(bme68x_data * const data[]);
		void sortSensorData(uint8_t low_index, uint8_t high_index, bme68x_data *field[]);
		void swapFields(uint8_t index1, uint8_t index2, bme68x_data *field[]);
		int8_t setHeaterConf(uint8_t op_mode, struct bme68x_heatr_conf &conf);
		int8_t setConf(const bme68x_heatr_conf &conf, uint8_t op_mode, uint8_t &nb_conv);
		uint8_t calculateHeaterDurationShared(uint16_t dur);

		int8_t init();
		int8_t softReset();
		int8_t readVariantID();
		int8_t getCalibData();
};
