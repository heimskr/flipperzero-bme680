// Credit: Adafruit BME680 library

#include "bme680.h"

BME680::BME680(): I2C(BME680_ADDRESS) {}

bool BME680::begin() {
	setIIRFilterSize(BME68X_FILTER_SIZE_3);
	setODR(BME68X_ODR_NONE);
	setHumidityOversampling(BME68X_OS_2X);
	setPressureOversampling(BME68X_OS_4X);
	setTemperatureOversampling(BME68X_OS_8X);
	// setGasHeater(320, 150); // 320*C for 150 ms

	// int8_t result = 

	return true;
}

bool BME680::setIIRFilterSize(uint8_t filtersize) {
	if (filtersize > BME68X_FILTER_SIZE_127)
		return false;
	gas_conf.filter = filtersize;
	return setConf(gas_conf) == 0;
}

int8_t BME680::setConf(bme68x_conf &conf) {
	int8_t result = 0;
	uint8_t odr20 = 0, odr3 = 1;
	uint8_t current_op_mode = 0;

	// Register data starting from BME68X_REG_CTRL_GAS_1(0x71) up to BME68X_REG_CONFIG(0x75)
	uint8_t reg_array[BME68X_LEN_CONFIG]  = {0x71, 0x72, 0x73, 0x74, 0x75};
	uint8_t data_array[BME68X_LEN_CONFIG] = {0};

	result = getOpMode(&current_op_mode);

	if (result == BME68X_OK)
		// Configure only in the sleep mode
		result = setOpMode(BME68X_SLEEP_MODE);

	if (result == BME68X_OK) {
		result = getRegs(reg_array[0], data_array, BME68X_LEN_CONFIG);
		infoMessage = BME68X_OK;

		if (result == BME68X_OK) {
			boundaryCheck(conf.filter, BME68X_FILTER_SIZE_127);
			boundaryCheck(conf.os_temp, BME68X_OS_16X);
			boundaryCheck(conf.os_pres, BME68X_OS_16X);
			boundaryCheck(conf.os_hum, BME68X_OS_16X);
			boundaryCheck(conf.odr, BME68X_ODR_NONE);

			data_array[4] = BME68X_SET_BITS(data_array[4], BME68X_FILTER, conf.filter);
			data_array[3] = BME68X_SET_BITS(data_array[3], BME68X_OST, conf.os_temp);
			data_array[3] = BME68X_SET_BITS(data_array[3], BME68X_OSP, conf.os_pres);
			data_array[1] = BME68X_SET_BITS_POS_0(data_array[1], BME68X_OSH, conf.os_hum);
			if (conf.odr != BME68X_ODR_NONE) {
				odr20 = conf.odr;
				odr3  = 0;
			}

			data_array[4] = BME68X_SET_BITS(data_array[4], BME68X_ODR20, odr20);
			data_array[0] = BME68X_SET_BITS(data_array[0], BME68X_ODR3, odr3);
		}
	}

	if (result == BME68X_OK)
		result = setRegs(reg_array, data_array, BME68X_LEN_CONFIG);

	if ((current_op_mode != BME68X_SLEEP_MODE) && (result == BME68X_OK))
		result = setOpMode(current_op_mode);

	return result;
}

int8_t BME680::getOpMode(uint8_t *op_mode) {
	int8_t result;
	uint8_t mode;

	if (op_mode) {
		result = getRegs(BME68X_REG_CTRL_MEAS, &mode, 1);
		// Masking the other register bit info
		*op_mode = mode & BME68X_MODE_MSK;
	} else
		result = BME68X_E_NULL_PTR;

	return result;
}

int8_t BME680::setOpMode(uint8_t op_mode) {
	int8_t result = BME68X_OK;
	uint8_t tmp_pow_mode = 0;
	uint8_t pow_mode = 0;
	uint8_t reg_addr = BME68X_REG_CTRL_MEAS;

	// Call until in sleep
	do {
		result = getRegs(BME68X_REG_CTRL_MEAS, &tmp_pow_mode, 1);
		if (result == BME68X_OK) {
			// Put to sleep before changing mode
			pow_mode = (tmp_pow_mode & BME68X_MODE_MSK);
			if (pow_mode != BME68X_SLEEP_MODE) {
				tmp_pow_mode &= ~BME68X_MODE_MSK; // Set to sleep
				result = setRegs(&reg_addr, &tmp_pow_mode, 1);
				furi_delay_us(BME68X_PERIOD_POLL);
			}
		}
	} while ((pow_mode != BME68X_SLEEP_MODE) && (result == BME68X_OK));

	// Already in sleep
	if ((op_mode != BME68X_SLEEP_MODE) && (result == BME68X_OK)) {
		tmp_pow_mode = (tmp_pow_mode & ~BME68X_MODE_MSK) | (op_mode & BME68X_MODE_MSK);
		result = setRegs(&reg_addr, &tmp_pow_mode, 1);
	}

	return result;
}

int8_t BME680::getRegs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len) {
	interfaceResult = writeThenRead(&reg_addr, 1, reg_data, len);
	if (interfaceResult != 0)
		return BME68X_E_COM_FAIL;

	return BME68X_OK;
}

int8_t BME680::setRegs(const uint8_t *reg_addrs, const uint8_t *reg_data, uint8_t len) {
	int8_t result = BME68X_OK;

	// Check for null pointer in the device structure
	if (reg_addrs != nullptr && reg_data != nullptr) {
		if ((0 < len) && (len <= (BME68X_LEN_INTERLEAVE_BUFF / 2))) {
			// Length of the temporary buffer is 2*(length of register)
			uint8_t	tmp_buff[BME68X_LEN_INTERLEAVE_BUFF] = {0};

			// Interleave the 2 arrays
			for (uint16_t index = 0; index < len; ++index) {
				tmp_buff[(2 * index)] = reg_addrs[index];
				tmp_buff[(2 * index) + 1] = reg_data[index];
			}

			if (result == BME68X_OK) {
				// interfaceResult = dev->write(tmp_buff[0], &tmp_buff[1], (2 * len) - 1, dev->intf_ptr);
				interfaceResult = write(&tmp_buff[0], 1);
				if (interfaceResult != 0) {
					result = BME68X_E_COM_FAIL;
				} else {
					interfaceResult = write(&tmp_buff[1], (2 * len) - 1);
					if (interfaceResult != 0)
						result = BME68X_E_COM_FAIL;
				}
			}
		} else
			result = BME68X_E_INVALID_LENGTH;
	} else
		result = BME68X_E_NULL_PTR;

	return result;
}

void BME680::boundaryCheck(uint8_t &value, uint8_t max) {
	// Check whether value is above maximum value
	if (max < value) {
		// Auto correct the invalid value to maximum value
		value = max;
		infoMessage |= BME68X_I_PARAM_CORR;
	}
}
