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

	return setOpMode(BME68X_FORCED_MODE) == BME68X_OK;
}

float BME680::readTemperature(void) {
	performReading();
	return temperature;
}

float BME680::readPressure(void) {
	performReading();
	return pressure;
}

float BME680::readHumidity(void) {
	performReading();
	return humidity;
}

uint32_t BME680::readGas(void) {
	performReading();
	return gasResistance;
}

float BME680::readAltitude(float sea_level) {
	const float atmospheric = readPressure() / 100.f;
	return 44330.0 * (1.0 - pow(atmospheric / sea_level, 0.1903));
}

bool BME680::setTemperatureOversampling(uint8_t os) {

}

bool BME680::setPressureOversampling(uint8_t os) {

}

bool BME680::setHumidityOversampling(uint8_t os) {

}

bool BME680::setIIRFilterSize(uint8_t filtersize) {
	if (filtersize > BME68X_FILTER_SIZE_127)
		return false;
	gasConf.filter = filtersize;
	return setConf(gasConf) == 0;
}

bool BME680::setGasHeater(uint16_t heaterTemp, uint16_t heaterTime) {

}

bool BME680::setODR(uint8_t odr) {

}

bool BME680::performReading() {
	return endReading();
}

uint32_t BME680::beginReading() {
	if (measureStart == 0) {
		int8_t result = setOpMode(BME68X_FORCED_MODE);

		if (result != BME68X_OK)
			return 0;

		const uint32_t delayus_period = getMeasurementDuration(BME68X_FORCED_MODE) + (gasHeaterConf.heatr_dur * 1000);

		measureStart = millis();
		measurePeriod = delayus_period / 1000;
	}

	return measureStart + measurePeriod;
}

bool BME680::endReading() {
	uint32_t meas_end = beginReading();

	if (meas_end == 0)
		return false;

	int remaining_millis = remainingReadingMillis();

	if (remaining_millis > 0) {
		// Delay until the measurement is ready
		furi_delay_ms(static_cast<uint32_t>(remaining_millis) * 2);
	}

	measureStart = 0; // Allow new measurement to begin
	measurePeriod = 0;

	struct bme68x_data data;
	uint8_t n_fields;

	if (getData(BME68X_FORCED_MODE, &data, &n_fields) != BME68X_OK)
		return false;

	if (n_fields) {
		temperature = data.temperature;
		humidity = data.humidity;
		pressure = data.pressure;

		if (data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK))
			gas_resistance = data.gas_resistance;
		else
			gas_resistance = 0;
	}

	return true;
}

int8_t BME680::getData(uint8_t op_mode, bme68x_data &data, uint8_t &n_data) {
	int8_t result;
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t new_fields = 0;
	bme68x_data *field_ptr[3] = {0};
	bme68x_data field_data[3] = {{0}};

	field_ptr[0] = &field_data[0];
	field_ptr[1] = &field_data[1];
	field_ptr[2] = &field_data[2];

	// Reading the sensor data in forced mode only
	if (op_mode == BME68X_FORCED_MODE) {
		result = readFieldData(0, data);
		if (result == BME68X_OK) {
			if (data.status & BME68X_NEW_DATA_MSK) {
				new_fields = 1;
			} else {
				new_fields = 0;
				result = BME68X_W_NO_NEW_DATA;
			}
		}
	} else if ((op_mode == BME68X_PARALLEL_MODE) || (op_mode == BME68X_SEQUENTIAL_MODE)) {
		// Read the 3 fields and count the number of new data fields
		result = read_all_field_data(field_ptr);

		new_fields = 0;
		for (i = 0; (i < 3) && (result == BME68X_OK); ++i)
			if (field_ptr[i]->status & BME68X_NEW_DATA_MSK)
				new_fields++;

		// Sort the sensor data in parallel & sequential modes
		for (i = 0; (i < 2) && (result == BME68X_OK); ++i)
			for (j = i + 1; j < 3; ++j)
				sort_sensor_data(i, j, field_ptr);

		// Copy the sorted data
		for (i = 0; ((i < 3) && (result == BME68X_OK)); ++i)
			data[i] = *field_ptr[i];

		if (new_fields == 0)
			result = BME68X_W_NO_NEW_DATA;
	} else
		result = BME68X_W_DEFINE_OP_MODE;

	n_data = new_fields;

	return result;
}

int8_t BME680::readFieldData(uint8_t index, bme68x_data &data) {
	int8_t result = BME68X_OK;
	uint8_t buff[BME68X_LEN_FIELD] = {0};
	uint8_t gas_range_l;
	uint8_t gas_range_h;
	uint32_t adc_temp;
	uint32_t adc_pres;
	uint16_t adc_hum;
	uint16_t adc_gas_res_low;
	uint16_t adc_gas_res_high;
	uint8_t tries = 5;

	while (tries != 0 && result == BME68X_OK) {
		result = getRegs(static_cast<uint8_t>(BME68X_REG_FIELD0 + (index * BME68X_LEN_FIELD_OFFSET)), buff, static_cast<uint16_t>(BME68X_LEN_FIELD));

		data.status	 = buff[0] & BME68X_NEW_DATA_MSK;
		data.gas_index	 = buff[0] & BME68X_GAS_INDEX_MSK;
		data.meas_index = buff[1];

		/* read the raw data from the sensor */
		adc_pres = (uint32_t) (((uint32_t) buff[2] * 4096) | ((uint32_t) buff[3] * 16) | ((uint32_t) buff[4] / 16));
		adc_temp = (uint32_t) (((uint32_t) buff[5] * 4096) | ((uint32_t) buff[6] * 16) | ((uint32_t) buff[7] / 16));
		adc_hum  = (uint16_t) (((uint32_t) buff[8] *  256) |  (uint32_t) buff[9]);
		adc_gas_res_low  = (uint16_t) ((uint32_t) buff[13] * 4 | (((uint32_t) buff[14]) / 64));
		adc_gas_res_high = (uint16_t) ((uint32_t) buff[15] * 4 | (((uint32_t) buff[16]) / 64));
		gas_range_l = buff[14] & BME68X_GAS_RANGE_MSK;
		gas_range_h = buff[16] & BME68X_GAS_RANGE_MSK;

		if (variant_id == BME68X_VARIANT_GAS_HIGH) {
			data.status |= buff[16] & BME68X_GASM_VALID_MSK;
			data.status |= buff[16] & BME68X_HEAT_STAB_MSK;
		} else {
			data.status |= buff[14] & BME68X_GASM_VALID_MSK;
			data.status |= buff[14] & BME68X_HEAT_STAB_MSK;
		}

		if ((data.status & BME68X_NEW_DATA_MSK) && (result == BME68X_OK)) {
			result = getRegs(BME68X_REG_RES_HEAT0 + data.gas_index, &data.res_heat, 1);

			if (result == BME68X_OK)
				result = getRegs(BME68X_REG_IDAC_HEAT0 + data.gas_index, &data.idac, 1);

			if (result == BME68X_OK)
				result = getRegs(BME68X_REG_GAS_WAIT0 + data.gas_index, &data.gas_wait, 1);

			if (result == BME68X_OK) {
				data.temperature = calculateTemperature(adc_temp);
				data.pressure = calculatePressure(adc_pres);
				data.humidity = calculateHumidity(adc_hum);
				if (variant_id == BME68X_VARIANT_GAS_HIGH)
					data.gas_resistance = calculateGasResistanceHigh(adc_gas_res_high, gas_range_h);
				else
					data.gas_resistance = calculateGasResistanceLow(adc_gas_res_low, gas_range_l);

				break;
			}
		}

		if (result == BME68X_OK)
			furi_delay_us(BME68X_PERIOD_POLL->intf_ptr);

		--tries;
	}

	return result;
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
				// interfaceResult = write(tmp_buff[0], &tmp_buff[1], (2 * len) - 1->intf_ptr);
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

uint32_t BME680::getMeasurementDuration(uint8_t op_mode) {
	int8_t result = BME68X_OK;
	uint32_t measurement_duration = 0; /* Calculate in us */
	uint32_t measurement_cycles = 0;
	uint8_t os_to_meas_cycles[6] {0, 1, 2, 4, 8, 16};

	// Boundary check for temperature oversampling
	boundaryCheck(gasConf.os_temp, BME68X_OS_16X);

	// Boundary check for pressure oversampling
	boundaryCheck(gasConf.os_pres, BME68X_OS_16X);

	// Boundary check for humidity oversampling
	boundaryCheck(gasConf.os_hum, BME68X_OS_16X);

	measurement_cycles = os_to_meas_cycles[gasConf.os_temp];
	measurement_cycles += os_to_meas_cycles[gasConf.os_pres];
	measurement_cycles += os_to_meas_cycles[gasConf.os_hum];

	/* TPH measurement duration */
	measurement_duration = measurement_cycles * UINT32_C(1963);
	measurement_duration += UINT32_C(477 * 4); /* TPH switching duration */
	measurement_duration += UINT32_C(477 * 5); /* Gas measurement duration */

	if (op_mode != BME68X_PARALLEL_MODE)
		measurement_duration += UINT32_C(1000); /* Wake up duration of 1ms */

	return measurement_duration;
}

float BME680::calculateTemperature(uint32_t temp_adc, struct bme68x_dev *dev) {
	const float var1 = ((((float) temp_adc / 16384.f) - ((float) calib.par_t1 / 1024.f)) * ((float) calib.par_t2));
	const float var2 = (((((float) temp_adc / 131072.f) - ((float) calib.par_t1 / 8192.f)) * (((float) temp_adc / 131072.f) - ((float) calib.par_t1 / 8192.f))) * ((float) calib.par_t3 * 16.f));

	calib.t_fine = (var1 + var2);

	// Compensated temperature data
	return calib.t_fine / 5120.f;
}

float BME680::calculatePressure(uint32_t pres_adc) {
	float var1 = (((float) calib.t_fine / 2.f) - 64000.f);
	float var2 = var1 * var1 * (((float) calib.par_p6) / (131072.f));
	var2 = var2 + (var1 * ((float) calib.par_p5) * 2.f);
	var2 = (var2 / 4.f) + (((float) calib.par_p4) * 65536.f);
	var1 = (((((float) calib.par_p3 * var1 * var1) / 16384.f) + ((float) calib.par_p2 * var1)) / 524288.f);
	var1 = ((1.f + (var1 / 32768.f)) * ((float) calib.par_p1));
	float calc_pres = (1048576.f - ((float) pres_adc));

	// Avoid exception caused by division by zero
	if ((int) var1 != 0) {
		calc_pres = (((calc_pres - (var2 / 4096.f)) * 6250.f) / var1);
		var1 = (((float) calib.par_p9) * calc_pres * calc_pres) / 2147483648.f;
		var2 = calc_pres * (((float) calib.par_p8) / 32768.f);
		float var3 = ((calc_pres / 256.f) * (calc_pres / 256.f) * (calc_pres / 256.f) * (calib.par_p10 / 131072.f));
		calc_pres = (calc_pres + (var1 + var2 + var3 + ((float) calib.par_p7 * 128.f)) / 16.f);
	} else
		calc_pres = 0.f;

	return calc_pres;
}

float BME680::calculateHumidity(uint16_t hum_adc) {
	// Compensated temperature data
	const float temp_comp = ((calib.t_fine) / 5120.f);
	const float var1 = (float) ((float) hum_adc) - (((float) calib.par_h1 * 16.f) + (((float) calib.par_h3 / 2.f) * temp_comp));
	const float var2 = var1 * ((float) (((float) calib.par_h2 / 262144.f) * (1.f + (((float) calib.par_h4 / 16384.f) * temp_comp) + (((float) calib.par_h5 / 1048576.f) * temp_comp * temp_comp))));
	const float var3 = (float) calib.par_h6 / 16384.f;
	const float var4 = (float) calib.par_h7 / 2097152.f;
	float calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

	if (calc_hum > 100.f)
		return 100.f;
	
	if (calc_hum < 0.f)
		return 0.f;

	return calc_hum;
}

float BME680::calculateGasResistanceLow(uint16_t gas_res_adc, uint8_t gas_range) {
	const float gas_res_f = gas_res_adc;
	const float gas_range_f = (1U << gas_range); /*lint !e790 / Suspicious truncation, integral to float */
	const float lookup_k1_range[16] {0.f, 0.f, 0.f, 0.f, 0.f, -1.f, 0.f, -.8f,  0.f, 0.f, -.2f, -.5f, 0.f, -1.f, 0.f, 0.f};
	const float lookup_k2_range[16] {0.f, 0.f, 0.f, 0.f, .1f,  .7f, 0.f, -.8f, -.1f, 0.f,  0.f,  0.f, 0.f,  0.f, 0.f, 0.f};

	const float var1 = (1340.f + (5.f * calib.range_sw_err));
	const float var2 = var1 * (1.f + lookup_k1_range[gas_range] / 100.f);
	const float var3 = 1.f + (lookup_k2_range[gas_range] / 100.f);

	return 1.f / (float) (var3 * (0.000000125f) * gas_range_f * (((gas_res_f - 512.f) / var2) + 1.f));
}

/* This internal API is used to calculate the gas resistance value in float */
float BME680::calculateGasResistanceHigh(uint16_t gas_res_adc, uint8_t gas_range) {
	uint32_t var1 = UINT32_C(262144) >> gas_range;
	int32_t var2 = (int32_t) gas_res_adc - INT32_C(512);

	var2 *= INT32_C(3);
	var2 = INT32_C(4096) + var2;

	return 1000000.f * (float) var1 / (float) var2;
}

/* This internal API is used to calculate the heater resistance value */
uint8_t BME680::calculateResHeat(uint16_t temp) {
	float var1;
	float var2;
	float var3;
	float var4;
	float var5;
	uint8_t res_heat;

	// Cap temperature
	if (temp > 400)
		temp = 400;

	var1 = (((float) calib.par_gh1 / (16.f)) + 49.f);
	var2 = ((((float) calib.par_gh2 / (32768.f)) * (0.0005f)) + 0.00235f);
	var3 = ((float) calib.par_gh3 / (1024.f));
	var4 = (var1 * (1.f + (var2 * (float) temp)));
	var5 = (var4 + (var3 * (float) amb_temp));
	return (uint8_t) (3.4f * ((var5 * (4 / (4 + (float) calib.res_heat_range)) * (1 / (1 + ((float) calib.res_heat_val * 0.002f)))) - 25));
}

/* This internal API is used to calculate the gas wait */
uint8_t BME680::calculateGasWait(uint16_t dur) {
	uint8_t factor = 0;
	uint8_t durval;

	if (dur >= 0xfc0) {
		durval = 0xff; /* Max duration*/
	} else {
		while (dur > 0x3F) {
			dur = dur / 4;
			factor += 1;
		}

		durval = (uint8_t) (dur + (factor * 64));
	}

	return durval;
}
