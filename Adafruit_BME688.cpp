/*!
 * @file Adafruit_BME688.cpp
 * BME688 temperature, humidity, barometric pressure and gas sensor driver
 * Originally written as Adafruit_BME680.h by Ladyada for Adafruit Industries.
 * Modified by Nik Dennler, University of Hertfordshire, 2021
 * n.dennler2@herts.ac.uk
 * 
 * ORIGINAL DESCRIPTION:
 * @mainpage Adafruit BME680 temperature, humidity, barometric pressure and gas
 * sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's BME680 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BME680 breakout: https://www.adafruit.com/products/3660
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_BME688.h"
#include "Arduino.h"

#define BIT(n,i) (n>>i&1)
#define BME680_DEBUG
/*
 * Macro definition for valid new data (0x80) AND
 * heater stability (0x10) AND gas resistance (0x20) values
 */
#define BME68X_VALID_DATA  UINT8_C(0xB0)

/* Macro for count of samples to be displayed */
#define SAMPLE_COUNT       UINT8_C(50)


/** Our hardware interface functions **/
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *interface);
static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *interface);
static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *interface);
static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *interface);
static void delay_usec(uint32_t us, void *intf_ptr);

// PUBLIC FUNCTIONS

/*!
 *  @brief  Instantiates sensor with i2c.
 *  @param  *theWire
 *          optional Wire object
 */
Adafruit_BME688::Adafruit_BME688(TwoWire *theWire)
    : _meas_start(0), _meas_period(0) {
  _wire = theWire;
}

/*!
 *  @brief  Instantiates sensor with Hardware SPI.
 *  @param  cspin
 *          SPI chip select.
 *  @param  theSPI
 *          optional SPI object
 */
Adafruit_BME688::Adafruit_BME688(int8_t cspin, SPIClass *theSPI)
    : _meas_start(0), _meas_period(0) {
  _spidev = new Adafruit_SPIDevice(cspin, 1000000, SPI_BITORDER_MSBFIRST,
                                   SPI_MODE0, theSPI);
}

/*!
 *  @brief  Instantiates sensor with Software (bit-bang) SPI.
 *  @param  cspin
 *          SPI chip select
 *  @param  mosipin
 *          SPI MOSI (Data from microcontroller to sensor)
 *  @param  misopin
 *          SPI MISO (Data to microcontroller from sensor)
 *  @param  sckpin
 *          SPI Clock
 */
Adafruit_BME688::Adafruit_BME688(int8_t cspin, int8_t mosipin, int8_t misopin,
                                 int8_t sckpin)
    : _meas_start(0), _meas_period(0) {
  _spidev = new Adafruit_SPIDevice(cspin, sckpin, misopin, mosipin, 1000000,
                                   SPI_BITORDER_MSBFIRST, SPI_MODE0);
}

/*!
 *  @brief  Initializes the sensor
 *          Hardware ss initialized, verifies it is in the I2C or SPI bus, then
 * reads calibration data in preparation for sensor reads.
 *  @param  addr
 *          Optional parameter for the I2C address of BME680. Default is 0x77
 *  @param  initSettings
 *          Optional parameter for initializing the sensor settings.
 *          Default is true.
 *  @return True on sensor initialization success. False on failure.
 */
bool Adafruit_BME688::begin(uint8_t addr, bool initSettings) {
  int8_t rslt;

  if (!_spidev) { // i2c
    if (_i2cdev) {
      delete _i2cdev;
    }
    _i2cdev = new Adafruit_I2CDevice(addr, _wire);
    if (!_i2cdev->begin()) {
      return false;
    }

    gas_sensor.chip_id = addr;
    gas_sensor.intf = BME68X_I2C_INTF;
    gas_sensor.intf_ptr = (void *)_i2cdev;
    gas_sensor.read = &i2c_read;
    gas_sensor.write = &i2c_write;

  } else {
    if (!_spidev->begin()) {
      return false;
    }

    gas_sensor.chip_id = 0;
    gas_sensor.intf = BME68X_SPI_INTF;
    gas_sensor.intf_ptr = (void *)_spidev;
    gas_sensor.read = &spi_read;
    gas_sensor.write = &spi_write;
  }

  gas_sensor.amb_temp = 25; /* The ambient temperature in deg C is used for
                               defining the heater temperature */
  gas_sensor.delay_us = delay_usec;

  rslt = bme68x_init(&gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("Init Result: "));
  Serial.println(rslt);
#endif

  if (rslt != BME68X_OK)
    return false;

#ifdef BME680_DEBUG
  Serial.print("T1 = ");
  Serial.println(gas_sensor.calib.par_t1);
  Serial.print("T2 = ");
  Serial.println(gas_sensor.calib.par_t2);
  Serial.print("T3 = ");
  Serial.println(gas_sensor.calib.par_t3);
  Serial.print("P1 = ");
  Serial.println(gas_sensor.calib.par_p1);
  Serial.print("P2 = ");
  Serial.println(gas_sensor.calib.par_p2);
  Serial.print("P3 = ");
  Serial.println(gas_sensor.calib.par_p3);
  Serial.print("P4 = ");
  Serial.println(gas_sensor.calib.par_p4);
  Serial.print("P5 = ");
  Serial.println(gas_sensor.calib.par_p5);
  Serial.print("P6 = ");
  Serial.println(gas_sensor.calib.par_p6);
  Serial.print("P7 = ");
  Serial.println(gas_sensor.calib.par_p7);
  Serial.print("P8 = ");
  Serial.println(gas_sensor.calib.par_p8);
  Serial.print("P9 = ");
  Serial.println(gas_sensor.calib.par_p9);
  Serial.print("P10 = ");
  Serial.println(gas_sensor.calib.par_p10);
  Serial.print("H1 = ");
  Serial.println(gas_sensor.calib.par_h1);
  Serial.print("H2 = ");
  Serial.println(gas_sensor.calib.par_h2);
  Serial.print("H3 = ");
  Serial.println(gas_sensor.calib.par_h3);
  Serial.print("H4 = ");
  Serial.println(gas_sensor.calib.par_h4);
  Serial.print("H5 = ");
  Serial.println(gas_sensor.calib.par_h5);
  Serial.print("H6 = ");
  Serial.println(gas_sensor.calib.par_h6);
  Serial.print("H7 = ");
  Serial.println(gas_sensor.calib.par_h7);
  Serial.print("G1 = ");
  Serial.println(gas_sensor.calib.par_gh1);
  Serial.print("G2 = ");
  Serial.println(gas_sensor.calib.par_gh2);
  Serial.print("G3 = ");
  Serial.println(gas_sensor.calib.par_gh3);
  Serial.print("G1 = ");
  Serial.println(gas_sensor.calib.par_gh1);
  Serial.print("G2 = ");
  Serial.println(gas_sensor.calib.par_gh2);
  Serial.print("G3 = ");
  Serial.println(gas_sensor.calib.par_gh3);
  Serial.print("Heat Range = ");
  Serial.println(gas_sensor.calib.res_heat_range);
  Serial.print("Heat Val = ");
  Serial.println(gas_sensor.calib.res_heat_val);
  Serial.print("SW Error = ");
  Serial.println(gas_sensor.calib.range_sw_err);
#endif

  if (initSettings) {
    setIIRFilterSize(BME68X_FILTER_SIZE_3);
    setODR(BME68X_ODR_NONE);
    setHumidityOversampling(BME68X_OS_2X);
    setPressureOversampling(BME68X_OS_4X);
    setTemperatureOversampling(BME68X_OS_8X);
    setGasHeater(320, 150); // 320*C for 150 ms
  } else {
    setGasHeater(0, 0);
  }
  // don't do anything till we request a reading
  rslt = bme68x_set_op_mode(BME688_DEFAULT_OP_MODE, &gas_sensor);
  // rslt = bme68x_set_op_mode(BME68X_SEQUENTIAL_MODE, &gas_sensor);


#ifdef BME680_DEBUG
  Serial.print(F("Opmode Result: "));
  Serial.println(rslt);
#endif

  if (rslt != BME68X_OK)
    return false;

  return true;
}

/*!
 *  @brief  Performs a reading and returns the ambient temperature.
 *  @return Temperature in degrees Centigrade
 */
float Adafruit_BME688::readTemperature(void) {
  performReading();
  return temperature;
}

/*!
 *  @brief Performs a reading and returns the barometric pressure.
 *  @return Barometic pressure in Pascals
 */
float Adafruit_BME688::readPressure(void) {
  performReading();
  return pressure;
}

/*!
 *  @brief  Performs a reading and returns the relative humidity.
 *  @return Relative humidity as floating point
 */
float Adafruit_BME688::readHumidity(void) {
  performReading();
  return humidity;
}

/*!
 *  @brief Calculates the resistance of the MOX gas sensor.
 *  @return Resistance in Ohms
 */
uint32_t Adafruit_BME688::readGas(void) {
  performReading();
  return gas_resistance;
}

/*!
 *  @brief  Calculates the altitude (in meters).
 *          Reads the current atmostpheric pressure (in hPa) from the sensor and
 * calculates via the provided sea-level pressure (in hPa).
 *  @param  seaLevel
 *          Sea-level pressure in hPa
 *  @return Altitude in meters
 */
float Adafruit_BME688::readAltitude(float seaLevel) {
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  float atmospheric = readPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/*!
 *  @brief  Performs a full reading of all 4 sensors in the BME680.
 *          Assigns the internal Adafruit_BME688#temperature,
 * Adafruit_BME688#pressure, Adafruit_BME688#humidity and
 * Adafruit_BME688#gas_resistance member variables
 *  @return True on success, False on failure
 */
bool Adafruit_BME688::performReading(void) { return endReading(); }

/*! @brief Begin an asynchronous reading.
 *  @return When the reading would be ready as absolute time in millis().
 */
uint32_t Adafruit_BME688::beginReading(void) {
  if (_meas_start != 0) {
    /* A measurement is already in progress */
    return _meas_start + _meas_period;
  }

  int8_t rslt = bme68x_set_op_mode(BME688_DEFAULT_OP_MODE, &gas_sensor);
  // int8_t rslt = bme68x_set_op_mode(BME68X_SEQUENTIAL_MODE, &gas_sensor);

#ifdef BME680_DEBUG
  Serial.print(F("Opmode Result: "));
  Serial.println(rslt);
#endif
  if (rslt != BME68X_OK)
    return false;

  /* Calculate delay period in microseconds */
  uint32_t delayus_period = 0;
  if (BME688_DEFAULT_OP_MODE==BME68X_FORCED_MODE){
      delayus_period = (uint32_t)bme68x_get_meas_dur(
                                BME688_DEFAULT_OP_MODE, &gas_conf, &gas_sensor) +
                            ((uint32_t)gas_heatr_conf.heatr_dur * 1000);
  }
  else {
    delayus_period = (uint32_t)bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &gas_conf, &gas_sensor) + ((uint32_t)gas_heatr_conf.shared_heatr_dur * 1000);
  }
#ifdef BME680_DEBUG
  Serial.print("meas_dur: ");
  Serial.println(bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &gas_conf, &gas_sensor));
  Serial.print("shared_heatr_dur: ");
  Serial.println(gas_heatr_conf.shared_heatr_dur*1000);
  Serial.print(F("delayus_period: "));
  Serial.println(delayus_period);
#endif
  // Serial.print("measure: ");
  // Serial.println(bme68x_get_meas_dur(BME68X_FORCED_MODE, &gas_conf,
  // &gas_sensor)); Serial.print("heater: ");
  // Serial.println((uint32_t)gas_heatr_conf.heatr_dur * 1000);
  // Serial.print("delay: ");
  // Serial.println(delayus_period);


  _meas_start = millis();
  _meas_period = delayus_period / 1000;

  return _meas_start + _meas_period;
}

/*! @brief  End an asynchronous reading.
 *          If the asynchronous reading is still in progress, block until it
 * ends. If no asynchronous reading has started, this is equivalent to
 * performReading().
 *  @return Whether success.
 */
bool Adafruit_BME688::endReading(void) {
  uint32_t meas_end = beginReading();

  if (meas_end == 0) {
    return false;
  }

  int remaining_millis = remainingReadingMillis();

  if (remaining_millis > 0) {
#ifdef BME680_DEBUG
    Serial.print(F("Waiting (ms) "));
    Serial.println(remaining_millis);
#endif
    delay(static_cast<unsigned int>(remaining_millis) *
          2); /* Delay till the measurement is ready */
  }
  _meas_start = 0; /* Allow new measurement to begin */
  _meas_period = 0;

#ifdef BME680_DEBUG
  Serial.print(F("t_fine = "));
  Serial.println(gas_sensor.calib.t_fine);
#endif

  uint8_t n_fields;

#ifdef BME680_DEBUG
  Serial.print(("n_fields: "));
  Serial.println(n_fields);
  Serial.println(F("Getting sensor data"));
#endif 

  // struct bme68x_data data;
  // struct bme68x_data data;
  struct bme68x_data data_single;
  struct bme68x_data data[3];
  int8_t rslt;
  if (BME688_DEFAULT_OP_MODE==BME68X_FORCED_MODE){
    Serial.println("FORCED MODE");
    rslt = bme68x_get_data(BME688_DEFAULT_OP_MODE, &data_single, &n_fields, &gas_sensor);
    Serial.print("Temperature: ");
    Serial.println(data_single.temperature);

    Serial.print("humidity: ");
    Serial.println(data_single.humidity);

    Serial.print("pressure: ");
    Serial.println(data_single.pressure);

    Serial.print("gas_resistance: ");
    Serial.println(data_single.gas_resistance);

    Serial.print("status: ");
    Serial.println(data_single.status);
    Serial.print("->");
    Serial.print(BIT(data_single.status,7));
    Serial.print(BIT(data_single.status,6));
    Serial.print(BIT(data_single.status,5));
    Serial.print(BIT(data_single.status,4));
    Serial.print(BIT(data_single.status,3));
    Serial.print(BIT(data_single.status,2));
    Serial.print(BIT(data_single.status,1));
    Serial.println(BIT(data_single.status,0));

    Serial.print("gas_index: ");
    Serial.println(data_single.gas_index);    

    Serial.print("meas_index: ");
    Serial.println(data_single.meas_index);
    }
  else {
    Serial.println("PARALLEL MODE");
    // gas_sensor.delay_us(remaining_millis, gas_sensor.intf_ptr);
    rslt = bme68x_get_data(BME688_DEFAULT_OP_MODE, data, &n_fields, &gas_sensor);
    Serial.print("Temperature: ");
    Serial.print(data[0].temperature);
    Serial.print(", ");
    Serial.print(data[1].temperature);
    Serial.print(", ");
    Serial.println(data[2].temperature);

    Serial.print("humidity: ");
    Serial.print(data[0].humidity);
    Serial.print(", ");
    Serial.print(data[1].humidity);
    Serial.print(", ");
    Serial.println(data[2].humidity);

    Serial.print("pressure: ");
    Serial.print(data[0].pressure);
    Serial.print(", ");
    Serial.print(data[1].pressure);
    Serial.print(", ");
    Serial.println(data[2].pressure);

    Serial.print("gas_resistance: ");
    Serial.print(data[0].gas_resistance);
    Serial.print(", ");
    Serial.print(data[1].gas_resistance);
    Serial.print(", ");
    Serial.println(data[2].gas_resistance);

    Serial.print("status: ");
    Serial.print(data[0].status);
    Serial.print("->");
    Serial.print(BIT(data[0].status,7));
    Serial.print(BIT(data[0].status,6));
    Serial.print(BIT(data[0].status,5));
    Serial.print(BIT(data[0].status,4));
    Serial.print(BIT(data[0].status,3));
    Serial.print(BIT(data[0].status,2));
    Serial.print(BIT(data[0].status,1));
    Serial.print(BIT(data[0].status,0));
    Serial.print(", ");
    Serial.print(data[1].status);
    Serial.print("->");
    Serial.print(BIT(data[1].status,7));
    Serial.print(BIT(data[1].status,6));
    Serial.print(BIT(data[1].status,5));
    Serial.print(BIT(data[1].status,4));
    Serial.print(BIT(data[1].status,3));
    Serial.print(BIT(data[1].status,2));
    Serial.print(BIT(data[1].status,1));
    Serial.print(BIT(data[1].status,0));    
    Serial.print(", ");
    Serial.println(data[2].status);
    Serial.print("->");
    Serial.print(BIT(data[2].status,7));
    Serial.print(BIT(data[2].status,6));
    Serial.print(BIT(data[2].status,5));
    Serial.print(BIT(data[2].status,4));
    Serial.print(BIT(data[2].status,3));
    Serial.print(BIT(data[2].status,2));
    Serial.print(BIT(data[2].status,1));
    Serial.print(BIT(data[2].status,0));

    Serial.print("gas_index: ");
    Serial.print(data[0].gas_index);
    Serial.print(", ");
    Serial.print(data[1].gas_index);
    Serial.print(", ");
    Serial.println(data[2].gas_index);    

    Serial.print("meas_index: ");
    Serial.print(data[0].meas_index);
    Serial.print(", ");
    Serial.print(data[1].meas_index);
    Serial.print(", ");
    Serial.println(data[2].meas_index);  
        /* Check if rslt == BME68X_OK, report or handle if otherwise */
    for (uint8_t i = 0; i < n_fields; i++)
    {
        if (data[i].status == BME68X_VALID_DATA)
        {
#ifdef BME68X_USE_FPU
            printf("%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x, %d, %d\n",
                    // sample_count,
                    // (long unsigned int)time_ms,
                    data[i].temperature,
                    data[i].pressure,
                    data[i].humidity,
                    data[i].gas_resistance,
                    data[i].status,
                    data[i].gas_index,
                    data[i].meas_index);
#else
            printf("%u, %lu, %d, %lu, %lu, %lu, 0x%x, %d, %d\n",
                    // sample_count,
                    // (long unsigned int)time_ms,
                    (data[i].temperature / 100),
                    (long unsigned int)data[i].pressure,
                    (long unsigned int)(data[i].humidity / 1000),
                    (long unsigned int)data[i].gas_resistance,
                    data[i].status,
                    data[i].gas_index,
                    data[i].meas_index);
#endif
            // sample_count++;
        }
    }

  }

#ifdef BME680_DEBUG
  Serial.print(F("GetData Result: "));
  Serial.println(rslt);
#endif

  if (rslt != BME68X_OK)
    return false;

//   if (n_fields) {
//     temperature = data.temperature;
//     humidity = data.humidity;
//     pressure = data.pressure;

// #ifdef BME680_DEBUG
//     Serial.print(F("data.status 0x"));
//     Serial.println(data.status, HEX);
// #endif
//     if (data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK)) {
//       // Serial.print("Gas resistance: "); Serial.println(data.gas_resistance);
//       gas_resistance = data.gas_resistance;
//     } else {
//       gas_resistance = 0;
//       // Serial.println("Gas reading unstable!");
//     }
//   }

  return true;
}

/*! @brief  Get remaining time for an asynchronous reading.
 *          If the asynchronous reading is still in progress, how many millis
 * until its completion. If the asynchronous reading is completed, 0. If no
 * asynchronous reading has started, -1 or
 * Adafruit_BME688::reading_not_started. Does not block.
 *  @return Remaining millis until endReading will not block if invoked.
 */
int Adafruit_BME688::remainingReadingMillis(void) {
  if (_meas_start != 0) {
    /* A measurement is already in progress */
    int remaining_time = (int)_meas_period - (millis() - _meas_start);
    return remaining_time < 0 ? reading_complete : remaining_time;
  }
  return reading_not_started;
}

/*!
 *  @brief  Enable and configure gas reading + heater
 *  @param  heaterTemp
 *          Desired temperature in degrees Centigrade
 *  @param  heaterTime
 *          Time to keep heater on in milliseconds
 *  @return True on success, False on failure
 */
bool Adafruit_BME688::setGasHeater(uint16_t heaterTemp, uint16_t heaterTime) {
  const uint8_t op_mode = BME68X_FORCED_MODE;
  if ((heaterTemp == 0) || (heaterTime == 0)) {
    gas_heatr_conf.enable = BME68X_DISABLE;
  } else {
    gas_heatr_conf.enable = BME68X_ENABLE;
    gas_heatr_conf.heatr_temp = heaterTemp;
    gas_heatr_conf.heatr_dur = heaterTime;
  }

  int8_t rslt =
      bme68x_set_heatr_conf(op_mode, &gas_heatr_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetHeaterConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Enable and configure gas reading + heater
 *  @param  temp_prof
 *          Desired temperature profile in degrees Centigrade
 *  @param  mul_prof
 *          Time profile to keep heater on. Multiplicators.
 *  @return True on success, False on failure
 */
bool Adafruit_BME688::setGasHeaterProfile(uint16_t temp_prof[10], uint16_t mul_prof[10]) {
  const uint8_t op_mode = BME688_DEFAULT_OP_MODE;
  if (op_mode==BME68X_PARALLEL_MODE || op_mode==BME68X_SEQUENTIAL_MODE){
    gas_heatr_conf.enable = BME68X_ENABLE;
    gas_heatr_conf.heatr_temp_prof = temp_prof;
    gas_heatr_conf.heatr_dur_prof = mul_prof;

      /* Shared heating duration in milliseconds */
    gas_heatr_conf.shared_heatr_dur = 140 - (bme68x_get_meas_dur(op_mode, &gas_conf, &gas_sensor) / 1000);
    // gas_heatr_conf.shared_heatr_dur = 150;
    Serial.print("Shared heater dur: ");
    Serial.println(gas_heatr_conf.shared_heatr_dur);
    gas_heatr_conf.profile_len = 1;

    int8_t rslt =
        bme68x_set_heatr_conf(op_mode, &gas_heatr_conf, &gas_sensor);  
  
  #ifdef BME680_DEBUG
    Serial.print(F("SetHeaterConf Result: "));
    Serial.println(rslt);
  #endif
    return rslt == 0;
  }
  else{
    return false;
  }
}

/*!
 *  @brief  Setter for Output Data Rate
 *  @param  odr
 *          Output data rate setting, can be BME68X_ODR_NONE,
 * BME68X_ODR_0_59_MS, BME68X_ODR_10_MS, BME68X_ODR_20_MS, BME68X_ODR_62_5_MS,
 * BME68X_ODR_125_MS, BME68X_ODR_250_MS, BME68X_ODR_500_MS, BME68X_ODR_1000_MS
 *  @return True on success, False on failure
 */

bool Adafruit_BME688::setODR(uint8_t odr) {
  if (odr > BME68X_ODR_NONE)
    return false;

  gas_conf.odr = odr;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Setter for Temperature oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Temperature
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */

bool Adafruit_BME688::setTemperatureOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X)
    return false;

  gas_conf.os_temp = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Setter for Humidity oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Humidity
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */
bool Adafruit_BME688::setHumidityOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X)
    return false;

  gas_conf.os_hum = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Setter for Pressure oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Pressure
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */
bool Adafruit_BME688::setPressureOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X)
    return false;

  gas_conf.os_pres = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Setter for IIR filter.
 *  @param  filtersize
 *          Size of the filter (in samples).
 *          Can be BME68X_FILTER_SIZE_0 (no filtering), BME68X_FILTER_SIZE_1,
 * BME68X_FILTER_SIZE_3, BME68X_FILTER_SIZE_7, BME68X_FILTER_SIZE_15,
 * BME68X_FILTER_SIZE_31, BME68X_FILTER_SIZE_63, BME68X_FILTER_SIZE_127
 *  @return True on success, False on failure
 */
bool Adafruit_BME688::setIIRFilterSize(uint8_t filtersize) {
  if (filtersize > BME68X_FILTER_SIZE_127)
    return false;
  gas_conf.filter = filtersize;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Reads 8 bit values over I2C
 */
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf) {

  Adafruit_I2CDevice *_dev = (Adafruit_I2CDevice *)intf;

  if (!_dev->write_then_read(&reg_addr, 1, reg_data, len, true)) {
    return -1;
  }

  return 0;
}

/*!
 *  @brief  Writes 8 bit values over I2C
 */
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                 void *intf) {
  Adafruit_I2CDevice *_dev = (Adafruit_I2CDevice *)intf;

  if (!_dev->write((uint8_t *)reg_data, len, true, &reg_addr, 1)) {
    return -1;
  }
  return 0;
}

/*!
 *  @brief  Reads 8 bit values over SPI
 */
static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr) {
  Adafruit_SPIDevice *_dev = (Adafruit_SPIDevice *)intf_ptr;

  reg_addr |= 0x80;

  if (!_dev->write_then_read(&reg_addr, 1, reg_data, len, 0x0)) {
    return -1;
  }

  return 0;
}

/*!
 *  @brief  Writes 8 bit values over SPI
 */
static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr) {
  Adafruit_SPIDevice *_dev = (Adafruit_SPIDevice *)intf_ptr;

  if (!_dev->write((uint8_t *)reg_data, len, &reg_addr, 1)) {
    return -1;
  }

  return 0;
}

static void delay_usec(uint32_t us, void *intf_ptr) {
  delayMicroseconds(us);
  yield();
}


/*!
 *  @brief  Setter for Temperature oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Temperature
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */

int8_t Adafruit_BME688::get_conf() {
  int8_t rslt = bme68x_get_conf(&gas_conf, &gas_sensor);
  return rslt;
}
