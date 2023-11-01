#include "espAds1x.h"

ADS1x::ADS1x() {
  // Initialize Conversion buffer
  _ptrConvBuff = new int16_t[ADS1x_CONV_BUF_SIZE];
  _iBuffCnt = -1;
  _iBuffMaxFillIndex = 0;
  _iValFroozenDebCnt = 0;

  for (int i_elem=0; i_elem<ADS1x_CONV_BUF_SIZE; i_elem++){_ptrConvBuff[i_elem]=0;}

  // Initialize I2C
  ESP_ERROR_CHECK(initI2CMaster());
  setDefault();
}

void ADS1x::setDefault() {
  /**
   * Bring ADS1x back to default settings
  */
  getRegisterValue(ADS1x_CONFIG_REG, &_iI2cConfigRegValue);

  setCompPolarity(ADS1x_CMP_POL_SETTINGS);
  setMux(ADS1x_MUX_SETTINGS);
  setRate(ADS1x_MEAS_RATE);
  setPGA(ADS1x_PGA_SETTINGS);
  setCompLatchingMode(ADS1x_CMP_LAT_SETTINGS);
  setPinRdyMode(ADS1x_CMP_QUE_SETTINGS);
  setOpMode(ADS1x_MEAS_MODE_SETTINGS);
  setRegisterValue(ADS1x_CONFIG_REG, _iI2cConfigRegValue);
  printConfigReg();
}


void ADS1x::startSingleShotMeas() {
  /**
   * Single-shot conversion start
   * This bit determines the operational status of the device. OS can only be written when in power-down state and has
   * no effect when a conversion is ongoing.
   *
  */
    getRegisterValue(ADS1x_CONFIG_REG, &_iI2cConfigRegValue);
    writeBit(_iI2cConfigRegValue, ADS1x_OS, true);
    setRegisterValue(ADS1x_CONFIG_REG, _iI2cConfigRegValue);
};

bool ADS1x::getOpStatus(void){
  /**
   * Get Operational status
   * @return: 0 : Device is currently performing a conversion, 1 : Device is not currently performing a conversion
  */
  getRegisterValue(ADS1x_CONFIG_REG, &_iI2cConfigRegValue);
  return (_iI2cConfigRegValue & 1<<ADS1x_OS) >> ADS1x_OS;
}


void ADS1x::setMux(uint8_t b_mux) {
  /**
   * Set input multiplexer configuration
   * @param b_mux:
   *    ADS1x_MUX_AIN0_AIN1 AINp = AIN0 and AINn = AIN1
   *    ADS1x_MUX_AIN0_AIN3 AINp = AIN0 and AINn = AIN3
   *    ADS1x_MUX_AIN1_AIN3 AINp = AIN1 and AINn = AIN3
   *    ADS1x_MUX_AIN2_AIN3 AINp = AIN2 and AINn = AIN3
   *    ADS1x_MUX_AIN0_GND AINp = AIN0 and AINn = GND
   *    ADS1x_MUX_AIN1_GND AINp = AIN1 and AINn = GND
   *    ADS1x_MUX_AIN2_GND AINp = AIN2 and AINn = GND
   *    ADS1x_MUX_AIN3_GND AINp = AIN3 and AINn = GND
  */
  bool b2 = readBit(b_mux, 2);
  bool b1 = readBit(b_mux, 1);
  bool b0 = readBit(b_mux, 0);

  writeBit(_iI2cConfigRegValue, ADS1x_MUX2, b2);
  writeBit(_iI2cConfigRegValue, ADS1x_MUX1, b1);
  writeBit(_iI2cConfigRegValue, ADS1x_MUX0, b0);
}


uint8_t ADS1x::getMux() {
  /**
   * Get Input multiplexer configuration
   * @return:
   *    0b000 AINp = AIN0 and AINn = AIN1
   *    0b001 AINp = AIN0 and AINn = AIN3
   *    0b010 AINp = AIN1 and AINn = AIN3
   *    0b011 AINp = AIN2 and AINn = AIN3
   *    0b100 AINp = AIN0 and AINn = GND
   *    0b101 AINp = AIN1 and AINn = GND
   *    0b110 AINp = AIN2 and AINn = GND
   *    0b111 AINp = AIN3 and AINn = GND
  */
  getRegisterValue(ADS1x_CONFIG_REG, &_iI2cConfigRegValue);
  return (_iI2cConfigRegValue & 0b111<<ADS1x_MUX0) >> ADS1x_MUX0;
}


void ADS1x::setPGA(uint8_t cmp_gain) {
  /**
   * Set the FSR of the programmable gain amplifier
   * @param cmp_gain:
   *    ADS1x_PGA_6P144 : FSR = +-6.144V
   *    ADS1x_PGA_4P096 : FSR = +-4.096V
   *    ADS1x_PGA_2P048 : FSR = +-2.048V
   *    ADS1x_PGA_1P024 : FSR = +-1.024V
   *    ADS1x_PGA_0P512 : FSR = +-0.512V
   *    ADS1x_PGA_0P256 : FSR = +-0.256V
  */

  bool b2 = readBit(cmp_gain, 2);
  bool b1 = readBit(cmp_gain, 1);
  bool b0 = readBit(cmp_gain, 0);

  writeBit(_iI2cConfigRegValue, ADS1x_PGA2, b2);
  writeBit(_iI2cConfigRegValue, ADS1x_PGA1, b1);
  writeBit(_iI2cConfigRegValue, ADS1x_PGA0, b0);
}


uint8_t ADS1x::getPGA() {
  /**
   * Get the FSR of the programmable gain amplifier
   * @return:
   *    0b000 : FSR = +-6.144V
   *    0b001 : FSR = +-4.096V
   *    0b010 : FSR = +-2.048V
   *    0b011 : FSR = +-1.024V
   *    0b100 : FSR = +-0.512V
   *    0b101 : FSR = +-0.256V
  */
  getRegisterValue(ADS1x_CONFIG_REG, &_iI2cConfigRegValue);

  return (_iI2cConfigRegValue & 0b111<<ADS1x_PGA0) >> ADS1x_PGA0;
}


void ADS1x::setOpMode(bool b_mode) {
  /**
   * Set the operating mode
   * @param b_mode
   *    ADS1x_MODE_CONTINUOUS : Continuous-conversion mode
   *    ADS1x_MODE_SINGLESHOT : Single-shot mode or power-down state (default)
  */
  writeBit(_iI2cConfigRegValue, ADS1x_MODE, b_mode);
}


uint8_t ADS1x::getOpMode() {
  /**
   * get operating mode
   * @return
   *    0 : Continuous-conversion mode
   *    1 : Single-shot mode or power-down state (default)
  */

  getRegisterValue(ADS1x_CONFIG_REG, &_iI2cConfigRegValue);
  return (_iI2cConfigRegValue & 1<<ADS1x_MODE) >> ADS1x_MODE;
}

void ADS1x::setRate(uint8_t cmp_rate) {
  /**
   * Set the data rate
   * @param cmp_rate
   *    ADS1x_RATE_8    : 8 samples per second  (SPS)
   *    ADS1x_RATE_16   : 16 SPS
   *    ADS1x_RATE_32   : 32 SPS
   *    ADS1x_RATE_64   : 64 SPS
   *    ADS1x_RATE_128  : 128 SPS (default)
   *    ADS1x_RATE_250  : 250 SPS
   *    ADS1x_RATE_475  : 475 SPS
   *    ADS1x_RATE_860  : 860 SPS
  */

  bool b2 = readBit(cmp_rate, 2);
  bool b1 = readBit(cmp_rate, 1);
  bool b0 = readBit(cmp_rate, 0);

  writeBit(_iI2cConfigRegValue, ADS1x_DR2, b2);
  writeBit(_iI2cConfigRegValue, ADS1x_DR1, b1);
  writeBit(_iI2cConfigRegValue, ADS1x_DR0, b0);
}


uint8_t ADS1x::getRate() {
  /**
   * Get the data rate
   * @return
   *    0b000 : 8 samples per second  (SPS)
   *    0b001 : 16 SPS
   *    0b010 : 32 SPS
   *    0b011 : 64 SPS
   *    0b100 : 128 SPS (default)
   *    0b101 : 250 SPS
   *    0b110 : 475 SPS
   *    0b111 : 860 SPS
  */

  getRegisterValue(ADS1x_CONFIG_REG, &_iI2cConfigRegValue);
  return (_iI2cConfigRegValue & 0b111<<ADS1x_DR0) >> ADS1x_DR0;
}


void ADS1x::setCompMode(bool b_mode) {
  /**
   * Set the comparator operating mode
   * @b_mode:
   *    ADS1x_CMP_MODE_TRADITIONAL  : Traditional comparator (default)
   *    ADS1x_CMP_MODE_WINDOW       : Window comparator
  */

  writeBit(_iI2cConfigRegValue, ADS1x_CMP_MDE, b_mode);
}


uint8_t ADS1x::getCompMode() {
   /**
   * Get the comparator operating mode
   * @return:
   *    0  : Traditional comparator (default)
   *    1  : Window comparator
  */

  getRegisterValue(ADS1x_CONFIG_REG, &_iI2cConfigRegValue);
  return (_iI2cConfigRegValue & 1<<ADS1x_CMP_MDE) >> ADS1x_CMP_MDE;
}


void ADS1x::setCompPolarity(bool b_polarity) {
  /**
   * Set polarity of the ALERT/RDY pin
   * @param b_polarity
   *    ADS1x_CMP_POL_ACTIVE_LOW  : Active low (default)
   *    ADS1x_CMP_POL_ACTIVE_HIGH : Active high
  */

  writeBit(_iI2cConfigRegValue, ADS1x_CMP_POL, b_polarity);
}


uint8_t ADS1x::getCompPolarity() {
  /**
   * Get polarity of the ALERT/RDY pin
   * @return
   *    0  : Active low (default)
   *    1  : Active high
  */

  getRegisterValue(ADS1x_CONFIG_REG, &_iI2cConfigRegValue);
  return (_iI2cConfigRegValue & (1<<ADS1x_CMP_POL)) >> ADS1x_CMP_POL;
}

void ADS1x::setCompLatchingMode(bool b_mode) {
  /**
   * Set whether the ALERT/RDY pin latches after being asserted or clears after conversions are within the margin of
   * the upper and lower threshold values.
   * @param b_mode
   *    ADS1x_CMP_LAT_NOT_ACTIVE : Nonlatching comparator . The ALERT/RDY pin does not latch when asserted (default).
   *    ADS1x_CMP_LAT_ACTIVE     : Latching comparator. The asserted ALERT/RDY pin remains latched until conversion data
   *                                 are read by the master or an appropriate SMBus alert response is sent by the master.
   *                                 The device responds with its address, and it is the lowest address currently asserting the
   *                                 ALERT/RDY bus line.
  */

  writeBit(_iI2cConfigRegValue, ADS1x_CMP_LAT, b_mode);
}


uint8_t ADS1x::getCompLatchingMode() {
   /**
   * Get whether the ALERT/RDY pin latches after being asserted or clears after conversions are within the margin of
   * the upper and lower threshold values.
   * @return
   *    0 : Nonlatching comparator. The ALERT/RDY pin does not latch when asserted (default).
   *    1 : Latching comparator. The asserted ALERT/RDY pin remains latched until conversion data
   *        are read by the master or an appropriate SMBus alert response is sent by the master.
   *        The device responds with its address, and it is the lowest address currently asserting the
   *        ALERT/RDY bus line.
  */

  getRegisterValue(ADS1x_CONFIG_REG, &_iI2cConfigRegValue);
  return (_iI2cConfigRegValue & (1<<ADS1x_CMP_LAT)) >> ADS1x_CMP_LAT;
}


void ADS1x::setCompQueueMode(uint8_t b_mode) {
  /**
   * Perform two functions. When set to ADS1x_CMP_DISABLE, the comparator is disabled and the ALERT/RDY pin is set to a high-impedance state.
   * When set to any other value, the ALERT/RDY pin and the comparator function are enabled, and the set value determines the
   * number of successive conversions exceeding the upper or lower threshold required before asserting the ALERT/RDY pin.
   * @param b_mode
   *    ADS1x_CMP_QUE_ASSERT_1_CONV : Assert after one conversion
   *    ADS1x_CMP_QUE_ASSERT_2_CONV : Assert after two conversions
   *    ADS1x_CMP_QUE_ASSERT_4_CONV : Assert after four conversions
   *    ADS1x_CMP_DISABLE           : Disable comparator and set ALERT/RDY pin to high-impedance (default)
  */
  bool b1 = readBit(b_mode, 1);
  bool b0 = readBit(b_mode, 0);

  writeBit(_iI2cConfigRegValue, ADS1x_CMP_QUE1, b1);
  writeBit(_iI2cConfigRegValue, ADS1x_CMP_QUE0, b0);
}


uint8_t ADS1x::getCompQueueMode() {
  /**
   * Get Queue mode of comparator. When set to 11, the comparator is disabled and the ALERT/RDY pin is set to a high-impedance state.
   * When set to any other value, the ALERT/RDY pin and the comparator function are enabled, and the set value determines the
   * number of successive conversions exceeding the upper or lower threshold required before asserting the ALERT/RDY pin.
   * @param b_mode
   *    0b00 : Assert after one conversion
   *    0b01 : Assert after two conversions
   *    0b10 : Assert after four conversions
   *    0b11 : Disable comparator and set ALERT/RDY pin to high-impedance (default)
  */

  getRegisterValue(ADS1x_CONFIG_REG, &_iI2cConfigRegValue);
  return (_iI2cConfigRegValue & (0b11<<ADS1x_CMP_QUE0)>>ADS1x_CMP_QUE0);
}


void ADS1x::setCompLowThreshBit(bool b_value, int i_bit_num){
  /**
   * Set the lower threshold values used by the comparator. The comparator is implemented as a digital comparator; therefore,
   * the values in these registers must be updated whenever the PGA settings are changed.
   * @param b_value: value for the low threshold register
   * @param i_bit_num: bit number to change, LSF bit is 0
  */

  getRegisterValue(ADS1x_LOW_THRESH_REG, &iLowThreshReg);

  writeBit(iLowThreshReg, i_bit_num, b_value);
  setRegisterValue(ADS1x_LOW_THRESH_REG, iLowThreshReg);
}

uint8_t ADS1x::getCompLowThreshBit(int i_bit_num){
  /**
   * Get the lower threshold values used by the comparator. The comparator is implemented as a digital comparator; therefore,
   * the values in these registers must be updated whenever the PGA settings are changed.
   * @param i_bit_num: bit number to read, LSF bit is 0
   * @return bit value on i_bit_num
  */

  iLowThreshReg = getRegisterValue(ADS1x_LOW_THRESH_REG, &iLowThreshReg);
  return readBit(iLowThreshReg, i_bit_num);
}


void ADS1x::setCompHighThreshBit(bool b_value, int i_bit_num){
  /**
   * Set the higher threshold values used by the comparator. The comparator is implemented as a digital comparator; therefore,
   * the values in these registers must be updated whenever the PGA settings are changed.
   * @param b_value: value for the high threshold register
   * @param i_bit_num: bit number to change, LSF bit is 0
  */

  iHighThreshReg = getRegisterValue(ADS1x_HIGH_THRESH_REG, &iHighThreshReg);
  writeBit(iHighThreshReg, i_bit_num, b_value);
  setRegisterValue(ADS1x_HIGH_THRESH_REG, iHighThreshReg);
}

uint8_t ADS1x::getCompHighThreshBit(int i_bit_num){
  /**
   * Get the high threshold values used by the comparator. The comparator is implemented as a digital comparator; therefore,
   * the values in these registers must be updated whenever the PGA settings are changed.
   * @param i_bit_num: bit number to read, LSF bit is 0
   * @return bit value on i_bit_num
  */

  getRegisterValue(ADS1x_HIGH_THRESH_REG, &iHighThreshReg);
  return readBit(iHighThreshReg, i_bit_num);
}


void ADS1x::setPinRdyMode(uint8_t b_comp_queue_mode){
  /**
   * Set pin ready mode. When set to RDY mode, the ALERT/RDY pin outputs the OS bit when in single-shot mode, and provides a
   * continuous-conversion ready pulse when in continuous-conversion mode. Latching comparator is activated in this mode.
   * @param b_activate
   *    ADS1x_CONV_READY_ACTIVE     : pin ready mode is activated
   *    ADS1x_CONV_READY_NOT_ACTIVE : pin ready mode is deactivated
  */

  setCompQueueMode(b_comp_queue_mode);

  iHighThreshReg = 0b1111111111111111;
  setRegisterValue(ADS1x_HIGH_THRESH_REG, iHighThreshReg);

  iLowThreshReg = 0b0000000000000000;
  setRegisterValue(ADS1x_LOW_THRESH_REG, iLowThreshReg);
}


uint8_t ADS1x::getPinRdyMode() {
  /**
   * Set pin ready mode. When set to RDY mode, the ALERT/RDY pin outputs the OS bit when in single-shot mode, and provides a
   * continuous-conversion ready pulse when in continuous-conversion mode. Latching comparator is activated in this mode.
   * @return
   *    true     : pin ready mode is activated
   *    false    : pin ready mode is deactivated
  */
  uint8_t b_cmp_queue_mode = getCompQueueMode();

   getRegisterValue(ADS1x_LOW_THRESH_REG, &iLowThreshReg);
   getRegisterValue(ADS1x_HIGH_THRESH_REG, &iHighThreshReg);

  if (!readBit(iLowThreshReg, 15) && readBit(iHighThreshReg, 15) && !(b_cmp_queue_mode==ADS1x_CMP_DISABLE)) {
    return true;
  } else {
    return false;
  }
}

void ADS1x::writeBit(uint16_t &i_register, int i_pos, bool b_value){
  /** Write a bit in a uint16_t number at a given position
   * @param i_register: uint16_t register as reference
   * @param i_pos: position of the bit
   * @param b_value: value of the bit, 0 or 1
  */
 if (b_value) {
   i_register |= (1<<i_pos);
 } else {
   i_register &= ~(1<<i_pos);
 }
}

bool ADS1x::readBit(uint16_t i_register, int i_pos){
  /**
   * read bit in uint16_t number at a given position
   * @param: i_register: number where to read out the bit
   * @param i_pos: bit position which should be read out
   * @return: bit value of given position
  */
  return ((i_register & (1<<i_pos)) >> i_pos);
}

uint16_t ADS1x::readConversionRegister() {
  /**
   * read conversion data from the conversion register as int value. Size can be maximum 16bit due to register length of the ADS1x
  */
  uint16_t conversion_value;
  getRegisterValue(ADS1x_CONVERSION_REG, &conversion_value);
  return conversion_value;
}


float ADS1x::getVoltVal() {
  /**
   * returns voltage level, based on the adc value of the ADS1x.
   * @return measured voltage
  */
  float f_conv_volt;

  f_conv_volt = getConvVal() * ADS1x_LSB_SETTINGS;

  return f_conv_volt;
}

int ADS1x::getLatestBufVal(){
  /**
   * @brief Get latest buffer value / latest conversion (unfiltered raw value)
   *
   */
  if (_iBuffCnt>=0){
    return (int)_ptrConvBuff[_iBuffCnt];
  }
  else{
    return 0;
  }
}


void ADS1x::setPhysConv(const float f_x_1, const float f_0) {
  /**
   * set factors for conversion from voltage to physical value
   * @param f_gradient: gradient of the conversion function
   * @param f_offset: (y-)Offset of the conversion function
  */
  initConvTable(1);

  _ptrConvTable[0][0] = 0.0;
  _ptrConvTable[0][1] = f_x_1;
  _ptrConvTable[0][2] = f_0;

}

void ADS1x::setPhysConv(const float f_x_2, const float f_x_1, const float f_0) {
  /**
   * set factors for conversion from voltage to physical value
   * @param f_gradient: gradient of the conversion function
   * @param f_offset: (y-)Offset of the conversion function
  */
  initConvTable(1);

  _ptrConvTable[0][0] = f_x_2;
  _ptrConvTable[0][1] = f_x_1;
  _ptrConvTable[0][2] = f_0;
}

void ADS1x::setPhysConv(const float arr_conv_table[][2], size_t i_size_conv) {
  /**
   * set factors for conversion from voltage to physical value
   * @param arr_conv_table: table for conversion, 1st dim is x value, 2nd dim is y value
   * @param i_size_conv: (row) size of conversion table
  */

  float f_prev_x;
  float f_act_x;
  float f_prev_y;
  float f_act_y;

  // Initialize member _ptrConvTable
  initConvTable(i_size_conv);

  // calculate gradient and offset and write it to array
  for (int i_row=1; i_row<i_size_conv; i_row++){
    f_prev_x = arr_conv_table[i_row-1][0];
    f_act_x = arr_conv_table[i_row][0];
    f_prev_y = arr_conv_table[i_row-1][1];
    f_act_y = arr_conv_table[i_row][1];

    // start range
    _ptrConvTable[i_row-1][0] = f_prev_x;
    // gradient
    _ptrConvTable[i_row-1][1] = (f_act_y-f_prev_y)/(f_act_x-f_prev_x);
    // offset
    _ptrConvTable[i_row-1][2] = f_prev_y - _ptrConvTable[i_row-1][1]*f_prev_x;
  }
}


float ADS1x::getPhysVal(void){
  /**
   * calculate physical value based on defined conversion and adc value
   * NOTE: readConversionRegister() must be called before to get adc value from ADS1x register over I2C
   * @return: physical value based on voltage read out
  */

  float f_voltage = getVoltVal();
  float f_physical = 0.F;

  if (_iSizeConvTable==1){
    // polynom or linear regression
    f_physical = f_voltage * f_voltage * _ptrConvTable[0][0] + f_voltage * _ptrConvTable[0][1] + _ptrConvTable[0][2];
  } else {

    if (f_voltage < _ptrConvTable[0][0]) {
      // left outside

    } else {
      // lookup table is given
      for (int i_idx = 1; i_idx < _iSizeConvTable; i_idx++) {
        if( (f_voltage >= _ptrConvTable[i_idx-1][0]) && (f_voltage < _ptrConvTable[i_idx][0]) ) {
          f_physical = f_voltage * _ptrConvTable[i_idx-1][1] + _ptrConvTable[i_idx-1][2];
          break;
        }
      }
    }
  }
  return f_physical;
}

void ADS1x::printConfigReg() {
  /**
   * Dump Config register to Serial output
  */
  ESP_LOGI(strLogTag, "ADS1x Conf.Reg.: %X", _iI2cConfigRegValue);
}

esp_err_t ADS1x::getRegisterValue(uint8_t i_reg, uint16_t * read_value) {
  /**
   * @brief Return a specified register value of ADS1x (only 2 uint8_t register are supported yet.)
   *
   * @param i_reg: Register to be readout
   */

  uint8_t ptr_data[2];
  esp_err_t esp_ret_value;

  esp_ret_value = i2c_master_write_read_device(ADS1x_I2C_PORT_NUM, _iI2cAddress, &i_reg, 1, ptr_data, 2,
                                               100 / portTICK_PERIOD_MS);
  if(esp_ret_value == ESP_ERR_TIMEOUT){
    ESP_LOGE(strLogTag, "Error Timeout in I2C communication.\n");
  } else if(esp_ret_value == ESP_FAIL){
    ESP_LOGE(strLogTag, "Error in I2C communication.Slave not acknowledged.\n");
  }
  *read_value = (uint16_t)(ptr_data[0]<<8) | ptr_data[1];

  return esp_ret_value;
}

void ADS1x::setRegisterValue(uint8_t i_reg, uint16_t i_data) {
  /**
   * @brief Return a specified register value of ADS1x (only 2 uint8_t register are supported yet.)
   *
   * @param i_reg: Register to be readout
   */

  uint8_t * ptr_data = new uint8_t[2];

  ptr_data[0] = _lowbyte(i_data);
  ptr_data[1] = _highbyte(i_data);
  sendI2CFrame(i_reg, ptr_data, 2);
}

esp_err_t ADS1x::initI2CMaster(void)
{
  i2c_port_t i2c_master_port = ADS1x_I2C_PORT_NUM;
  esp_err_t esp_err;

  i2c_config_t conf;

  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = ADS1x_I2C_SDA_PIN;
  conf.scl_io_num = ADS1x_I2C_SCL_PIN;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 100000;
  conf.clk_flags = 0;

  i2c_param_config(i2c_master_port, &conf);
  esp_err = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);

  return esp_err;
}

esp_err_t ADS1x::stop(void)
{
  esp_err_t esp_err;
  esp_err = i2c_driver_delete(ADS1x_I2C_PORT_NUM);

  return esp_err;
}


void ADS1x::sendI2CFrame(uint8_t i_reg, uint8_t* data_write, size_t data_len)
{
  // Link i2c ressource
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  // Change address register pointer of ADS
  // Put Start command in queue
  i2c_master_start(cmd);
  // Initiate communication with start address and indicating read request, no Acknoledgement
  i2c_master_write_byte(cmd, (_iI2cAddress<<1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);

  i2c_master_write_byte(cmd, i_reg, I2C_MASTER_ACK);

  for (int i_step=data_len; i_step>0; i_step--){
    // Write MSB from ADS1x and acknowledge it
    i2c_master_write_byte(cmd, *(data_write + i_step - 1), I2C_MASTER_ACK);
  }
  // Put Stop command in queue
  i2c_master_stop(cmd);

  // Execute all queued commands, 1000ms timeout
  i2c_master_cmd_begin(ADS1x_I2C_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
}


void ADS1x::initConvTable(size_t i_size_conv) {
  /**
   * Initialize pointer for conversion table
   * @param i_size_conv: row of the conversion table
  */

  // Make (row) size of conversion table in class available
  _iSizeConvTable=i_size_conv;
  // assign memory to the pointer, pointer in pointer element
  _ptrConvTable = new float*[_iSizeConvTable];

  // assign second pointer in pointer to get a 2dim field
  for(int i_row=0;i_row<_iSizeConvTable;i_row++) {
    _ptrConvTable[i_row]=new float[3];
  }
}

void ADS1x::activateFilter(){
  /**
   * @brief Activate the conversion filter and define Savitzky-Golay filter-variables
   *
   */

  _bFilterActive = true;
  //_iBuffMaxFillIndex=0;

  if (ADS1x_CONV_BUF_SIZE == 5){
      _ptrFilterCoeff = new float[5];
      _ptrFilterCoeff[0] = -3.0F;
      _ptrFilterCoeff[1] = 12.0F;
      _ptrFilterCoeff[2] = 17.0F;
      _ptrFilterCoeff[3] = 12.0F;
      _ptrFilterCoeff[4] = -3.0F;
      _fFilterNormCoeff = 35.F;
      _bSavGolFilterActive = true;
  } else if(ADS1x_CONV_BUF_SIZE == 7) {
      _ptrFilterCoeff = new float[7];
      _ptrFilterCoeff[0] = -2.0F;
      _ptrFilterCoeff[1] = 3.0F;
      _ptrFilterCoeff[2] = 6.0F;
      _ptrFilterCoeff[3] = 7.0F;
      _ptrFilterCoeff[4] = 6.0F;
      _ptrFilterCoeff[5] = 3.0F;
      _ptrFilterCoeff[6] = -2.0F;
      _fFilterNormCoeff = 21.F;
      _bSavGolFilterActive = true;
  } else if(ADS1x_CONV_BUF_SIZE == 9) {
      _ptrFilterCoeff = new float[9];
      _ptrFilterCoeff[0] = -21.0F;
      _ptrFilterCoeff[1] = 14.0F;
      _ptrFilterCoeff[2] = 39.0F;
      _ptrFilterCoeff[3] = 54.0F;
      _ptrFilterCoeff[4] = 59.0F;
      _ptrFilterCoeff[5] = 54.0F;
      _ptrFilterCoeff[6] = 39.0F;
      _ptrFilterCoeff[7] = 14.0F;
      _ptrFilterCoeff[8] = -21.0F;
      _fFilterNormCoeff = 231.F;
      _bSavGolFilterActive = true;
  } else if(ADS1x_CONV_BUF_SIZE == 11) {
      _ptrFilterCoeff = new float[11];
      _ptrFilterCoeff[0] = -36.0F;
      _ptrFilterCoeff[1] = 9.0F;
      _ptrFilterCoeff[2] = 44.0F;
      _ptrFilterCoeff[3] = 69.0F;
      _ptrFilterCoeff[4] = 84.0F;
      _ptrFilterCoeff[5] = 89.0F;
      _ptrFilterCoeff[6] = 84.0F;
      _ptrFilterCoeff[7] = 69.0F;
      _ptrFilterCoeff[8] = 44.0F;
      _ptrFilterCoeff[9] = 9.0F;
      _ptrFilterCoeff[10] = -36.0F;
      _fFilterNormCoeff = 429.F;
      _bSavGolFilterActive = true;
  } else {
      _bSavGolFilterActive = false;
  }
}


void ADS1x::deactivateFilter(){
  /**
   * @brief deactivat signal filter
   *
   */

  _bFilterActive = false;
}


bool ADS1x::getFilterStatus(){
  /**
   * @brief get actual filter status. True if filter is active
   *
   */

  return _bFilterActive;
}

int ADS1x::getAbsBufSize(){
  /**
   * @brief get the absolute buffer size of the filter
   *
   */

  return (int)ADS1x_CONV_BUF_SIZE;
}

int16_t* ADS1x::getBuffer(){
  /**
   * @brief get the pointer to the buffer
   *
   */

  return _ptrConvBuff;
}


float ADS1x::_getSavGolFilterVal(){
  /**
   * @brief get filtered value via Savitzky-Golay filter
   *
   */

  int i_index;
  float f_filter_value = 0.0F;

  for (int i_row=0; i_row<ADS1x_CONV_BUF_SIZE; i_row++){
    i_index = (_iBuffCnt+i_row) % ADS1x_CONV_BUF_SIZE;
    f_filter_value += (_ptrConvBuff[i_index]*_ptrFilterCoeff[i_index]);
  }
  f_filter_value /= _fFilterNormCoeff;

  return f_filter_value;
}


float ADS1x::_getAvgFilterVal(){
  /**
   * @brief get filtered value via average filter
   *
   */

  float f_filter_value = 0.0F;

  // when the filter is fully filled _iBuffMaxFillIndex is 1 below the filter size
  for (int i_row=0; i_row<=_iBuffMaxFillIndex; i_row++){
    f_filter_value += _ptrConvBuff[i_row];
  }

  f_filter_value /= (float)(_iBuffMaxFillIndex+1);

  return f_filter_value;
}

bool ADS1x::isValueFrozen(){
  /**
   * @brief check if the Sensor raw value is frozen. Is only active when the the filter is active.
   * An error is detected when the max value and the min value of te unfiltered filter-buffer is the same
   * (no change in the signal)
   *
   * @return: true: value is frozen; false: value is not frozen
   *
   */

  bool b_val_equal = false;
  bool b_status = false;

  if (_iBuffMaxFillIndex>=9){
    // filter is active and filled enough -> check if value is frozen

    int16_t i_last_val = _ptrConvBuff[0];
    b_val_equal = true;
    b_status = true;

    for (int i_row=1; i_row<=_iBuffMaxFillIndex; i_row++){
      // if two values are not the same, break the for loop and return false
      // _iBuffMaxFillIndex is a index not a counter
      if (_ptrConvBuff[i_row] != i_last_val){
        // values are different -> found change -> ok
        b_val_equal = false;
        break;
      }
      i_last_val = _ptrConvBuff[i_row]; // set last value to current value
    }

    if (b_val_equal){
      _iValFroozenDebCnt++;

      if (_iValFroozenDebCnt == ADS1x_DEB_VALUE_FROZEN) {
        b_status = false;
        ESP_LOGE(strLogTag, "Sensor raw values in buffer are equal. Last raw value: %d.\n", i_last_val);
        _iValFroozenDebCnt = 0;
      }
    } else {
      _iValFroozenDebCnt = 0;
    }
  }
  return b_status;
}


float ADS1x::getConvVal(){
  /**
   * @brief get the filtered conversion value
   *
   *
   */

  float f_conversion_value;

  // fill the filter buffer an increment the ring buffer counter
  _iBuffCnt = (_iBuffCnt+1) % ADS1x_CONV_BUF_SIZE; // ring buffer
  _iBuffMaxFillIndex = std::max(_iBuffMaxFillIndex,_iBuffCnt); // get fill index of filter. Used for error detection or filter selsction
  _ptrConvBuff[_iBuffCnt] = readConversionRegister(); // read the register

  // Filter
  if (_bFilterActive){
    // if filter is not fully filled for savitzky golay filter use average filter
    if (_bSavGolFilterActive && (_iBuffMaxFillIndex +1) == ADS1x_CONV_BUF_SIZE){
    // apply savitzky golay filter
      f_conversion_value = _getSavGolFilterVal();
    } else {
    // apply avg filter
      f_conversion_value = _getAvgFilterVal();
    }
  } else {
    f_conversion_value = (float)getLatestBufVal();
  }
  return f_conversion_value;
}


bool ADS1x::getConnectionStatus(){
  /**
   * @brief get Connection status of ADS1x. If true, connection is OK
   *
   */

  return _bConnectStatus;
}

uint8_t ADS1x::_lowbyte(uint16_t data){
  /**
   * @brief returns the low byte of a given data
   *
   */
  return (uint8_t)(data & 0xFF);
}

uint8_t ADS1x::_highbyte(uint16_t data){
  /**
   * @brief returns the high byte of a given data
   *
   */
  return (uint8_t)((data>>8) & 0xFF);
}