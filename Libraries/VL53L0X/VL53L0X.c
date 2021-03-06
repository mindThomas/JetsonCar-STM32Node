// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include "VL53L0X.h"
#include "cmsis_os.h"
#include "main.h"

// Defines /////////////////////////////////////////////////////////////////////

// A 7-bit number is used for the address which defines the upper 7-bit in the address.
// The last bit defines whether the transaction is a read or write
#define ADDRESS_DEFAULT 0b0101001

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = xTaskGetTickCount())

// Check if timeout is enabled (set to nonzero value) and has expired
#define VL53L0X_checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)xTaskGetTickCount() - timeout_start_ms) > io_timeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define VL53L0X_decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// Constructors ////////////////////////////////////////////////////////////////

// TCC: Target CentreCheck
// MSRC: Minimum Signal Rate Check
// DSS: Dynamic Spad Selection

typedef struct SequenceStepEnables
{
	bool tcc, msrc, dss, pre_range, final_range;
} SequenceStepEnables;

typedef struct SequenceStepTimeouts
{
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} SequenceStepTimeouts;

uint8_t address;
uint16_t io_timeout;
bool did_timeout;
uint16_t timeout_start_ms;

uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
uint32_t measurement_timing_budget_us;

uint8_t VL53L0X_last_status;

bool VL53L0X_getSpadInfo(uint8_t * count, bool * type_is_aperture);

void VL53L0X_getSequenceStepEnables(SequenceStepEnables * enables);
void VL53L0X_getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

bool VL53L0X_performSingleRefCalibration(uint8_t vhv_init_byte);

static uint16_t VL53L0X_decodeTimeout(uint16_t value);
static uint16_t VL53L0X_encodeTimeout(uint16_t timeout_mclks);
static uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

// Public Methods //////////////////////////////////////////////////////////////

uint8_t VL53L0X_getAddress(void) { return address; }
void VL53L0X_setTimeout(uint16_t timeout) { io_timeout = timeout; }
uint16_t VL53L0X_getTimeout(void) { return io_timeout; }

void VL53L0X_setAddress(uint8_t new_addr)
{
  VL53L0X_writeReg(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
  address = new_addr;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool VL53L0X_init(bool io_2v8)
{
  // VL53L0X_DataInit() begin

	address = ADDRESS_DEFAULT;
	io_timeout = 0; // no timeout
	did_timeout = false;

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    VL53L0X_writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
      VL53L0X_readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
  }

  // "Set I2C standard mode"
  VL53L0X_writeReg(0x88, 0x00);

  VL53L0X_writeReg(0x80, 0x01);
  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);
  stop_variable = VL53L0X_readReg(0x91);
  VL53L0X_writeReg(0x00, 0x01);
  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  VL53L0X_writeReg(MSRC_CONFIG_CONTROL, VL53L0X_readReg(MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  VL53L0X_setSignalRateLimit(0.25);

  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!VL53L0X_getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  VL53L0X_readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  VL53L0X_writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  VL53L0X_writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x09, 0x00);
  VL53L0X_writeReg(0x10, 0x00);
  VL53L0X_writeReg(0x11, 0x00);

  VL53L0X_writeReg(0x24, 0x01);
  VL53L0X_writeReg(0x25, 0xFF);
  VL53L0X_writeReg(0x75, 0x00);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x4E, 0x2C);
  VL53L0X_writeReg(0x48, 0x00);
  VL53L0X_writeReg(0x30, 0x20);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x30, 0x09);
  VL53L0X_writeReg(0x54, 0x00);
  VL53L0X_writeReg(0x31, 0x04);
  VL53L0X_writeReg(0x32, 0x03);
  VL53L0X_writeReg(0x40, 0x83);
  VL53L0X_writeReg(0x46, 0x25);
  VL53L0X_writeReg(0x60, 0x00);
  VL53L0X_writeReg(0x27, 0x00);
  VL53L0X_writeReg(0x50, 0x06);
  VL53L0X_writeReg(0x51, 0x00);
  VL53L0X_writeReg(0x52, 0x96);
  VL53L0X_writeReg(0x56, 0x08);
  VL53L0X_writeReg(0x57, 0x30);
  VL53L0X_writeReg(0x61, 0x00);
  VL53L0X_writeReg(0x62, 0x00);
  VL53L0X_writeReg(0x64, 0x00);
  VL53L0X_writeReg(0x65, 0x00);
  VL53L0X_writeReg(0x66, 0xA0);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x22, 0x32);
  VL53L0X_writeReg(0x47, 0x14);
  VL53L0X_writeReg(0x49, 0xFF);
  VL53L0X_writeReg(0x4A, 0x00);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x7A, 0x0A);
  VL53L0X_writeReg(0x7B, 0x00);
  VL53L0X_writeReg(0x78, 0x21);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x23, 0x34);
  VL53L0X_writeReg(0x42, 0x00);
  VL53L0X_writeReg(0x44, 0xFF);
  VL53L0X_writeReg(0x45, 0x26);
  VL53L0X_writeReg(0x46, 0x05);
  VL53L0X_writeReg(0x40, 0x40);
  VL53L0X_writeReg(0x0E, 0x06);
  VL53L0X_writeReg(0x20, 0x1A);
  VL53L0X_writeReg(0x43, 0x40);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x34, 0x03);
  VL53L0X_writeReg(0x35, 0x44);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x31, 0x04);
  VL53L0X_writeReg(0x4B, 0x09);
  VL53L0X_writeReg(0x4C, 0x05);
  VL53L0X_writeReg(0x4D, 0x04);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x44, 0x00);
  VL53L0X_writeReg(0x45, 0x20);
  VL53L0X_writeReg(0x47, 0x08);
  VL53L0X_writeReg(0x48, 0x28);
  VL53L0X_writeReg(0x67, 0x00);
  VL53L0X_writeReg(0x70, 0x04);
  VL53L0X_writeReg(0x71, 0x01);
  VL53L0X_writeReg(0x72, 0xFE);
  VL53L0X_writeReg(0x76, 0x00);
  VL53L0X_writeReg(0x77, 0x00);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x0D, 0x01);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x01);
  VL53L0X_writeReg(0x01, 0xF8);

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x8E, 0x01);
  VL53L0X_writeReg(0x00, 0x01);
  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  VL53L0X_writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  VL53L0X_writeReg(GPIO_HV_MUX_ACTIVE_HIGH, VL53L0X_readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  VL53L0X_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  measurement_timing_budget_us = VL53L0X_getMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  VL53L0X_setMeasurementTimingBudget(measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!VL53L0X_performSingleRefCalibration(0x40)) { return false; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!VL53L0X_performSingleRefCalibration(0x00)) { return false; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  return true;
}

// Write an 8-bit register
void VL53L0X_writeReg(uint8_t reg, uint8_t value)
{
  uint8_t buf[] = {reg, value};

  VL53L0X_last_status = 0; // sucess
  if (HAL_I2C_Master_Transmit(&hi2c2, address << 1, buf, 2, VL53L0X_I2C_TIMEOUT) != HAL_OK) {
	  VL53L0X_last_status = 0xFF;
  }
}

// Write a 16-bit register
void VL53L0X_writeReg16Bit(uint8_t reg, uint16_t value)
{
  uint8_t buf[] = {reg, (value >> 8) & 0xFF, value & 0xFF};

  VL53L0X_last_status = 0; // sucess
  if (HAL_I2C_Master_Transmit(&hi2c2, address << 1, buf, 3, VL53L0X_I2C_TIMEOUT) != HAL_OK) {
	  VL53L0X_last_status = 0xFF;
  }
}

// Write a 32-bit register
void VL53L0X_writeReg32Bit(uint8_t reg, uint32_t value)
{
  uint8_t buf[] = {reg, (value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >>  8) & 0xFF, value & 0xFF};

  VL53L0X_last_status = 0; // sucess
  if (HAL_I2C_Master_Transmit(&hi2c2, address << 1, buf, 5, VL53L0X_I2C_TIMEOUT) != HAL_OK) {
	  VL53L0X_last_status = 0xFF;
  }
}

// Read an 8-bit register
uint8_t VL53L0X_readReg(uint8_t reg)
{
  uint8_t value;

  uint8_t ret = 0; // sucess
  HAL_StatusTypeDef errCode;

  //__disable_irq();

  //while(HAL_I2C_IsDeviceReady(mpu_i2c_handle, MPU_Addr << 1, trials, timeout) != HAL_OK) ;
  errCode = HAL_I2C_Master_Transmit(&hi2c2, address << 1, &reg, 1, VL53L0X_I2C_TIMEOUT);
  if (errCode != HAL_OK) {
  	ret = 0xFF;
    goto error_w;
  }
  if (HAL_I2C_Master_Receive(&hi2c2, address << 1, &value, 1, VL53L0X_I2C_TIMEOUT) != HAL_OK) {
  	ret = 0xFF;
  	goto error_w;
  }

  return value;

error_w:
  //__enable_irq();
	VL53L0X_last_status = ret;
	return 0;
}

// Read a 16-bit register
uint16_t VL53L0X_readReg16Bit(uint8_t reg)
{
  uint16_t value;
  uint8_t buffer[2];

  uint8_t ret = 0; // sucess
  HAL_StatusTypeDef errCode;

  //__disable_irq();

  //while(HAL_I2C_IsDeviceReady(mpu_i2c_handle, MPU_Addr << 1, trials, timeout) != HAL_OK) ;
  errCode = HAL_I2C_Master_Transmit(&hi2c2, address << 1, &reg, 1, VL53L0X_I2C_TIMEOUT);
  if (errCode != HAL_OK) {
  	ret = 0xFF;
    goto error_w;
  }
  if (HAL_I2C_Master_Receive(&hi2c2, address << 1, buffer, 2, VL53L0X_I2C_TIMEOUT) != HAL_OK) {
  	ret = 0xFF;
  	goto error_w;
  }

  value  = (uint16_t)buffer[0] << 8; // value high byte
  value |=           buffer[1];      // value low byte

  return value;

error_w:
  //__enable_irq();
	VL53L0X_last_status = ret;
	return 0;
}

// Read a 32-bit register
uint32_t VL53L0X_readReg32Bit(uint8_t reg)
{
  uint32_t value;
  uint8_t buffer[4];

  uint8_t ret = 0; // sucess
  HAL_StatusTypeDef errCode;

  //__disable_irq();

  //while(HAL_I2C_IsDeviceReady(mpu_i2c_handle, MPU_Addr << 1, trials, timeout) != HAL_OK) ;
  errCode = HAL_I2C_Master_Transmit(&hi2c2, address << 1, &reg, 1, VL53L0X_I2C_TIMEOUT);
  if (errCode != HAL_OK) {
  	ret = 0xFF;
    goto error_w;
  }
  if (HAL_I2C_Master_Receive(&hi2c2, address << 1, buffer, 4, VL53L0X_I2C_TIMEOUT) != HAL_OK) {
  	ret = 0xFF;
  	goto error_w;
  }

  value  = (uint32_t)buffer[0] << 24; // value highest byte
  value |= (uint32_t)buffer[1] << 16;
  value |= (uint16_t)buffer[2] <<  8;
  value |=           buffer[3];       // value lowest byte

  return value;

error_w:
  //__enable_irq();
	VL53L0X_last_status = ret;
	return 0;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void VL53L0X_writeMulti(uint8_t reg, uint8_t const * src, uint8_t count)
{
  uint8_t ret = 0; // sucess
  uint8_t * buf = (uint8_t *)pvPortMalloc(count+1);
  uint8_t i;

  buf[0] = reg;
  for (i = 0; i < count; i++) {
  	buf[i+1] = src[i];
  }

  if (HAL_I2C_Master_Transmit(&hi2c2, address << 1, buf, (count + 1), VL53L0X_I2C_TIMEOUT) != HAL_OK) {
	  VL53L0X_last_status = 0xFF;
  }
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void VL53L0X_readMulti(uint8_t reg, uint8_t * dst, uint8_t count)
{
  uint8_t ret = 0; // sucess
  HAL_StatusTypeDef errCode;

  //__disable_irq();

  //while(HAL_I2C_IsDeviceReady(mpu_i2c_handle, MPU_Addr << 1, trials, timeout) != HAL_OK) ;
  errCode = HAL_I2C_Master_Transmit(&hi2c2, address << 1, &reg, 1, VL53L0X_I2C_TIMEOUT);
  if (errCode != HAL_OK) {
  	ret = 0xFF;
    goto error_w;
  }
  if (HAL_I2C_Master_Receive(&hi2c2, address << 1, dst, count, VL53L0X_I2C_TIMEOUT) != HAL_OK) {
  	ret = 0xFF;
  	goto error_w;
  }

error_w:
  //__enable_irq();
	VL53L0X_last_status = ret;
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool VL53L0X_setSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  VL53L0X_writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}

// Get the return signal rate limit check value in MCPS
float VL53L0X_getSignalRateLimit(void)
{
  return (float)VL53L0X_readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 xTaskGetTickCounteconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool VL53L0X_setMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  VL53L0X_getSequenceStepEnables(&enables);
  VL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
      VL53L0X_timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    VL53L0X_writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      VL53L0X_encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t VL53L0X_getMeasurementTimingBudget(void)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  VL53L0X_getSequenceStepEnables(&enables);
  VL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool VL53L0X_setVcselPulsePeriod(VL53L0X_vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  VL53L0X_getSequenceStepEnables(&enables);
  VL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VL53L0X_VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        VL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        VL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        VL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        VL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    VL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    VL53L0X_writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      VL53L0X_timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    VL53L0X_writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      VL53L0X_encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      VL53L0X_timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    VL53L0X_writeReg(MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VL53L0X_VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        VL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        VL53L0X_writeReg(0xFF, 0x01);
        VL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x30);
        VL53L0X_writeReg(0xFF, 0x00);
        break;

      case 10:
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        VL53L0X_writeReg(0xFF, 0x01);
        VL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_writeReg(0xFF, 0x00);
        break;

      case 12:
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        VL53L0X_writeReg(0xFF, 0x01);
        VL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_writeReg(0xFF, 0x00);
        break;

      case 14:
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        VL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        VL53L0X_writeReg(0xFF, 0x01);
        VL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_writeReg(0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    VL53L0X_writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      VL53L0X_timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    VL53L0X_writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      VL53L0X_encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  VL53L0X_setMeasurementTimingBudget(measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = VL53L0X_readReg(SYSTEM_SEQUENCE_CONFIG);
  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  VL53L0X_performSingleRefCalibration(0x0);
  VL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t VL53L0X_getVcselPulsePeriod(VL53L0X_vcselPeriodType type)
{
  if (type == VL53L0X_VcselPeriodPreRange)
  {
    return VL53L0X_decodeVcselPeriod(VL53L0X_readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VL53L0X_VcselPeriodFinalRange)
  {
    return VL53L0X_decodeVcselPeriod(VL53L0X_readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in xTaskGetTickCounteconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void VL53L0X_startContinuous(uint32_t period_ms)
{
  VL53L0X_writeReg(0x80, 0x01);
  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);
  VL53L0X_writeReg(0x91, stop_variable);
  VL53L0X_writeReg(0x00, 0x01);
  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodxTaskGetTickCounteconds() begin

    uint16_t osc_calibrate_val = VL53L0X_readReg16Bit(OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    VL53L0X_writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    // VL53L0X_SetInterMeasurementPeriodmilliseconds() end

    VL53L0X_writeReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    VL53L0X_writeReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void VL53L0X_stopContinuous(void)
{
  VL53L0X_writeReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);
  VL53L0X_writeReg(0x91, 0x00);
  VL53L0X_writeReg(0x00, 0x01);
  VL53L0X_writeReg(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t VL53L0X_readRangeContinuousMillimeters(void)
{
  startTimeout();
  while ((VL53L0X_readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (VL53L0X_checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = VL53L0X_readReg16Bit(RESULT_RANGE_STATUS + 10);

  VL53L0X_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t VL53L0X_readRangeSingleMillimeters(void)
{
  VL53L0X_writeReg(0x80, 0x01);
  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);
  VL53L0X_writeReg(0x91, stop_variable);
  VL53L0X_writeReg(0x00, 0x01);
  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x00);

  VL53L0X_writeReg(SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout();
  while (VL53L0X_readReg(SYSRANGE_START) & 0x01)
  {
    if (VL53L0X_checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  return VL53L0X_readRangeContinuousMillimeters();
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool VL53L0X_timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool VL53L0X_getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  VL53L0X_writeReg(0x80, 0x01);
  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x00);

  VL53L0X_writeReg(0xFF, 0x06);
  VL53L0X_writeReg(0x83, VL53L0X_readReg(0x83) | 0x04);
  VL53L0X_writeReg(0xFF, 0x07);
  VL53L0X_writeReg(0x81, 0x01);

  VL53L0X_writeReg(0x80, 0x01);

  VL53L0X_writeReg(0x94, 0x6b);
  VL53L0X_writeReg(0x83, 0x00);
  startTimeout();
  while (VL53L0X_readReg(0x83) == 0x00)
  {
    if (VL53L0X_checkTimeoutExpired()) { return false; }
  }
  VL53L0X_writeReg(0x83, 0x01);
  tmp = VL53L0X_readReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  VL53L0X_writeReg(0x81, 0x00);
  VL53L0X_writeReg(0xFF, 0x06);
  VL53L0X_writeReg(0x83, VL53L0X_readReg(0x83)  & ~0x04);
  VL53L0X_writeReg(0xFF, 0x01);
  VL53L0X_writeReg(0x00, 0x01);

  VL53L0X_writeReg(0xFF, 0x00);
  VL53L0X_writeReg(0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void VL53L0X_getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = VL53L0X_readReg(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void VL53L0X_getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(VL53L0X_VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = VL53L0X_readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    VL53L0X_timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    VL53L0X_decodeTimeout(VL53L0X_readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    VL53L0X_timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(VL53L0X_VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    VL53L0X_decodeTimeout(VL53L0X_readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    VL53L0X_timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t VL53L0X_decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t VL53L0X_encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
bool VL53L0X_performSingleRefCalibration(uint8_t vhv_init_byte)
{
  VL53L0X_writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  while ((VL53L0X_readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (VL53L0X_checkTimeoutExpired()) { return false; }
  }

  VL53L0X_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  VL53L0X_writeReg(SYSRANGE_START, 0x00);

  return true;
}



#define LONG_RANGE
#define HIGH_ACCURACY
#include <string.h>
uint16_t range;
osThreadId I2C_TestTaskHandle;

void VL53L0X_LaunchTest(void)
{
	osThreadDef(I2C_TestTask, VL53L0X_TestTask, osPriorityNormal, 0, 256);
	I2C_TestTaskHandle = osThreadCreate(osThread(I2C_TestTask), NULL);
}

void VL53L0X_TestTask(void const * argument)
{
	uint8_t buffer[50];
	while (1) {
		osDelay(100);
	}

	VL53L0X_init(true);
	VL53L0X_setTimeout(500);
	/*VL53L0X_startContinuous(0);

	while (1) {
		range = VL53L0X_readRangeContinuousMillimeters();
		//osDelay(100);
	}*/

#ifdef LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
	VL53L0X_setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
	VL53L0X_setVcselPulsePeriod(VL53L0X_VcselPeriodPreRange, 18);
	VL53L0X_setVcselPulsePeriod(VL53L0X_VcselPeriodFinalRange, 14);
#endif

#ifdef HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
	VL53L0X_setMeasurementTimingBudget(20000);
#elif defined(HIGH_ACCURACY)
  // increase timing budget to 200 ms
	VL53L0X_setMeasurementTimingBudget(200000);
#endif

	while (1) {
		range = VL53L0X_readRangeSingleMillimeters();
		sprintf(buffer, "Distance: %d\n", range);
		//CDC_Transmit_FS(buffer, strlen(buffer));
		osDelay(100);
	}
}
