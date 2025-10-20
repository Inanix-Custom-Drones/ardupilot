/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RPM_config.h"

#if AP_RPM_AS5600_ENABLED

#include "RPM_AS5600.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_Servo_Telem/AP_Servo_Telem.h>

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_RPM_AS5600::AP_RPM_AS5600(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State &_state) :
    AP_RPM_Backend(_ap_rpm, instance, _state)
{
	if(ap_rpm._params[state.instance].as5600_busid >= 0){
		_dev = hal.i2c_mgr->get_device_ptr(
										ap_rpm._params[state.instance].as5600_busid, 
										ap_rpm._params[state.instance].as5600_addr);
										//ap_rpm._params[state.instance].as5600_busspeed);
		if (! _dev) {
			GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AS5600 Not found at %u:%u", unsigned(ap_rpm._params[state.instance].as5600_busid), unsigned(ap_rpm._params[state.instance].as5600_addr));
			//printf("AS5600 Not found at %u:%u\n", unsigned(ap_rpm._params[state.instance].as5600_busid), unsigned(ap_rpm._params[state.instance].as5600_addr));
			return;
		}
		 
		WITH_SEMAPHORE(_dev->get_semaphore());
        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
        _dev->set_retries(5);

		_dev->set_device_type(AS5600_DEFAULT_ADDRESS);
		
		GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AS5600 found at %u:%u", unsigned(ap_rpm._params[state.instance].as5600_busid), unsigned(ap_rpm._params[state.instance].as5600_addr));
		//printf("AS5600 found at %u:%u\n", unsigned(ap_rpm._params[state.instance].as5600_busid), unsigned(ap_rpm._params[state.instance].as5600_addr));
		uint8_t status = readStatus();
		if(status != 0){
			_connected = true;
			
			setConfigure(0x1f00);
			//setFastFilter(AS5600_FAST_FILT_LSB10);
			
			GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AS5600 status %u", unsigned(status));
			//printf("AS5600 status %u\n", unsigned(status));
		}else{
			GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AS5600 Issue reading status %u", unsigned(status));
			//printf("AS5600 Issue reading status %u\n", unsigned(status));
			delete _dev;
	        return;
		}
		//1000 Hz
        _dev->register_periodic_callback(ap_rpm._params[state.instance].as5600_timer,
                                        FUNCTOR_BIND_MEMBER(&AP_RPM_AS5600::_timer, void));
		
	}
}

void AP_RPM_AS5600::update(void)
{
	if(isConnected()){
	    state.rate_rpm = getAngularSpeed(AS5600_MODE_RPM, false);
	    state.signal_quality = 0.5f;
	    state.last_reading_ms = AP_HAL::millis();
		 
		//GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AS5600 motor pos %f at rpm %f", (_lastReadAngle * AS5600_RAW_TO_DEGREES), state.rate_rpm);
	}
}

void AP_RPM_AS5600::_timer(void)
{
	if(isConnected()){
		uint32_t timerStart = AP_HAL::micros64();
		if(calculationStartTimerActr == 0){
			calculationStartTimerActr = AP_HAL::millis();;
		}
		
		readAngle();
		
		//Telemetry du servo pour affichage de la pos du moteur
		AP_Servo_Telem *servoTelem = AP_Servo_Telem::get_singleton();
		
		const AP_Servo_Telem::TelemetryData telem_data {
	        .measured_position = _lastReadAngle * AS5600_RAW_TO_DEGREES,
			.speed = getAngularSpeed(AS5600_MODE_DEGREES, false),
	        .present_types = AP_Servo_Telem::TelemetryData::Types::MEASURED_POSITION |
	                         AP_Servo_Telem::TelemetryData::Types::SPEED
	    };
		servoTelem->update_telem_data(ap_rpm._params[state.instance].as5600_servoidx, telem_data);
		
		calculationTimeActr += AP_HAL::micros64() - timerStart;
		calculationIdxActr ++;
		if(calculationIdxActr >= NBRE_CALC_TIME_ACTR){
			float calculationAvg = calculationTimeActr/NBRE_CALC_TIME_ACTR;
			uint16_t timer = AP_HAL::millis() - calculationStartTimerActr;
			
			gcs().send_text(MAV_SEVERITY_DEBUG, "AS56 read time avg %f us %i loops %ims", calculationAvg, NBRE_CALC_TIME_ACTR, timer);
			calculationIdxActr = 0;
			calculationTimeActr = 0;
			calculationStartTimerActr = 0;
		}
	}
}

bool AP_RPM_AS5600::begin(uint8_t directionPin)
{
  setDirection(AS5600_CLOCK_WISE);

  if (! isConnected()) return false;
  return true;
}


bool AP_RPM_AS5600::isConnected()
{
  return _connected;
}


uint8_t AP_RPM_AS5600::getAddress()
{
  return _address;
}


/////////////////////////////////////////////////////////
//
//  CONFIGURATION REGISTERS + direction pin
//
void AP_RPM_AS5600::setDirection(uint8_t direction)
{
  _direction = direction;
}


uint8_t AP_RPM_AS5600::getDirection()
{
  return _direction;
}


uint8_t AP_RPM_AS5600::getZMCO()
{
  uint8_t value = readReg(AS5600_ZMCO);
  return value;
}


bool AP_RPM_AS5600::setZPosition(uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(AS5600_ZPOS, value);
  return true;
}


uint16_t AP_RPM_AS5600::getZPosition()
{
  uint16_t value = readReg2(AS5600_ZPOS) & 0x0FFF;
  return value;
}


bool AP_RPM_AS5600::setMPosition(uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(AS5600_MPOS, value);
  return true;
}


uint16_t AP_RPM_AS5600::getMPosition()
{
  uint16_t value = readReg2(AS5600_MPOS) & 0x0FFF;
  return value;
}


bool AP_RPM_AS5600::setMaxAngle(uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(AS5600_MANG, value);
  return true;
}


uint16_t AP_RPM_AS5600::getMaxAngle()
{
  uint16_t value = readReg2(AS5600_MANG) & 0x0FFF;
  return value;
}


/////////////////////////////////////////////////////////
//
//  CONFIGURATION
//
bool AP_RPM_AS5600::setConfigure(uint16_t value)
{
  if (value > 0x3FFF) return false;
  writeReg2(AS5600_CONF, value);
  return true;
}


uint16_t AP_RPM_AS5600::getConfigure()
{
  uint16_t value = readReg2(AS5600_CONF) & 0x3FFF;
  return value;
}


//  details configure
bool AP_RPM_AS5600::setPowerMode(uint8_t powerMode)
{
  if (powerMode > 3) return false;
  uint8_t value = readReg(AS5600_CONF + 1);
  value &= ~AS5600_CONF_POWER_MODE;
  value |= powerMode;
  writeReg(AS5600_CONF + 1, value);
  return true;
}


uint8_t AP_RPM_AS5600::getPowerMode()
{
  return readReg(AS5600_CONF + 1) & 0x03;
}


bool AP_RPM_AS5600::setHysteresis(uint8_t hysteresis)
{
  if (hysteresis > 3) return false;
  uint8_t value = readReg(AS5600_CONF + 1);
  value &= ~AS5600_CONF_HYSTERESIS;
  value |= (hysteresis << 2);
  writeReg(AS5600_CONF + 1, value);
  return true;
}


uint8_t AP_RPM_AS5600::getHysteresis()
{
  return (readReg(AS5600_CONF + 1) >> 2) & 0x03;
}


bool AP_RPM_AS5600::setOutputMode(uint8_t outputMode)
{
  if (outputMode > 2) return false;
  uint8_t value = readReg(AS5600_CONF + 1);
  value &= ~AS5600_CONF_OUTPUT_MODE;
  value |= (outputMode << 4);
  writeReg(AS5600_CONF + 1, value);
  return true;
}


uint8_t AP_RPM_AS5600::getOutputMode()
{
  return (readReg(AS5600_CONF + 1) >> 4) & 0x03;
}


bool AP_RPM_AS5600::setPWMFrequency(uint8_t pwmFreq)
{
  if (pwmFreq > 3) return false;
  uint8_t value = readReg(AS5600_CONF + 1);
  value &= ~AS5600_CONF_PWM_FREQUENCY;
  value |= (pwmFreq << 6);
  writeReg(AS5600_CONF + 1, value);
  return true;
}


uint8_t AP_RPM_AS5600::getPWMFrequency()
{
  return (readReg(AS5600_CONF + 1) >> 6) & 0x03;
}


bool AP_RPM_AS5600::setSlowFilter(uint8_t mask)
{
  if (mask > 3) return false;
  uint8_t value = readReg(AS5600_CONF);
  value &= ~AS5600_CONF_SLOW_FILTER;
  value |= mask;
  writeReg(AS5600_CONF, value);
  return true;
}


uint8_t AP_RPM_AS5600::getSlowFilter()
{
  return readReg(AS5600_CONF) & 0x03;
}


bool AP_RPM_AS5600::setFastFilter(uint8_t mask)
{
  if (mask > 7) return false;
  uint8_t value = readReg(AS5600_CONF);
  value &= ~AS5600_CONF_FAST_FILTER;
  value |= (mask << 2);
  writeReg(AS5600_CONF, value);
  return true;
}


uint8_t AP_RPM_AS5600::getFastFilter()
{
  return (readReg(AS5600_CONF) >> 2) & 0x07;
}


bool AP_RPM_AS5600::setWatchDog(uint8_t mask)
{
  if (mask > 1) return false;
  uint8_t value = readReg(AS5600_CONF);
  value &= ~AS5600_CONF_WATCH_DOG;
  value |= (mask << 5);
  writeReg(AS5600_CONF, value);
  return true;
}


uint8_t AP_RPM_AS5600::getWatchDog()
{
  return (readReg(AS5600_CONF) >> 5) & 0x01;
}


/////////////////////////////////////////////////////////
//
//  OUTPUT REGISTERS
//
uint16_t AP_RPM_AS5600::rawAngle()
{
  int16_t value = readReg2(AS5600_RAW_ANGLE);
  if (getOffset() > 0) value += getOffset();
  value &= 0x0FFF;

  if ((_directionPin == AS5600_SW_DIRECTION_PIN) &&
      (_direction == AS5600_COUNTERCLOCK_WISE))
  {
    value = (AS5600_RESO - value) & 0x0FFF;
  }
  return value;
}


uint16_t AP_RPM_AS5600::readAngle()
{
  uint16_t value = readReg2(AS5600_ANGLE);
  if (_error != AS5600_OK)
  {
    return _lastReadAngle;
  }
  if (getOffset() > 0) value += getOffset();
  value &= 0x0FFF;

  if ((_directionPin == AS5600_SW_DIRECTION_PIN) &&
      (_direction == AS5600_COUNTERCLOCK_WISE))
  {
    //  mask needed for value == 0.
    value = (AS5600_RESO - value) & 0x0FFF;
  }
  _lastReadAngle = value;
  return value;
}


float AP_RPM_AS5600::getOffset()
{
	return ap_rpm._params[state.instance].as5600_offset;
}


/////////////////////////////////////////////////////////
//
//  STATUS REGISTERS
//
uint8_t AP_RPM_AS5600::readStatus()
{
  uint8_t value = readReg(AS5600_STATUS);
  return value;
}


uint8_t AP_RPM_AS5600::readAGC()
{
  uint8_t value = readReg(AS5600_AGC);
  return value;
}


uint16_t AP_RPM_AS5600::readMagnitude()
{
  uint16_t value = readReg2(AS5600_MAGNITUDE) & 0x0FFF;
  return value;
}


bool AP_RPM_AS5600::detectMagnet()
{
  return (readStatus() & AS5600_MAGNET_DETECT) > 1;
}


bool AP_RPM_AS5600::magnetTooStrong()
{
  return (readStatus() & AS5600_MAGNET_HIGH) > 1;
}


bool AP_RPM_AS5600::magnetTooWeak()
{
  return (readStatus() & AS5600_MAGNET_LOW) > 1;
}


/////////////////////////////////////////////////////////
//
//  BURN COMMANDS
//
//  DO NOT UNCOMMENT - USE AT OWN RISK - READ DATASHEET
//
//  void AS5600::burnAngle()
//  {
//    writeReg(AS5600_BURN, x0x80);
//    delay(2);
//  }
//
//
//  See https://github.com/RobTillaart/AS5600/issues/38
//  void AS5600::burnSetting()
//  {
//    writeReg(AS5600_BURN, 0x40);
//    delay(5);
//    //  read back the OTP values (non-volatile RAM)
//    writeReg(AS5600_BURN, 0x01);
//    writeReg(AS5600_BURN, 0x11);
//    writeReg(AS5600_BURN, 0x10);
//    delay(5);
//  }


float AP_RPM_AS5600::getAngularSpeed(uint8_t mode, bool update)
{
  if (update)
  {
    _lastReadAngle = readAngle();
    if (_error != AS5600_OK)
    {
      return NAN;
    }
  }
  //  default behaviour
  uint32_t now     = AP_HAL::micros();
  int      angle   = _lastReadAngle;
  uint32_t deltaT  = now - _lastMeasurement;
  int      deltaA  = angle - _lastAngle;

  //  assumption is that there is no more than 180° rotation
  //  between two consecutive measurements.
  //  => at least two measurements per rotation (preferred 4).
  if (deltaA >  AS5600_RESO_DIV2)      deltaA -= AS5600_RESO;
  else if (deltaA < -AS5600_RESO_DIV2) deltaA += AS5600_RESO;
  float speed = (deltaA * 1e6) / deltaT;

  //  remember last time & angle
  _lastMeasurement = now;
  _lastAngle       = angle;

  //  return radians, RPM or degrees.
  if (mode == AS5600_MODE_RADIANS)
  {
    return speed * AS5600_RAW_TO_RADIANS;
  }
  if (mode == AS5600_MODE_RPM)
  {
    return speed * AS5600_RAW_TO_RPM;
  }
  //  default return degrees
  return speed * AS5600_RAW_TO_DEGREES;
}


/////////////////////////////////////////////////////////
//
//  POSITION cumulative
//
int32_t AP_RPM_AS5600::getCumulativePosition(bool update)
{
  if (update)
  {
    _lastReadAngle = readAngle();
    if (_error != AS5600_OK)
    {
      return _position;  //  last known position.
    }
  }
  int16_t value = _lastReadAngle;

  //  whole rotation CW?
  //  less than half a circle
  if ((_lastPosition > AS5600_RESO_DIV2) && ( value < (_lastPosition - AS5600_RESO_DIV2)))
  {
    _position = _position + AS5600_RESO - _lastPosition + value;
  }
  //  whole rotation CCW?
  //  less than half a circle
  else if ((value > AS5600_RESO_DIV2) && ( _lastPosition < (value - AS5600_RESO_DIV2)))
  {
    _position = _position - AS5600_RESO - _lastPosition + value;
  }
  else
  {
    _position = _position - _lastPosition + value;
  }
  _lastPosition = value;

  return _position;
}


int32_t AP_RPM_AS5600::getRevolutions()
{
  int32_t p = _position >> AS5600_RESO_BITS;  //  divide by 4096
  if (p < 0) p++;  //  correct negative values, See #65
  return p;
}


int32_t AP_RPM_AS5600::resetPosition(int32_t position)
{
  int32_t old = _position;
  _position = position;
  return old;
}


int32_t AP_RPM_AS5600::resetCumulativePosition(int32_t position)
{
  _lastPosition = readAngle();
  int32_t old = _position;
  _position = position;
  return old;
}


int AP_RPM_AS5600::lastError()
{
  int value = _error;
  _error = AS5600_OK;
  return value;
}


/////////////////////////////////////////////////////////
//
//  PROTECTED AS5600
//
uint8_t AP_RPM_AS5600::readReg(uint8_t reg)
{
  uint8_t buf;
  _dev->read_registers(reg, &buf, 1);
  return buf;
} 

uint16_t AP_RPM_AS5600::readReg2(uint8_t reg)
{
  uint8_t buf[2];
  _dev->read_registers(reg, buf, 2);
  return (buf[0] << 8) | buf[1];
}


uint8_t AP_RPM_AS5600::writeReg(uint8_t reg, uint8_t value)
{
  _error = AS5600_OK;
  if(! _dev->write_register(reg, value)){
	_error = AS5600_ERROR_I2C_WRITE_0;
  }
  return _error;
}


uint8_t AP_RPM_AS5600::writeReg2(uint8_t reg, uint16_t value)
{
  _error = AS5600_OK;
  if(_dev->write_register(reg, value >> 8)){
	if(! _dev->write_register(reg+1, value & 0xFF)){
	  _error = AS5600_ERROR_I2C_WRITE_0;
    }
  }else{
	_error = AS5600_ERROR_I2C_WRITE_0;
  }

  return _error;
}

#endif // AP_RPM_AS5600_ENABLED
