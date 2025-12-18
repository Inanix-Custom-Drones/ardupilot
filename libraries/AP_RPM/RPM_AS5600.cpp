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

	// --- Fonctions utilitaires de Matrices et Filtre Kalman ---

	// Constructeur
    KalmanFilter2D::KalmanFilter2D(double initial_theta, double initial_omega) {
	    // Initialisation de l'état
	    x[0] = initial_theta;
	    x[1] = initial_omega;
	
	    // Initialisation de la Covariance (Grande incertitude initiale)
	    for (int i = 0; i < STATE_SIZE; i++) {
	        for (int j = 0; j < STATE_SIZE; j++) {
	            P[i][j] = (i == j) ? 10.0 : 0.0;
	        }
	    }
	    
	    // Initialisation du bruit de mesure R
	    R[0][0] = R_VAL;
	    
	    //std::cout << "Filtre de Kalman 2D initialisé : theta=" << x[0] << ", omega=" << x[1] << std::endl;
	}
	
    // Calcule A = B * C
    void KalmanFilter2D::matrix_mult(int m, int n, int p, double A[STATE_SIZE][STATE_SIZE], const double B[][STATE_SIZE], const double C[][STATE_SIZE]) {
        // Redéfini pour les dimensions de l'état (2x2 * 2x2)
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < p; j++) {
                A[i][j] = 0;
                for (int k = 0; k < n; k++) {
                    A[i][j] += B[i][k] * C[k][j];
                }
            }
        }
    }
    
    // Surcharge pour K * H (2x1 * 1x2 -> 2x2)
    void KalmanFilter2D::matrix_mult_kh(double A[STATE_SIZE][STATE_SIZE], const double K[STATE_SIZE][MEASUREMENT_SIZE], const double H[MEASUREMENT_SIZE][STATE_SIZE]) {
        for (int i = 0; i < STATE_SIZE; i++) { // m=2
            for (int j = 0; j < STATE_SIZE; j++) { // p=2
                A[i][j] = K[i][0] * H[0][j]; // n=1
            }
        }
    }

    // Calcule A = B + C (n x n)
    void KalmanFilter2D::matrix_add(int n, double A[STATE_SIZE][STATE_SIZE], const double B[STATE_SIZE][STATE_SIZE], const double C[STATE_SIZE][STATE_SIZE]) {
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                A[i][j] = B[i][j] + C[i][j];
    }

    // Calcule A = B - C (n x n)
    void KalmanFilter2D::matrix_subtract(int n, double A[STATE_SIZE][STATE_SIZE], const double B[STATE_SIZE][STATE_SIZE], const double C[STATE_SIZE][STATE_SIZE]) {
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                A[i][j] = B[i][j] - C[i][j];
    }

    // Calcule la transpose de B
    void KalmanFilter2D::matrix_transpose(int m, int n, double A[STATE_SIZE][STATE_SIZE], const double B[STATE_SIZE][STATE_SIZE]) {
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                A[j][i] = B[i][j];
    }
    
    // Calcule l'inverse d'une matrice 1x1 (scalaire)
    double KalmanFilter2D::inverse_1x1(double S) const {
        return (S != 0.0) ? 1.0 / S : 0.0;
    }

	void KalmanFilter2D::calculate_Q_with_time_jitter(double Q_out[STATE_SIZE][STATE_SIZE], double dt, double omega) const {
	    double dt2 = dt * dt;
	    double dt3 = dt2 * dt;
	    double qc = QC_SPECTRAL_DENSITY; // Bruit de processus classique (accélération)
	    
	    // Bruit induit par le jitter temporel sur la position : (omega * sigma_dt)^2
	    double q_jitter = (omega * omega) * SIGMA_DT2;

	    // Q classique + composante de jitter sur la diagonale de la position
	    Q_out[0][0] = qc * (dt3 / 3.0) + q_jitter; 
	    Q_out[0][1] = qc * (dt2 / 2.0);
	    Q_out[1][0] = qc * (dt2 / 2.0);
	    Q_out[1][1] = qc * dt;
	}
	
    // Calcule la matrice Q dynamique
    void KalmanFilter2D::calculate_Q_dynamic(double Q_out[STATE_SIZE][STATE_SIZE], double dt) const {
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double qc = QC_SPECTRAL_DENSITY;

        // Q = qc * [ dt^3/3  dt^2/2 ]
        //        [ dt^2/2  dt       ]
        Q_out[0][0] = qc * (dt3 / 3.0);
        Q_out[0][1] = qc * (dt2 / 2.0);
        Q_out[1][0] = qc * (dt2 / 2.0);
        Q_out[1][1] = qc * dt;
    }

    // --- Méthodes publiques pour obtenir l'état ---
    double KalmanFilter2D::getTheta() const { return x[0]; }
    double KalmanFilter2D::getOmega() const { return x[1]; }

    // --- Fonction principale de mise à jour ---
    void KalmanFilter2D::update(double delta_theta_mesure, double dt_k) {
        if (dt_k <= 0.0) return;

        // ------------------------------------
        // PHASE 1 : PRÉDICTION
        // ------------------------------------
        
        // 1. Mise à jour des Matrices F et Q
        double F[STATE_SIZE][STATE_SIZE] = {
            {1, dt_k},
            {0, 1}
        };
        double Q_dynamic[STATE_SIZE][STATE_SIZE];
        //calculate_Q_dynamic(Q_dynamic, dt_k);
		calculate_Q_with_time_jitter(Q_dynamic, dt_k, x[1]);
		
        // 2. Estimation de l'état projeté (x_k|k-1)
        double x_pred[STATE_SIZE];
        x_pred[0] = x[0] + x[1] * dt_k; // theta_pred = theta + omega * dt
        x_pred[1] = x[1];               // omega_pred = omega (modèle CV)
        
        // 3. Covariance projetée (P_k|k-1)
        // P_pred = F * P * F_T + Q_dynamic
        double F_T[STATE_SIZE][STATE_SIZE];
        double FP[STATE_SIZE][STATE_SIZE];
        double FPF_T[STATE_SIZE][STATE_SIZE];
        double P_pred[STATE_SIZE][STATE_SIZE];

        matrix_transpose(STATE_SIZE, STATE_SIZE, F_T, F);
        matrix_mult(STATE_SIZE, STATE_SIZE, STATE_SIZE, FP, F, P);
        matrix_mult(STATE_SIZE, STATE_SIZE, STATE_SIZE, FPF_T, FP, F_T);
        matrix_add(STATE_SIZE, P_pred, FPF_T, Q_dynamic);

        // ------------------------------------
        // PHASE 2 : CORRECTION
        // ------------------------------------

        // 4. Matrice H pour la mesure Delta Theta : H = [ 0, dt_k ]
        double H[MEASUREMENT_SIZE][STATE_SIZE] = { {0.0, dt_k} };
        
        // 5. Calcul de S (Covariance de l'Innovation)
        // S = H * P_pred * H_T + R
        double H_T[STATE_SIZE][STATE_SIZE];
        //double HP_pred[MEASUREMENT_SIZE][STATE_SIZE];
        matrix_transpose(MEASUREMENT_SIZE, STATE_SIZE, H_T, H); // H_T est 2x1
        
        // HP_pred est 1x2. P_pred * H_T est 2x1.
        // H * P_pred est 1x2.
        double HP_pred_H_T_scalar;
        HP_pred_H_T_scalar = H[0][0] * (P_pred[0][0] * H_T[0][0] + P_pred[0][1] * H_T[1][0]) +
                             H[0][1] * (P_pred[1][0] * H_T[0][0] + P_pred[1][1] * H_T[1][0]);
        
        double S_scalar = HP_pred_H_T_scalar + R[0][0];

        // 6. Calcul du Gain de Kalman (K)
        // K = P_pred * H_T * S^-1
        double S_inv_scalar = inverse_1x1(S_scalar);
        double K[STATE_SIZE][MEASUREMENT_SIZE]; // K est 2 x 1
        
        // P_pred * H_T est 2x1
        for (int i = 0; i < STATE_SIZE; i++) {
            double P_pred_H_T_i = P_pred[i][0] * H_T[0][0] + P_pred[i][1] * H_T[1][0];
            K[i][0] = P_pred_H_T_i * S_inv_scalar;
        }

        // 7. Mise à jour de l'état (x_k|k)
        // Innovation: y = delta_theta_mesure - (omega_pred * dt_k)
        double delta_theta_predit = x_pred[1] * dt_k;
        double innovation = delta_theta_mesure - delta_theta_predit;
        
        // x_new = x_pred + K * y
        x[0] = x_pred[0] + K[0][0] * innovation;
        x[1] = x_pred[1] + K[1][0] * innovation;
        
        // 8. Mise à jour de la Covariance (P_k|k)
        // P_new = (I - K * H) * P_pred
        double I[STATE_SIZE][STATE_SIZE] = {{1, 0}, {0, 1}};
        double KH[STATE_SIZE][STATE_SIZE];
        double I_minus_KH[STATE_SIZE][STATE_SIZE];
        
        matrix_mult_kh(KH, K, H); // K * H
        matrix_subtract(STATE_SIZE, I_minus_KH, I, KH); // I - KH
        
        // Mise à jour de la matrice P interne
        matrix_mult(STATE_SIZE, STATE_SIZE, STATE_SIZE, P, I_minus_KH, P_pred); 
    }


// TODO : Add calibration
// Listen for message MAV_CMD_PREFLIGHT_CALIBRATION
// Calibrate with avg value _sumMeasurement
// Write offset parameter AS56_OFST

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
			
			if(ap_rpm._params[state.instance].as5600_cwccw == 0){
				setDirection(AS5600_CLOCK_WISE);
			}else{
				setDirection(AS5600_COUNTERCLOCK_WISE);
			}
			
			setPowerMode(AS5600_POWERMODE_NOMINAL);
			setZPosition(0);
			setMPosition(AS5600_RESO-1);
			setMaxValue(AS5600_RESO-1);
			setSlowFilter(AS5600_SLOW_FILT_2X);
			setFastFilter(AS5600_FAST_FILT_LSB10);
			
			GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AS5600 status %u", unsigned(status));
			//printf("AS5600 status %u\n", unsigned(status));
			
			if(detectMagnet(status)){
				if(magnetTooStrong(status)){
					_statusMagnet = 0.8f;
				}else if(magnetTooWeak(status)){
					_statusMagnet = 0.6f;
				}else{
					_statusMagnet = 1.0f;
				}
			}
			readValue();
			_kf = KalmanFilter2D(_lastReadValue * AS5600_RAW_TO_RADIANS, 0.0);

		}else{
			GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AS5600 Issue reading status %u", unsigned(status));
			//printf("AS5600 Issue reading status %u\n", unsigned(status));
			delete _dev;
	        return;
		}
		//1000 Hz voir plus si possible
        _dev->register_periodic_callback(ap_rpm._params[state.instance].as5600_timer,
                                        FUNCTOR_BIND_MEMBER(&AP_RPM_AS5600::_timer, void));
	}
}

void AP_RPM_AS5600::update(void)
{
	if(isConnected()){
	    state.rate_rpm = getAngularSpeed(_lastSpeed, AS5600_MODE_RPM);
	    state.last_reading_ms = _lastMeasurementTime / 1000; //pour des millis

		float signalQuality = _statusMagnet;
		if(_numberRead>0){
			signalQuality -= _numberReadError/_numberRead;
		}
		_numberRead = 0;
		_numberReadError = 0;
		
  		if(state.rate_rpm > ap_rpm._params[state.instance].maximum ||
  				state.rate_rpm < ap_rpm._params[state.instance].minimum){
			signalQuality -= 0.1;
  		}
		
		state.signal_quality = signalQuality;
		
		if(ap_rpm._params[state.instance].as5600_debug == 3){
			GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "AS5600 val %i motpos %fdeg rpm %f", _lastReadValue, (_lastReadValue * AS5600_RAW_TO_DEGREES), state.rate_rpm);
		}
	}
}

void AP_RPM_AS5600::_timer(void)
{
	if(isConnected() && ap_rpm._params[state.instance].as5600_servoidx > 0){
		uint32_t timerStart = 0;
		if(ap_rpm._params[state.instance].as5600_debug > 0){
			timerStart = AP_HAL::micros64();
			if(calculationStartTimerActr == 0){
				calculationStartTimerActr = AP_HAL::millis();
			}
		}
		
		readValue();
		
		//Telemetry du servo pour affichage de la pos du moteur
		AP_Servo_Telem *servoTelem = AP_Servo_Telem::get_singleton();
		
		AP_Servo_Telem::TelemetryData telem_data {
	        .measured_position = _lastReadValue * AS5600_RAW_TO_DEGREES,
			.speed = getAngularSpeed(_lastSpeed, AS5600_MODE_DEGREES),
	        .present_types = AP_Servo_Telem::TelemetryData::Types::MEASURED_POSITION |
	                         AP_Servo_Telem::TelemetryData::Types::SPEED
	    };
		
		servoTelem->update_telem_data(ap_rpm._params[state.instance].as5600_servoidx-1, telem_data);
		
		if(ap_rpm._params[state.instance].as5600_debug > 0){
			calculationTimeActr += AP_HAL::micros64() - timerStart;
			calculationIdxActr ++;
			_sumMeasurement += _lastReadValue;
			if(calculationIdxActr >= NBRE_CALC_TIME_ACTR){
				uint16_t timer = AP_HAL::millis() - calculationStartTimerActr;
				uint16_t calculationAvgTime = (uint16_t) (calculationTimeActr/NBRE_CALC_TIME_ACTR);
				uint16_t calculationAvgVal = (uint16_t) (_sumMeasurement/NBRE_CALC_TIME_ACTR);
				
				uint8_t status = readStatus();
				uint8_t agc = readAGC();
				
				if(ap_rpm._params[state.instance].as5600_debug == 2){
					gcs().send_text(MAV_SEVERITY_DEBUG, "AS56 rdtmavg %i us %i lps %ims sts %i agc %i avgval %i lstval %i", calculationAvgTime, NBRE_CALC_TIME_ACTR, timer, status, agc, calculationAvgVal, _lastReadValue);
				}else if(ap_rpm._params[state.instance].as5600_debug == 1){
					gcs().send_text(MAV_SEVERITY_DEBUG, "AS56 rdtmavg %i us %i lps %ims sts %i agc %i", calculationAvgTime, NBRE_CALC_TIME_ACTR, timer, status, agc);
				}
				
				calculationIdxActr = 0;
				calculationTimeActr = 0;
				calculationStartTimerActr = 0;
				_sumMeasurement = 0;
			}
			
		}
	}
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


bool AP_RPM_AS5600::setMaxValue(uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(AS5600_MANG, value);
  return true;
}


uint16_t AP_RPM_AS5600::getMaxValue()
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
uint16_t AP_RPM_AS5600::rawValue()
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


uint16_t AP_RPM_AS5600::readValue()
{
  _numberRead++;
  uint64_t now = AP_HAL::micros64();
  uint16_t value = readReg2(AS5600_ANGLE);
  if (_error != AS5600_OK)
  {
	_numberReadError ++;
    return _lastReadValue;
  }
  
  uint64_t deltaT = now - _lastMeasurementTime;
  uint16_t deltaA = 0;
  _lastMeasurementTime = now;
  
  if (getOffset() > 0) {
	if(value - getOffset() < 0){
		value += AS5600_RESO;
	}
	value -= getOffset();
  }

  if ((_directionPin == AS5600_SW_DIRECTION_PIN) &&
      (_direction == AS5600_COUNTERCLOCK_WISE))
  {
    //  mask needed for value == 0.
    //value = (AS5600_RESO - value) & 0x0FFF;
	value = (AS5600_RESO - value) ;
  }
  
  
  //Calcul de la vitesse
  if(_lastReadValue < AS5600_RESO){ //On filtre la première lecture de valeur car on ne pourra pas calculer de vitesse
	int tmpNewVal = value;
	int tmpLastVal = _lastReadValue;
	
	if(tmpLastVal > AS5600_RESO_DIV2 && tmpNewVal < AS5600_RESO_DIV2) { //On vient de passer par le 0
		tmpLastVal -= AS5600_RESO;
	}
	
	deltaA = abs(tmpNewVal - tmpLastVal) ;
			
	if(ap_rpm._params[state.instance].as5600_debug == 5){
		GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "AS56 lstval %i newval %i dT %lli dA %i", _lastReadValue, value, deltaT, deltaA);
	}	
	
    if(deltaA <= AP_RPM_AS5600::AS5600_DZ){
		
		// On n'a pas de mouvement... 
	  	// soit parce qu'on a executé trop vite l'appel au calcul de la vitesse (entre 2 lecture d'angle)
	  	// soit parce que le moteur est arreté
	  	uint64_t motorStoppedTime = now - _lastSpeedCalculateTime;
	  	if(motorStoppedTime >  ap_rpm._params[state.instance].as5600_timer * 5){ //le délai est dépassé, le moteur est arrêté
	  		if(ap_rpm._params[state.instance].as5600_debug == 4 && motorStoppedTime > 2500000 && ! _msgStoppedSent){ //Ttes les 2.5sec
	  			GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "AS56 spd motor stopped since %lli", motorStoppedTime);
				_msgStoppedSent = true;
	  		}
	  		_lastSpeed = 0;
	  	}else{
	  		if(ap_rpm._params[state.instance].as5600_debug == 4  && motorStoppedTime > 2500000 && ! _msgStoppedSent){
	  			GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "AS56 spd not calc last %f", _lastSpeed);
	  		}
	  	}
    }else{
      	if(deltaT > 0){
			double dtAngleRad = deltaA*AS5600_RAW_TO_RADIANS;
			double dtTempsSec = deltaT*AS5600_US_TO_S;
			_kf.update(dtAngleRad, dtTempsSec);
			_lastSpeedCalculateTime = now;
			//value = _kf.getTheta();
			
			double speedRadSec = _kf.getOmega();
			
	  		_lastSpeed = _kf.getOmega()/AS5600_RAW_TO_RADIANS;
	  		
			if(ap_rpm._params[state.instance].as5600_debug == 4){
				_msgStoppedSent = false; 
	  			GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "AS56 dT %.6f dA %.3f", dtTempsSec, dtAngleRad);
				GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "AS56 kfPos %.3f kfSpd %.3f", _kf.getTheta(), speedRadSec);
	  		}
	  	}else{
	  		if(ap_rpm._params[state.instance].as5600_debug == 4){
	  			GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "AS56 spd not calc dT %lli", deltaT);
	  		}
	  	}
	  }
	  
	  /*if((! (_lastSpeed > 0.0)) && ap_rpm._params[state.instance].as5600_debug >= 0){
		GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "AS56 spd0 lstv %i val %i dT %lli dA %i", _lastReadValue, value, deltaT, deltaA);
	  }*/
	  
  }else{
	if(ap_rpm._params[state.instance].as5600_debug == 4){
		GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "AS56 spd 1e calc no read");
	}
  }

  _lastReadValue = value;
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


bool AP_RPM_AS5600::detectMagnet(uint8_t status)
{
  return (status & AS5600_MAGNET_DETECT) > 1;
}


bool AP_RPM_AS5600::magnetTooStrong(uint8_t status)
{
  return (status & AS5600_MAGNET_HIGH) > 1;
}


bool AP_RPM_AS5600::magnetTooWeak(uint8_t status)
{
  return (status & AS5600_MAGNET_LOW) > 1;
}

bool AP_RPM_AS5600::detectMagnet()
{
  return detectMagnet(readStatus());
}


bool AP_RPM_AS5600::magnetTooStrong()
{
  return magnetTooStrong(readStatus());
}


bool AP_RPM_AS5600::magnetTooWeak()
{
  return magnetTooWeak(readStatus());
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
  if (update || _lastMeasurementTime == 0){
    readValue();
  }

  return getAngularSpeed(_lastSpeed, mode);
}

float AP_RPM_AS5600::getAngularSpeed(double value, uint8_t mode)
{

  double rtnSpeed;
  //  return radians, RPM or degrees.
  switch(mode) {
    case AS5600_MODE_RADIANS:
      rtnSpeed = value * AS5600_RAW_TO_RADIANS;
      break;
    case AS5600_MODE_RPM:
      rtnSpeed = value * AS5600_RAW_TO_RPM;
      break;
    case AS5600_MODE_DEGREES:
    default:
      rtnSpeed = value * AS5600_RAW_TO_DEGREES;
  }

  return (float) rtnSpeed;
}


/////////////////////////////////////////////////////////
//
//  POSITION cumulative
//
int32_t AP_RPM_AS5600::getCumulativePosition(bool update)
{
  if (update)
  {
    readValue();
    if (_error != AS5600_OK)
    {
      return _position;  //  last known position.
    }
  }
  int16_t value = _lastReadValue;

  //  whole rotation CW?
  //  less than half a circle
  if ((_lastPosition > AS5600_VAL_MIDDLE) && ( value < (_lastPosition - AS5600_VAL_MIDDLE)))
  {
    _position = _position + AS5600_RESO - _lastPosition + value;
  }
  //  whole rotation CCW?
  //  less than half a circle
  else if ((value > AS5600_VAL_MIDDLE) && ( _lastPosition < (value - AS5600_VAL_MIDDLE)))
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
  _lastPosition = readValue();
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
