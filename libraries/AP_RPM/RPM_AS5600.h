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
#pragma once

#include "AP_RPM_config.h"

#if AP_RPM_AS5600_ENABLED

#include "RPM_Backend.h"

#define AS5600_LIB_VERSION              (F("0.6.6"))


// --- Configuration du Filtre Kalman ---
#define STATE_SIZE 2     // x: [theta, omega]
#define MEASUREMENT_SIZE 1 // z: [delta_theta_mesure]
// Constante pour le bruit de processus (à régler dans le .cpp)
#define QC_SPECTRAL_DENSITY 0.1 
// Constante pour le bruit de mesure (à régler dans le .cpp)
#define R_VAL 0.000001
// Variance supposée de votre horloge/système (ex: 1ms de gigue = 0.001^2)
#define SIGMA_DT2 0.000001

/**
 * @class KalmanFilter2D
 * @brief Implémente un Filtre de Kalman 2D (Position angulaire et Vitesse angulaire)
 * utilisant une mesure de différence de position (Delta Theta) et gérant un DT variable.
 */
class KalmanFilter2D {
private:
    // --- État du Filtre ---
    double x[STATE_SIZE];             // [theta, omega] (Estimation de l'état)
    double P[STATE_SIZE][STATE_SIZE]; // Matrice de Covariance d'Erreur
    double R[MEASUREMENT_SIZE][MEASUREMENT_SIZE]; // Matrice de Covariance du Bruit de Mesure

    // --- Fonctions Privées (Utilitaires de Matrices et Calcul Q) ---

    // Calcule A = B * C (redéfini pour la clarté)
    void matrix_mult(int m, int n, int p, double A[STATE_SIZE][STATE_SIZE], const double B[][STATE_SIZE], const double C[][STATE_SIZE]);
    
    // Calcule K * H (2x1 * 1x2 -> 2x2)
    void matrix_mult_kh(double A[STATE_SIZE][STATE_SIZE], const double K[STATE_SIZE][MEASUREMENT_SIZE], const double H[MEASUREMENT_SIZE][STATE_SIZE]);

    // Calcule A = B + C
    void matrix_add(int n, double A[STATE_SIZE][STATE_SIZE], const double B[STATE_SIZE][STATE_SIZE], const double C[STATE_SIZE][STATE_SIZE]);

    // Calcule A = B - C
    void matrix_subtract(int n, double A[STATE_SIZE][STATE_SIZE], const double B[STATE_SIZE][STATE_SIZE], const double C[STATE_SIZE][STATE_SIZE]);

    // Calcule la transpose de B
    void matrix_transpose(int m, int n, double A[STATE_SIZE][STATE_SIZE], const double B[STATE_SIZE][STATE_SIZE]);
    
    // Calcule l'inverse d'une matrice 1x1
    double inverse_1x1(double S) const;

    // Calcule la matrice Q dynamique en fonction de dt
    void calculate_Q_dynamic(double Q_out[STATE_SIZE][STATE_SIZE], double dt) const;
	void calculate_Q_with_time_jitter(double Q_out[STATE_SIZE][STATE_SIZE], double dt, double omega) const;
public:
    /**
     * @brief Constructeur du Filtre de Kalman.
     * @param initial_theta Position angulaire initiale estimée (rad).
     * @param initial_omega Vitesse angulaire initiale estimée (rad/s).
     */
    KalmanFilter2D(double initial_theta, double initial_omega);

    /**
     * @brief Exécute une étape de prédiction et de correction du Filtre de Kalman.
     * @param delta_theta_mesure La nouvelle mesure de déplacement angulaire (Delta Theta).
     * @param dt_k Le temps écoulé depuis la dernière mesure (Delta t).
     */
    void update(double delta_theta_mesure, double dt_k);

    /**
     * @brief Retourne l'estimation lissée de la position angulaire.
     */
    double getTheta() const;

    /**
     * @brief Retourne l'estimation lissée de la vitesse angulaire.
     */
    double getOmega() const;
};

class AP_RPM_AS5600 : public AP_RPM_Backend
{
	
	
public:
	static const uint8_t AS5600_RESO_BITS = 12;
	static const uint16_t AS5600_RESO = pow(2,AS5600_RESO_BITS);
	static const uint16_t AS5600_RESO_DIV2 = AS5600_RESO/2.0f;
	static const uint16_t AS5600_VAL_MIDDLE = AS5600_RESO_DIV2 - 1;
	static const uint8_t AS5600_DZ = 5; //Dead zone
	static constexpr double AS5600_US_TO_S = 1/pow(10,6); //Conversion de microsec a sec
	
	//  default addresses
	static const uint8_t AS5600_DEFAULT_ADDRESS    = 0x36;
	static const uint8_t AS5600L_DEFAULT_ADDRESS   = 0x40;
	static const uint8_t AS5600_SW_DIRECTION_PIN   = 255;
	
	//  setDirection
	static const uint8_t AS5600_CLOCK_WISE         = 0;  //  LOW
	static const uint8_t AS5600_COUNTERCLOCK_WISE  = 1;  //  HIGH
	
	//  0.087890625;
	static constexpr float   AS5600_RAW_TO_DEGREES     = 360.0 / AS5600_RESO;
	static constexpr float   AS5600_DEGREES_TO_RAW     = AS5600_RESO / 360.0;
	//  0.00153398078788564122971808758949;
	static constexpr float   AS5600_RAW_TO_RADIANS     = M_PI * 2.0 / AS5600_RESO;
	//  4.06901041666666e-6
	static constexpr float   AS5600_RAW_TO_RPM         = 60.0 / AS5600_RESO;
	//  getAngularSpeed
	static const uint8_t AS5600_MODE_DEGREES       = 0;
	static const uint8_t AS5600_MODE_RADIANS       = 1;
	static const uint8_t AS5600_MODE_RPM           = 2;
	
	//  ERROR CODES
	static const int     AS5600_OK                 = 0;
	static const int     AS5600_ERROR_I2C_READ_0   = -100;
	static const int     AS5600_ERROR_I2C_READ_1   = -101;
	static const int     AS5600_ERROR_I2C_READ_2   = -102;
	static const int     AS5600_ERROR_I2C_READ_3   = -103;
	static const int     AS5600_ERROR_I2C_WRITE_0  = -200;
	static const int     AS5600_ERROR_I2C_WRITE_1  = -201;
	
	//  CONFIGURE CONSTANTS
	//  check datasheet for details
	
	//  setOutputMode
	static const uint8_t AS5600_OUTMODE_ANALOG_100 = 0;
	static const uint8_t AS5600_OUTMODE_ANALOG_90  = 1;
	static const uint8_t AS5600_OUTMODE_PWM        = 2;
	
	//  setPowerMode
	static const uint8_t AS5600_POWERMODE_NOMINAL  = 0;
	static const uint8_t AS5600_POWERMODE_LOW1     = 1;
	static const uint8_t AS5600_POWERMODE_LOW2     = 2;
	static const uint8_t AS5600_POWERMODE_LOW3     = 3;
	
	//  setPWMFrequency
	static const uint8_t AS5600_PWM_115            = 0;
	static const uint8_t AS5600_PWM_230            = 1;
	static const uint8_t AS5600_PWM_460            = 2;
	static const uint8_t AS5600_PWM_920            = 3;
	
	//  setHysteresis
	static const uint8_t AS5600_HYST_OFF           = 0;
	static const uint8_t AS5600_HYST_LSB1          = 1;
	static const uint8_t AS5600_HYST_LSB2          = 2;
	static const uint8_t AS5600_HYST_LSB3          = 3;
	
	//  setSlowFilter
	static const uint8_t AS5600_SLOW_FILT_16X      = 0;
	static const uint8_t AS5600_SLOW_FILT_8X       = 1;
	static const uint8_t AS5600_SLOW_FILT_4X       = 2;
	static const uint8_t AS5600_SLOW_FILT_2X       = 3;
	
	//  setFastFilter
	static const uint8_t AS5600_FAST_FILT_NONE     = 0;
	static const uint8_t AS5600_FAST_FILT_LSB6     = 1;
	static const uint8_t AS5600_FAST_FILT_LSB7     = 2;
	static const uint8_t AS5600_FAST_FILT_LSB9     = 3;
	static const uint8_t AS5600_FAST_FILT_LSB18    = 4;
	static const uint8_t AS5600_FAST_FILT_LSB21    = 5;
	static const uint8_t AS5600_FAST_FILT_LSB24    = 6;
	static const uint8_t AS5600_FAST_FILT_LSB10    = 7;
	
	//  setWatchDog
	static const uint8_t AS5600_WATCHDOG_OFF       = 0;
	static const uint8_t AS5600_WATCHDOG_ON        = 1;

	//  CONFIGURATION REGISTERS
	const uint8_t AS5600_ZMCO = 0x00;
	const uint8_t AS5600_ZPOS = 0x01;   //  + 0x02
	const uint8_t AS5600_MPOS = 0x03;   //  + 0x04
	const uint8_t AS5600_MANG = 0x05;   //  + 0x06
	const uint8_t AS5600_CONF = 0x07;   //  + 0x08

	//  CONFIGURATION BIT MASKS - byte level
	const uint8_t AS5600_CONF_POWER_MODE    = 0x03;
	const uint8_t AS5600_CONF_HYSTERESIS    = 0x0C;
	const uint8_t AS5600_CONF_OUTPUT_MODE   = 0x30;
	const uint8_t AS5600_CONF_PWM_FREQUENCY = 0xC0;
	const uint8_t AS5600_CONF_SLOW_FILTER   = 0x03;
	const uint8_t AS5600_CONF_FAST_FILTER   = 0x1C;
	const uint8_t AS5600_CONF_WATCH_DOG     = 0x20;

	//  UNKNOWN REGISTERS 0x09-0x0A

	//  OUTPUT REGISTERS
	const uint8_t AS5600_RAW_ANGLE = 0x0C;   //  + 0x0D
	const uint8_t AS5600_ANGLE     = 0x0E;   //  + 0x0F

	// I2C_ADDRESS REGISTERS (AS5600L)
	const uint8_t AS5600_I2CADDR   = 0x20;
	const uint8_t AS5600_I2CUPDT   = 0x21;

	//  STATUS REGISTERS
	const uint8_t AS5600_STATUS    = 0x0B;
	const uint8_t AS5600_AGC       = 0x1A;
	const uint8_t AS5600_MAGNITUDE = 0x1B;   //  + 0x1C
	const uint8_t AS5600_BURN      = 0xFF;

	//  STATUS BITS
	const uint8_t AS5600_MAGNET_HIGH   = 0x08;
	const uint8_t AS5600_MAGNET_LOW    = 0x10;
	const uint8_t AS5600_MAGNET_DETECT = 0x20;
	
	
	// constructor
	AP_RPM_AS5600(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State &_state);
	
	// update state
    void update(void) override;

  //  made virtual, see #66
  virtual bool isConnected();
  
  int16_t  getLastReadValue(){ return  _lastReadValue;}

  //  address = fixed   0x36 for AS5600,
  //          = default 0x40 for AS5600L
  uint8_t  getAddress();


  //  SET CONFIGURE REGISTERS
  //  read datasheet first

  //  0         = AS5600_CLOCK_WISE
  //  1         = AS5600_COUNTERCLOCK_WISE
  //  all other = AS5600_COUNTERCLOCK_WISE
  void     setDirection(uint8_t direction = AS5600_CLOCK_WISE);
  uint8_t  getDirection();

  //  returns how many times ZPOS and MPOS have been permanently written.
  uint8_t  getZMCO();

  //  0 .. 4095
  //  returns false if parameter out of range
  bool     setZPosition(uint16_t value);
  uint16_t getZPosition();

  //  0 .. 4095
  //  returns false if parameter out of range
  bool     setMPosition(uint16_t value);
  uint16_t getMPosition();

  //  0 .. 4095
  //  returns false if parameter out of range
  bool     setMaxValue(uint16_t value);
  uint16_t getMaxValue();

  //  access the whole configuration register
  //  check datasheet for bit fields
  //  returns false if parameter out of range
  bool     setConfigure(uint16_t value);
  uint16_t getConfigure();

  //  access details of the configuration register
  //  0 = Normal
  //  1,2,3 are low power mode - check datasheet
  //  returns false if parameter out of range
  bool     setPowerMode(uint8_t powerMode);
  uint8_t  getPowerMode();

  //  0 = off    1 = lsb1    2 = lsb2    3 = lsb3
  //  returns false if parameter out of range
  //  suppresses noise when the magnet is not moving.
  bool     setHysteresis(uint8_t hysteresis);
  uint8_t  getHysteresis();

  //  0 = analog 0-100%
  //  1 = analog 10-90%
  //  2 = PWM
  //  returns false if parameter out of range
  bool     setOutputMode(uint8_t outputMode);
  uint8_t  getOutputMode();

  //  0 = 115    1 = 230    2 = 460    3 = 920 (Hz)
  //  returns false if parameter out of range
  bool     setPWMFrequency(uint8_t pwmFreq);
  uint8_t  getPWMFrequency();

  //  0 = 16x    1 = 8x     2 = 4x     3 = 2x
  //  returns false if parameter out of range
  bool     setSlowFilter(uint8_t mask);
  uint8_t  getSlowFilter();

  //  0 = none   1 = LSB6   2 = LSB7   3 = LSB9
  //  4 = LSB18  5 = LSB21  6 = LSB24  7 = LSB10
  //  returns false if parameter out of range
  bool     setFastFilter(uint8_t mask);
  uint8_t  getFastFilter();

  //  0 = OFF
  //  1 = ON   (auto low power mode)
  //  returns false if parameter out of range
  bool     setWatchDog(uint8_t mask);
  uint8_t  getWatchDog();


  //  READ OUTPUT REGISTERS
  uint16_t rawValue();
  uint16_t readValue();

  //  software based offset.
  //  degrees = -359.99 .. 359.99 (preferred)
  //  returns false if abs(parameter) > 36000
  //          => expect loss of precision
  float    getOffset();


  //  READ STATUS REGISTERS
  uint8_t  readStatus();
  uint8_t  readAGC();
  uint16_t readMagnitude();

  //  access detail status register
  bool     detectMagnet();
  bool     magnetTooStrong();
  bool     magnetTooWeak();
  
  bool     detectMagnet(uint8_t status);
  bool     magnetTooStrong(uint8_t status);
  bool     magnetTooWeak(uint8_t status);


  //  BURN COMMANDS
  //  DO NOT UNCOMMENT - USE AT OWN RISK - READ DATASHEET
  //  use getZMCO() to get the counter how often ZPOS/MPOS is "burned".
  //  void burnAngle();
  //  void burnSetting();


  //  EXPERIMENTAL 0.1.2 - to be tested.
  //  approximation of the angular speed in rotations per second.
  //  mode == 1: radians /second
  //  mode == 0: degrees /second  (default)
  float    getAngularSpeed(uint8_t mode = AS5600_MODE_DEGREES, bool update = true);				   
  float    getAngularSpeed(double value, uint8_t mode = AS5600_MODE_DEGREES);

  //  EXPERIMENTAL CUMULATIVE POSITION
  //  reads sensor and updates cumulative position
  int32_t  getCumulativePosition(bool update = true);
  //  converts last position to whole revolutions.
  int32_t  getRevolutions();
  //  resets position only (not the i)
  //  returns last position but not internal lastPosition.
  int32_t  resetPosition(int32_t position = 0);
  //  resets position and internal lastPosition
  //  returns last position.
  int32_t  resetCumulativePosition(int32_t position = 0);

  //  EXPERIMENTAL 0.5.2
  int      lastError();


protected:
  //  made virtual, see #66
  virtual uint8_t  readReg(uint8_t reg);
  virtual uint16_t readReg2(uint8_t reg);
  virtual uint8_t  writeReg(uint8_t reg, uint8_t value);
  virtual uint8_t  writeReg2(uint8_t reg, uint16_t value);
  
  void _timer(void);

  bool _connected = false;
  
  uint8_t  _address         = AS5600_DEFAULT_ADDRESS;
  uint8_t  _directionPin    = AS5600_SW_DIRECTION_PIN;
  uint8_t  _direction       = AS5600_CLOCK_WISE;
  int      _error           = AS5600_OK;

  //  for getAngularSpeed()
  uint64_t _lastMeasurementTime = 0;
  uint64_t _lastSpeedCalculateTime = 0;
  bool _msgStoppedSent = false;
  uint16_t _lastReadValue   = AS5600_RESO;
  float _lastSpeed = 0;

  //  for readValue() and rawValue()
  uint16_t _offset          = 0;

  //For debug and timing mesure
  uint64_t _sumMeasurement = 0;
  uint64_t calculationTimeActr = 0;
  uint32_t calculationStartTimerActr = 0;
  uint16_t calculationIdxActr = 0;
  const uint16_t NBRE_CALC_TIME_ACTR = 2500;
  
  //For signal quality
  float _statusMagnet = 0;
  uint16_t _numberRead = 0;
  uint16_t _numberReadError = 0;
  
  //  EXPERIMENTAL
  //  cumulative position counter
  //  works only if the sensor is read often enough.
  int32_t  _position        = 0;
  int16_t  _lastPosition    = 0;

  
  //Filtre Kalman
  KalmanFilter2D _kf = KalmanFilter2D(0.0, 0.0);;
  
private:
  AP_HAL::I2CDevice *_dev;
};

#endif
