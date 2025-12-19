// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

	struct PACKED log_actr {
	    LOG_PACKET_HEADER;
	    uint64_t time_us;
	    float    throttle;
		uint16_t pwm;
		float    posRead;
	    float    pos;
		float    speed;
		float	 dpitch;
		float 	 droll;
	};

	SRV_Channel *actrChannel;
	
	double radStep;
	double *cosTable; 
	double *sinTable;
	double *tanTable;
	
	const uint8_t MIN_PRECISION = 2;
	
	uint64_t calculationTimeActr = 0;
	uint64_t _lastTimeCalc = 0;
	uint32_t calculationStartTimerActr = 0;
	uint16_t calculationIdxActr = 0;
	const uint16_t NBRE_CALC_TIME_ACTR = 2500;
	const uint16_t PWM_OFF = 1000;
	
	const double M_1DOT5_PI = M_PI + M_PI_2; //270 degres

	/**************  Time Based Phase Shifter     ********************/
	//Algorithme pour estimer le décalage entre l'ordre et le changement de vitesse
	static const uint32_t TEN_POW_SIX = pow(10,6);
	uint64_t _tbpsTimeDelayUSeconds = 0.0f; // La constante de temps apprise (s)
    float _tbpsLearningRate = 0.05f;    // Adaptation lente pour la stabilité
    // Analyseurs
    float _tbpsAvgThrottle = 0.5f;
    float _tbpsAvgSpeed = 0.0f;
    float _tbpsAngleAtCrossThrottle = 0;
    uint64_t _tbpsTimeAtCrossThrottle = 0;
    bool _tbpsThrottleWasBelow = true;
    bool _tbpsSpeedWasBelow = true;
	
	// Paramètres de boost et de filtrage du boost
    float filteredDerivative = 0.0f;
	float lastRawThrottle = 0.0f;
    const double refSpeedRadSec = 1000.0f / 60 * M_2PI ; //1000rpm en rad/sec
	
#endif  // USERHOOK_VARIABLES


