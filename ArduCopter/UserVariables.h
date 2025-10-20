// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#if HAL_LOGGING_ENABLED

	const uint8_t LOG_ACTR = 0;

	struct PACKED log_actr {
	    LOG_PACKET_HEADER;
	    uint64_t time_us;
	    float    thrust;
	    float    pos;
	};
	
	// @LoggerMessage: ACTR
	// @Description: Information from acuated rotor
	// @Field: TimeUS: Time since system startup
	// @Field: thrust: oscillation thrust calculated
	// @Field: pos: position of the motor in rads

	const LogStructure LOG_ACTR_STRCT = {LOG_ACTR, sizeof(log_actr), "ACTR", "Qff", "TimeUS,Thrust,Pos", "s-r", "F00" , true };
#endif 

	SRV_Channel *actrChannel;
	
	double radStep;
	double *cosTable; 
	double *sinTable;
	double *tanTable;
	
	uint64_t calculationTimeActr = 0;
	uint32_t calculationStartTimerActr = 0;
	uint16_t calculationIdxActr = 0;
	const uint16_t NBRE_CALC_TIME_ACTR = 2500;
	

#endif  // USERHOOK_VARIABLES


