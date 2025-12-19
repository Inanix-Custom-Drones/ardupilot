#include "Copter.h"
#include <AP_RPM/RPM_AS5600.h>

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
	
	if(g2.user_parameters.getAcrtServo() > 0){
		//On retrouve le channel du moteur
		actrChannel = SRV_Channels::srv_channel(g2.user_parameters.getAcrtServo()-1);
	}
	
	if(g2.user_parameters.getAcrtPrecision() > MIN_PRECISION){
		//Stockage des cos/sin pour accélérer les traitements ensuite
		cosTable = (double *)malloc(g2.user_parameters.getAcrtPrecision() * sizeof(double));
		sinTable = (double *)malloc(g2.user_parameters.getAcrtPrecision() * sizeof(double));
		//TODO retrouver les tangeantes au lien de les calculer dans la boucle de calculs
		//tanTable = (double *)malloc(g2.user_parameters.getAcrtPrecision() * sizeof(double));
		
		radStep = M_2PI / g2.user_parameters.getAcrtPrecision();
		
		for(uint16_t i = 0; i < g2.user_parameters.getAcrtPrecision() ; i++){
			cosTable[i] = cos(i * radStep);
			sinTable[i] = sin(i * radStep);
			//tanTable[i] = tan(i * radStep);
		}
	}
	
	gcs().send_text(MAV_SEVERITY_INFO, "ACTR initied");
	
}
#endif

#ifdef USERHOOK_SUPERFASTLOOP
void Copter::userhook_SuperFastLoop()
{
	/**
	Code spécifique Acuated Rotor
	*/ 
	//Valeur à calculer
	double thrustOscillant = 0;
	double motorPosReadRads = 0.0;
	double motorPosRads = 0.0;
	float angularSpeed = 0.0;
	
	uint64_t timerStart = AP_HAL::micros64();
	uint64_t deltaT = timerStart - _lastTimeCalc;
	_lastTimeCalc = timerStart;
	if(g2.user_parameters.getAcrtDebug()  == 1){
		if(calculationStartTimerActr == 0){
			calculationStartTimerActr = AP_HAL::millis();
		}
	}
	
	float pitchCTRL = motors->get_pitch();
	float rollCTRL = motors->get_roll();
	
	AP_Servo_Telem *servoTelem = AP_Servo_Telem::get_singleton();
	AP_Servo_Telem::TelemetryData telem_data;
	
	if(g2.user_parameters.getAcrtEnable() 
			&& g2.user_parameters.getAcrtServo() > 0
			&& motors->armed() 
			&& is_tradheli() 
			&& motors->get_interlock()){
		
		//On retrouve la position du moteur
		servoTelem->get_telem(g2.user_parameters.getAcrtServo()-1, telem_data);
		
		if (! telem_data.present(AP_Servo_Telem::TelemetryData::Types::MEASURED_POSITION)) {
			return;
		}
		
		if (telem_data.present(AP_Servo_Telem::TelemetryData::Types::SPEED)) {
			angularSpeed = radians(telem_data.speed);
		}
		
		motorPosReadRads = radians(telem_data.measured_position);
		
		//On estime la poistion qu'aura le moteur lorsque l'on everra l'ordre
		//En fonction des différents délais de traitement
		if(g2.user_parameters.getAcrtDelay() >= 0 && angularSpeed > 0){
			uint32_t motorDecalageTimeUs = (timerStart - telem_data.last_update_us) + g2.user_parameters.getAcrtDelay();
			motorPosRads = motorPosReadRads + angularSpeed*motorDecalageTimeUs/TEN_POW_SIX;
		}
		
		/**************  Time Based Phase Shifter     ********************/
        // Calcule l'avance angulaire nécessaire pour compenser le temps de retard
        float angularAdvance = angularSpeed * _tbpsTimeDelayUSeconds / TEN_POW_SIX;
        
        // Limite l'avance à 90° pour éviter les instabilités
        if (angularAdvance > M_PI_2) angularAdvance = M_PI_2;

        motorPosRads = fmod(motorPosRads + angularAdvance, M_2PI);
		/**************  FIN Time Based Phase Shifter     ********************/
		
		uint16_t motorPosTabIdx = -1;
		if(g2.user_parameters.getAcrtPrecision() > MIN_PRECISION){
			motorPosTabIdx = (uint16_t) (motorPosRads / radStep);
		}
		
		while(motorPosTabIdx >= g2.user_parameters.getAcrtPrecision()){
			motorPosTabIdx -= g2.user_parameters.getAcrtPrecision();
		}
		
		if(g2.user_parameters.getAcrtDebug() == 2){
			gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR motpos %f idx %u spd %f", motorPosRads, motorPosTabIdx, angularSpeed);
		}
		
		//Valeurs de commande
		//valeur entre 0 et 1
		float throttleIN = motors->get_throttle();

		if(throttleIN > 0){	
			//valeurs entre -1 et 1
			
			pitchCTRL=0.0f;
			
			//Angle du déplacement demandé
			double angleDep = 0;
			
			if(g2.user_parameters.getAcrtDebug() == 3){
				gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR ch# %i ch.min %i ch.max %i th %f pi %f ro %f",
													actrChannel->get_function(), actrChannel->get_output_min(), actrChannel->get_output_max(), throttleIN, pitchCTRL, rollCTRL);	   
			}
			
			if (pitchCTRL > 0.0 || pitchCTRL < 0.0) {
				if (rollCTRL > 0.0 || rollCTRL < 0.0){
					angleDep = atan2F(rollCTRL, pitchCTRL);
					if (angleDep < 0){
			          angleDep += M_2PI; // on ajoute 360deg pour le rendre entre 0 et 360
			        }
				}else{
			        //On n'a pas de roll, mais on doit checker si on veut aller vers l'avant ou vers l'arriere
			        if (pitchCTRL > 0.0) {
			          angleDep = 0;
			        }else if (pitchCTRL < 0.0) {
			          angleDep = M_PI;
			        }
			    }
			}else{
			    //On n'a pas de pitch, mais on doit checker si on veut aller vers la droite ou gauche
				if (rollCTRL > 0.0){
			    	angleDep = M_PI_2; // 90 degres
				}else if (rollCTRL < 0.0 ){
			    	angleDep = 3*M_PI_2; // 270 degrés
				}
			}

			//Calcul des minis et maxi pour mapper le résultat
			double angleDepForMin = angleDep + M_PI; //on ajoute 180 deg pour trouver l'opposé du déplacement qui est soit le mini soit le maxi
		    if (angleDepForMin > M_2PI){
		      angleDepForMin = angleDepForMin - M_2PI;
		    }
			
			//calcul des valeurs min/max de commande que l'on peut atteindre pour ce déplacement
			double cosAngleDep = 0.0;
			double sinAngleDep = 0.0;
			double cosAngleDepMin = 0.0;
			double sinAngleDepMin = 0.0;
			double cosMotPos = 0.0;
			double sinMotPos = 0.0;
			if(g2.user_parameters.getAcrtPrecision() > MIN_PRECISION){
				uint16_t radStepForDepMax = (uint16_t) (angleDep/radStep);
			    cosAngleDep = cosTable[radStepForDepMax];
			    sinAngleDep = sinTable[radStepForDepMax];
		
			    uint16_t radStepForDepMin = (uint16_t) (angleDepForMin/radStep);
			    cosAngleDepMin = cosTable[radStepForDepMin];
			    cosAngleDepMin = sinTable[radStepForDepMin];
				
				cosMotPos = cosTable[motorPosTabIdx];
				sinMotPos = sinTable[motorPosTabIdx];
			}else{
				cosAngleDep = cos(angleDep);
				sinAngleDep = sin(angleDep);
				cosAngleDepMin = cos(angleDepForMin);
				cosAngleDepMin = sin(angleDepForMin);
				
				cosMotPos = cos(motorPosRads);
				sinMotPos = sin(motorPosRads);
			}
		    float absolutePicth = abs(pitchCTRL);
		    float absolutRoll = abs(rollCTRL);
	
			// calcul de la commande a envoyer au moteur en fonction de la position de l'hélice
		    thrustOscillant = throttleIN + throttleIN * ((pitchCTRL * cosMotPos) + (rollCTRL * sinMotPos)) * g2.user_parameters.getAcrtFactorRollPitch();
		    
			double maxCmdVal = throttleIN + throttleIN * ((absolutePicth * cosAngleDep) + (absolutRoll * sinAngleDep)) * g2.user_parameters.getAcrtFactorRollPitch();
		    double minCmdVal = throttleIN + throttleIN * ((absolutePicth * cosAngleDepMin) + (absolutRoll * sinAngleDepMin)) * g2.user_parameters.getAcrtFactorRollPitch();

			if (maxCmdVal < minCmdVal){
		      double tmpCmdval = minCmdVal;
		      minCmdVal = maxCmdVal;
		      maxCmdVal = tmpCmdval;
		    }
			
			double outputMaxCmdVal = MIN(1, maxCmdVal);
		    double outputMinCmdVal = MAX(throttleIN*g2.user_parameters.getAcrtMinThrottleRatio(), minCmdVal);
			
			//On map le thrust sur le min/max du thrust possible
			thrustOscillant = outputMinCmdVal + (thrustOscillant - minCmdVal)*(outputMaxCmdVal - outputMinCmdVal)/(maxCmdVal - minCmdVal);
			
			/**************  Time Based Phase Shifter     ********************/
			_tbpsAvgThrottle = (_tbpsAvgThrottle * 0.99f) + (thrustOscillant * 0.01f);
	        _tbpsAvgSpeed = (_tbpsAvgSpeed * 0.99f) + (angularSpeed * 0.01f);

	        // Détection passage moyenne - Commande
	        if (_tbpsThrottleWasBelow && thrustOscillant > _tbpsAvgThrottle) {
	            _tbpsAngleAtCrossThrottle = motorPosReadRads;
	            _tbpsTimeAtCrossThrottle = timerStart; // On note le temps précis
	            _tbpsThrottleWasBelow = false;
	        } else if (thrustOscillant < _tbpsAvgThrottle) {
	            _tbpsThrottleWasBelow = true;
	        }

	        // Détection passage moyenne - Vitesse réelle
	        if (_tbpsSpeedWasBelow && angularSpeed > _tbpsAvgSpeed) {
	            float angleAtCrossSpeed = motorPosReadRads;
	            _tbpsSpeedWasBelow = false;
	            
	            // Calcul du retard temporel constaté sur ce cycle
	            float measuredDelayRad = fmod(angleAtCrossSpeed - _tbpsAngleAtCrossThrottle + M_2PI, M_2PI);
	            if (angularSpeed > 0) {
	                double measuredTimeDelayUS = (measuredDelayRad / angularSpeed) * TEN_POW_SIX ;
	                
	                // Mise à jour de la constante de temps (Lissage)
	                _tbpsTimeDelayUSeconds = (_tbpsTimeDelayUSeconds * (1.0f - _tbpsLearningRate)) + (measuredTimeDelayUS * _tbpsLearningRate);
	            }
	        } else if (angularSpeed < _tbpsAvgSpeed) {
	            _tbpsSpeedWasBelow = true;
	        }
			/**************  Fin Time Based Phase Shifter     ********************/
			//On boost en demande d'acceleration et on deboost en décélération
			// BOOST PRÉDICTIF (Vertical / Dynamique)
	        // On calcule la vitesse de variation de l'ordre
			// CALCUL DE LA DÉRIVÉE FILTRÉE
	        double instantDerivative = (thrustOscillant - lastRawThrottle) * TEN_POW_SIX / deltaT ;
			if(instantDerivative == instantDerivative){ //Si NaN on ne passe pas dans le filtre
				// Filtre passe-bas pour éviter les vibrations
		        filteredDerivative = (g2.user_parameters.getAcrtBoostAlphaFilter() * instantDerivative) + ((1.0f - g2.user_parameters.getAcrtBoostAlphaFilter()) * filteredDerivative);
		        lastRawThrottle = thrustOscillant;

		        // 3. ADAPTATION DU BOOST AU RÉGIME
		        // Le boost augmente proportionnellement aux RPM
		        //float adaptiveKBoost = kBoostBase * (angularSpeed / refRPM);
		        // Si l'effet aéro est très fort, utilisez le carré :
		        double adaptiveKBoost =  pow(angularSpeed / refSpeedRadSec, 2);
				double kBoost = kBoostBase;
				if (filteredDerivative > 0) {
			        // Gain plus fort pour l'accélération
			        kBoost *= 1.5f; 
			    } else {
			        // Gain plus faible pour la décélération (l'air aide déjà)
			        kBoost *=  0.8f;
			    }

		        thrustOscillant += (g2.user_parameters.getAcrtBoostK() * filteredDerivative * adaptiveKBoost);
				
				if(g2.user_parameters.getAcrtDebug() == 6){
					gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR instdrv %f drv %f bst %f",
														instantDerivative, filteredDerivative, adaptiveKBoost);

				}
			}
			
			thrustOscillant = MAX(0, MIN(1, thrustOscillant));
			
			if(g2.user_parameters.getAcrtDebug() == 4){
				gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR %f < cmdVal < %f %f < output < %f thr %f",
													minCmdVal, maxCmdVal, outputMinCmdVal, outputMaxCmdVal, thrustOscillant);

			}
			 
		}	
	    
	}
	if(g2.user_parameters.getAcrtServo() > 0){
		if(thrustOscillant > 0) {
			//On calcule le PWM
			//Min et max du channel out
			uint16_t channelOutMin = actrChannel->get_output_min();
			uint16_t channelOutMax = actrChannel->get_output_max();
			
	     	uint16_t pwmOscillant = (uint16_t) (channelOutMin + thrustOscillant * (channelOutMax - channelOutMin));
			if(g2.user_parameters.getAcrtDebug() == 5){
				uint32_t timerEnd = AP_HAL::micros64() - timerStart;
				
				gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR pwm %i calc in %li us", pwmOscillant, timerEnd);
			}
			actrChannel->set_output_pwm(pwmOscillant);

			#if HAL_LOGGING_ENABLED
			if(g2.user_parameters.getAcrtDebug() >= 0){
				
				const log_actr pkt {
			      LOG_PACKET_HEADER_INIT(LOG_ACTR),
			      time_us         : AP_HAL::micros64(),
			      throttle 		  : (float) thrustOscillant,
				  pwm 	 		  : pwmOscillant,
			      posRead         : (float) motorPosReadRads,
				  pos          	  : (float) motorPosRads,
				  speed			  : angularSpeed,
				  dpitch 		  : pitchCTRL,
				  droll			  : rollCTRL,
			    };
			    logger.WriteBlock(&pkt, sizeof(pkt));
			}
			#endif
	    }else{
	    	actrChannel->set_output_pwm(PWM_OFF);
		}
	}
	
	
	if(g2.user_parameters.getAcrtDebug() == 1){
		calculationTimeActr += AP_HAL::micros64() - timerStart;
		calculationIdxActr ++;
		if(calculationIdxActr >= NBRE_CALC_TIME_ACTR){
			uint16_t calculationAvg = (uint16_t) calculationTimeActr/NBRE_CALC_TIME_ACTR;
			uint16_t timer = AP_HAL::millis() - calculationStartTimerActr;
			
			gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR time avg %i us %i loops %i ms", calculationAvg, NBRE_CALC_TIME_ACTR, timer);
			calculationIdxActr = 0;
			calculationTimeActr = 0;
			calculationStartTimerActr = 0;
		}
	}
	
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
