#include "Copter.h"
#include <AP_RPM/RPM_AS5600.h>

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
	if(g2.user_parameters.getAcrtDebug()){
		gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR Activated");
	}
	
	//printf("userhook_init 1");
	//On retrouve le channel du moteur
	actrChannel = SRV_Channels::srv_channel(g2.user_parameters.getAcrtServo());
	
	//printf("userhook_init 2");
	
	//Stockage des cos/sin pour accélérer les traitements ensuite
	cosTable = (double *)malloc(g2.user_parameters.getAcrtPrecision() * sizeof(double));
	sinTable = (double *)malloc(g2.user_parameters.getAcrtPrecision() * sizeof(double));
	tanTable = (double *)malloc(g2.user_parameters.getAcrtPrecision() * sizeof(double));
	
	//printf("userhook_init 3");
	
	radStep = M_2PI / g2.user_parameters.getAcrtPrecision();
	
	for(uint16_t i = 0; i < g2.user_parameters.getAcrtPrecision() ; i++){
		//printf("userhook_init 4");
		cosTable[i] = cos(i * radStep);
		sinTable[i] = sin(i * radStep);
		tanTable[i] = tan(i * radStep);
	}
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
	
	uint32_t timerStart = 0;
	if(g2.user_parameters.getAcrtDebug()){
		timerStart = AP_HAL::micros64();
		if(calculationStartTimerActr == 0){
			calculationStartTimerActr = AP_HAL::millis();;
		}
	}
	
	if(g2.user_parameters.getAcrtEnable()){
		
		//On retrouve la position du moteur
		AP_Servo_Telem *servoTelem = AP_Servo_Telem::get_singleton();
		AP_Servo_Telem::TelemetryData telem_data;
		servoTelem->get_telem(g2.user_parameters.getAcrtServo(), telem_data);
		
		if (! telem_data.present(AP_Servo_Telem::TelemetryData::Types::MEASURED_POSITION)) {
			return;
		}
		
		float motorPosRads = radians(telem_data.measured_position);
		
		//On estime la poistion qu'aura le moteur lorsque l'on everra l'ordre
		//En fonction des différents délais de traitement
		float motorSpeed = radians(telem_data.speed);
		uint32_t motorDecalageTimeUs = (AP_HAL::micros() - telem_data.last_update_us) + g2.user_parameters.getAcrtDelay();
		motorPosRads += (motorSpeed/1000000)*motorDecalageTimeUs;
		
		uint16_t motorPosTabIdx = (uint16_t) (motorPosRads / radStep);
		
		if(g2.user_parameters.getAcrtDebug() == 2){
			gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR motor position %f and array idx %u", motorPosRads, motorPosTabIdx);
		}
		
		//Valeurs de commande
		//valeur entre 0 et 1
		float throttleIN = motors->get_throttle();
		
		//TODO a supprimer DEBUG pour avoir une valeur
		throttleIN = 0.5;

		if(throttleIN > 0){	
			//valeurs entre -1 et 1
			float pitchCTRL = motors->get_pitch();
			float rollCTRL = motors->get_roll();
			
			//Angle du déplacement
			double angleDep = 0;
			
			if(g2.user_parameters.getAcrtDebug() == 2){
				gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR actrChannel.num %i actrChannel.min %i actrChannel.max %i throttleIN %f pitchCTRL %f rollCTRL %f",
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
		    uint16_t radStepForDepMax = (uint16_t) (angleDep/radStep);
		    double cosAngleDep = cosTable[radStepForDepMax];
		    double sinAngleDep = sinTable[radStepForDepMax];
	
		    uint16_t radStepForDepMin = (uint16_t) (angleDepForMin/radStep);
		    double cosAngleDepMin = cosTable[radStepForDepMin];
		    double sinAngleDepMin = sinTable[radStepForDepMin];
		    float absolutePicth = abs(pitchCTRL);
		    float absolutRoll = abs(rollCTRL);
		    double maxCmdVal = throttleIN + throttleIN * ((absolutePicth * cosAngleDep) + (absolutRoll * sinAngleDep)) * g2.user_parameters.getAcrtFactorRollPitch();
		    double minCmdVal = throttleIN + throttleIN * ((absolutePicth * cosAngleDepMin) + (absolutRoll * sinAngleDepMin)) * g2.user_parameters.getAcrtFactorRollPitch();
	

			if (maxCmdVal < minCmdVal){
		      double tmpCmdval = minCmdVal;
		      minCmdVal = maxCmdVal;
		      maxCmdVal = tmpCmdval;
		    }
			
			double outputMaxCmdVal = MIN(1, maxCmdVal);
		    double outputMinCmdVal = MAX(0, minCmdVal);
	
			// calcul de la commande a envoyer au moteur en fonction de la position de l'hélice
		    thrustOscillant = throttleIN + throttleIN * ((pitchCTRL * cosTable[motorPosTabIdx]) + (rollCTRL * sinTable[motorPosTabIdx])) * g2.user_parameters.getAcrtFactorRollPitch();
		    
			//On map le thrust sur le min/max du thrust possible
			thrustOscillant = outputMinCmdVal + (thrustOscillant - minCmdVal)*(outputMaxCmdVal - outputMinCmdVal)/(maxCmdVal - minCmdVal);
	
			if(g2.user_parameters.getAcrtDebug() == 2){
				gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR radStepForDepMax %i minCmdVal %f maxCmdVal %f outputMinCmdVal %f outputMaxCmdVal %f thrustOscillant %f",
													radStepForDepMax, minCmdVal, maxCmdVal, outputMinCmdVal, outputMaxCmdVal, thrustOscillant);

			}
		
			 
			#if HAL_LOGGING_ENABLED
			    struct log_actr pkt = {
			      LOG_PACKET_HEADER_INIT(LOG_ACTR),
			      time_us         : AP_HAL::micros64(),
			      thrust 		  : (float) thrustOscillant,
			      pos          	  : motorPosRads
			    };
			    logger.WriteBlock(&pkt, sizeof(pkt));
			#endif
		}	
	    
	}
	if(thrustOscillant > 0) {
		//On calcule le PWM
		//Min et max du channel out
		uint16_t channelOutMin = actrChannel->get_output_min();
		uint16_t channelOutMax = actrChannel->get_output_max();
		
     	uint16_t pwmOscillant = (uint16_t) (channelOutMin + thrustOscillant * (channelOutMax - channelOutMin));
		if(g2.user_parameters.getAcrtDebug() == 2){
			uint32_t timerEnd = AP_HAL::micros64() - timerStart;
			
			gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR pwmOscillant %ipwm in %lius start at %li", pwmOscillant, timerEnd, (uint32_t) (timerStart/1000));
		}
		actrChannel->set_output_pwm(pwmOscillant);
    }else{
    	actrChannel->set_output_pwm(0);
	}
	
	if(g2.user_parameters.getAcrtDebug()){
		calculationTimeActr += AP_HAL::micros64() - timerStart;
		calculationIdxActr ++;
		if(calculationIdxActr >= NBRE_CALC_TIME_ACTR){
			uint16_t calculationAvg = (uint16_t) calculationTimeActr/NBRE_CALC_TIME_ACTR;
			uint16_t timer = AP_HAL::millis() - calculationStartTimerActr;
			
			gcs().send_text(MAV_SEVERITY_DEBUG, "ACTR time avg %i us %i loops %ims", calculationAvg, NBRE_CALC_TIME_ACTR, timer);
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
