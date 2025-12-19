#include "UserParameters.h"

// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

	// @Param: USRACRT_ENABLE
	// @DisplayName: Enable Acuated Rotor
	// @Description: Enable the actuated rotor
	// @Range: 0 1
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO_FLAGS("ACRT_ENABLE", 0, UserParameters, acrtEnable, 0, AP_PARAM_FLAG_ENABLE),
	// @Param: USRACRT_DEBUG
	// @DisplayName: Enable Acuated Rotor debug
	// @Description: -1 pas de debug ni de logs, 0 pas de debug mais log, 1 uniquement le temps de traitement, 2 motor position, 3 donnees IN, 4 calculs, 5 donnees OUT
	// @Range: -1 10
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("ACRT_DEBUG", 1, UserParameters, acrtDebug, -1), // -1 pas de debug ni de logs, 0 pas de debug mais log, 1 uniquement le temps de traitement, 2 motor position, 3 donnees IN, 4 calculs, 5 donnees OUT
	// @Param: USRACRT_PRECI
	// @DisplayName: Precision of calculation
	// @Description: Nombre de division de 360deg pour approximation des calculs. > 2 pour activer
	// @Range: 0 10000
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("ACRT_PRECI", 2, UserParameters, acrtPrecision, 720),
	// @Param: USRACRT_SRV
	// @DisplayName: Servo to output 
	// @Description: Servo to read position and to output PWM
	// @Range: 0 32
	// @Increment: 1
	// @User: Advanced
    AP_GROUPINFO("ACRT_SRV", 3, UserParameters, acrtServo, 0),
	// @Param: USRACRT_FCTRP
	// @DisplayName: Roll/Pitch Factor
	// @Description: Roll/Pitch Factor to apply to PWM
	// @Range: 0 1
	// @Increment: 0.01
	// @User: Advanced
    AP_GROUPINFO("ACRT_FCTRP", 4, UserParameters, acrtFactorRollPitch, 0.25),
	// @Param: USRACRT_DELAY
	// @DisplayName: Delay
	// @Description: Position read and calculation delay to estimate real motor position at time to send order. 0 to take into account only times elapsed from mesurment to now. - 1 to desactivate
	// @Range: 0 1
	// @Increment: 0.01
	// @Units: us
	// @User: Advanced
	AP_GROUPINFO("ACRT_DELAY", 5, UserParameters, acrtDelay, 6), //us
	//TODO voir l'utilité
	// @Param: USRACRT_MINTHR
	// @DisplayName: Min throttle to output
	// @Description: Minimal throttle ratio to output related to throttle input
	// @Range: 0 1
	// @Units: %
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("ACRT_MINTHR", 6, UserParameters, acrtMinThrottleRatio, 0.9),
	// @Param: USRACRT_BSTAL
	// @DisplayName: Boost Alpha
	// @Description: Filtre la dérivée du changement de throttle d'entrée pour le boost (0.1 = très filtré, 1.0 = pas de filtre)
	// @Range: 0 1
	// @Units: %
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("ACRT_BSTAL", 7, UserParameters, acrtBoostAlphaFilter, 0.3),
	// @Param: USRACRT_BSTK
	// @DisplayName: Boost Factor
	// @Description: Le facteur de boost à un régime de référence (ex: 1000 RPM)
	// @Range: 0 1
	// @Units: %
	// @Increment: 0.001
	// @User: Advanced
    AP_GROUPINFO("ACRT_BSTK", 8, UserParameters, acrtBoostK, 0.001),

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}