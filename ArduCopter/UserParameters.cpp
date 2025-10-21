#include "UserParameters.h"

// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

	AP_GROUPINFO("ACRT_ENABLE", 0, UserParameters, acrtEnable, 0),
    AP_GROUPINFO("ACRT_DEBUG", 1, UserParameters, acrtDebug, 0), // 0 pas de debug, 1 uniquement le temps de traitement, 2 traces de debug
    AP_GROUPINFO("ACRT_PRECI", 2, UserParameters, acrtPrecision, 720),
    AP_GROUPINFO("ACRT_SRV", 3, UserParameters, acrtServo, 0),
    AP_GROUPINFO("ACRT_FCTRP", 4, UserParameters, acrtFactorRollPitch, 0.25),
	AP_GROUPINFO("ACRT_DELAY", 4, UserParameters, acrtDelay, 6), //us

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}