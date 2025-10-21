#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

	AP_Int8 getAcrtEnable(){return acrtEnable;} // = 0;
	AP_Int8 getAcrtDebug(){return acrtDebug;} // = 0;
	AP_Int16 getAcrtPrecision(){return acrtPrecision;} // = 720;
	AP_Int8 getAcrtServo(){return acrtServo;} // = 94;
	AP_Float getAcrtFactorRollPitch(){return acrtFactorRollPitch;} // = 0.25;
	AP_Int16 getAcrtDelay(){return acrtDelay;}

private:
    // Put your parameter variable definitions here
	
	AP_Int8 acrtEnable; // = 0;
	AP_Int8 acrtDebug; // = 0;
	AP_Int16 acrtPrecision; // = 720;
	AP_Int8 acrtServo; // = 94;
	AP_Float acrtFactorRollPitch; // = 0.25;
	AP_Int16 acrtDelay;

};
