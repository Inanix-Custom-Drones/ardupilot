#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    /*AP_Int8 get_int8Param() const { return _int8; }
    AP_Int16 get_int16Param() const { return _int16; }
    AP_Float get_floatParam() const { return _float; }
*/

	AP_Int8 getAcrtEnable(){return acrtEnable;} // = 0;
	AP_Int8 getAcrtDebug(){return acrtDebug;} // = 0;
	AP_Int16 getAcrtPrecision(){return acrtPrecision;} // = 720;
	AP_Int8 getAcrtServo(){return acrtServo;} // = 94;
	AP_Float getAcrtFactorRollPitch(){return acrtFactorRollPitch;} // = 0.25;

private:
    // Put your parameter variable definitions here
	
	AP_Int8 acrtEnable; // = 0;
	AP_Int8 acrtDebug; // = 0;
	AP_Int16 acrtPrecision; // = 720;
	AP_Int8 acrtServo; // = 94;
	AP_Float acrtFactorRollPitch; // = 0.25;

};
