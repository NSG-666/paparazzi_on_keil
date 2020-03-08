#include "dl_protocol.h"
#include "firmwares/rotorcraft/autopilot.h"

inline float DL_GUIDED_SETPOINT_NED_x(uint8_t* _payload)
{ 
	union { uint32_t u; float f; } _f; 
	_f.u = (uint32_t)(*((uint8_t*)_payload+4)|*((uint8_t*)_payload+4+1)<<8|((uint32_t)*((uint8_t*)_payload+4+2))<<16|((uint32_t)*((uint8_t*)_payload+4+3))<<24);
	return _f.f; 
}
inline float DL_GUIDED_SETPOINT_NED_y(uint8_t* _payload)
{ 
	union { uint32_t u; float f; } _f;
	_f.u = (uint32_t)(*((uint8_t*)_payload+8)|*((uint8_t*)_payload+8+1)<<8|((uint32_t)*((uint8_t*)_payload+8+2))<<16|((uint32_t)*((uint8_t*)_payload+8+3))<<24);
	return _f.f; 
}
inline float DL_GUIDED_SETPOINT_NED_z(uint8_t* _payload)
{
	union { uint32_t u; float f; } _f; 
	_f.u = (uint32_t)(*((uint8_t*)_payload+12)|*((uint8_t*)_payload+12+1)<<8|((uint32_t)*((uint8_t*)_payload+12+2))<<16|((uint32_t)*((uint8_t*)_payload+12+3))<<24);
	return _f.f; 
}
inline float DL_GUIDED_SETPOINT_NED_yaw(uint8_t* _payload)
{ 
	union { uint32_t u; float f; } _f;
	_f.u = (uint32_t)(*((uint8_t*)_payload+16)|*((uint8_t*)_payload+16+1)<<8|((uint32_t)*((uint8_t*)_payload+16+2))<<16|((uint32_t)*((uint8_t*)_payload+16+3))<<24);
	return _f.f; 
}

inline float DL_WIND_INFO_east(uint8_t* _payload) 
{ 
	union { uint32_t u; float f; } _f;
	_f.u = (uint32_t)(*((uint8_t*)_payload+4)|*((uint8_t*)_payload+4+1)<<8|((uint32_t)*((uint8_t*)_payload+4+2))<<16|((uint32_t)*((uint8_t*)_payload+4+3))<<24);
	return _f.f; 
}
inline float DL_WIND_INFO_north(uint8_t*_payload)
{ 
	union { uint32_t u; float f; } _f;
	_f.u = (uint32_t)(*((uint8_t*)_payload+8)|*((uint8_t*)_payload+8+1)<<8|((uint32_t)*((uint8_t*)_payload+8+2))<<16|((uint32_t)*((uint8_t*)_payload+8+3))<<24);
	return _f.f;
}
 
inline float DL_WIND_INFO_airspeed(uint8_t*_payload)
{ 
	union { uint32_t u; float f; } _f;
	_f.u = (uint32_t)(*((uint8_t*)_payload+12)|*((uint8_t*)_payload+12+1)<<8|((uint32_t)*((uint8_t*)_payload+12+2))<<16|((uint32_t)*((uint8_t*)_payload+12+3))<<24);
	return _f.f;
}

inline float DL_SETTING_value(uint8_t* _payload)
{ 
	union { uint32_t u; float f; } _f;
	_f.u = (uint32_t)(*((uint8_t*)_payload+4)|*((uint8_t*)_payload+4+1)<<8|((uint32_t)*((uint8_t*)_payload+4+2))<<16|((uint32_t)*((uint8_t*)_payload+4+3))<<24);
	return _f.f;
}

bool_t NavKillThrottle()
{
	if (autopilot_mode == AP_MODE_NAV) 
	{ 
		autopilot_set_motors_on(FALSE); 
	} 
	
	return FALSE;
}

bool_t NavResurrect()
{ 
	if (autopilot_mode == AP_MODE_NAV) 
	{ 
		autopilot_set_motors_on(TRUE); 
	} 
	
	return FALSE; 
}

//#define NavSetGroundReferenceHere() ({ nav_reset_reference(); FALSE; })
//#define NavSetAltitudeReferenceHere() ({ nav_reset_alt(); FALSE; })

bool_t NavSetGroundReferenceHere() 
{ 
	nav_reset_reference(); 
	return FALSE; 
}

bool_t NavSetAltitudeReferenceHere() 
{ 
	nav_reset_alt(); 
	return FALSE; 
}

//#define NavSetWaypointHere(_wp) ({ waypoint_set_here_2d(_wp); FALSE; })
//#define NavCopyWaypoint(_wp1, _wp2) ({ waypoint_copy(_wp1, _wp2); FALSE; })
//#define NavCopyWaypointPositionOnly(_wp1, _wp2) ({ waypoint_position_copy(_wp1, _wp2); FALSE; })

bool_t NavSetWaypointHere(uint8_t _wp)
{ 
	waypoint_set_here_2d(_wp); 
	return FALSE;
}

inline bool_t NavCopyWaypoint(uint8_t _wp1, uint8_t _wp2)
{
	waypoint_copy(_wp1, _wp2); 
	return FALSE;
}

inline bool_t NavCopyWaypointPositionOnly(uint8_t _wp1, uint8_t _wp2)
{
	waypoint_position_copy(_wp1, _wp2); 
	return FALSE;
}

bool_t NavStartDetectGround()
{
	autopilot_detect_ground_once = TRUE; 
	return FALSE;
}

void SystemInit()
{

}


