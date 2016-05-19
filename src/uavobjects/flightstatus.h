#ifndef _HEADER_YJ0SXEK9K074_INCLUDED_
#define _HEADER_YJ0SXEK9K074_INCLUDED_

#include "uavtalk.h"
#include <stdint.h>

enum
{
    FLIGHTSTATUS_OBJID = 0x2E6D4754,
    FLIGHTSTATUS_ISSINGLEINST = 1,
    FLIGHTSTATUS_ISSETTINGS = 0,
    FLIGHTSTATUS_ISPRIORITY = 1,
    FLIGHTSTATUS_NUMBYTES = 10, // TODO calculate proper size

    // Number of elements for field ControlChain
    FLIGHTSTATUS_CONTROLCHAIN_NUMELEM = 3
};

/* Field Armed information */

// Enumeration options for field Armed
typedef enum {
    FLIGHTSTATUS_ARMED_DISARMED=0,
    FLIGHTSTATUS_ARMED_ARMING=1,
    FLIGHTSTATUS_ARMED_ARMED=2
} FlightStatusArmedOptions;

/* Field AlwaysStabilizeWhenArmed information */

// Enumeration options for field AlwaysStabilizeWhenArmed
typedef enum {
    FLIGHTSTATUS_ALWAYSSTABILIZEWHENARMED_FALSE=0,
    FLIGHTSTATUS_ALWAYSSTABILIZEWHENARMED_TRUE=1
} FlightStatusAlwaysStabilizeWhenArmedOptions;

/* Field FlightMode information */

// Enumeration options for field FlightMode
typedef enum {
    FLIGHTSTATUS_FLIGHTMODE_MANUAL=0,
    FLIGHTSTATUS_FLIGHTMODE_STABILIZED1=1,
    FLIGHTSTATUS_FLIGHTMODE_STABILIZED2=2,
    FLIGHTSTATUS_FLIGHTMODE_STABILIZED3=3,
    FLIGHTSTATUS_FLIGHTMODE_STABILIZED4=4,
    FLIGHTSTATUS_FLIGHTMODE_STABILIZED5=5,
    FLIGHTSTATUS_FLIGHTMODE_STABILIZED6=6,
    FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD=7,
    FLIGHTSTATUS_FLIGHTMODE_COURSELOCK=8,
    FLIGHTSTATUS_FLIGHTMODE_VELOCITYROAM=9,
    FLIGHTSTATUS_FLIGHTMODE_HOMELEASH=10,
    FLIGHTSTATUS_FLIGHTMODE_ABSOLUTEPOSITION=11,
    FLIGHTSTATUS_FLIGHTMODE_RETURNTOBASE=12,
    FLIGHTSTATUS_FLIGHTMODE_LAND=13,
    FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER=14,
    FLIGHTSTATUS_FLIGHTMODE_POI=15,
    FLIGHTSTATUS_FLIGHTMODE_AUTOCRUISE=16,
    FLIGHTSTATUS_FLIGHTMODE_AUTOTAKEOFF=17
} FlightStatusFlightModeOptions;

/* Field FlightModeAssist information */

// Enumeration options for field FlightModeAssist
typedef enum {
    FLIGHTSTATUS_FLIGHTMODEASSIST_NONE=0,
    FLIGHTSTATUS_FLIGHTMODEASSIST_GPSASSIST_PRIMARYTHRUST=1,
    FLIGHTSTATUS_FLIGHTMODEASSIST_GPSASSIST=2
} FlightStatusFlightModeAssistOptions;

/* Field AssistedControlState information */

// Enumeration options for field AssistedControlState
typedef enum {
    FLIGHTSTATUS_ASSISTEDCONTROLSTATE_PRIMARY=0,
    FLIGHTSTATUS_ASSISTEDCONTROLSTATE_BRAKE=1,
    FLIGHTSTATUS_ASSISTEDCONTROLSTATE_HOLD=2
} FlightStatusAssistedControlStateOptions;

/* Field AssistedThrottleState information */

// Enumeration options for field AssistedThrottleState
typedef enum {
    FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL=0,
    FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_AUTO=1,
    FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_AUTOOVERRIDE=2
} FlightStatusAssistedThrottleStateOptions;

/* Field ControlChain information */

// Enumeration options for field ControlChain
typedef enum {
    FLIGHTSTATUS_CONTROLCHAIN_FALSE=0,
    FLIGHTSTATUS_CONTROLCHAIN_TRUE=1
} FlightStatusControlChainOptions;

// Array element names for field ControlChain
typedef enum {
    FLIGHTSTATUS_CONTROLCHAIN_STABILIZATION=0,
    FLIGHTSTATUS_CONTROLCHAIN_PATHFOLLOWER=1,
    FLIGHTSTATUS_CONTROLCHAIN_PATHPLANNER=2
} FlightStatusControlChainElem;



typedef struct {
    FlightStatusControlChainOptions Stabilization;
    FlightStatusControlChainOptions PathFollower;
    FlightStatusControlChainOptions PathPlanner;
}  FlightStatusControlChainData;

typedef struct {
    FlightStatusControlChainOptions array[3];
} FlightStatusControlChainDataArray;

//#define FlightStatusControlChainToArray( var ) UAVObjectFieldToArray( FlightStatusControlChainData, var )


/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
    FlightStatusArmedOptions Armed;
    FlightStatusAlwaysStabilizeWhenArmedOptions AlwaysStabilizeWhenArmed;
    FlightStatusFlightModeOptions FlightMode;
    FlightStatusFlightModeAssistOptions FlightModeAssist;
    FlightStatusAssistedControlStateOptions AssistedControlState;
    FlightStatusAssistedThrottleStateOptions AssistedThrottleState;
    FlightStatusControlChainData ControlChain;

} uavtalk_FlightStatus;

inline void uavtalk_FlightStatus_parse(uint8_t* source, uavtalk_FlightStatus* msg)
{
    msg->Armed = (FlightStatusArmedOptions)uavtalk_get_uint8(&source);
    msg->AlwaysStabilizeWhenArmed = (FlightStatusAlwaysStabilizeWhenArmedOptions)uavtalk_get_uint8(&source);
    msg->FlightMode = (FlightStatusFlightModeOptions)uavtalk_get_uint8(&source);
}

inline void uavtalk_FlightStatus_emit(uint8_t* dest, uavtalk_FlightStatus* msg,
                                      uavtalk_msg_t type, size_t* final_size);

#endif // _HEADER_YJ0SXEK9K074_INCLUDED_
