#ifndef _HEADER_B5HVEPRRVCKO_INCLUDED_
#define _HEADER_B5HVEPRRVCKO_INCLUDED_

#include "uavtalk.h"
#include <stdint.h>

enum
{
    MANUALCONTROLCOMMAND_OBJID = 0x161A2C98,
    MANUALCONTROLCOMMAND_ISSINGLEINST = 1,
    MANUALCONTROLCOMMAND_ISSETTINGS = 0,
    MANUALCONTROLCOMMAND_ISPRIORITY = 0,
    MANUALCONTROLCOMMAND_NUMBYTES = 44 // 42
};

typedef enum {
    MANUALCONTROLCOMMAND_CONNECTED_FALSE = 0,
    MANUALCONTROLCOMMAND_CONNECTED_TRUE = 1
} ManualControlCommandConnectedOptions;


typedef struct
{
    float Throttle;
    float Roll;
    float Pitch;
    float Yaw;
    float Collective;
    float Thrust;
    uint16_t Channel[9];
    ManualControlCommandConnectedOptions Connected;
    uint8_t FlightModeSwitchPosition;
}
uavtalk_ManualControlCommand;

inline void uavtalk_ManualControlCommand_parse(uint8_t *source, uavtalk_ManualControlCommand *msg)
{
    // TODO parse
    msg->Throttle = uavtalk_get_float(&source);
    msg->Roll = uavtalk_get_float(&source);
    msg->Pitch = uavtalk_get_float(&source);
    msg->Yaw = uavtalk_get_float(&source);
    msg->Collective = uavtalk_get_float(&source);
    msg->Thrust = uavtalk_get_float(&source);
}

inline void uavtalk_ManualControlCommand_emit(uint8_t *dest,
                                              uavtalk_ManualControlCommand *msg,
                                              uavtalk_msg_t type,
                                              size_t *final_size)
{
    uint8_t crc = 0;
    uint8_t *iter = dest;
    uavtalk_emit_header(&iter, type,
                        MANUALCONTROLCOMMAND_OBJID,
                        MANUALCONTROLCOMMAND_NUMBYTES,
                        &crc);
    uavtalk_emit_float(msg->Throttle, &iter, &crc);
    uavtalk_emit_float(msg->Roll, &iter, &crc);
    uavtalk_emit_float(msg->Pitch, &iter, &crc);
    uavtalk_emit_float(msg->Yaw, &iter, &crc);
    uavtalk_emit_float(msg->Collective, &iter, &crc);
    uavtalk_emit_float(msg->Thrust, &iter, &crc);

    size_t i;
    for (i = 0; i < 9; ++i) uavtalk_emit_uint16(msg->Channel[i], &iter, &crc);

    uavtalk_emit_uint8(msg->Connected, &iter, &crc);
    uavtalk_emit_uint8(msg->FlightModeSwitchPosition, &iter, &crc);

    uavtalk_emit_checksum(crc, &iter);
    *final_size = (iter - dest);
}

#if 0
#include <cstdint>
#include "uavtalk.hpp"
#include "uavobject.hpp"

namespace copexp
{
namespace uavobjects
{

class ManualControlCommand
{
public:
    ManualControlCommand(uint8_t* data) : _data(data) {}

    enum
    {
        OBJID = 0xC4107480
        , ISSINGLEINST = 1
        , ISSETTINGS = 0
        , ISPRIORITY = 0
        , NUMBYTES = 46
    };

    typedef enum
    {
        False = 0,
        True = 1
    }
    ConnectedOptions;

    inline float getThrottle()
    {
        return get<float>(this, FIELD_THROTTLE);
    }

    inline float getRoll()
    {
        return *reinterpret_cast<float*>(_data + FIELD_ROLL);
    }

    inline float getPitch()
    {
        return *reinterpret_cast<float*>(_data + FIELD_PITCH);
    }

    inline float getCollective()
    {
        return *reinterpret_cast<float*>(_data + FIELD_COLLECTIVE);
    }

    inline float getThrust()
    {
        return *reinterpret_cast<float*>(_data + FIELD_THRUST);
    }

    uint8_t* content() { return _data; }

private:
    friend class copexp::UAVObjectParser;
    enum
    {
        DATA_LAYOUT
        , FIELD_THROTTLE                 =  0 /* float */
        , FIELD_ROLL                     =  4 /* float */
        , FIELD_PITCH                    =  8 /* float */
        , FIELD_YAW                      = 12 /* float */
        , FIELD_COLLECTIVE               = 16 /* float */
        , FIELD_THRUST                   = 20 /* float */
        , FIELD_CHANNEL                  = 24 /* uint16[10] */
        , FIELD_CONNECTED                = 44 /* False,True */
        , FIELD_FLIGHTMODESWITCHPOSITION = 45 /* uint8 */
    };

    uint8_t* _data;
};

class ManualControlCommandData : ManualControlCommand
{
public:
    ManualControlCommandData() : ManualControlCommand(_real_data) {}

private:
    uint8_t _real_data[ManualControlCommand::NUMBYTES];
};

} // namespace uavobjects
} // namespace copexp

#endif

#endif // _HEADER_B5HVEPRRVCKO_INCLUDED_
