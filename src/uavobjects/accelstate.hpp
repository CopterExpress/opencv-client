#ifndef _HEADER_23GZ751JQGZI_INCLUDED_
#define _HEADER_23GZ751JQGZI_INCLUDED_

#include <cstdint>
#include "uavtalk.hpp"
#include "uavobject.hpp"

namespace copexp
{
namespace uavobjects
{

class AccelState
{
public:
    AccelState(uint8_t* data) : _data(data) {}

    enum
    {
        OBJID = 0xAD3C0E06
        , ISSINGLEINST = 1
        , ISSETTINGS = 0
        , ISPRIORITY = 0
        , NUMBYTES = 22
    };

    typedef enum
    {
        False = 0,
        True = 1
    }
    ConnectedOptions;

    inline float X();
    inline void setX(float value);

    inline float Y();
    inline void setY(float value);

    inline float Z();
    inline void setZ(float value);

    uint8_t* content() { return _data; }

private:
    enum
    {
        DATA_LAYOUT
        , FIELD_X                =  0 /* float */
        , FIELD_Y                =  4 /* float */
        , FIELD_Z                =  8 /* float */
    };

    uint8_t* _data;
};

class AccelStateData : public AccelState
{
public:
    AccelStateData() : AccelState(_real_data) {}

private:
    uint8_t _real_data[AccelState::NUMBYTES];
};

float AccelState::X()
{
    return get<float>(this, FIELD_X);
}

void AccelState::setX(float value)
{
    set(this, value, FIELD_X);
}

float AccelState::Y()
{
    return get<float>(this, FIELD_Y);
}

void AccelState::setY(float value)
{
    set(this, value, FIELD_Y);
}

float AccelState::Z()
{
    return get<float>(this, FIELD_Z);
}

void AccelState::setZ(float value)
{
    set(this, value, FIELD_Z);
}

} // namespace uavobjects
} // namespace copexp

#endif // _HEADER_23GZ751JQGZI_INCLUDED_
