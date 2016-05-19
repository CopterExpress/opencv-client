#ifndef _HEADER_40146204X8LO_INCLUDED_
#define _HEADER_40146204X8LO_INCLUDED_

#include <cstring>
#include <endian.h>

namespace copexp
{
namespace uavobjects
{

template <typename Value, typename UAVObject>
inline void set(UAVObject *object, Value value, size_t offset)
{
#if BYTE_ORDER == LITTLE_ENDIAN
    memcpy(&value, object->content() + offset, sizeof(value));
#else
#error Big endian CPUs not supported at the time
#endif
}

template <typename Value, typename UAVObject>
inline Value get(UAVObject *object, size_t offset)
{
#if BYTE_ORDER == LITTLE_ENDIAN
    return *reinterpret_cast<float*>(object->content() + offset);
#else
#error Big endian CPUs not supported at the time
#endif
}

} // uavobjects
} // copexp

#endif // _HEADER_40146204X8LO_INCLUDED_
