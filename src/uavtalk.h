#ifndef _HEADER_LIL3XNMHDXI3_INCLUDED_
#define _HEADER_LIL3XNMHDXI3_INCLUDED_

#include <stdint.h>
#include <string.h>

#define uavtalk_memset(UAVObject) memset(&UAVObject, 0, sizeof(UAVObject))

typedef enum {
    UAVTALK_OBJ     = 0x20,
    UAVTALK_OBJ_REQ = 0x21,
    UAVTALK_OBJ_ACK = 0x22,
    UAVTALK_ACK     = 0x23,
    UAVTALK_NACK    = 0x24
}
uavtalk_msg_t;

inline void uavtalk_emit_header(uint8_t **source,
                                uavtalk_msg_t type,
                                uint32_t id,
                                size_t size,
                                uint8_t *crc);

inline float uavtalk_get_float(uint8_t **source);
inline uint32_t uavtalk_get_uint32(uint8_t **source);
inline uint16_t uavtalk_get_uint16(uint8_t **source);
inline uint8_t uavtalk_get_uint8(uint8_t **source);
inline int32_t uavtalk_get_int32(uint8_t **source);
inline int16_t uavtalk_get_int16(uint8_t **source);
inline int8_t uavtalk_get_int8(uint8_t **source);

inline void uavtalk_emit_float(float source, uint8_t **dest, uint8_t *crc);
inline void uavtalk_emit_uint32(uint32_t source, uint8_t **dest, uint8_t *crc);
inline void uavtalk_emit_uint16(uint16_t source, uint8_t **dest, uint8_t *crc);
inline void uavtalk_emit_uint8(uint8_t source, uint8_t **dest, uint8_t *crc);
inline void uavtalk_emit_checksum(uint8_t crc, uint8_t **dest);
inline void uavtalk_mod_checksum(uint8_t *crc, void *input, size_t len);

extern const uint8_t uavtalk_crc_table[256];

inline void uavtalk_emit_request(uint8_t *dest,
                                 uint32_t id,
                                 uavtalk_msg_t type,
                                 size_t*final_size)
{
    uint8_t crc = 0;
    uint8_t *iter = dest;
    uavtalk_emit_header(&iter, type, id, 0, &crc);
    uavtalk_emit_checksum(crc, &iter);
    *final_size = (iter - dest);
}

inline void uavtalk_emit_header(uint8_t **source,
                                uavtalk_msg_t type,
                                uint32_t id,
                                size_t size,
                                uint8_t *crc)
{
    (*source)[0] = 0x3C;
    (*source)[1] = type;
    (*source)[2] = 10 + size; // TODO check instance parameter?
    (*source)[3] = 0;
    memcpy((*source) + 4, &id, sizeof(id));
    (*source)[8] = 0;
    (*source)[9] = 0;
    uavtalk_mod_checksum(crc, *source, 10);
    *source += 10;
}

inline float uavtalk_get_float(uint8_t **source)
{
    float value = *(float*)(*source);
    *source += sizeof(float);

    return value;
}

inline uint32_t uavtalk_get_uint32(uint8_t **source)
{
    uint32_t value = *(uint32_t*)(*source);
    *source += sizeof(uint32_t);

    return value;
}

inline uint16_t uavtalk_get_uint16(uint8_t **source);

inline uint8_t uavtalk_get_uint8(uint8_t **source)
{
    uint8_t value = **source;
    *source += sizeof(uint8_t);

    return value;
}

inline void uavtalk_emit_float(float source, uint8_t **dest, uint8_t *crc)
{
    memcpy(*dest, &source, sizeof(source));
    *dest += sizeof(source);
    uavtalk_mod_checksum(crc, &source, sizeof(source));
}

inline void uavtalk_emit_uint32(uint32_t source, uint8_t **dest, uint8_t *crc)
{
    memcpy(*dest, &source, sizeof(source));
    *dest += sizeof(source);
    uavtalk_mod_checksum(crc, &source, sizeof(source));
}

inline void uavtalk_emit_uint16(uint16_t source, uint8_t **dest, uint8_t *crc)
{
    memcpy(*dest, &source, sizeof(source));
    *dest += sizeof(source);
    uavtalk_mod_checksum(crc, &source, sizeof(source));
}

inline void uavtalk_emit_uint8(uint8_t source, uint8_t **dest, uint8_t *crc)
{
    memcpy(*dest, &source, sizeof(source));
    *dest += sizeof(source);
    uavtalk_mod_checksum(crc, &source, sizeof(source));
}

inline void uavtalk_emit_checksum(uint8_t crc, uint8_t **dest)
{
    **dest = crc;
    *dest += sizeof(crc);
}

inline void uavtalk_mod_checksum(uint8_t *crc, void *input, size_t len)
{
    size_t i;
    for (i = 0; i < len; ++i)
    {
        *crc = uavtalk_crc_table[*crc ^ ((uint8_t*)input)[i]];
    }
}

#endif // _HEADER_LIL3XNMHDXI3_INCLUDED_
