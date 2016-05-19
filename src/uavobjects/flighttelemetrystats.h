#ifndef _HEADER_Q4MD8IJQ4YOR_INCLUDED_
#define _HEADER_Q4MD8IJQ4YOR_INCLUDED_

#include "uavtalk.h"
#include <stdint.h>

enum
{
    FLIGHTTELEMETRYSTATS_OBJID = 0x6737BB5A,
    FLIGHTTELEMETRYSTATS_ISSINGLEINST = 1,
    FLIGHTTELEMETRYSTATS_ISSETTINGS = 0,
    FLIGHTTELEMETRYSTATS_ISPRIORITY = 1,
    FLIGHTTELEMETRYSTATS_NUMBYTES = 37
};

typedef enum {
    FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED = 0,
    FLIGHTTELEMETRYSTATS_STATUS_HANDSHAKEREQ = 1,
    FLIGHTTELEMETRYSTATS_STATUS_HANDSHAKEACK = 2,
    FLIGHTTELEMETRYSTATS_STATUS_CONNECTED = 3
} FlightTelemetryStatsStatusOptions;

typedef struct
{
    float TxDataRate;
    uint32_t TxBytes;
    uint32_t TxFailures;
    uint32_t TxRetries;
    float RxDataRate;
    uint32_t RxBytes;
    uint32_t RxFailures;
    uint32_t RxSyncErrors;
    uint32_t RxCrcErrors;
    FlightTelemetryStatsStatusOptions Status;
}
uavtalk_FlightTelemetryStats;


inline void uavtalk_FlightTelemetryStats_parse(uint8_t* source, uavtalk_FlightTelemetryStats* msg)
{
    msg->TxDataRate = uavtalk_get_float(&source);
    msg->TxBytes = uavtalk_get_uint32(&source);
    msg->TxFailures = uavtalk_get_uint32(&source);
    msg->TxRetries = uavtalk_get_uint32(&source);
    msg->RxDataRate = uavtalk_get_float(&source);
    msg->RxBytes = uavtalk_get_uint32(&source);
    msg->RxFailures = uavtalk_get_uint32(&source);
    msg->RxSyncErrors = uavtalk_get_uint32(&source);
    msg->RxCrcErrors = uavtalk_get_uint32(&source);
    msg->Status = (FlightTelemetryStatsStatusOptions)uavtalk_get_uint8(&source);
}

inline void uavtalk_FlightTelemetryStats_emit(uint8_t* dest,
                                           uavtalk_FlightTelemetryStats* msg,
                                           uavtalk_msg_t type,
                                           size_t* final_size)
{
    uint8_t crc = 0;
    uint8_t *iter = dest;
    uavtalk_emit_header(&iter, type,
                        FLIGHTTELEMETRYSTATS_OBJID,
                        FLIGHTTELEMETRYSTATS_NUMBYTES,
                        &crc);
    uavtalk_emit_float(msg->TxDataRate, &iter, &crc);
    uavtalk_emit_uint32(msg->TxBytes, &iter, &crc);
    uavtalk_emit_uint32(msg->TxFailures, &iter, &crc);
    uavtalk_emit_uint32(msg->TxRetries, &iter, &crc);
    uavtalk_emit_float(msg->RxDataRate, &iter, &crc);
    uavtalk_emit_uint32(msg->RxBytes, &iter, &crc);
    uavtalk_emit_uint32(msg->RxFailures, &iter, &crc);
    uavtalk_emit_uint32(msg->RxSyncErrors, &iter, &crc);
    uavtalk_emit_uint32(msg->RxCrcErrors, &iter, &crc);
    uavtalk_emit_uint8(msg->Status, &iter, &crc);
    uavtalk_emit_checksum(crc, &iter);
    *final_size = (iter - dest);
}

#endif // _HEADER_Q4MD8IJQ4YOR_INCLUDED_
