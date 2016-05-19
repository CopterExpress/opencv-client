#ifndef _HEADER_KEWDHGJ1AJEC_INCLUDED_
#define _HEADER_KEWDHGJ1AJEC_INCLUDED_

#include "uavtalk.h"
#include <stdint.h>

enum
{
    GCSTELEMETRYSTATS_OBJID = 0xCAD1DC0A,
    GCSTELEMETRYSTATS_ISSINGLEINST = 1,
    GCSTELEMETRYSTATS_ISSETTINGS = 0,
    GCSTELEMETRYSTATS_ISPRIORITY = 1,
    GCSTELEMETRYSTATS_NUMBYTES = 37
};

typedef enum {
    GCSTELEMETRYSTATS_STATUS_DISCONNECTED = 0,
    GCSTELEMETRYSTATS_STATUS_HANDSHAKEREQ = 1,
    GCSTELEMETRYSTATS_STATUS_HANDSHAKEACK = 2,
    GCSTELEMETRYSTATS_STATUS_CONNECTED = 3
} GCSTelemetryStatsStatusOptions;

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
    GCSTelemetryStatsStatusOptions Status;
}
uavtalk_GCSTelemetryStats;


inline void uavtalk_GCSTelemetryStats_parse(uint8_t* source, uavtalk_GCSTelemetryStats* msg)
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
    msg->Status = (GCSTelemetryStatsStatusOptions)uavtalk_get_uint8(&source);
}

inline void uavtalk_GCSTelemetryStats_emit(uint8_t* dest,
                                           uavtalk_GCSTelemetryStats* msg,
                                           uavtalk_msg_t type,
                                           size_t* final_size)
{
    uint8_t crc = 0;
    uint8_t *iter = dest;
    uavtalk_emit_header(&iter, type,
                        GCSTELEMETRYSTATS_OBJID,
                        GCSTELEMETRYSTATS_NUMBYTES,
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

#endif // _HEADER_KEWDHGJ1AJEC_INCLUDED_
