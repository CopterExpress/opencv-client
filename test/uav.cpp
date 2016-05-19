#include "catch/catch.hpp"
#include "uavtalk.hpp"
#include "uavobjects/gcstelemetrystats.h"
#include <cstring>
using namespace copexp;

TEST_CASE("Sending some uavobjects", "[uav]")
{
    uint8_t data[128];
    memset(data, 0, sizeof(data));
    data[0] = 0x3c;
    data[1] = 0x20;
    // length
    data[2] = 12;
    data[3] = 0;
    // objid
    data[4] = 0x0a;
    data[5] = 0xdc;
    data[6] = 0xd1;
    data[7] = 0xca;
    // 2 bytes
    data[8] = 0;
    data[9] = 0;
    // payload
    data[10] = 0;
    data[11] = 0;
    // crc
    data[12] = 0;

    bool parsed = false;

    UAVObjectParser parser;
    parser.include(GCSTELEMETRYSTATS_OBJID, [&](uint8_t mtype) {
        parsed = true;
    });

    // parse all
    parser.parse(data, 73);
    REQUIRE( parsed == true );
    REQUIRE( parser.length() == 12 );

    // parse by small chunks
    parsed = false;
    parser.parse(data, 3);
    parser.parse(data + 3, 7);
    parser.parse(data + 3 + 7, 56);
    REQUIRE( parsed == true );
    REQUIRE( parser.length() == 12 );

    uint8_t msg_buffer[256];
    memset(msg_buffer, 0, sizeof(msg_buffer));

    uavtalk_GCSTelemetryStats telemetry;
    uavtalk_memset(telemetry);
    telemetry.Status = GCSTELEMETRYSTATS_STATUS_HANDSHAKEREQ;

    size_t msg_size;
    uavtalk_GCSTelemetryStats_emit(msg_buffer, &telemetry,
                                   UAVTALK_OBJ_ACK, &msg_size);

    REQUIRE( msg_buffer[0] == 0x3c );
    REQUIRE( msg_buffer[1] == UAVTALK_OBJ_ACK );
    REQUIRE( msg_size == 48 ); // 10 + 37 + 1

    memset(msg_buffer, 0, sizeof(msg_buffer));
    uavtalk_emit_request(msg_buffer, GCSTELEMETRYSTATS_OBJID,
                         UAVTALK_OBJ_REQ, &msg_size);
    REQUIRE( msg_size == 11 );
    REQUIRE( msg_buffer[1] == UAVTALK_OBJ_REQ );
}

TEST_CASE("Emiting and parsing some objects", "[uav]")
{
    UAVObjectParser parser;
}
