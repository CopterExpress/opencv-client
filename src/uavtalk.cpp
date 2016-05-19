#include "spdlog/spdlog.h"
#include "uavtalk.hpp"
#include <endian.h>
using namespace copexp;


void UAVObjectParser::parse(const uint8_t *buffer, size_t bytes_read)
{
    auto log = spdlog::get("parser");
    _read_position = 0;

    while (_read_position < bytes_read)
    {
        parseField(buffer, bytes_read);
    }
}

void UAVObjectParser::parseField(const uint8_t *buffer, size_t bytes_read)
{
    auto log = spdlog::get("parser");
    if (_msg_position == 0) // sync byte
    {
        for (;_read_position < bytes_read; ++_read_position)
        {
            if (buffer[_read_position] == 0x3C)
            {
                _msg_position = 1;
                ++_read_position;
                break;
            }
        }
    }

    if (_msg_position < 4)
    {
        do
        {
            if (_read_position >= bytes_read) return;
            _msg_buffer[_msg_position] = buffer[_read_position];
            ++_msg_position; ++_read_position;

        } while (_msg_position < 4);

        _msg_type = _msg_buffer[1];
#if BYTE_ORDER == LITTLE_ENDIAN
        _msg_length = *reinterpret_cast<uint16_t*>(_msg_buffer + 2);
#else
#error Big endian CPUs not supported at the time
#endif
        if (_msg_length > UAVOBJECT_MAXLEN)
        {
            log->warn("Broken package length");
            _msg_position = 0;
            return;
        }
    }

    // TODO replace with memcopy
    for (;_msg_position < _msg_length + 1; ++_msg_position, ++_read_position)
    {
        if (_read_position >= bytes_read) return;
        _msg_buffer[_msg_position] = buffer[_read_position];
    }



#if BYTE_ORDER == LITTLE_ENDIAN
    _msg_id = *reinterpret_cast<uint32_t*>(_msg_buffer + 4);
#else
#error Big endian CPUs not supported at the time
#endif

#if 0
    log->info("Package processed: {0} bytes, id {1:x}", _msg_length, _msg_id);
#endif

    execHandlers();
    _msg_position = 0;
}

void UAVObjectParser::execHandlers()
{
    auto m = _uavobjects.find(_msg_id);
    if (m == _uavobjects.end()) return;

    if (m->second.call)
    {
        m->second.call(_msg_buffer[1]);
    }
}
