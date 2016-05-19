#ifndef _HEADER_YFF6QH4DWKL5_INCLUDED_
#define _HEADER_YFF6QH4DWKL5_INCLUDED_

#include <map>
#include <functional>
#include <cstdint>

namespace copexp {

class UAVObjectParser
{
public:
    typedef int EventFlags;

    template <typename Obj, typename CallObj>
    void include(CallObj call)
    {
        auto id = Obj::OBJID;
        includeEvent(_uavobjects[id], std::forward<CallObj>(call));
    }

    template <typename CallObj>
    void include(uint32_t id, CallObj call)
    {
        includeEvent(_uavobjects[id], std::forward<CallObj>(call));
    }

    void parse(const uint8_t *buffer, size_t bytes_read);

    uint8_t* content() { return _msg_buffer + _msg_head_length; }
    size_t length() { return _msg_length; }

private:
    void parseField(const uint8_t *buffer, size_t buffer_read);
    void execHandlers();

    typedef std::function<void(uint8_t)> MessageCallback;
    struct MessageHandler
    {
        MessageCallback call = nullptr;
        bool timestamped = false;
        bool single_instance = true;
        size_t package_length;
    };

    inline void includeEvent(MessageHandler &handler, MessageCallback call);

    std::map<std::uint32_t, MessageHandler> _uavobjects;

    enum
    {
        UAVOBJECT_MAXLEN = 256,
        UAVOBJECT_SYNC_VAL = 0x3C
    };

    uint8_t _msg_buffer[UAVOBJECT_MAXLEN];
    size_t  _msg_position = 0;
    size_t  _read_position = 0;

    uint8_t  _msg_type;
    uint32_t _msg_id = 0;
    uint16_t _msg_length = 0;
    uint16_t _msg_head_length = 10;
};

void UAVObjectParser::includeEvent(MessageHandler &handler,
        UAVObjectParser::MessageCallback call)
{
    handler.call = call;
}

} // namespace copexp

#endif // _HEADER_YFF6QH4DWKL5_INCLUDED_
