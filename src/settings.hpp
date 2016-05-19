#ifndef _HEADER_GUEI8U1HS9LF_INCLUDED_
#define _HEADER_GUEI8U1HS9LF_INCLUDED_

#include <cstdint>
#include <string>

namespace copexp
{

class Settings
{
public:
    void parse(int argc, char *argv[]) {}

    std::int32_t        timeout() const { return _timeout; }
    std::int32_t        baud() const { return _baud; }
    const std::string & tty() const { return _tty; }
    const std::string & altTty() const { return _tty; }
    bool                keyboardTelemetry() const { return _keyboard_telemetry; }

private:
    std::int32_t _timeout = 200;
    std::int32_t _baud = 115200;
    std::string  _tty = "/dev/ttyUSB0";
    std::string _alt_tty = "/dev/ttyUSB1";
    bool _keyboard_telemetry = true;
};

} // namespace copexp

#endif // _HEADER_GUEI8U1HS9LF_INCLUDED_
