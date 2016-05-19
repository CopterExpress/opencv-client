#ifndef _HEADER_1BZERWCK647W7333MY6Y_INCLUDED_
#define _HEADER_1BZERWCK647W7333MY6Y_INCLUDED_

#include "settings.hpp"
#include <condition_variable>
#include <mutex>
#include <queue>

namespace copexp
{

class RC2Queue
{
public:
    RC2Queue(short timeout) : _timeout(timeout) {}
    struct RCCommand
    {
        uint16_t roll;
        uint16_t pitch;
        uint16_t yaw;
        uint16_t throttle;
        uint8_t armed;
    };

    bool wait(RCCommand &command)
    {
        std::unique_lock<std::mutex> lk(_mutex);
        _cv.wait_for(lk, _timeout);

        if (_queue.empty())
        {
            return false;
        }

        command = _queue.back();
        _queue.pop();
        return true;
    }

    void exec(const RCCommand& command)
    {
        std::lock_guard<std::mutex> lg(_mutex);
        _queue.push(command);
        _cv.notify_one();
    }

private:
    std::chrono::milliseconds _timeout;
    std::queue<RCCommand> _queue;
    std::mutex _mutex;
    std::condition_variable _cv;
};

int rc2Open(Settings& settings, RC2Queue& queue);

} // namespace copexp

#endif // _HEADER_1BZERWCK647W7333MY6Y_INCLUDED_
