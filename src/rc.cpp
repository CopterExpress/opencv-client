#include "rc.hpp"
#include "serial/serial.h"
#include "spdlog/spdlog.h"
#include "msppg/msppg.h"
#include <memory>
#include <signal.h>
using namespace std;

namespace copexp {
namespace signal {
extern volatile sig_atomic_t interrupt;
}
}

class RC_ATTITUDE_Handler : public ATTITUDE_Handler
{
public:
    void handle_ATTITUDE(short angx, short angy, short heading) override
    {
        auto log = spdlog::get("rc");
        log->info("ATTITUDE: {}, {}, {}", angx, angy, heading);
    }
};

class RC_RC_Handler : public RC_Handler
{
public:
    void handle_RC(short c1, short c2, short c3, short c4,
                   short c5, short c6, short c7, short c8,
                   short c9, short c10, short c11, short c12,
                   short c13, short c14, short c15, short c16,
                   short c17, short c18) override
    {
#if 1
        auto log = spdlog::get("rc");
        log->info("RC: [{}, {}, {}, {}], {}", c1, c2, c3, c4, c5);
#endif
    }
};

class RC_MOTOR_Handler : public MOTOR_Handler
{
public:
    void handle_MOTOR(short c1, short c2, short c3, short c4,
                      short c5, short c6, short c7, short c8) override
    {
        auto log = spdlog::get("rc");
        log->info("MOTOR: {}, {}, {}, {}, {}, {}, {}, {}",
                  c1, c2, c3, c4, c5, c6, c7, c8);
    }
};

int copexp::rc2Open(Settings &settings, RC2Queue &queue)
{
    auto log = spdlog::get("rc");

    MSP_Parser parser;
    MSP_Message message = parser.serialize_ATTITUDE(59, 76, 1);

    unique_ptr<serial::Serial> port;
    try {
        port.reset(new serial::Serial(settings.tty(), settings.baud(),
                                      serial::Timeout::simpleTimeout
                                      (settings.timeout())));
    }
    catch (...)
    {
        log->error("Unable to open port {}, trying alternative {}",
                   settings.tty(),
                   settings.altTty());

        try {
            port.reset(new serial::Serial(settings.altTty(), settings.baud(),
                                          serial::Timeout::simpleTimeout
                                          (settings.timeout())));
        }
        catch (...)
        {
            log->error("Unable to open alternative port {}, aborting.",
                       settings.altTty());
            return EXIT_FAILURE;
        }
    }

    log->info("RC2 serial device {} ready, baud rate {}",
              settings.tty(),
              settings.baud());

    uint8_t read_buffer[256];
    size_t bytes_read, bytes_write;

    RC_ATTITUDE_Handler attitude_handler;
    parser.set_ATTITUDE_Handler(&attitude_handler);

    RC_RC_Handler rc_handler;
    parser.set_RC_Handler(&rc_handler);

    RC_MOTOR_Handler motor_handler;
    parser.set_MOTOR_Handler(&motor_handler);

    auto time = chrono::system_clock::now();

    copexp::RC2Queue::RCCommand command = { 1000, 1000, 1000, 1000, 0 };
    while (true)
    {
        if (queue.wait(command))
        {
            log->info("Send manual command {}:{}:{}:{}",
                      command.roll, command.pitch,
                      command.yaw, command.throttle);
        }

        // send last command
        auto rc_msg = parser.serialize_SET_RAW_RC(command.roll, command.pitch,
                                                  command.throttle, command.yaw,
                                                  command.armed ? 1800 : 1000,
                                                  1000, 1000, 1000);
        port->write(rc_msg.data(), rc_msg.length());

        auto rq_msg = parser.serialize_RC_Request();
        port->write(rq_msg.data(), rq_msg.length());

        bytes_read = port->read(read_buffer, min(port->available(),
                                                 sizeof(read_buffer)));
        for (auto it = read_buffer, end = read_buffer + bytes_read;
             it != end; ++it)
        {
            parser.parse(*it);
        }

        if (copexp::signal::interrupt)
        {
            break;
        }
    }
    log->info("RC2 service shutting down.");
}

#if 0
#if 0
        {
            auto message_rc_lib = parser.serialize_SET_RAW_RC(1000, 1000, 1000, 1000,
                                                              1000, 1000, 1000, 1000);
            for (byte b = message_rc_lib.start();
                 message_rc_lib.hasNext();
                 b = message_rc_lib.getNext())
            {
                port->write(&b, 1);
            }
        }

        this_thread::sleep_for(chrono::seconds(2));

        {
            auto message_rc_lib = parser.serialize_SET_RAW_RC(1500, 1500, 1300, 1267,
                                                              1660, 1000, 1000, 1000);
            for (byte b = message_rc_lib.start();
                 message_rc_lib.hasNext();
                 b = message_rc_lib.getNext())
            {
                port->write(&b, 1);
            }
        }

        this_thread::sleep_for(chrono::seconds(2));
#endif

    // read data from serial and parse
    while (true)
    {
        port->waitReadable();
        bytes_read = port->read(read_buffer, min(port->available(),
                                                 sizeof(read_buffer)));
#if 1
        log->info("Read {} bytes from {}", bytes_read, settings.tty());
#endif

        for (auto it = read_buffer, end = read_buffer + bytes_read;
             it != end; ++it)
        {
            parser.parse(*it);
        }

#if 0
        auto message = parser.serialize_ATTITUDE_Request();
        for (byte b = message.start(); message.hasNext(); b = message.getNext())
        {
            port->write(&b, 1);
        }
#endif

#if 1
        auto message2 = parser.serialize_RC_Request();
        for (byte b = message2.start(); message2.hasNext(); b = message2.getNext())
        {
            port->write(&b, 1);
        }
#endif

#if 1
        auto message_motor = parser.serialize_MOTOR_Request();
        for (byte b = message_motor.start();
             message_motor.hasNext(); b = message_motor.getNext())
        {
            port->write(&b, 1);
        }
#endif

        byte message[] =
                { 0x24
                , 0x4d
                , 0x3c
                , 0x10
                , 0xc8
                , 0xdc
                , 0x05
                , 0xdc
                , 0x05
                , 0x14
                , 0x05
                , 0xf3
                , 0x04
                , 0x7c
                , 0x06
                , 0xe8
                , 0x03
                , 0xe8
                , 0x03
                , 0xe8
                , 0x03
                , 0xaf };

#if 0
        auto message_rc_lib = parser.serialize_SET_RAW_RC(1500, 1500, 1300, 1267, 1660, 1000, 1000, 1000);
        for (byte b = message_rc_lib.start();
             message_rc_lib.hasNext();
             b = message_rc_lib.getNext())
        {
            port->write(&b, 1);
        }
#endif

//        port->write(message, sizeof(message));
        log->info("Written RC_RAW command, size {}", sizeof(message));

//        auto since_time = chrono::system_clock::now() - time;
//        short rcval = 1000 + ((since_time.count() % 10000) / 1000) * 100;
//        auto message3 = parser.serialize_SET_RAW_RC(1100, 1100, 1100, 1100,
//                                                    1700, 1500, 1500, 1500);
//        for (byte b = message3.start(); message3.hasNext(); b = message3.getNext())
//        {
//            port->write(&b, 1);
//        }
//        log->info("Written RC_RAW command, value {}, sec {}", rcval, since_time.count());

#if 0
        auto message_motor = parser.serialize_SET_MOTOR(1600, 0, 0, 0, 0, 0, 0, 0);
        for (byte b = message_motor.start(); message_motor.hasNext(); b = message_motor.getNext())
        {
            port->write(&b, 1);
        }
#endif

        if (copexp::signal::interrupt)
        {
            break;
        }
    }
#endif
