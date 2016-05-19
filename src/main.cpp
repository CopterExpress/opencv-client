#include "camera.hpp"
#include "uavtalk.hpp"
#include "settings.hpp"
#include "rc.hpp"
#include "scope_guard.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/null_sink.h"
#include "serial/serial.h"

#include <iostream>
#include <memory>
#include <thread>
#include <signal.h>
#include <stdlib.h>

#include "uavobjects/accelstate.hpp"
#include "uavobjects/manualcontrolcommand.h"
#include "uavobjects/gcstelemetrystats.h"
#include "uavobjects/flighttelemetrystats.h"
#include "uavobjects/flightstatus.h"
using namespace std;
namespace uav = copexp::uavobjects;

namespace copexp
{
namespace signal
{
    volatile sig_atomic_t interrupt = 0;
    void set()
    {
        struct sigaction action;
        memset(&action, 0, sizeof(action));
        action.sa_handler = [](int)
        {
            interrupt = true;
        };
        sigaction(SIGINT, &action, nullptr);
        sigaction(SIGTERM, &action, nullptr);
    }

} // namespace signal

void kbdOpen(copexp::RC2Queue& queue);

} // namespace copexp

#ifdef _NO_MAIN
int _main(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
{
    // set signals
    copexp::signal::set();

    // parse settings
    copexp::Settings settings;
    settings.parse(argc, argv);

    spdlog::set_pattern("[%H:%M:%S] %v\r");

    spdlog::stdout_logger_mt("rc");
    spdlog::stdout_logger_mt("opencv");
    spdlog::stdout_logger_mt("kbd");

#if 0
    auto stdout_sink = make_shared<spdlog::sinks::stdout_sink_mt>();
    auto null_sink = make_shared<spdlog::sinks::null_sink_mt>();

    spdlog::register_logger()
    auto log_rc = make_shared<spdlog::logger>("rc", stdout_sink);
    auto log_opencv = make_shared<spdlog::logger>("opencv", stdout_sink);
    auto log = make_shared<spdlog::logger>("main", stdout_sink);
#endif

    copexp::RC2Queue queue(settings.timeout());

    thread cvthread(copexp::cameraOpen, ref(queue));
    thread rcthread(copexp::rc2Open, ref(settings), ref(queue));

    if (settings.keyboardTelemetry())
    {
        copexp::kbdOpen(queue);
    }
    else
    {
        while (!copexp::signal::interrupt)
        {
            this_thread::sleep_for(chrono::seconds(1));
        }
    }

    rcthread.join();
    cvthread.join();

    return EXIT_SUCCESS;
}

void copexp::kbdOpen(copexp::RC2Queue& queue)
{
    auto log = spdlog::get("kbd");
    log->info("Keyboard input service started");

    system("/bin/stty raw");
    td::scope_guard ttyg([&]()
    {
        system("/bin/stty cooked");
        log->info("Keyboard service stopped");
    });

    while (true)
    {
        char input;
        cin >> input;

        switch (input)
        {
        // roll pitch yaw throttle
        case '0':
            queue.exec({ 1000, 1000, 1000, 1000, 0 });
            break;

        case '1':
            queue.exec({ 1500, 1500, 1500, 1000, 1 });
            break;

        case '2':
            queue.exec({ 1500, 1500, 1500, 1400, 1 });
            break;

        case '3':
            queue.exec({ 1500, 1500, 1500, 1500, 1 });
            break;

        case '4':
            queue.exec({ 1500, 1500, 1500, 1600, 1 });
            break;

        case '5':
            queue.exec({ 1500, 1500, 1500, 1650, 1 });
            break;

        case '6':
            queue.exec({ 1500, 1500, 1500, 1700, 1 });
            break;

        case '7':
            queue.exec({ 1500, 1500, 1500, 1800, 1 });
            break;

        case '8':
            queue.exec({ 1500, 1500, 1500, 1900, 1 });
            break;

        case '9':
            queue.exec({ 1500, 1500, 1500, 2000, 1 });
            break;

        case 'r':
            queue.exec({ 1600, 1500, 1500, 1600, 1 });
            break;

        case 'p':
            queue.exec({ 1500, 1600, 1500, 1600, 1 });
            break;

        case 'q':
            return;
            break;

        default:
            break;
        }

        if (copexp::signal::interrupt)
        {
            break;
        }
    }
}

#if 0

int copexp::rcOpen(copexp::Settings& settings)
{
    auto log = spdlog::get("main");
    log->info("RC service started");

    // open serial
    unique_ptr<serial::Serial> port;

    try {
        port.reset(new serial::Serial(settings.tty(), settings.baud(),
                                      serial::Timeout::simpleTimeout(200)));
    }
    catch (...)
    {
        log->error("Unable to open port {}, trying alternative {}", settings.tty(),
                   settings.altTty());

        try {
            port.reset(new serial::Serial(settings.altTty(), settings.baud(),
                                          serial::Timeout::simpleTimeout(200)));
        }
        catch (...)
        {
            log->error("Unable to open alternative port {}, aborting.", settings.altTty());
            return EXIT_FAILURE;
        }
    }

    uint8_t write_buffer[256];
    uint8_t read_buffer[512];
    size_t bytes_read, bytes_write;

    if (!port->isOpen())
    {
        log->error("Unable to open serial port {} on baudrate {}",
                   settings.tty(), settings.baud());
        return EXIT_FAILURE;
    }
    log->info("Serial device {} ready, baud rate {}",
              settings.tty(),
              settings.baud());

    copexp::UAVObjectParser parser;
    parser.include<uav::AccelState>([&](uint8_t mtype)
    {
        uav::AccelState accel(parser.content());
#if 1
        log->info("AccelState object: {} {} {}", accel.X(), accel.Y(), accel.Z());
#endif
    });

    parser.include(uint32_t(GCSTELEMETRYSTATS_OBJID), [&](uint8_t mtype)
    {
        uavtalk_GCSTelemetryStats telemetry;
        uavtalk_GCSTelemetryStats_parse(parser.content(), &telemetry);
        log->info("GCSTelemetry object: {}", telemetry.Status);

        if (telemetry.Status == GCSTELEMETRYSTATS_STATUS_HANDSHAKEREQ)
        {
            size_t msg_size;
            uavtalk_emit_request(write_buffer, GCSTELEMETRYSTATS_OBJID,
                                 UAVTALK_ACK, &msg_size);
            bytes_write = port->write(write_buffer, msg_size);
            log->info("Wrote ACK, {} bytes", bytes_write);

            uavtalk_emit_request(write_buffer, GCSTELEMETRYSTATS_OBJID,
                                 UAVTALK_OBJ_REQ, &msg_size);
            bytes_write = port->write(write_buffer, msg_size);
            log->info("Wrote second REQ, {} bytes", bytes_write);
        }
    });

    char a = 0x03 ^ 0x01 ^ 0x00 ^ 0x01 ^ 0x0e;

//    > 	0x03     // size
//    > 	0x01     // command
//    > 	0x00
//    > 	0x01
//    > 	0x0e

    bool send_empty = false;
    parser.include(uint32_t(FLIGHTTELEMETRYSTATS_OBJID),
                   [&](uint8_t mtype)
    {
        uavtalk_FlightTelemetryStats status;
        uavtalk_FlightTelemetryStats_parse(parser.content(), &status);
        log->info("FlightTelemetry status object: {}", status.Status);

        if (status.Status == FLIGHTTELEMETRYSTATS_STATUS_HANDSHAKEACK)
        {
            log->info("Senging second telemetry handshake request");
            uavtalk_GCSTelemetryStats gcs;
            uavtalk_memset(gcs);
            gcs.Status = GCSTELEMETRYSTATS_STATUS_CONNECTED;
            uavtalk_GCSTelemetryStats_emit(write_buffer, &gcs,
                                           UAVTALK_OBJ,
                                           &bytes_write);
            bytes_write = port->write(write_buffer, bytes_write);
            log->info("Second handshake request written, {} bytes", bytes_write);
        }
        else if (status.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED)
        {
            log->info("Senging manual control command");
            uavtalk_ManualControlCommand command;
            uavtalk_memset(command);

            command.Roll = 0;
            command.Pitch = 0;
            command.Yaw = 0;
            command.Collective = 0;
            command.FlightModeSwitchPosition = 0;

            if (!send_empty)
            {
               command.Throttle = 0.0;
               command.Thrust = 0.0;
               send_empty = true;
            }
            else
            {
                command.Throttle = 0.07;
                command.Thrust = 0.07;
            }

            command.Connected = MANUALCONTROLCOMMAND_CONNECTED_TRUE;
            command.Channel[0] = 65535;
            command.Channel[1] = 65535;
            command.Channel[2] = 65535;
            command.Channel[3] = 65535;
            command.Channel[4] = 65535;
            command.Channel[5] = 65535;
            command.Channel[6] = 65535;
            command.Channel[7] = 65535;
            command.Channel[8] = 65535;

            uavtalk_ManualControlCommand_emit(write_buffer, &command,
                                              UAVTALK_OBJ_ACK, &bytes_write);
            bytes_write = port->write(write_buffer, bytes_write);
            log->info("Manual control command, size {} bytes", bytes_write);

            // receive manualcontrol command
            uavtalk_emit_request(write_buffer,
                                 MANUALCONTROLCOMMAND_OBJID, UAVTALK_OBJ_REQ,
                                 &bytes_write);
            bytes_write = port->write(write_buffer, bytes_write);

            // receive flight status
            uavtalk_emit_request(write_buffer,
                                 FLIGHTSTATUS_OBJID, UAVTALK_OBJ_REQ,
                                 &bytes_write);
            bytes_write = port->write(write_buffer, bytes_write);
        }
    });

    parser.include(uint32_t(MANUALCONTROLCOMMAND_OBJID),
                   [&](uint8_t mtype)
    {
        if (mtype == UAVTALK_NACK)
        {
            log->warn("ManualControl object 0x{0:x} not found (NACK)", MANUALCONTROLCOMMAND_OBJID);
            return;
        }
        else if (mtype == UAVTALK_ACK)
        {
            log->info("ManualControl command acknoledge (ACK)");
            return;
        }

        uavtalk_ManualControlCommand command;
        uavtalk_ManualControlCommand_parse(parser.content(), &command);
        log->info("Manual control thrust: {}", command.Thrust);
    });

    parser.include(uint32_t(FLIGHTSTATUS_OBJID), [&](uint8_t mtype)
    {
        if (mtype == UAVTALK_NACK)
        {
            log->warn("Flightstatus object not found (NACK)");
            return;
        }

        uavtalk_FlightStatus status;
        uavtalk_FlightStatus_parse(parser.content(), &status);
        log->info("Flightstatus info: ARMED {}", status.Armed);
    });

    log->info("Senging telemetry handshake request");
    uavtalk_GCSTelemetryStats gcs;
    uavtalk_memset(gcs);
    gcs.Status = GCSTELEMETRYSTATS_STATUS_HANDSHAKEREQ;

    uavtalk_GCSTelemetryStats_emit(write_buffer, &gcs,
                                   UAVTALK_OBJ,
                                   &bytes_write);
    bytes_write = port->write(write_buffer, bytes_write);
    log->info("Handshake request written, {} bytes", bytes_write);


    log->info("Senging flight status request");
    uavtalk_emit_request(write_buffer, FLIGHTTELEMETRYSTATS_NUMBYTES,
                         UAVTALK_OBJ_REQ, &bytes_write);
    bytes_write = port->write(write_buffer, bytes_write);
    log->info("Handshake request written, {} bytes", bytes_write);

    while (true)
    {
        port->waitReadable();
        bytes_read = port->read(read_buffer, min(port->available(),
                                                 sizeof(read_buffer)));
#if 0
        log->info("Read {} bytes", bytes_read);
#endif

        parser.parse(read_buffer, bytes_read);

        if (copexp::signal::interrupt)
        {
            break;
        }
    }

    log->info("RC service will exit now");
    return EXIT_SUCCESS;
}
#endif
