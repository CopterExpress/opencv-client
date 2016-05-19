// AUTO-GENERATED CODE: DO NOT EDIT!!!


#include "msppg.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static byte CRC8(byte * data, int n) {

    byte crc = 0x00;

    for (int k=0; k<n; ++k) {

        crc ^= data[k];
    }

    return crc;
}

byte MSP_Message::start() {

    this->pos = 0;
    return this->getNext();
}

bool MSP_Message::hasNext() {

    return this->pos <= this->len;
}


byte MSP_Message::getNext() {

    return this->bytes[this->pos++];
}

MSP_Parser::MSP_Parser() {

    this->state = 0;
}

void MSP_Parser::parse(byte b) {

    switch (this->state) {

        case 0:               // sync char 1
            if (b == 36) { // $
                this->state++;
            }
            break;        

        case 1:               // sync char 2
            if (b == 77) { // M
                this->state++;
            }
            else {            // restart and try again
                this->state = 0;
            }
            break;

        case 2:               // direction (should be >)
            if (b == 62) { // >
                this->message_direction = 1;
            }
            else {            // <
                this->message_direction = 0;
            }
            this->state++;
            break;

        case 3:
            this->message_length_expected = b;
            this->message_checksum = b;
            // setup arraybuffer
            this->message_length_received = 0;
            this->state++;
            break;

        case 4:
            this->message_id = b;
            this->message_checksum ^= b;
            if (this->message_length_expected > 0) {
                // process payload
                this->state++;
            }
            else {
                // no payload
                this->state += 2;
            }
            break;

        case 5: // payload
            this->message_buffer[this->message_length_received] = b;
            this->message_checksum ^= b;
            this->message_length_received++;
            if (this->message_length_received >= this->message_length_expected) {
                this->state++;
            }
            break;

        case 6:
            this->state = 0;
            if (this->message_checksum == b) {
                // message received, process
                switch (this->message_id) {
                
                    case 125: {

                        short framecount;
                        memcpy(&framecount,  &this->message_buffer[0], sizeof(short));

                        short pixel_flow_x_sum;
                        memcpy(&pixel_flow_x_sum,  &this->message_buffer[2], sizeof(short));

                        short pixel_flow_y_sum;
                        memcpy(&pixel_flow_y_sum,  &this->message_buffer[4], sizeof(short));

                        short flow_comp_m_x;
                        memcpy(&flow_comp_m_x,  &this->message_buffer[6], sizeof(short));

                        short flow_comp_m_y;
                        memcpy(&flow_comp_m_y,  &this->message_buffer[8], sizeof(short));

                        short qual;
                        memcpy(&qual,  &this->message_buffer[10], sizeof(short));

                        short gyro_x_rate;
                        memcpy(&gyro_x_rate,  &this->message_buffer[12], sizeof(short));

                        short gyro_y_rate;
                        memcpy(&gyro_y_rate,  &this->message_buffer[14], sizeof(short));

                        short gyro_z_rate;
                        memcpy(&gyro_z_rate,  &this->message_buffer[16], sizeof(short));

                        byte gyro_range;
                        memcpy(&gyro_range,  &this->message_buffer[18], sizeof(byte));

                        byte sonar_timestamp;
                        memcpy(&sonar_timestamp,  &this->message_buffer[19], sizeof(byte));

                        short ground_distance;
                        memcpy(&ground_distance,  &this->message_buffer[20], sizeof(short));

                        this->handlerForPX4FLOW->handle_PX4FLOW(framecount, pixel_flow_x_sum, pixel_flow_y_sum, flow_comp_m_x, flow_comp_m_y, qual, gyro_x_rate, gyro_y_rate, gyro_z_rate, gyro_range, sonar_timestamp, ground_distance);
                        } break;

                    case 105: {

                        short c1;
                        memcpy(&c1,  &this->message_buffer[0], sizeof(short));

                        short c2;
                        memcpy(&c2,  &this->message_buffer[2], sizeof(short));

                        short c3;
                        memcpy(&c3,  &this->message_buffer[4], sizeof(short));

                        short c4;
                        memcpy(&c4,  &this->message_buffer[6], sizeof(short));

                        short c5;
                        memcpy(&c5,  &this->message_buffer[8], sizeof(short));

                        short c6;
                        memcpy(&c6,  &this->message_buffer[10], sizeof(short));

                        short c7;
                        memcpy(&c7,  &this->message_buffer[12], sizeof(short));

                        short c8;
                        memcpy(&c8,  &this->message_buffer[14], sizeof(short));

                        short c9;
                        memcpy(&c9,  &this->message_buffer[16], sizeof(short));

                        short c10;
                        memcpy(&c10,  &this->message_buffer[18], sizeof(short));

                        short c11;
                        memcpy(&c11,  &this->message_buffer[20], sizeof(short));

                        short c12;
                        memcpy(&c12,  &this->message_buffer[22], sizeof(short));

                        short c13;
                        memcpy(&c13,  &this->message_buffer[24], sizeof(short));

                        short c14;
                        memcpy(&c14,  &this->message_buffer[26], sizeof(short));

                        short c15;
                        memcpy(&c15,  &this->message_buffer[28], sizeof(short));

                        short c16;
                        memcpy(&c16,  &this->message_buffer[30], sizeof(short));

                        short c17;
                        memcpy(&c17,  &this->message_buffer[32], sizeof(short));

                        short c18;
                        memcpy(&c18,  &this->message_buffer[34], sizeof(short));

                        this->handlerForRC->handle_RC(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18);
                        } break;

                    case 101: {

                        short cycleTime;
                        memcpy(&cycleTime,  &this->message_buffer[0], sizeof(short));

                        short i2c_errors_count;
                        memcpy(&i2c_errors_count,  &this->message_buffer[2], sizeof(short));

                        short sensor;
                        memcpy(&sensor,  &this->message_buffer[4], sizeof(short));

                        int flag;
                        memcpy(&flag,  &this->message_buffer[6], sizeof(int));

                        byte currentSet;
                        memcpy(&currentSet,  &this->message_buffer[10], sizeof(byte));

                        this->handlerForMSP_STATUS->handle_MSP_STATUS(cycleTime, i2c_errors_count, sensor, flag, currentSet);
                        } break;

                    case 104: {

                        short c1;
                        memcpy(&c1,  &this->message_buffer[0], sizeof(short));

                        short c2;
                        memcpy(&c2,  &this->message_buffer[2], sizeof(short));

                        short c3;
                        memcpy(&c3,  &this->message_buffer[4], sizeof(short));

                        short c4;
                        memcpy(&c4,  &this->message_buffer[6], sizeof(short));

                        short c5;
                        memcpy(&c5,  &this->message_buffer[8], sizeof(short));

                        short c6;
                        memcpy(&c6,  &this->message_buffer[10], sizeof(short));

                        short c7;
                        memcpy(&c7,  &this->message_buffer[12], sizeof(short));

                        short c8;
                        memcpy(&c8,  &this->message_buffer[14], sizeof(short));

                        this->handlerForMOTOR->handle_MOTOR(c1, c2, c3, c4, c5, c6, c7, c8);
                        } break;

                    case 108: {

                        short angx;
                        memcpy(&angx,  &this->message_buffer[0], sizeof(short));

                        short angy;
                        memcpy(&angy,  &this->message_buffer[2], sizeof(short));

                        short heading;
                        memcpy(&heading,  &this->message_buffer[4], sizeof(short));

                        this->handlerForATTITUDE->handle_ATTITUDE(angx, angy, heading);
                        } break;

                }
            }

            break;

        default:
            break;
    }
}

void MSP_Parser::set_PX4FLOW_Handler(class PX4FLOW_Handler * handler) {

    this->handlerForPX4FLOW = handler;
}

MSP_Message MSP_Parser::serialize_PX4FLOW_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 125;
    msg.bytes[5] = 125;

    msg.len = 6;

    return msg;
}MSP_Message MSP_Parser::serialize_PX4FLOW(short framecount, short pixel_flow_x_sum, short pixel_flow_y_sum, short flow_comp_m_x, short flow_comp_m_y, short qual, short gyro_x_rate, short gyro_y_rate, short gyro_z_rate, byte gyro_range, byte sonar_timestamp, short ground_distance) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 22;
    msg.bytes[4] = 125;

    memcpy(&msg.bytes[5], &framecount, sizeof(short));
    memcpy(&msg.bytes[7], &pixel_flow_x_sum, sizeof(short));
    memcpy(&msg.bytes[9], &pixel_flow_y_sum, sizeof(short));
    memcpy(&msg.bytes[11], &flow_comp_m_x, sizeof(short));
    memcpy(&msg.bytes[13], &flow_comp_m_y, sizeof(short));
    memcpy(&msg.bytes[15], &qual, sizeof(short));
    memcpy(&msg.bytes[17], &gyro_x_rate, sizeof(short));
    memcpy(&msg.bytes[19], &gyro_y_rate, sizeof(short));
    memcpy(&msg.bytes[21], &gyro_z_rate, sizeof(short));
    memcpy(&msg.bytes[23], &gyro_range, sizeof(byte));
    memcpy(&msg.bytes[24], &sonar_timestamp, sizeof(byte));
    memcpy(&msg.bytes[25], &ground_distance, sizeof(short));

    msg.bytes[27] = CRC8(&msg.bytes[3], 24);

    msg.len = 28;

    return msg;
}

void MSP_Parser::set_RC_Handler(class RC_Handler * handler) {

    this->handlerForRC = handler;
}

MSP_Message MSP_Parser::serialize_RC_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 105;
    msg.bytes[5] = 105;

    msg.len = 6;

    return msg;
}MSP_Message MSP_Parser::serialize_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8, short c9, short c10, short c11, short c12, short c13, short c14, short c15, short c16, short c17, short c18) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 36;
    msg.bytes[4] = 105;

    memcpy(&msg.bytes[5], &c1, sizeof(short));
    memcpy(&msg.bytes[7], &c2, sizeof(short));
    memcpy(&msg.bytes[9], &c3, sizeof(short));
    memcpy(&msg.bytes[11], &c4, sizeof(short));
    memcpy(&msg.bytes[13], &c5, sizeof(short));
    memcpy(&msg.bytes[15], &c6, sizeof(short));
    memcpy(&msg.bytes[17], &c7, sizeof(short));
    memcpy(&msg.bytes[19], &c8, sizeof(short));
    memcpy(&msg.bytes[21], &c9, sizeof(short));
    memcpy(&msg.bytes[23], &c10, sizeof(short));
    memcpy(&msg.bytes[25], &c11, sizeof(short));
    memcpy(&msg.bytes[27], &c12, sizeof(short));
    memcpy(&msg.bytes[29], &c13, sizeof(short));
    memcpy(&msg.bytes[31], &c14, sizeof(short));
    memcpy(&msg.bytes[33], &c15, sizeof(short));
    memcpy(&msg.bytes[35], &c16, sizeof(short));
    memcpy(&msg.bytes[37], &c17, sizeof(short));
    memcpy(&msg.bytes[39], &c18, sizeof(short));

    msg.bytes[41] = CRC8(&msg.bytes[3], 38);

    msg.len = 42;

    return msg;
}

MSP_Message MSP_Parser::serialize_SET_RAW_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 16;
    msg.bytes[4] = 200;

    memcpy(&msg.bytes[5], &c1, sizeof(short));
    memcpy(&msg.bytes[7], &c2, sizeof(short));
    memcpy(&msg.bytes[9], &c3, sizeof(short));
    memcpy(&msg.bytes[11], &c4, sizeof(short));
    memcpy(&msg.bytes[13], &c5, sizeof(short));
    memcpy(&msg.bytes[15], &c6, sizeof(short));
    memcpy(&msg.bytes[17], &c7, sizeof(short));
    memcpy(&msg.bytes[19], &c8, sizeof(short));

    msg.bytes[21] = CRC8(&msg.bytes[3], 18);

    msg.len = 22;

    return msg;
}

void MSP_Parser::set_MSP_STATUS_Handler(class MSP_STATUS_Handler * handler) {

    this->handlerForMSP_STATUS = handler;
}

MSP_Message MSP_Parser::serialize_MSP_STATUS_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 101;
    msg.bytes[5] = 101;

    msg.len = 6;

    return msg;
}MSP_Message MSP_Parser::serialize_MSP_STATUS(short cycleTime, short i2c_errors_count, short sensor, int flag, byte currentSet) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 11;
    msg.bytes[4] = 101;

    memcpy(&msg.bytes[5], &cycleTime, sizeof(short));
    memcpy(&msg.bytes[7], &i2c_errors_count, sizeof(short));
    memcpy(&msg.bytes[9], &sensor, sizeof(short));
    memcpy(&msg.bytes[11], &flag, sizeof(int));
    memcpy(&msg.bytes[15], &currentSet, sizeof(byte));

    msg.bytes[16] = CRC8(&msg.bytes[3], 13);

    msg.len = 17;

    return msg;
}

void MSP_Parser::set_MOTOR_Handler(class MOTOR_Handler * handler) {

    this->handlerForMOTOR = handler;
}

MSP_Message MSP_Parser::serialize_MOTOR_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 104;
    msg.bytes[5] = 104;

    msg.len = 6;

    return msg;
}MSP_Message MSP_Parser::serialize_MOTOR(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 16;
    msg.bytes[4] = 104;

    memcpy(&msg.bytes[5], &c1, sizeof(short));
    memcpy(&msg.bytes[7], &c2, sizeof(short));
    memcpy(&msg.bytes[9], &c3, sizeof(short));
    memcpy(&msg.bytes[11], &c4, sizeof(short));
    memcpy(&msg.bytes[13], &c5, sizeof(short));
    memcpy(&msg.bytes[15], &c6, sizeof(short));
    memcpy(&msg.bytes[17], &c7, sizeof(short));
    memcpy(&msg.bytes[19], &c8, sizeof(short));

    msg.bytes[21] = CRC8(&msg.bytes[3], 18);

    msg.len = 22;

    return msg;
}

MSP_Message MSP_Parser::serialize_SET_MOTOR(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 16;
    msg.bytes[4] = 214;

    memcpy(&msg.bytes[5], &c1, sizeof(short));
    memcpy(&msg.bytes[7], &c2, sizeof(short));
    memcpy(&msg.bytes[9], &c3, sizeof(short));
    memcpy(&msg.bytes[11], &c4, sizeof(short));
    memcpy(&msg.bytes[13], &c5, sizeof(short));
    memcpy(&msg.bytes[15], &c6, sizeof(short));
    memcpy(&msg.bytes[17], &c7, sizeof(short));
    memcpy(&msg.bytes[19], &c8, sizeof(short));

    msg.bytes[21] = CRC8(&msg.bytes[3], 18);

    msg.len = 22;

    return msg;
}

void MSP_Parser::set_ATTITUDE_Handler(class ATTITUDE_Handler * handler) {

    this->handlerForATTITUDE = handler;
}

MSP_Message MSP_Parser::serialize_ATTITUDE_Request() {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 60;
    msg.bytes[3] = 0;
    msg.bytes[4] = 108;
    msg.bytes[5] = 108;

    msg.len = 6;

    return msg;
}MSP_Message MSP_Parser::serialize_ATTITUDE(short angx, short angy, short heading) {

    MSP_Message msg;

    msg.bytes[0] = 36;
    msg.bytes[1] = 77;
    msg.bytes[2] = 62;
    msg.bytes[3] = 6;
    msg.bytes[4] = 108;

    memcpy(&msg.bytes[5], &angx, sizeof(short));
    memcpy(&msg.bytes[7], &angy, sizeof(short));
    memcpy(&msg.bytes[9], &heading, sizeof(short));

    msg.bytes[11] = CRC8(&msg.bytes[3], 8);

    msg.len = 12;

    return msg;
}

