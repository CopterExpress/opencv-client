// AUTO-GENERATED CODE: DO NOT EDIT!!!\n\n'

static const int MAXBUF = 256;

typedef unsigned char byte;

class MSP_Message {

    friend class MSP_Parser;

    protected:

        MSP_Message() { }
        byte bytes[MAXBUF];
        int pos;
        int len;

    public:

        byte start();
        bool hasNext();
        byte getNext();
        int length() { return len; }
        const byte* data() { return bytes; }

};

class MSP_Parser {

    private:

        int state;
        byte message_direction;
        byte message_id;
        byte message_length_expected;
        byte message_length_received;
        byte message_buffer[MAXBUF];
        byte message_checksum;

    public:

        MSP_Parser();

        void parse(byte b);


        MSP_Message serialize_PX4FLOW(short framecount, short pixel_flow_x_sum, short pixel_flow_y_sum, short flow_comp_m_x, short flow_comp_m_y, short qual, short gyro_x_rate, short gyro_y_rate, short gyro_z_rate, byte gyro_range, byte sonar_timestamp, short ground_distance);

        MSP_Message serialize_PX4FLOW_Request();

        void set_PX4FLOW_Handler(class PX4FLOW_Handler * handler);

        MSP_Message serialize_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8, short c9, short c10, short c11, short c12, short c13, short c14, short c15, short c16, short c17, short c18);

        MSP_Message serialize_RC_Request();

        void set_RC_Handler(class RC_Handler * handler);

        MSP_Message serialize_SET_RAW_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8);

        MSP_Message serialize_MSP_STATUS(short cycleTime, short i2c_errors_count, short sensor, int flag, byte currentSet);

        MSP_Message serialize_MSP_STATUS_Request();

        void set_MSP_STATUS_Handler(class MSP_STATUS_Handler * handler);

        MSP_Message serialize_MOTOR(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8);

        MSP_Message serialize_MOTOR_Request();

        void set_MOTOR_Handler(class MOTOR_Handler * handler);

        MSP_Message serialize_SET_MOTOR(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8);

        MSP_Message serialize_ATTITUDE(short angx, short angy, short heading);

        MSP_Message serialize_ATTITUDE_Request();

        void set_ATTITUDE_Handler(class ATTITUDE_Handler * handler);

    private:

        class PX4FLOW_Handler * handlerForPX4FLOW;

        class RC_Handler * handlerForRC;

        class MSP_STATUS_Handler * handlerForMSP_STATUS;

        class MOTOR_Handler * handlerForMOTOR;

        class ATTITUDE_Handler * handlerForATTITUDE;

};


class PX4FLOW_Handler {

    public:

        PX4FLOW_Handler() {}

        virtual void handle_PX4FLOW(short framecount, short pixel_flow_x_sum, short pixel_flow_y_sum, short flow_comp_m_x, short flow_comp_m_y, short qual, short gyro_x_rate, short gyro_y_rate, short gyro_z_rate, byte gyro_range, byte sonar_timestamp, short ground_distance){ }

};



class RC_Handler {

    public:

        RC_Handler() {}

        virtual void handle_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8, short c9, short c10, short c11, short c12, short c13, short c14, short c15, short c16, short c17, short c18){ }

};



class MSP_STATUS_Handler {

    public:

        MSP_STATUS_Handler() {}

        virtual void handle_MSP_STATUS(short cycleTime, short i2c_errors_count, short sensor, int flag, byte currentSet){ }

};



class MOTOR_Handler {

    public:

        MOTOR_Handler() {}

        virtual void handle_MOTOR(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8){ }

};



class ATTITUDE_Handler {

    public:

        ATTITUDE_Handler() {}

        virtual void handle_ATTITUDE(short angx, short angy, short heading){ }

};

