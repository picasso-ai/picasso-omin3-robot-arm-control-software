#include "test.h"

int txid;

// Emergency stop motor but keep it active
void stop(int id)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x01;
    tx_frame.FIR.B.DLC = 0;
    txid = tx_frame.MsgID;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*
* Set motor status 
* 0x00: disable motor
* 0x01: enable motor
* 0x02: restart motor
* 0x03: reset motor parameters
* 0x04: reset error status
*/
void set_status(int id, float status)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x03;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = status;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Read motor status
int read_status(int id)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x05;
    tx_frame.FIR.B.DLC = 0;
    txid = tx_frame.MsgID;
    ESP32Can.CANWriteFrame(&tx_frame);
    unsigned long currentMillis = millis();
    while (1)
    {
        xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS);
        if (txid == rx_frame.MsgID)
        {
            break;
        }
    }
    return rx_frame.data.u8[0];
}

/*
* Set motor mode 
* 0x00: torque mode
* 0x01: speed mode
* 0x02: position mode
*/
void set_mode(int id, float mode)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x07;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = mode;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Read motor mode
int read_mode(int id)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x09;
    tx_frame.FIR.B.DLC = 0;
    txid = tx_frame.MsgID;
    ESP32Can.CANWriteFrame(&tx_frame);
    unsigned long currentMillis = millis();
    while (1)
    {
        xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS);
        if (txid == rx_frame.MsgID)
        {
            break;
        }
    }
    return rx_frame.data.u8[0];
}

// Set current position of motor to zero
void set_zp(int id)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x0b;
    tx_frame.FIR.B.DLC = 0;
    txid = tx_frame.MsgID;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*
* Set PID parameters 
*
* Position loop parameters
* 0x00: set P parameter
* 0x01: set I parameter
* 0x02: set D parameter
* 0x03: set slope parameter
* 0x04: set filter period parameter
*
* Speed loop parameters
* 0x05: set P parameter
* 0x06: set I parameter
* 0x07: set D parameter
* 0x08: set slope parameter
* 0x09: set filter period parameter
* 
* Q-axis current loop parameters
* 0x0A: set P parameter
* 0x0B: set I parameter
* 0x0C: set D parameter
* 0x0D: set slope parameter
* 0x0E: set filter period parameter
* 
* D-axis current loop parameters
* 0x0F: set P parameter
* 0x10: set I parameter
* 0x11: set D parameter
* 0x12: set slope parameter
* 0x13: set filter period parameter
*/
void set_PID(int id, float pid_parameter, float value)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x0d;
    tx_frame.FIR.B.DLC = 5;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = pid_parameter;
    float *p2 = (float *)&tx_frame.data.u8[1];
    *p2 = value;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Read PID data; uses same schema for pid_parameter as set_PID
float read_PID(int id, float pid_parameter)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    unsigned int d;
    float rxdata;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x0f;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = pid_parameter;
    ESP32Can.CANWriteFrame(&tx_frame);
    unsigned long currentMillis = millis();
    while (1)
    {
        xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS);
        if (txid == rx_frame.MsgID)
        {
            d = rx_frame.data.u8[3];
            d <<= 8;
            d |= rx_frame.data.u8[2];
            d <<= 8;
            d |= rx_frame.data.u8[1];
            d <<= 8;
            d |= rx_frame.data.u8[0];
            rxdata = *(float *)&d;
            break;
        }
    }
    return rxdata;
}

/*
* Set motor limit parameters 
*
* 0x01: temperature
* 0x02: voltage
* 0x03: current
* 0x04: speed
* 0x05: position (minimum)
* 0x06: position (maximum)
* 0x07: brake start
* 0x08: brake maintenance
* 0x09: overvoltage
*/ 
void set_lim(int id, float limit_id, float value)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x11;
    tx_frame.FIR.B.DLC = 5;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = limit_id;
    float *p2 = (float *)&tx_frame.data.u8[1];
    *p2 = value;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Read limit values; uses same schema for limit_id as above
float read_lim(int id, float limit_id)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    unsigned int d;
    float rxdata;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x13;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = limit_id;
    ESP32Can.CANWriteFrame(&tx_frame);
    unsigned long currentMillis = millis();
    while (1)
    {
        xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS);
        if (txid == rx_frame.MsgID)
        {
            d = rx_frame.data.u8[3];
            d <<= 8;
            d |= rx_frame.data.u8[2];
            d <<= 8;
            d |= rx_frame.data.u8[1];
            d <<= 8;
            d |= rx_frame.data.u8[0];
            rxdata = *(float *)&d;
            break;
        }
    }
    return rxdata;
}

// Run to target value
void spr(int id, float value)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x15;
    tx_frame.FIR.B.DLC = 4;
    txid = tx_frame.MsgID;
    float *p1 = (float *)&tx_frame.data.u8[0];
    *p1 = value;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Run to specified position at given speed
void spt(int id, float position, float speed)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x17;
    tx_frame.FIR.B.DLC = 8;
    txid = tx_frame.MsgID;
    float *p1 = (float *)&tx_frame.data.u8[0];
    float *p2 = (float *)&tx_frame.data.u8[4];
    *p1 = position;
    *p2 = speed;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Set position trajectory data. Note that 00 00 returns to the zero point. 
void set_ctp(int id, float index, float value)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x19;
    tx_frame.FIR.B.DLC = 6;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = index;
    float *p2 = (float *)&tx_frame.data.u8[2];
    *p2 = value;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Set speed trajectory data. Note that 00 00 returns to the zero point. 
void set_ctv(int id, float index, float value)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x1b;
    tx_frame.FIR.B.DLC = 6;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = index;
    float *p2 = (float *)&tx_frame.data.u8[2];
    *p2 = value;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Set torque trajectory data. Note that 00 00 returns to the zero point. 
void set_ctmf(int id, float index, float value)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x1d;
    tx_frame.FIR.B.DLC = 6;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = index;
    float *p2 = (float *)&tx_frame.data.u8[2];
    *p2 = value;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Run according to specified trajectory data
void tdr(int id, float index)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x1f;
    tx_frame.FIR.B.DLC = 2;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = index;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Record current position, speed, and torque data to the specified index 
void record(int id, float index)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x21;
    tx_frame.FIR.B.DLC = 2;
    txid = tx_frame.MsgID;
    printf("%d\n", txid);
    tx_frame.data.u8[0] = index;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*
* Read state of running motor 
*
* 0x00: position
* 0x01: speed
* 0x02: Q-axis current
* 0x03: Q-axis voltage
* 0x04: D-axis current
* 0x05: D-axis voltage
* 0x06: temperature
* 0x07: program version
*
* 0x0A: position + speed 
* 0x0B: Q-axis voltage + Q-axis current 
* 0x0C: D-axis voltage + D-axis current 
*/ 
float read_rd(int id, float motor_param)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    unsigned int d;
    float rxdata;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x23;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = motor_param;
    ESP32Can.CANWriteFrame(&tx_frame);
    unsigned long currentMillis = millis();
    while (1)
    {
        xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS);
        if (txid == rx_frame.MsgID)
        {
            d = rx_frame.data.u8[3];
            d <<= 8;
            d |= rx_frame.data.u8[2];
            d <<= 8;
            d |= rx_frame.data.u8[1];
            d <<= 8;
            d |= rx_frame.data.u8[0];
            rxdata = *(float *)&d;
            break;
        }
    }
    return rxdata;
}

/*
* Set CAN ID. Starting at 0x01, each additional index increments corresponding ID by 4. 
*
* 0x01: ID becomes 57
* 0x02: ID becomes 97
* 0x03: ID becomes D7
* ...
*/
void set_CAN_ID(int id, float CAN_id)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x25;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = CAN_id;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Restore motor parameters by restarting and recalibrating 
void reset(int id)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x27;
    tx_frame.FIR.B.DLC = 0;
    txid = tx_frame.MsgID;
    ESP32Can.CANWriteFrame(&tx_frame);
}

// Receive motor data and display current status and function. Note: data must be checked for validity 
void receive_data()
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;

    unsigned long currentMillis = millis();

    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    {
        if (rx_frame.FIR.B.FF == CAN_frame_std)
        {
            printf("----------------\n");
        }
        else
        {
            printf("New extened frame:\n");
        }
        if (rx_frame.FIR.B.RTR == CAN_RTR)
        {
            printf("RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID, rx_frame.FIR.B.DLC);
        }
        else
        {
            printf("tx_frame.MsgID: %x\n", tx_frame.MsgID);
            printf("rx_frame.MsgID: %x\n", rx_frame.MsgID);

            if (txid == rx_frame.MsgID)
            {
                rx_frame.MsgID = rx_frame.MsgID & (~(0x3f << 6));
                switch (rx_frame.MsgID)
                {
                case 0x01:
                    printf("%s\n", "Success\n");
                    break;
                case 0x03:
                    printf("%s\n", "Success\n");
                    break;
                case 0x05:
                    switch (rx_frame.data.u8[0])
                    {
                    case 0x00:
                        printf("%s\n", "NO error\n");
                        break;
                    case 0x81:
                        printf("%s\n", "OTP error\n");
                        break;
                    case 0x83:
                        printf("%s\n", "OV error\n");
                        break;
                    case 0x84:
                        printf("%s\n", "UV error\n");
                        break;
                    default:
                        break;
                    }
                    break;
                case 0x07:
                    printf("%s\n", "Success\n");
                    break;
                case 0x09:
                    printf("%x\n", rx_frame.data.u8[0]);
                    switch (rx_frame.data.u8[0])
                    {
                    case 0x00:
                        printf("%s\n", "torque\n");
                        break;
                    case 0x01:
                        printf("%s\n", "Speed\n");
                        break;
                    case 0x02:
                        printf("%s\n", "Position\n");
                        break;
                    default:
                        break;
                    }
                    break;
                case 0x0B:
                    printf("%s\n", "Success\n");
                    break;
                case 0x0D:
                    printf("%s\n", "Success\n");
                    break;
                case 0x0F:
                    for (int i = 0; i < rx_frame.FIR.B.DLC; i++)
                    {
                        printf("0x%02X ", rx_frame.data.u8[i]);
                    }
                    printf("\n");
                    break;
                case 0x11:
                    printf("%s", "Success\n");
                    break;
                case 0x13:
                    for (int i = 0; i < rx_frame.FIR.B.DLC; i++)
                    {
                        printf("0x%02X ", rx_frame.data.u8[i]);
                    }
                    printf("\n");
                    break;
                case 0x15:
                    printf("%s", "Success\n");
                    break;
                case 0x17:
                    printf("%s", "Success\n");
                    break;
                case 0x19:
                    printf("%s", "Success\n");
                    break;
                case 0x1b:
                    printf("%s", "Success\n");
                    break;
                case 0x1d:
                    printf("%s", "Success\n");
                    break;
                case 0x1f:
                    printf("%s", "Success\n");
                    break;
                case 0x21:
                    printf("%s", "Success\n");
                    break;
                case 0x23:
                    for (int i = 0; i < rx_frame.FIR.B.DLC; i++)
                    {
                        printf("0x%02X ", rx_frame.data.u8[i]);
                    }
                    printf("\n");
                    break;
                case 0x65:
                    printf("%s", "Success\n");
                    break;
                case 0x40:
                    printf("%s", "Success\n");
                    break;
                default:
                    break;
                }
            }
            else
            {
                printf("%s", "Reply Error\n");
            }
        }
    }
}
