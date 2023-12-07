#include "test.h"

int txid;
// 发送指令使电机紧急停止运行并保持使能状态
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

/*发送指令设置电机状态-data[0]代表索引值，发送0x00是使电机失能,发送0x01是使电机使能,
0x02：将电机主控重启；0x03：将电机参数重置；0x04：将电机错误状态重置*/

void set_status(int id, float suoyin)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x03;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*发送指令读取电机状态*/
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

/*发送指令设置电机运行模式-Data[0]代表索引值，发送0x00是设置电机运行模式为力矩模式，
发送0x01是设置电机运行模式为速度模式，发送0x02是设置电机运行模式为位置模式*/
void set_mode(int id, float suoyin)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x07;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*发送指令读取电机运行模式*/
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

/*发送指令设置电机当前位置为零点*/
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

/*发送指令设置电机PID参数,Data[0]代表索引值，发送0x00代表设置位置环P的参数，
0x01：位置环I；0x02：位置环D；0x03：位置环斜率；0x04：位置环滤波周期；
0x05：速度环P；0x06：速度环I；0x07：速度环D；0x08：速度环斜率；0x09：速度环滤波周期；
0x0A：Q轴电流环P；0x0B：Q轴电流环I；0x0C：Q轴电流环D；0x0D：Q轴电流环斜率；0x0E：Q轴电流环滤波周期；
0x0F：D轴电流环P；0x10：D轴电流环I；0x11：D轴电流环D；0x12：D轴电流环斜率；0x13：D轴电流环滤波周期。*/
void set_PID(int id, float suoyin, float shuju)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x0d;
    tx_frame.FIR.B.DLC = 5;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
    float *p2 = (float *)&tx_frame.data.u8[1];
    *p2 = shuju;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*发送指令读取PID参数,Data[0]代表索引值，发送0x00是读取位置环P参数,
0x01：位置环I；0x02：位置环D；0x03：位置环斜率；0x04：位置环滤波周期；
0x05：速度环P；0x06：速度环I；0x07：速度环D；0x08：速度环斜率；0x09：速度环滤波周期；
0x0A：Q轴电流环P；0x0B：Q轴电流环I；0x0C：Q轴电流环D；0x0D：Q轴电流环斜率；0x0E：Q轴电流环滤波周期；
0x0F：D轴电流环P；0x10：D轴电流环I；0x11：D轴电流环D；0x12：D轴电流环斜率；0x13：D轴电流环滤波周期*/
float read_PID(int id, float suoyin)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    unsigned int d;
    float rxdata;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x0f;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
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

/*发送指令设置电机限制参数,Data[0]代表索引值，
0x01：电机温度限制；0x02：电压限制；0x03：电流限制；0x04：速度限制；
0x05：位置限制-最小值；0x06：位置限制-最大值；0x07：抱闸启动；0x08：抱闸维持；0x09：过压值；
Data[1]—Data[4]数据从低位到高位，输入转换后的十六进制数*/
void set_lim(int id, float suoyin, float shuju)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x11;
    tx_frame.FIR.B.DLC = 5;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
    float *p2 = (float *)&tx_frame.data.u8[1];
    *p2 = shuju;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*发送指令读取限制参数,Data[0]代表索引值，
0x01：电机温度限制；0x02：电压限制；0x03：电流限制；0x04：速度限制；
0x05：位置限制-最小值；0x06：位置限制-最大值；0x07：抱闸启动；0x08：抱闸维持；0x09：过压值；*/
float read_lim(int id, float suoyin)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    unsigned int d;
    float rxdata;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x13;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
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

/*发送指令使电机根据当前模式按照目标值运行,Data[0]—Data[3]数据从低位到高位，输入转换后的十六进制数*/
void spr(int id, float shuju)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x15;
    tx_frame.FIR.B.DLC = 4;
    txid = tx_frame.MsgID;
    float *p1 = (float *)&tx_frame.data.u8[0];
    *p1 = shuju;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*位置模式下，发送指令使电机按照指定速度和限制电流参数运行到指定位置
Data[0]—Data[3]的值代表电机运行到的位置，Data[4]—Data[7]的值代表电机运行的速度*/
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

/*发送指令设置连续轨迹运行的位置数据参数
Data[0]—Data[1]代表索引值，00 00为设置第0号点，Data[2]—Data[5]数据从低位到高位，输入转换后的十六进制数。*/
void set_ctp(int id, float suoyin, float shuju)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x19;
    tx_frame.FIR.B.DLC = 6;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
    float *p2 = (float *)&tx_frame.data.u8[2];
    *p2 = shuju;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*发送指令设置连续轨迹运行的速度数据参数
Data[0]—Data[1]代表索引值，00 00为设置第0号点，Data[2]—Data[5]数据从低位到高位，输入转换后的十六进制数。*/
void set_ctv(int id, float suoyin, float shuju)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x1b;
    tx_frame.FIR.B.DLC = 6;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
    float *p2 = (float *)&tx_frame.data.u8[2];
    *p2 = shuju;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*发送指令设置连续轨迹运行的矩流数据参数
Data[0]—Data[1]代表索引值，00为设置第0号点，Data[2]—Data[5]数据从低位到高位，输入转换后的十六进制数。*/
void set_ctmf(int id, float suoyin, float shuju)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x1d;
    tx_frame.FIR.B.DLC = 6;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
    float *p2 = (float *)&tx_frame.data.u8[2];
    *p2 = shuju;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*发送指令使电机按照指定的轨迹数据运行
Data[0]—Data[1]代表索引值，00为按0号点的数据运行*/
void tdr(int id, float suoyin)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x1f;
    tx_frame.FIR.B.DLC = 2;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
    ESP32Can.CANWriteFrame(&tx_frame);
}
/*发送指令记录当前位置速度和矩流数据到指定的数据索引位置
Data[0]—Data[1]代表索引值，00为记录到0号点*/
void record(int id, float suoyin)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x21;
    tx_frame.FIR.B.DLC = 2;
    txid = tx_frame.MsgID;
    printf("%d\n", txid);
    tx_frame.data.u8[0] = suoyin;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*发送指令读取电机运行时的数据
Data[0]代表索引值,0x00表示读取电机当前位置数据
0x01：当前速度；0x02：Q轴电流；0x03：Q轴电压；0x04：D轴电流；
0x05：D轴电压；0x06：当前电机温度；0x07：程序版本；
0x0A：位置+速度；0x0B：Q轴电压+Q轴电流；0x0C：D轴电压+D轴电流；*/
float read_rd(int id, float suoyin)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    unsigned int d;
    float rxdata;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x23;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
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

/*发送指令设置CAN ID,Data[0]代表设置的电机序号
0x01表示设置电机ID为57，0x02表示设置电机ID为97，
0x03表示设置电机ID为d7，以此类推，序号每加1，对应的ID加4（ID为十六进制）*/
void set_CAN_ID(int id, float suoyin)
{
    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id << 6 | 0x25;
    tx_frame.FIR.B.DLC = 1;
    txid = tx_frame.MsgID;
    tx_frame.data.u8[0] = suoyin;
    ESP32Can.CANWriteFrame(&tx_frame);
}

/*发送指令还原电机参数，重启电机重新校准*/
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

/*接收数据，通过判断接收的ID来确定是在使用哪个功能，进一步根据返回的数据，来判断显示出当前的状态
注：对于接收的数据需进行一个有效性的判断*/
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
