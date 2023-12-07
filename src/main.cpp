#include "test.h"

CAN_device_t CAN_cfg;             // CAN Config (CAN配置)
unsigned long previousMillis = 0; // will store last time a CAN Message was send (将存储最后一次发送的CAN消息)
const int interval = 10;          // interval at which send CAN Messages (milliseconds) (发送CAN消息的时间间隔(毫秒))
const int rx_queue_size = 10;     // Receive Queue size (接收队列大小)

void setup()
{
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);

    pinMode(CAN_SE_PIN, OUTPUT);
    digitalWrite(CAN_SE_PIN, LOW);

    // 初始化IO32，开启引脚的输出模式
    pinMode(32, OUTPUT);
    digitalWrite(32, HIGH);

    Serial.begin(115200);
    CAN_cfg.speed = CAN_SPEED_1000KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_27;
    CAN_cfg.rx_pin_id = GPIO_NUM_26;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    // Init CAN Module (初始化CAN模块)
    ESP32Can.CANInit();

    // put your setup code here, to run once: (将您的设置代码放在此处，运行一次：)
}

void loop()
{
    // put your setup code here, to loop run: (将您的设置代码放在此处，循环运行:)
    float a, b, c, d, e;
    spt(1, 100, 3); // 使1号电机以3rpm的速度运动到100°。
    delay(4000);
    spt(1, 0, 3); // 使1号电机以3rpm的速度运动到0°。
    delay(4000);
    a = read_lim(1, 1); // 将读到的1号电机温度限制参数返回值传入变量a
    b = read_PID(1, 1); // 将读到的PID参数返回值传入变量b
    c = read_rd(1, 1);  // 将读到的电机运行速度返回值传入变量c
    d = read_status(1); // 将读到的1号电机状态返回值传入变量d
    e = read_mode(1);   // 将读到的1号电机模式返回值传入变量e；
    
    printf("%.2f, %.2f, %.2f, %.2x, %.2x\n", a, b, c, d, e);
}