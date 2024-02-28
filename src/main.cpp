#include "test.h"

CAN_device_t CAN_cfg;             // CAN Config 
unsigned long previousMillis = 0; // will store last time a CAN Message was send 
const int interval = 10;          // interval at which CAN Messages are sent in milliseconds 
const int rx_queue_size = 10;     // Size of the received messages queue

void setup()
{
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);

    pinMode(CAN_SE_PIN, OUTPUT);
    digitalWrite(CAN_SE_PIN, LOW);

    // Initialize pin 32 as output
    pinMode(32, OUTPUT);
    digitalWrite(32, HIGH);

    Serial.begin(115200);
    CAN_cfg.speed = CAN_SPEED_1000KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_27;
    CAN_cfg.rx_pin_id = GPIO_NUM_26;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    // Init CAN Module 
    ESP32Can.CANInit();
}

void loop()
{
    float a, b, c, d, e;
    spt(1, 100, 3); // Moves motor 1 to 100° at 3rpm
    delay(4000);
    spt(1, 0, 3); // Moves motor 1 back to 0° at 3rpm
    delay(4000);
    a = read_lim(1, 1); // Motor temperature limit
    b = read_PID(1, 1); // Motor PID parameter
    c = read_rd(1, 1);  // Motor running speed
    d = read_status(1); // Motor status 
    e = read_mode(1);   // Motor mode (torque, speed or position)
    
    printf("%.2f, %.2f, %.2f, %.2x, %.2x\n", a, b, c, d, e);
}