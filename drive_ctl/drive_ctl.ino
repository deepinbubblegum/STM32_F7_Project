#include <AccelStepper.h>
#include <STM32FreeRTOS.h>
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <EthernetUdp.h>

#define MOTOR_L_DIR_GPIO D7
#define MOTOR_L_PWM_GPIO D6
#define MOTOR_R_PWM_GPIO D5
#define MOTOR_R_DIR_GPIO D4

// handle task
TaskHandle_t encoder_wheel;
TaskHandle_t connection;
TaskHandle_t wheel_pwm;

// init header funtional
void initialize_GPIO();
void initial_task();
void vWheel_loop(void *pvParameters);
void vEnc_wheel(void *pvParameters);
void vConnect(void *pvParameters);

AccelStepper stepperL(AccelStepper::DRIVER, MOTOR_L_PWM_GPIO, MOTOR_L_DIR_GPIO);
AccelStepper stepperR(AccelStepper::DRIVER, MOTOR_R_PWM_GPIO, MOTOR_R_DIR_GPIO);
AccelStepper *stepper[] = {&stepperL, &stepperR};

String incoming = "";
float cmd_speed[2] = {0, 0};
long pos[] = {0, 0};
int arrSize = *(&pos + 1) - pos;
int counting_checker = 0;
int task_100ms = 0;

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

// initialize network
IPAddress ip(10, 1, 10, 183);
unsigned int localPort = 10000;
EthernetUDP Udp;

// start
void setup() {
  Serial.begin(9600);
  initialize_GPIO();
  Ethernet.begin(ip);
  Udp.begin(localPort);

  // set acc speed
  for (int i = 0; i < arrSize; i++) {
    stepper[i]->setMaxSpeed(30000.0);
    stepper[i]->setSpeed(0.0);
  }

  // init task
  initial_task();
}
// end oparation

// funtional programs
void initialize_GPIO() {
  pinMode(MOTOR_L_DIR_GPIO, OUTPUT);
  pinMode(MOTOR_L_PWM_GPIO, OUTPUT);
  pinMode(MOTOR_R_PWM_GPIO, OUTPUT);
  pinMode(MOTOR_R_DIR_GPIO, OUTPUT);
}

int count_pack = 0;
void vWheel_loop(void *pvParameters) {
  while (true) {
    for (int i = 0; i < arrSize; i++) {
      stepper[i]->runSpeed();
    }

    int packetSize = Udp.parsePacket();
    if (packetSize) {
      Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      incoming = String(packetBuffer);
      Serial.println();
      cmd_speed[0] = getValue(incoming,',',0).toFloat();
      cmd_speed[1] = getValue(incoming,',',1).toFloat();
      counting_checker = int();
      incoming = String();
    }
    vTaskDelay((1L * configTICK_RATE_HZ) / 1000L);
  }
}

void vConnect(void *pvParameters) {
  while (true) {
    if (counting_checker > 50) {
      for (int i = 0; i < arrSize; i++) {
        cmd_speed[i] = 0.00;
      }
    }
    vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);
  }
}

void vEnc_wheel(void *pvParameters) {
  while (true) {
    if (task_100ms++ >= 10) {
      char ReplyBuffer[255] = "";
      task_100ms = 0;
      sprintf(ReplyBuffer, "{\"en0\": %d, \"en1\": %d}\r\n", pos[0], pos[1]);
      Udp.beginPacket("10.1.10.255", localPort);
      Udp.write(ReplyBuffer);
      Udp.endPacket();
      counting_checker++;
    }
    for (int i = 0; i < arrSize; i++) {
      stepper[i]->setSpeed(cmd_speed[i]);
      pos[i] = stepper[i]->currentPosition();
    }
    vTaskDelay((5L * configTICK_RATE_HZ) / 1000L);
  }
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// create task funtional :D
void initial_task() {
  // task wheel
  xTaskCreate(vWheel_loop,
              "Wheel",
              configMINIMAL_STACK_SIZE + 130,
              NULL,
              tskIDLE_PRIORITY + 3,
              &wheel_pwm
             );

  // task encoder_wheel
  xTaskCreate(vEnc_wheel,
              "Enc_wheel",
              configMINIMAL_STACK_SIZE + 50,
              NULL,
              tskIDLE_PRIORITY + 2,
              &encoder_wheel
             );

  // task check connect
  xTaskCreate(vConnect,
              "Connect",
              configMINIMAL_STACK_SIZE + 10,
              NULL,
              tskIDLE_PRIORITY + 1,
              &connection
             );
  // start FreeRTOS
  vTaskStartScheduler();
  while (1);
}

void loop() {
  // not use
}
