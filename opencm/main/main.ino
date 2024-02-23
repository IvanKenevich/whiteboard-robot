#include <DynamixelWorkbench.h>

#define OPENCM_DEVICE "1"
#define MOTOR_BAUDRATE  1000000
#define SERIAL_BAUDRATE 57600
#define POS_FEEDBACK_RATE 500000 // in microseconds

const size_t N_ID = 5;
const uint8_t ID[N_ID] = {1, 2, 3, 4, 5};

uint8_t pos_low = 0, pos_high = 0;
uint8_t motor_id;
uint32_t get_data_1 = 0, get_data_2 = 0;

int32_t target_position = 0;

DynamixelWorkbench dxl_wb;
HardwareTimer Timer(TIMER_CH1);

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  while(!Serial); // Wait for Opening Serial Monitor

  const char *log = NULL;
  bool result = false;

  result = dxl_wb.init(OPENCM_DEVICE, MOTOR_BAUDRATE, &log);
  if (!result) {
    Serial.println("Failed to init");
    Serial.println(log);
  }

  for (int i = 0; i < N_ID; ++i) {
    motor_id = ID[i];
    result = dxl_wb.ping(motor_id, &log);
    if (!result) {
      Serial.print("Failed to ping motor id ");
      Serial.println(motor_id);
      Serial.println(log);
    }
  }

  for (int i = 0; i < N_ID; ++i) {
    motor_id = ID[i];
    uint16_t motor_speed = 100;
    dxl_wb.writeRegister(motor_id, (uint16_t) 32, 2, (uint8_t *) &motor_speed);
    if (!result) {
      Serial.println("Failed to set speed.");
      Serial.println(log);
    }
  }

//  Timer.stop();
//  Timer.setPeriod(POS_FEEDBACK_RATE);
//  Timer.attachInterrupt(sendPositions);
//  Timer.start();
}

void loop() {
  if(Serial.available() > 2) {
    pos_low = Serial.read();
    pos_high = Serial.read();
    motor_id = Serial.read();

    target_position = (int32_t)( (pos_high << 8) | pos_low);    
    dxl_wb.goalPosition(motor_id, target_position);
  } 
}

void sendPositions(void) {
  for (int i = 0; i < N_ID; ++i) {
    dxl_wb.readRegister(i+1, (uint16_t)36, (uint16_t)1, &get_data_1);
    dxl_wb.readRegister(i+1, (uint16_t)37, (uint16_t)1, &get_data_2);

    // write position
    Serial.write((int8_t) get_data_1);
    Serial.write((int8_t) get_data_2);
  }
}
