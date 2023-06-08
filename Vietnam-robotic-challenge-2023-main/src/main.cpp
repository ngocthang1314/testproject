#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

#define BEBUG_CTRL
#define X_JOY_CALIB 127
#define Y_JOY_CALIB 128

#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14
#define SERVO 2
#define SERVO2 3
#define SERVO3 4
#define SERVO4 5
#define SERVO_FREQ_MIN 200
#define SERVO_FREQ_MAX 550
#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 180

#define TOP_SPEED 4095
#define NORM_SPEED 2048
#define TURNING_FACTOR 1

#define SINGLE_HAND_DRIVING 0
#define TWO_HAND_DRIVING 1

#define ENA
#define ENB 

#define pressures false
#define rumble false
bool driving_mode = SINGLE_HAND_DRIVING;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2X ps2x;
bool mode_drive = false;
int servo1 = 0, servo2 = 0, servo3 = 0, servo4 = 0, collector_mode = 0, shooter_mode = 0;
void setup() 
{
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  pwm.setPWM(SERVO,0,0);

  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.print("Ket noi voi tay cam PS2)");

  int error = -1;
  for (int i = 0; i < 10; i++) 
  {
    delay(200);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
  }
  switch (error) 
  {
    case (0):
      Serial.println(" Ket noi tay cam PS2 thanh cong");
      break;
    case (1):
      Serial.println(" LOI) Khong tim thay tay cam, hay kiem tra day ket noi vơi tay cam ");
      break;
    case (2):
      Serial.println(" LOI) khong gui duoc lenh");
      break;
    case (3):
      Serial.println(" LOI) Khong vao duoc Pressures mode ");
      break;
  }

}
// servo setup
void servo_clockwise(uint8_t Servo)
{
  pwm.setPWM(Servo,0,410);
}
void servo_anticlockwise(uint8_t Servo1)
{
  pwm.setPWM(Servo1, 0, 204);
}
void stop_servo(uint8_t Servo2)
{
  pwm.setPWM(Servo2, 0, 0);
}
// shooter and collector setup
void collector()
{
  pwm.setPWM(8,0,0);
  pwm.setPWM(9,0,3000);
}
void reverse_collector()
{
  pwm.setPWM(8,0,3000);
  pwm.setPWM(9,0,0);
}
void collector_stop()
{
  pwm.setPWM(8,0,0);
  pwm.setPWM(9,0,0);
}
void shooter()
{
  pwm.setPWM(14,0,0);
  pwm.setPWM(15,0,4000);
}
void shooter_stop()
{
  pwm.setPWM(14,0,0);
  pwm.setPWM(15,0,0);
}
// control setup
bool ps2Control() 
{
  ps2x.read_gamepad(false, false);
  int speed = NORM_SPEED;
  if (ps2x.Button(PSB_CIRCLE))
    speed = TOP_SPEED;
   if (ps2x.ButtonPressed(PSB_SELECT))
    driving_mode =! driving_mode;
  
  int nJoyX = 128 - ps2x.Analog(PSS_RX);
  int nJoyY = 128 - (driving_mode ? ps2x.Analog(PSS_LY) :ps2x.Analog(PSS_RY));
  int nMotMixL;
  int nMotMixR; 
  bool temp = (nJoyY * nJoyX > 0);
  if (nJoyX) // Turning
  {
    nMotMixL = -nJoyX + (nJoyY * temp);
    nMotMixR = nJoyX + (nJoyY * !temp);
  }
  else // Forward or Reverse
  {
    nMotMixL = nJoyY;
    nMotMixR = nJoyY;
  }
 int c1 = 0, c2 = 0, c3 = 0, c4 = 0;

  if (nMotMixR > 0)
  {
    c3 = nMotMixR;
    c3 = map(c3, 0, 128, 0, speed);
  }

  else if (nMotMixR < 0)
  {
    c4 = abs(nMotMixR) + 1;
    c4 = map(c4, 0, 128, 0, speed);
  }

  if (nMotMixL > 0)
  {
    c1 = nMotMixL;
    c1 = map(c1, 0, 128, 0, speed);
  }
  else if (nMotMixL < 0)
  {
    c2 = abs(nMotMixL)+1;
    c2 = map(c2, 0, 128, 0, speed);
  }
  setPWMMotors(c1, c2, c3, c4);
  return 1;
//control servo 360 clockwise and anticlockwise 
  if(ps2x.ButtonPressed(PSB_PAD_UP))
  {
    servo1++;
  }
  switch(servo1 % 3)
  {
    case (0):
    stop_servo(SERVO);
    break;
    case(1):
    servo_clockwise(SERVO);
    break;
    case(2):
    servo_anticlockwise(SERVO);
    break;
  }
// control servo 360 (2) clockwise and anticlockwise
  if(ps2x.ButtonPressed(PSB_PAD_DOWN))
  {
    servo2++;
  }
  switch(servo2 % 3)
  {
    case (0):
    stop_servo(SERVO2);
    break;
    case(1):
    servo_clockwise(SERVO2);
    break;
    case(2):
    servo_anticlockwise(SERVO2);
    break;
  }
// control servo 180 clockwise and anticlockwise
  if(ps2x.ButtonPressed(PSB_PAD_LEFT))
  {
    servo3++;
  }
  switch(servo3 % 3)
  {
    case (0):
    stop_servo(SERVO3);
    break;
    case(1):
    servo_clockwise(SERVO3);
    break;
    case(2):
    servo_anticlockwise(SERVO3);
    break;
  }
// control servo 180 (2) clockwise and anticlockwise
  if(ps2x.ButtonPressed(PSB_PAD_RIGHT))
  {
    servo4++;
  }
  switch(servo4 % 3)
  {
    case (0):
    stop_servo(SERVO4);
    break;
    case(1):
    servo_clockwise(SERVO4);
    break;
    case(2):
    servo_anticlockwise(SERVO4);
    break;
  }
//shooting motor
  if(ps2x.ButtonPressed(PSB_R1))
  {
    shooter_mode++;
  }
  switch(shooter_mode % 2)
  {
    case(0):
      shooter_stop();
      break;
    case(1):
      shooter();
      break;
  }
//ball collector motor
  if(ps2x.Button(PSB_L2))
  {
    reverse_collector();
  }
  if(ps2x.ButtonPressed(PSB_L1))
  {
    collector_mode++;
  }
  switch(collector_mode % 2)
  {
    case (0):
      collector_stop();
      break;
    case(1):
      collector();
  }
  delay(50);
}
void loop() {ps2Control();}
