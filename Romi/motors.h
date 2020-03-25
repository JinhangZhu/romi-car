//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

void setupMotorPins()
{
  // Set our motor driver pins as outputs.
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  // Set initial direction for l and r
  // Which of these is foward, or backward?
  digitalWrite(L_DIR_PIN, LOW);
  digitalWrite(R_DIR_PIN, LOW);
}

void drive_motor_signed(int l_power, int r_power)
{
  l_power = constrain(l_power, -255, 255);
  r_power = constrain(r_power, -255, 255);

  digitalWrite(L_DIR_PIN, (l_power < 0));
  digitalWrite(R_DIR_PIN, (r_power < 0));

  analogWrite(L_PWM_PIN, abs(l_power));
  analogWrite(R_PWM_PIN, abs(r_power));
}
