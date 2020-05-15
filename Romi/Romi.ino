#include "encoders.h"
#include "lineSensors.h"
#include "motors.h"
#include "buzzer.h"
#include "pid.h"
#include "kinematics.h"

// Line sensor variables
const int upper_threshold[] = {250, 320, 360}; // Upper thresholds from left to right to keep detectable values
const int lower_threshold[] = {80, 80, 80};    // Lower thresholds from left to right to eliminate small noises.
int prev_sensor_values[] = {0, 0, 0};          // Last saved sensor values [left, centre, right]
int curr_sensor_values[] = {0, 0, 0};          // Currently saved sensor values [left, centre, right]

const int const_bias = 10;    // Constant basic wheel power
const int constrain_bias = 9; // Minimum of abs(power)
int bias = 10;                // Changeable basic wheel power
int l_power = 0;              // Demand for wheel PID
int r_power = 0;              // Demand for wheel PID

volatile long prev_count_eleft = 0;  // Previous count of the left encoder
volatile long prev_count_eright = 0; // Previous count of the right encoder
float l_speed = 0;                   // Measurement for wheel PID
float r_speed = 0;                   // Measurement for wheel PID

unsigned long speed_pid_prev_timestamp;       // Timestamp for the speed PID in ALL TIME
unsigned long heading_pid_prev_timestamp;     // Timestamp for the heaeding PID in LINE FOLLOWING
unsigned long straight_pid_prev_timestamp;    // Timestamp for the straight PID in GOING STRAIGHT
unsigned long home_pid_prev_timestamp;        // Timestamp for TURNING TOWARDS HOME
unsigned long stop_timer_start_timestamp = 0; // Timestamp for a millis-based PAUSING

LineSensor line_left(LINE_LEFT_PIN);     // Instance for the left linesensor
LineSensor line_centre(LINE_CENTRE_PIN); // Instance for the centre linesensor
LineSensor line_right(LINE_RIGHT_PIN);   // Instance for the right linesensor
float error_heading = 0;                 // Measurement of heading in LINE FOLLOWING
float touch_error_heading = 0;           // The very first Measurement of heading in LINE FOLLOWING

// Heading PID
float Kp_heading = 40.0;  //45
float Kd_heading = 0;     //2
float Ki_heading = 0.003; //0.001
float heading_output = 0;
PID heading_PID(Kp_heading, Ki_heading, Kd_heading);

// Straight PID
float Kp_straight = 10.0;
float Kd_straight = 0;
float Ki_straight = 0.1;
float straight_output = 0;
PID straight_PID(Kp_straight, Ki_straight, Kd_straight);

// Home PID
float Kp_home = 10.0; //15
float Kd_home = 0;
float Ki_home = 0.5; //0.3
float home_output = 0;
PID home_PID(Kp_home, Ki_home, Kd_home);

// Speed PID
float Kp_speed = 0.7;   //0.6
float Kd_speed = 0;     //0.05
float Ki_speed = 0.001; //0.001
float left_output = 0;
PID left_PID(Kp_speed, Ki_speed, Kd_speed);

// Right PID
float right_output = 0;
PID right_PID(Kp_speed, Ki_speed, Kd_speed);

Kinematics kinematics;
float pose_x = 0;                // Position x
float pose_y = 0;                // Position y
float pose_theta = 0;            // Orientation theta
float posi_theta = 0;            // Orientation towards home
float x_dest = 0;                // Destination x
float y_dest = 0;                // Destination y
float d_dest = 0;                // Distance to the destination
float d_right_margin = 83;       // Margin for the distance
float theta_right_margin = -4.1; // Margin for theta
float d_left_margin = 90;        // Margin for the distance
float theta_left_margin = 7;     // Margin for theta
float angle_threshold = 0.05;    // The threshold to decide the angles are the same

int stage = 0; // Stages for the motion
// 0: Start and Straight line
// 1: Joing the line
// 2: Line following
// 3: Stop, beep
// 4: Turn towards home
// 5: Go home
// 6: Stop at home

/*
* Setup
*/
void setup()
{
  setupEncoderLeft();
  setupEncoderRight();
  setupMotorPins();

  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  heading_pid_prev_timestamp = micros();
  speed_pid_prev_timestamp = micros();
}

/*
* Loop over stages 
*/
void loop()
{
  measure_error();

  switch (stage)
  {
    // 0: Start and Straight line
  case 0:
    if (curr_sensor_values[0] + curr_sensor_values[1] + curr_sensor_values[2] < 700)
    {
      straight_line(const_bias, 2000, 0, kinematics.get_pi_pose_theta());
    }
    else
    {
      touch_error_heading = error_heading;
      stage = 1;
    }
    break;
    // 1: Joing the line
  case 1:
    if (error_heading * touch_error_heading > 0)
    {
      bias = const_bias - 5;
    }
    else
    {
      pause_while(500, 2);
      bias = const_bias;
    }
    break;
    // 2: Line following
  case 2:
    if (check_leave_line() == LOW)
    {
      line_follow(10000);
    }
    else
    {
      stage = 3;
    }

    break;
    // 3: Stop, beep and turn towards home
  case 3:
    pause_while(500, 4);
    posi_theta = kinematics.get_posi_theta();
    break;
    // 4: Stop at the endpoint
  case 4:
    turn_home(2000, 5);
    //pause_while(500, 5);
    //assign_power(0, 0);
    break;
    // 5: Go home
  case 5:
    go_home(2000, 6);
    break;
    // 6: Stop at home
  case 6:
    assign_power(0, 0);
    left_output = 0;
    right_output = 0;
    break;

  default:
    assign_power(0, 0);
    break;
  }

  speed_pid(2000);
  drive_motor_signed(l_power - left_output, r_power - right_output);

  kinematics.update(count_eleft, count_eright);
  //kinematics.print_values();
  Serial.print("cur");
  Serial.print(",");
  Serial.print("dest");
  Serial.print(",");
  Serial.println("stage");
  Serial.print(kinematics.get_pose_x());
  Serial.print(",");
  Serial.print(d_dest);
  Serial.print(",");
  Serial.print(stage);
  Serial.println("\n");
}

/*
* PID for wheels
*/
void speed_pid(unsigned long interval)
{
  unsigned long speed_pid_timestamp = micros();
  unsigned long speed_pid_time_interval = speed_pid_timestamp - speed_pid_prev_timestamp;

  if (speed_pid_time_interval > interval)
  {
    volatile long change_count_eleft = count_eleft - prev_count_eleft;
    volatile long change_count_eright = count_eright - prev_count_eright;
    prev_count_eleft = count_eleft;
    prev_count_eright = count_eright;

    l_speed = change_count_eleft * pow(10, 6) / speed_pid_time_interval * 0.153;
    r_speed = change_count_eright * pow(10, 6) / speed_pid_time_interval * 0.153;

    left_output = left_PID.update(l_power * 5, l_speed);
    right_output = right_PID.update(r_power * 5, r_speed);

    speed_pid_prev_timestamp = speed_pid_timestamp;
  }
}

/*
* PID for moving straight 
*/
void straight_line(float input_bias, unsigned long interval, float angle, float measure_angle)
{
  /* input_bias: the constant speed | interval: time interval for straight-line PID | angle: demand of straight line=0 | measure_angle: measured orientation
  */
  unsigned long straight_pid_timestamp = micros();
  unsigned long straight_pid_time_interval = straight_pid_timestamp - straight_pid_prev_timestamp;

  if (straight_pid_time_interval > interval)
  {
    straight_output = straight_PID.update(angle, measure_angle);

    l_power = input_bias + straight_output;
    r_power = input_bias - straight_output;

    power_constrain(constrain_bias);  // Avoid low values that does not launch the car

    straight_pid_prev_timestamp = straight_pid_timestamp;
  }
}

/*
* Turning towards home 
*/
void turn_home(unsigned long interval, int next_stage)
{
  if (touch_error_heading < 0) // join from the right side
  {
    unsigned long home_pid_timestamp = micros();
    unsigned long home_pid_time_interval = home_pid_timestamp - home_pid_prev_timestamp;

    if (home_pid_time_interval > interval)
    {
      if (posi_theta + theta_right_margin > kinematics.get_pose_theta())
      { // anti-clockwise
        assign_power(-15, 15);
      }
      else
      { // clockwise
        assign_power(15, -15);
      }
      //    home_output = home_PID.update(kinematics.get_posi_theta() + theta_right_margin, kinematics.get_pose_theta());
      //
      //    l_power = 0 + home_output;
      //    r_power = 0 - home_output;
      //
      //    power_constrain(constrain_bias);

      home_pid_prev_timestamp = home_pid_timestamp;
    }

    if (abs(posi_theta + theta_right_margin - kinematics.get_pose_theta()) < angle_threshold)
    {
      assign_power(0, 0);
      stage = next_stage;
      d_dest = sqrt(pow(kinematics.get_pose_x(), 2) + pow(kinematics.get_pose_y(), 2)) + d_right_margin;
      kinematics.emtpy_values();
    }
  }
  else // join from the left side
  {
    unsigned long home_pid_timestamp = micros();
    unsigned long home_pid_time_interval = home_pid_timestamp - home_pid_prev_timestamp;

    if (home_pid_time_interval > interval)
    {
      if (posi_theta + theta_left_margin > kinematics.get_pose_theta())
      { // anti-clockwise
        assign_power(-15, 15);
      }
      else
      { // clockwise
        assign_power(15, -15);
      }
      home_pid_prev_timestamp = home_pid_timestamp;
    }

    if (abs(posi_theta + theta_left_margin - kinematics.get_pose_theta()) < angle_threshold)
    {
      assign_power(0, 0);
      stage = next_stage;
      d_dest = sqrt(pow(kinematics.get_pose_x(), 2) + pow(kinematics.get_pose_y(), 2)) + d_left_margin;
      kinematics.emtpy_values();
    }
  }
}

/*
* Go home 
*/
void go_home(unsigned long interval, int next_stage)
{
  // if (kinematics.get_pose_x() - d_dest < 50)
  if (kinematics.get_pose_x() > d_dest)
  {
    assign_power(0, 0);
    stage = next_stage;
  }
  else
  {
    straight_line(const_bias + 2, interval, 0, kinematics.get_pi_pose_theta());
  }
}

/*
* PID for line following
*/
void line_follow(unsigned long interval)
{
  unsigned long heading_pid_timestamp = micros();
  unsigned long heading_pid_time_interval = heading_pid_timestamp - heading_pid_prev_timestamp;

  if (heading_pid_time_interval > interval)
  {
    heading_output = heading_PID.update(0, error_heading);

    l_power = const_bias - heading_output;
    r_power = const_bias + heading_output;

    power_constrain(constrain_bias);

    //    Serial.print(l_power);
    //    Serial.print(",");
    //    Serial.print(r_power);
    //    Serial.println("\n");

    heading_pid_prev_timestamp = heading_pid_timestamp;
  }
}

/*
* Assign powers for the wheels 
*/
void assign_power(int assign_l_power, int assign_r_power)
{
  l_power = assign_l_power;
  r_power = assign_r_power;
}

/*
* Measure errors for heading in line following
*/
void measure_error()
{
  curr_sensor_values[0] = line_left.readCalibrated();
  curr_sensor_values[1] = line_centre.readCalibrated();
  curr_sensor_values[2] = line_right.readCalibrated();

  // Thresholding the sensing values
  // Eliminate small noises
  curr_sensor_values[0] = curr_sensor_values[0] > lower_threshold[2] ? curr_sensor_values[0] : 0;
  curr_sensor_values[1] = curr_sensor_values[1] > lower_threshold[1] ? curr_sensor_values[1] : 0;
  curr_sensor_values[2] = curr_sensor_values[2] > lower_threshold[0] ? curr_sensor_values[2] : 0;

  // Left -> Centre -> Right: +1 -> 0 -> -1
  float total_value = curr_sensor_values[0] + curr_sensor_values[1] + curr_sensor_values[2];
  float p_value[] = {curr_sensor_values[0] / total_value, curr_sensor_values[1] / total_value, curr_sensor_values[2] / total_value};
  error_heading = p_value[0] - p_value[2];

  if (isnan(error_heading))
  {
    error_heading = 0;
  }

  prev_sensor_values[0] = curr_sensor_values[0];
  prev_sensor_values[1] = curr_sensor_values[1];
  prev_sensor_values[2] = curr_sensor_values[2];

  //  Serial.print(curr_sensor_values[0]+curr_sensor_values[1]+curr_sensor_values[2]);
  //  Serial.print(",");
  //  Serial.print(error_heading);
  //  Serial.println("\n");
}

/*
* Check if the car leaves the line 
*/
bool check_leave_line()
{
  if (curr_sensor_values[0] < lower_threshold[0] && curr_sensor_values[1] < lower_threshold[1] && curr_sensor_values[2] < lower_threshold[2])
  {
    return HIGH;
  }
  else
  {
    return LOW;
  }
}

/*
* Pause for a while and move to the next stage 
*/
void pause_while(unsigned long interval, int next_stage)
{
  if (stop_timer_start_timestamp == 0)
  {
    stop_timer_start_timestamp = millis();
    pinMode(BUZZER_PIN, OUTPUT);
    analogWrite(BUZZER_PIN, 100);
  }
  assign_power(0, 0);
  if (millis() - stop_timer_start_timestamp > interval)
  {
    analogWrite(BUZZER_PIN, 0);
    stage = next_stage;
    stop_timer_start_timestamp = 0;
  }
}

/*
* Constrain the abs(power) to be no less than an value specified
*/
void power_constrain(int anti_jitter)
{
  if (l_power > 0 && l_power < anti_jitter)
  {
    l_power = constrain(l_power, anti_jitter, 255);
  }
  else if (l_power < 0 && l_power > -anti_jitter)
  {
    l_power = constrain(l_power, -255, -anti_jitter);
  }

  if (r_power > 0 && r_power < anti_jitter)
  {
    r_power = constrain(r_power, anti_jitter, 255);
  }
  else if (r_power < 0 && r_power > -anti_jitter)
  {
    r_power = constrain(r_power, -255, -anti_jitter);
  }
}
