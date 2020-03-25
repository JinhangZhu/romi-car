#ifndef _Kinematics
#define _Kinematics_h

// Constant variables
// const float WHEEL_DIAMETER = 70;  // (mm)
const float WHEEL_SEPARATION = 138; // (mm)
// const float GEAR_RATIO = 120;
// const float COUNTS_PER_SHAFT_REVOLUTION = 12;
// const float COUNTS_PER_WHEEL_REVOLUTION = 1440;
const float MM_PER_COUNT = 0.153;

class Kinematics
{
public:
  Kinematics(); // Constructor, required.

  // Write your method functions:
  void update(volatile long, volatile long);           // Update the kinematics values at every call
  float update_straight(volatile long, volatile long); // Update the kinematics while DEBUGGING straight line motion
  void set_pose_x(float);                              // Set the private variable: pose_x       NOT USED
  void set_pose_y(float);                              // Set the private variable: pose_y       NOT USED
  void set_pose_theta(float);                          // Set the private variable: pose_theta   NOT USED
  float get_pose_x();                                  // Get the private variable: pose_x
  float get_pose_y();                                  // Get the private variable: pose_y
  float get_pose_theta();                              // Get the private variable: pose_theta in the range of [0,360]
  float get_posi_theta();                              // Get the private variable: posi_theta
  float get_pi_pose_theta();                           // Get the private variable: pose_theta in the range of [-180,180]
  void print_values();                                 // Print private variables while DEBUGGING
  void emtpy_values();                                 // Empty private variables, prepared for GOING HOME

private:
  //Private variables and methods go here
  void rectify_pose_theta(); // Rectify the pose_theta to [0,360]
  void rectify_posi_theta(); // Rectify the posi_theta to [0,360]
  float pose_x = 0;          // Position in X global frame (mm)
  float pose_y = 0;          // Position in X global frame (mm)
  float pose_theta = 0;      // Orientation in global frame (deg)
  float posi_theta = 0;      // Orientation to the starting point of GOING HOME (deg)
  float pose_theta_stamp;    // Orientation right after line following (deg)

  // Encoder counts
  volatile long prev_count_eleft;
  volatile long prev_count_eright;
};

// Required constructor.  Initialise variables.
Kinematics::Kinematics()
{
  prev_count_eleft = 0;
  prev_count_eright = 0;
  pose_x = 0;
  pose_y = 0;
  pose_theta = 0;
}

/*
* Update the kinematics values at every call
* 
* count_eleft: Counts of the left encoder
* count_eright: Counts of the right encoder
*/
void Kinematics::update(volatile long count_eleft, volatile long count_eright)
{
  // Change in the encoder counts since the last update
  volatile long change_count_eleft = count_eleft - prev_count_eleft;
  volatile long change_count_eright = count_eright - prev_count_eright;

  float d_right = change_count_eright * MM_PER_COUNT;
  float d_left = change_count_eleft * MM_PER_COUNT;

  float delta_theta = (d_right - d_left) / WHEEL_SEPARATION * 180 / PI;

  if (d_left != d_right)
  {
    float radius = (d_right + d_left) / (d_right - d_left) * WHEEL_SEPARATION / 2;
    pose_x = radius * sin((pose_theta + delta_theta) * PI / 180) + pose_x - radius * sin((pose_theta)*PI / 180);
    pose_y = -radius * cos((pose_theta + delta_theta) * PI / 180) + pose_y + radius * cos((pose_theta)*PI / 180);
  }
  else
  {
    float d_straight = (d_left + d_right) / 2;

    pose_x = pose_x + d_straight * cos(pose_theta * PI / 180);
    pose_y = pose_y + d_straight * sin(pose_theta * PI / 180);
  }

  posi_theta = atan2(-pose_y, -pose_x) * 180 / PI;
  rectify_posi_theta();

  pose_theta += delta_theta;
  rectify_pose_theta();

  // Update stored encoder counts
  prev_count_eleft = count_eleft;
  prev_count_eright = count_eright;
}

/*
* Update the kinematics while DEBUGGING straight line motion
* 
* count_eleft: Counts of the left encoder
* count_eright: Counts of the right encoder 
*/
float Kinematics::update_straight(volatile long count_eleft, volatile long count_eright)
{
  // Change in the encoder counts since the last update
  volatile long change_count_eleft = count_eleft - prev_count_eleft;
  volatile long change_count_eright = count_eright - prev_count_eright;

  // Convert the counts into mm
  float d_straight = ((change_count_eleft + change_count_eright) / 2) * MM_PER_COUNT;

  pose_x = pose_x + d_straight * cos(pose_theta * PI / 180);
  pose_y = pose_y + d_straight * sin(pose_theta * PI / 180);

  // Update heading theta
  pose_theta = pose_theta + ((change_count_eright - change_count_eleft) * MM_PER_COUNT / WHEEL_SEPARATION) * 180 / PI;
  rectify_pose_theta();

  // Update stored encoder counts
  prev_count_eleft = count_eleft;
  prev_count_eright = count_eright;

  return d_straight;
}

/*
* Set the private variable: pose_x       NOT USED
* 
*/
void Kinematics::set_pose_x(float new_pose_x)
{
  pose_x = new_pose_x;
}

/*
* Set the private variable: pose_y       NOT USED
* 
*/
void Kinematics::set_pose_y(float new_pose_y)
{
  pose_x = new_pose_y;
}

/*
* Set the private variable: pose_theta   NOT USED
* 
*/
void Kinematics::set_pose_theta(float new_pose_theta)
{
  pose_theta = new_pose_theta;
  rectify_pose_theta();
}

/*
* Get the private variable: pose_x
* 
*/
float Kinematics::get_pose_x()
{
  return pose_x;
}

/*
* Get the private variable: pose_y
* 
*/
float Kinematics::get_pose_y()
{
  return pose_y;
}

/*
* Get the private variable: pose_theta in the range of [0,360]
* 
*/
float Kinematics::get_pose_theta()
{
  return pose_theta;
}

/*
* Get the private variable: pose_theta in the range of [-180,180]
* 
*/
float Kinematics::get_pi_pose_theta()
{
  if (pose_theta >= 180)
  {
    return (pose_theta - 360);
  }
  else
  {
    return pose_theta;
  }
}

/*
* Get the private variable: posi_theta
* 
*/
float Kinematics::get_posi_theta()
{
  return posi_theta;
}

/*
* Rectify the pose_theta to [0,360]
* 
*/
void Kinematics::rectify_pose_theta()
{
  // Rectify theta
  if (pose_theta >= 360)
  {
    pose_theta -= 360;
  }
  if (pose_theta < 0)
  {
    pose_theta += 360;
  }
}

/*
* Rectify the posi_theta to [0,360]
* 
*/
void Kinematics::rectify_posi_theta()
{
  // Rectify theta
  if (posi_theta >= 360)
  {
    posi_theta -= 360;
  }
  if (posi_theta < 0)
  {
    posi_theta += 360;
  }
}

/*
* Empty private variables, prepared for GOING HOME
* 
*/
void Kinematics::emtpy_values()
{
  pose_x = 0;
  pose_y = 0;
  pose_theta = 0;
  posi_theta = 0;
}

/*
* Print private variables while DEBUGGING
* 
*/
void Kinematics::print_values()
{
  Serial.print("x");
  Serial.print(",");
  Serial.print("y");
  Serial.print(",");
  Serial.println("theta");
  Serial.print(",");
  Serial.println("ptheta");
  Serial.print(pose_x);
  Serial.print(",");
  Serial.print(pose_y);
  Serial.print(",");
  Serial.print(pose_theta);
  Serial.print(",");
  Serial.println(posi_theta);
  Serial.println("\n");
}
#endif
