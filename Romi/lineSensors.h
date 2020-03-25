#ifndef _Line_follow_h
#define _Line_follow_h

// Line sensor.
#define LINE_LEFT_PIN A2   //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN A4  //Pin for the right line sensor

//Number of readings to take for calibration
const int NUM_CALIBRATIONS = 100;

/* 
 *  Class to represent a single line sensor
 */
class LineSensor
{
public:
  // Required function, class Constructor:
  // Saves the pin passed in as argument and sets to input
  LineSensor(int line_pin)
  {
    pin = line_pin;
    pinMode(pin, INPUT);
  }

  // Suggested functions.
  void calibrate();     //Calibrate
  int readRaw();        //Return the uncalibrated value from the sensor
  int readCalibrated(); //Return the calibrated value from the sensor

  // You may wish to add other functions!
  // ...

private:
  int pin;
  /*
     * Add any variables needed for calibration here
     */
  float bias;
};

// Returns unmodified reading.
int LineSensor::readRaw()
{
  return analogRead(pin);
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
void LineSensor::calibrate()
{
  /*
   * Write code to calibrate your sensor here
   */
  float cur_readings = 0;
  for (int i = 0; i < NUM_CALIBRATIONS; i++)
  {
    cur_readings += readRaw();
  }
  bias = cur_readings / NUM_CALIBRATIONS;
}

// Use the above bias values to return a
// compensated ("corrected") sensor reading.
int LineSensor::readCalibrated()
{
  /*
   * Write code to return a calibrated reading here
   */
  return readRaw() - bias;
}

#endif
