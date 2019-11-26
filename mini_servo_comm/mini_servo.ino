/***************************************************
  Simple arduino script for mini servo arm based on
  the Adafruit 16-channel PWM & Servo driver

  Main loop writes current setpoints out to the servos
  on each tick.  If serial commands arive, they
  are ready to update servo setpoints and a response
  is given which includes current servo angles.

  For now, servo response is just an echo of latest
  command, but in the future if feedback encoders
  or pots are added, this will change.
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// High level constants
#define NUM_DOF 6

// Startup Point
float startup[NUM_DOF] = {0.0, 0.0, -70.0, -15.0, 45.0, -5};

// Parameters for the servos
// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
// It's also necessary to know how many degrees of rotation each of these servos
// has, so that we can scale a angle into a servo signal
// Do this with a range and with a servo number corresponding to zero
#define SERVO_MIN_S0  116
#define SERVO_MAX_S0  475
#define ANGLE_MIN_S0 -124.0
#define ANGLE_MAX_S0  90.0

#define SERVO_MIN_S1  233
#define SERVO_MAX_S1  517
#define ANGLE_MIN_S1 -80.0
#define ANGLE_MAX_S1  63.0

#define SERVO_MIN_E0  177
#define SERVO_MAX_E0  449
#define ANGLE_MIN_E0 -63.0
#define ANGLE_MAX_E0  67.0

#define SERVO_MIN_E1  116
#define SERVO_MAX_E1  536
#define ANGLE_MIN_E1 -115.0
#define ANGLE_MAX_E1  100.0

#define SERVO_MIN_W0  160
#define SERVO_MAX_W0  592
#define ANGLE_MIN_W0 -66.0
#define ANGLE_MAX_W0  112.0

#define SERVO_MIN_W1  179
#define SERVO_MAX_W1  600
#define ANGLE_MIN_W1 -78.0
#define ANGLE_MAX_W1  123.0

// Put the above defines into arrays for easy access
float a_mins[NUM_DOF] = {ANGLE_MIN_S0, ANGLE_MIN_S1, ANGLE_MIN_E0, ANGLE_MIN_E1, ANGLE_MIN_W0, ANGLE_MIN_W1};
float a_maxs[NUM_DOF] = {ANGLE_MAX_S0, ANGLE_MAX_S1, ANGLE_MAX_E0, ANGLE_MAX_E1, ANGLE_MAX_W0, ANGLE_MAX_W1};
int s_mins[NUM_DOF] = {SERVO_MIN_S0, SERVO_MIN_S1, SERVO_MIN_E0, SERVO_MIN_E1, SERVO_MIN_W0, SERVO_MIN_W1};
int s_maxs[NUM_DOF] = {SERVO_MAX_S0, SERVO_MAX_S1, SERVO_MAX_E0, SERVO_MAX_E1, SERVO_MAX_W0, SERVO_MAX_W1};

// Location for the outgoing servo commands
int command_list[NUM_DOF] = {0, 0, 0, 0, 0, 0};

// Location for outgoing angle command
float setpoints[NUM_DOF];

// Helper variables
String input_string("");
String output_string("");
int word_count = 0;
bool string_complete = false;
float clock = 0.0;
bool paused = true;
unsigned long loop_time = 0;

int status_counter = 0;

// Helper function to convert one angle into the corresponding
// servo command for a given joint
int angleToServo(int index, float angle)
{
  int s_range = s_maxs[index] - s_mins[index];
//  Serial.print("A: ");
//  Serial.println(s_range);

  float f_range = a_maxs[index] - a_mins[index];
//  Serial.print("B: ");
//  Serial.println(f_range);

  float norm_dist = (angle-a_mins[index]) / f_range;
//  Serial.print("C: ");
//  Serial.println(norm_dist);

  return (int)round(norm_dist * s_range) + s_mins[index];
}

// Helper function to convert one array of angle commands
// into a corresponding array of integer servo commands
void anglesToServos(float* angles, int* servos)
{
  for (int i=0; i<NUM_DOF; i++)
  {
    servos[i] = angleToServo(i, angles[i]);
  }
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  pulselength /= 4096;  // 12 bits of resolution
  pulse *= 1000;
  pulse /= pulselength;
  pwm.setPWM(n, 0, pulse);
}

// Helper function to reset the buffer and pointers to a specific pointer
// useful for stopping as it can reset back to default
void resetBuffer(float* data)
{
  // Initialize waypoint
  setpoints[0] = data[0];
  setpoints[1] = data[1];
  setpoints[2] = data[2];
  setpoints[3] = data[3];
  setpoints[4] = data[4];
  setpoints[5] = data[5];
}


///////////////////////////////////////////////////////////////////////////////
void setup()
{
  // Start Serial comm
  Serial.begin(500000);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Initialize buffer
  resetBuffer(startup);
  anglesToServos(setpoints, command_list);

  // Start commands to servos
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void writeStatus()
{
  char buff[12];

  String output = "";

  dtostrf(setpoints[0], 4, 4, buff);
  output += String(buff);
  for (int i=1; i<NUM_DOF; i++)
  {
    output += ", ";
    dtostrf(setpoints[i], 4, 4, buff);
    output += String(buff);
  }
  Serial.println(output);
}

void writeCommands()
{
  char buff[12];

  String output = "";

  dtostrf(command_list[0], 4, 4, buff);
  output += String(buff);

  for (int i=1; i<NUM_DOF; i++)
  {
    output += ", ";
    dtostrf(command_list[i], 4, 4, buff);
    output += String(buff);
  }
  Serial.println(output);
}

void loop()
{
  if (status_counter == 100)
  {
    writeCommands();
    status_counter = 0;
  }
  status_counter++;

  // Drive each servo one at a time
  anglesToServos(setpoints, command_list);
  for (int i=0; i<NUM_DOF; i++)
  {
    pwm.setPWM(i, 0, command_list[i]);

  }

  // Read the incoming message
  if (Serial.available() > 0)
  {

    // get the new byte:
    char inChar = (char)Serial.read();

    // if the incoming character is a comma, get the
    // number
    if (inChar == ',')
    {
      setpoints[word_count] = input_string.toFloat();
      input_string = String("");
      word_count++;
    }

    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    else if (inChar == '\n')
    {
      // Grab the last character (which has no comma at the end)
      setpoints[word_count] = input_string.toFloat();
      input_string = String("");
      word_count++;

      anglesToServos(setpoints, command_list);
      string_complete = true;

      // reset all the variables
      input_string = String("");
      word_count = 0;
    }

    // Default case, just add the character to the string
    else
    {
      // add it to the inputString:
      input_string += inChar;
    }

  }


  if (string_complete)
  {
    writeStatus();
    string_complete = false;
  }

  delay(10);
}
