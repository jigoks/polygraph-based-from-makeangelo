
#define motor_setting

#include "Arduino.h"
#include "config_set.h"


extern void motor_settings();
void jogMotors();
//#include <robot_polargraph.h>
//#include "WString.h"


#define MAX_BUF 64

/*************** motor feed rate *************************/
#define NORMAL_MOTOR_STEPS 200
/*************** motor feed rate *************************/

#if NORMAL_MOTOR_STEPS == 200
#define DEFAULT_FEEDRATE     (180.0)
#define DEFAULT_ACCELERATION (300.0)
#define DEGREES_PER_STEP     (1.8)
#endif
#if NORMAL_MOTOR_STEPS == 400
#define DEFAULT_FEEDRATE     (100.0)
#define DEFAULT_ACCELERATION (300.0)
#define DEGREES_PER_STEP     (0.9)
#endif




const char *MotorNames = "LRUVWT";
const char *AxisNames = "XYZUVWT";


/*************** Constants *******************/
float max_jerk[NUM_MOTORS + NUM_SERVOS];
float max_feedrate_mm_s[NUM_MOTORS + NUM_SERVOS];
float tool_offset[NUM_TOOLS][NUM_AXIES];
char absolute_mode = 1; // absolute or incremental programming mode?
float acceleration;     // mm/sec^2
uint8_t current_tool = 0;
float feed_rate = DEFAULT_FEEDRATE;
int sofar;
char serialBuffer[MAX_BUF+1];
long last_cmd_time;
long line_number = 0;
int robot_uid = 0;

uint8_t lastGcommand=-1;

//Axis axies[NUM_AXIES];


struct Motor{
  uint8_t step_pin;
  uint8_t dir_pin;
  uint8_t enable_pin;
  uint8_t limit_switch_pin;
};

struct Motor motors[NUM_MOTORS+NUM_SERVOS];
struct Axis{
 uint8_t pos;
 uint8_t homePos;

};
struct Axis axies[NUM_AXIES];

extern void axis_setting()
{
  //axies[0]
}

extern void motor_settings()
{

  motors[0].step_pin        = MOTOR_0_STEP_PIN;
  motors[0].dir_pin         = MOTOR_0_DIR_PIN;
  motors[0].enable_pin      = MOTOR_0_ENABLE_PIN;
  motors[0].limit_switch_pin = MOTOR_0_LIMIT_SWITCH_PIN;

  motors[1].step_pin        = MOTOR_1_STEP_PIN;
  motors[1].dir_pin         = MOTOR_1_DIR_PIN;
  motors[1].enable_pin      = MOTOR_1_ENABLE_PIN;
  motors[1].limit_switch_pin = MOTOR_1_LIMIT_SWITCH_PIN;

  int i;
  for (i = 0; i < NUM_MOTORS; ++i) {
    // set the motor pin & scale
    pinMode(motors[i].step_pin, OUTPUT);
    pinMode(motors[i].dir_pin, OUTPUT);
    pinMode(motors[i].enable_pin, OUTPUT);

    // set the switch pin
    pinMode(motors[i].limit_switch_pin, INPUT);
    digitalWrite(motors[i].limit_switch_pin, HIGH);
  }


  long steps[NUM_MOTORS + NUM_SERVOS];
  memset(steps, 0, (NUM_MOTORS + NUM_SERVOS)*sizeof(long));

  for (i = 0; i < NUM_MOTORS + NUM_SERVOS; ++i) {
    max_jerk[i] = MAX_JERK;
    max_feedrate_mm_s[i] = MAX_FEEDRATE;
  }
  
}

// plotter position.
//float feed_rate = DEFAULT_FEEDRATE;
//float acceleration = DEFAULT_ACCELERATION;
float step_delay;
//float MM_PER_STEP= 0.025; //verify via equations 



void get_end_plus_offset(float *results) {
  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    results[i] = tool_offset[current_tool][i] + axies[i].pos;
   
  }
}

/**
   Move the pen holder in a straight line using bresenham's algorithm
   @input pos NUM_AXIES floats describing destination coordinates
   @input new_feed_rate speed to travel along arc
*/
void lineSafe(float *pos, float new_feed_rate) {
  float destination[NUM_AXIES];
  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    destination[i] = pos[i] - tool_offset[current_tool][i];
    // @TODO confirm destination is within max/min limits.
  }

//#ifdef SUBDIVIDE_LINES
  // split up long lines to make them straighter
  float delta[NUM_AXIES];
  float startPos[NUM_AXIES];
  float temp[NUM_AXIES];
  float len = 0;
  for (i = 0; i < NUM_AXIES; ++i) {
    startPos[i] = axies[i].pos;
    delta[i] = destination[i] - startPos[i];
    len += sq(delta[i]);
  }
  

  // What if some axies don't need subdividing?  like z axis on polargraph.
  // often SEGMENT_PER_CM_LINE is 10mm or 20mm.  but a servo movement can be 90-160=70, or 7 segments.  This is pretty nuts.
  // discount the z movement from the subdivision to use less segments and (I hope) move the servo faster.
  len -= sq(delta[2]);
  delta[2] = 0;


  len = sqrt(len);  //mm
  int pieces = ceil(len / SEGMENT_MAX_LENGTH_MM );
  float a;
  long j;

  // draw everything up to (but not including) the destination.
  for (j = 1; j < pieces; ++j) {
    a = (float)j / (float)pieces;
    for (i = 0; i < NUM_AXIES; ++i) {
      temp[i] = delta[i] * a + startPos[i];
    }
    //motor_line(temp, new_feed_rate);
  }
///#endif

  // guarantee we stop exactly at the destination (no rounding errors).
  //motor_line(destination, new_feed_rate);
}


float parseNumber(char code, float val) {
  char *ptr = serialBuffer; // start at the beginning of buffer
  char *finale = serialBuffer+sofar;
  for(ptr=serialBuffer; ptr<finale; ++ptr) {  // walk to the end
    if(*ptr==';') break;
    if(*ptr == code) { // if you find code on your walk,
      return atof(ptr + 1); // convert the digits that follow into a float and return it
    }
  }
  return val;  // end reached, nothing found, return default val.
}

/**
   G0-G1 [Xnnn] [Ynnn] [Znnn] [Unnn] [Vnnn] [Wnnn] [Ann] [Fnn]
   straight lines.  distance in mm.
*/

void parseLine() {
  float offset[NUM_AXIES];
  get_end_plus_offset(offset);
  acceleration = parseNumber('A', acceleration);
  acceleration = min(max(acceleration, (float)MIN_ACCELERATION), (float)MAX_ACCELERATION);
  float f = parseNumber('F', feed_rate);
  f = max(f, (float)MIN_FEEDRATE);

  int i;
  float pos[NUM_AXIES];
  for (i = 0; i < NUM_AXIES; ++i) {
    pos[i] = parseNumber(AxisNames[i], (absolute_mode ? offset[i] : 0)) + (absolute_mode ? 0 : offset[i]);
  }

  lineSafe( pos, f );
}




/**
   @param delay in microseconds
*/
void pause(const long us) {
  float delayTime = parseNumber('S', 0) + parseNumber('P', 0) * 1000.0f;
  pause(delayTime);
}


void findStepDelay() {
  //step_delay = 1000000.0f / (DEFAULT_FEEDRATE / MM_PER_STEP); //this is the correct one MM_PER_STEP changed to 1


   step_delay = 1000000.0f / (DEFAULT_FEEDRATE / MM_PER_STEP);
}


void jogMotors() {
  int i, j, amount;
   motor_settings(); 
  //motor_engage(); add this later
  findStepDelay();

  for (i = 0; i < NUM_MOTORS; ++i) {
    if (MotorNames[i] == 0) continue;
    amount = parseNumber(MotorNames[i], 0);
    if (amount != 0) {
      Serial.print("Moving ");
      Serial.print(MotorNames[i]);
      Serial.print(" (");
      Serial.print(i);
      Serial.print(") ");
      Serial.print(amount);
      Serial.print(" steps. Dir=");
      Serial.print(motors[i].step_pin);
      Serial.print(" Step=");
      Serial.print(motors[i].step_pin);
      Serial.print('\n');

      int x = amount < 0 ? STEPPER_DIR_HIGH  : STEPPER_DIR_LOW;
      digitalWrite(motors[i].dir_pin, x);

      amount = abs(amount);
      for (j = 0; j < amount; ++j) {
      digitalWrite(motors[i].step_pin, HIGH);
      digitalWrite(motors[i].step_pin, LOW);
        pause(step_delay);
      }
    }
  }
}


char checkLineNumberAndCRCisOK() {
  // is there a line number?
  long cmd = parseNumber('N', -1);
  if (cmd != -1 && serialBuffer[0] == 'N') { // line number must appear first on the line
    if ( cmd != line_number ) {
      // wrong line number error
      Serial.printf("BADLINENUM ");
      Serial.println(line_number);
      return 0;
    }

    // is there a checksum?
    int i;
    for (i = strlen(serialBuffer) - 1; i >= 0; --i) {
      if (serialBuffer[i] == '*') {
        break;
      }
    }

    if (i >= 0) {
      // yes.  is it valid?
      char checksum = 0;
      int c;
      for (c = 0; c < i; ++c) {
        checksum ^= serialBuffer[c];
      }
      c++; // skip *
      int against = strtod(serialBuffer + c, NULL);
      if ( checksum != against ) {
        Serial.print("BADCHECKSUM ");
        Serial.println(line_number);
        return 0;
      }
    } else {
      Serial.print("NOCHECKSUM ");
      Serial.println(line_number);
      return 0;
    }

    // remove checksum
    serialBuffer[i] = 0;

    line_number++;
  }

  return 1;  // ok!
}



void processCommand()
{
if((serialBuffer[0]== '\0') || (serialBuffer[0]== ';')+1);
if(!checkLineNumberAndCRCisOK()) return;

if(!strncmp(serialBuffer, "UID", 3)){robot_uid = atoi(strchr(serialBuffer, ' ')+1);}

  long cmd;

  // M codes
  cmd = parseNumber('M', -1);
  switch (cmd) {
    //case   6:  toolChange(parseNumber('T', current_tool));  break;
    //case  17:  motor_engage();  break;
  }

    if(cmd!=-1) return;  // M command processed, stop.
 // D codes
 cmd = parseNumber('D', -1);
  switch (cmd) {
    case  0:jogMotors();break;
  }
    if(cmd!=-1) return;  // D command processed, stop.

  cmd = parseNumber('G', lastGcommand);
  lastGcommand=-1;
  switch (cmd) {
    case  0:  
    case  1:  parseLine();  lastGcommand=cmd;  break;
    //case  2:  parseArc(1);  lastGcommand=cmd;  break;  // clockwise
  }


}

/**
   prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
*/
void parser_ready() {
  sofar = 0; // clear input buffer
  Serial.print("\n> "); // signal ready to receive input
  last_cmd_time = millis();
}

void Serial_listen()
{
  //listening to the serial command

  while(Serial.available() > 0)
  {
    char c = Serial.read();
    if(c== '\r') continue;
    if(sofar < MAX_BUF) serialBuffer[sofar++] = c;
    if(c == '\n')
    {
      serialBuffer[sofar - 1] = 0;
      processCommand();
      parser_ready();


    }
  }

}

void setup() {
  // put your setup code here, to run once:
}

void loop() {

  Serial_listen();
  // put your main code here, to run repeatedly:
}