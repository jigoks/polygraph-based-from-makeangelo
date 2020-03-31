//GlobalVariables

float MM_PER_STEP  = 0.025; //verify via equations 
#define STEPPER_DIR_HIGH   LOW
#define STEPPER_DIR_LOW    HIGH
#define MAX_ACCELERATION     (500.0)
#define MIN_ACCELERATION     (0.0)
#define NUM_TOOLS            (1)


/*********** measurements of the machine ******************/
// physical measurements of the machine
#define CENTER_TO_SHOULDER       ( 3.770f)  // cm (f)
#define SHOULDER_TO_ELBOW        ( 5.000f)  // cm (Rf)
#define ELBOW_TO_WRIST           (16.500f)  // cm (Re)
#define EFFECTOR_TO_WRIST        ( 1.724f)  // cm (e)
#define CENTER_TO_FLOOR          (18.900f)  // cm
#define DEGREES_ABOVE_HORIZONTAL (24.000f)
#define NUM_AXIES                (2)
#define NUM_MOTORS               (2)
#define NUM_SERVOS               (0)
#define DELTA_HOME_DIRECTION     LOW  // LOW or HIGH
/*************** measurements of the machine *************************/

#define MAX_SEGMENTS         (32)  // number of line segments to buffer ahead. must be a power of two.





#define MOTOR_0_DIR_PIN           (55)
#define MOTOR_0_STEP_PIN          (54)
#define MOTOR_0_ENABLE_PIN        (38)
#define MOTOR_0_LIMIT_SWITCH_PIN  (3)   // X min 


#define MOTOR_1_DIR_PIN           (61)
#define MOTOR_1_STEP_PIN          (60)
#define MOTOR_1_ENABLE_PIN        (56)
#define MOTOR_1_LIMIT_SWITCH_PIN  (14)  // Y min

#define MOTOR_2_DIR_PIN           (48)
#define MOTOR_2_STEP_PIN          (46)
#define MOTOR_2_ENABLE_PIN        (62)
#define MOTOR_2_LIMIT_SWITCH_PIN  (18)   // Z Min 


#define MAX_FEEDRATE         (100.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0.0)
//#define DEFAULT_FEEDRATE     (90.0)

#define MAX_JERK             (10.0)
#define MAX_Z_JERK           (0.3)



#define MOTOR_STEPS_PER_TURN          (200.0)  // motor full steps * microstepping setting

#define NEMA17_CYCLOID_GEARBOX_RATIO        (20.0)
#define NEMA23_CYCLOID_GEARBOX_RATIO_ELBOW  (35.0)
#define NEMA23_CYCLOID_GEARBOX_RATIO_ANCHOR (30.0)
#define NEMA24_CYCLOID_GEARBOX_RATIO        (40.0)

#define DM322T_MICROSTEP              (2.0)

#define ELBOW_DOWNGEAR_RATIO          (30.0/20.0)
#define NEMA17_RATIO                  (DM322T_MICROSTEP*NEMA17_CYCLOID_GEARBOX_RATIO*ELBOW_DOWNGEAR_RATIO)
#define NEMA23_RATIO_ELBOW            (NEMA23_CYCLOID_GEARBOX_RATIO_ELBOW)
#define NEMA23_RATIO_ANCHOR           (NEMA23_CYCLOID_GEARBOX_RATIO_ANCHOR)
#define NEMA24_RATIO                  (NEMA24_CYCLOID_GEARBOX_RATIO)

// Motors are numbered 0 (base) to 5 (hand)
#define MOTOR_0_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA23_RATIO_ANCHOR)  // anchor
#define MOTOR_1_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA24_RATIO)  // shoulder
//#define MOTOR_2_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA23_RATIO_ELBOW)  // elbow
//#define MOTOR_3_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA17_RATIO)  // ulna
//#define MOTOR_4_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA17_RATIO)  // wrist
//#define MOTOR_5_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA17_RATIO)  // hand


#define DEGREES_PER_STEP_0 (360.0/MOTOR_0_STEPS_PER_TURN)
#define DEGREES_PER_STEP_1 (360.0/MOTOR_1_STEPS_PER_TURN)
//#define DEGREES_PER_STEP_2 (360.0/MOTOR_2_STEPS_PER_TURN)
//#define DEGREES_PER_STEP_3 (360.0/MOTOR_3_STEPS_PER_TURN)
//#define DEGREES_PER_STEP_4 (360.0/MOTOR_4_STEPS_PER_TURN)
//#define DEGREES_PER_STEP_5 (360.0/MOTOR_5_STEPS_PER_TURN)



/**************** EXTERN VARIABLES ************************/
extern float max_jerk[NUM_MOTORS+NUM_SERVOS];
extern float acceleration;
extern float feed_rate;

/**************** EXTERN FUNCTIONS ************************/
extern float max_feedrate_mm_s[NUM_MOTORS+NUM_SERVOS];
extern void get_end_plus_offset(float *results);
extern void motor_settings();
extern void jogMotors();
extern void parseLine();



