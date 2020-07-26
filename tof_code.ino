/*
 * Author: Geffen Cooper
 *
 * Arduino Sensor Suite Code for RC Car robot
 */

 
#define RUN_ALL
//#define TEST_MODE



// ====================== Libs for I2C, TOF sensor, and Servo motor ==========================

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
#include <TimerOne.h> 



// ========================================= PINS ============================================
 
#define XSHUT1 9            // TOF1 reset
#define XSHUT2 10           // TOF2 reset
#define servoPWM 6          // TOF servo pwm signal
#define trigPin 5           // ultrasonic receive
#define echoPin 4           // ultrasonic send
#define encoder_intr 3      // encoder interrupt



// ======================================= Objects ===========================================

// TOF sensor objects
VL53L0X sensor;
VL53L0X sensor2;

// servo object
Servo tofMount;



// ================================== Global variables ========================================

double theta_turn = 72;           // servo position (0 - 180 degrees), tof_servo 'home state' is 72 degrees
double theta_raw = 0;             // servo displacement in radians, 0 displacement = at 'home state' position 
double tt_delta = 2;              // (2 degrees), incremental change of servo position
double tr_delta = 0.0349;         // (2 degrees = 0.0349 radians), incremental displacement
bool dir = true;                  // direction which servo is currently turning -->  true = CCW    false = CW
bool is_mapping = false;          // activate TOF mapping
float duration, distance;         // ultrasonic sensor sound wave readings
bool last_state = false;          // stores last encoder state
bool curr_state = false;          // stores current encoder state
int interrupt_count = 0;          // counts the number of times encoder state change triggered
int little_wheel_rotations = 0;   // 12 state changes is a rotation
float total_distance = 0;         // distance traveled by car







// ============================================= SETUP ========================================

void setup()
{
  Serial.begin(9600);
  
  #ifdef RUN_ALL

  Serial.println("INIT SENSORS");
  
  init_servo();
  init_tof_sensors();
  init_encoder();
  init_ultrasonic();

  delay(2000);

  Serial.println("initialized");

  // read initial encoder state
  last_state = curr_state;
  curr_state = digitalRead(encoder_intr);
  
  #endif
}



// ================================================ Main Loop =====================================

void loop()
{
  
  #ifdef RUN_ALL
  update_ultrasonic(); // sends distance value over serial
  update_mapping_state();
  // update_encoder(); // sends distance rotated
  
  if(is_mapping)
  {
    tofMount.write(theta_turn); // every iteration of the loop, turn the servo one increment

    update_tof();
  
    // once reach threshold, reverse the direction
    if(theta_turn > 106 && dir == true)
    {
      tt_delta*=-1;
      tr_delta*=-1;
      dir = false;
    }
    else if(theta_turn < 36 && dir == false)
    {
      tt_delta*=-1;
      tr_delta*=-1;
      dir = true;
    }
  
  
    // increment theta
    theta_turn += tt_delta;
    theta_raw += tr_delta;
    delay(5);
  }

  #endif



  #ifdef TEST_MODE
  run_tests();
  #endif
}



// ============== init functions ===============

void init_tof_sensors()
{
  Serial.println("INIT TOF SENSORS");

  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  
  // turn off both TOF sensors (pulled high by default)
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);

  delay(500);
  Wire.begin();
  
  
  // TOF SENSOR 1 i2c init address
  digitalWrite(XSHUT1, HIGH);
  delay(150);
  Serial.println("00");
  sensor.init(true);
  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);
  Serial.println("02");

  // TOF SENSOR 2 i2c init address
  digitalWrite(XSHUT2, HIGH);
  delay(150);
  sensor2.init(true);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");


  Serial.println("");
  Serial.println("addresses set");
  Serial.println("");
  
  sensor.setTimeout(500);
  sensor2.setTimeout(500);
}

void init_servo()
{
  Serial.println("INIT SERVO");
  
  tofMount.attach(servoPWM); // servo PWM pin
  tofMount.write(theta_turn); // start at home state
}

void init_encoder()
{
  Serial.println("INIT ENCODER");
  pinMode(encoder_intr, INPUT);
}

void init_ultrasonic()
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

// ========================= Sensor Funcs ===============


void update_ultrasonic()
{
  // send and receive wave
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // calculate distance
  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;

  // Send ultrasnoic sensor readings over serial
  Serial.println("HC"); Serial.println(distance);
}

void update_tof()
{
    Serial.println("TOF");
    // read the distance for each TOF sensor
    long d1 = (sensor.readRangeSingleMillimeters());
    long d2 = (sensor2.readRangeSingleMillimeters());
  
    // Sensor 1
    if (sensor.timeoutOccurred())
    {
      Serial.println("_________________________________");
      Serial.print("Distance FWD (READING): ");
      Serial.println(" TIMEOUT");
      Serial.println("_________________________________");
      Serial.println("");
    }
    else
    {
      // calculate coordinates
      double x = d1*sin(-1*theta_raw);
      double y = -1*d1*cos(-1*theta_raw);
      Serial.println(x);
      Serial.println(y);
    }
  
    // Sensor 2
    if (sensor2.timeoutOccurred())
    {
      Serial.println("_________________________________");
      Serial.print("Distance FLT (READING): ");
      Serial.println(" TIMEOUT");
      Serial.println("_________________________________");
      Serial.println("");
    }
    else
    {
      //Serial.println(d2);
      double x = d2*sin(theta_raw);
      double y = d2*cos(theta_raw);
      Serial.println(x);
      Serial.println(y);
    }
}

void update_mapping_state()
{
  if (Serial.available() > 0)
  {
    String data = Serial.readStringUntil('\n');
    if(data == "startmapping")
    {
      is_mapping = true;
      Serial.println("Started Mapping");
      delay(1000);
    }
    else if(data == "stopmapping")
    {
      is_mapping = false;
      Serial.println("Stoped Mapping");
      delay(5000);
    }
  }
}

void update_encoder()
{
  // 12 interrupts = 1 encoder rotation = 3.14159 in
  // 4 encoder rotations = 1 wheel rotation

  // 48 encoder rotations = 1 wheel rotation
  
  
  last_state = curr_state;
  curr_state = digitalRead(encoder_intr);

  if(curr_state != last_state)
  {
    interrupt_count += 1;
  }

  if(interrupt_count == 12)
  {
    little_wheel_rotations += 1;
    total_distance += 3.14159;
    interrupt_count = 0;
  } 
  Serial.print("EC============================================="); Serial.println(total_distance);
}




// ====================== Test Functions =================
void run_tests()
{
  // test_tof_servo();
  // test_tof_sensors();
  // test_tof_with_servo();
  // test_encoder();
  // test_ultrasonic();
}

void test_tof_servo()
{
  Serial.println("TEST SERVO");
  init_servo();
  
  while(true)
  {
    tofMount.write(theta_turn); // every iteration of the loop, turn the servo one increment
    delay(50);
    // once reach threshold, reverse the direction
    if(theta_turn > 106 && dir == true)
    {
      tt_delta*=-1;
      tr_delta*=-1;
      dir = false;
    }
    else if(theta_turn < 36 && dir == false)
    {
      tt_delta*=-1;
      tr_delta*=-1;
      dir = true;
    }
    
    // increment theta
    theta_turn += tt_delta;
    theta_raw += tr_delta;
  }
}


void test_tof_sensors()
{
  Serial.println("TESTING TOF SENSORS");
  init_tof_sensors();
  while(true)
  {
    update_tof(); 
  }
}

void test_tof_with_servo()
{
    Serial.println("TESTING TOF SENSORS WITH SERVO");
    
    init_servo();
    init_tof_sensors();
    delay(3000);

    while(true)
    {
      tofMount.write(theta_turn); // every iteration of the loop, turn the servo one increment
      
      update_tof();
    
      // once reach threshold, reverse the direction
      if(theta_turn > 106 && dir == true)
      {
        tt_delta*=-1;
        tr_delta*=-1;
        dir = false;
      }
      else if(theta_turn < 36 && dir == false)
      {
        tt_delta*=-1;
        tr_delta*=-1;
        dir = true;
      }
    
    
      // increment theta
      theta_turn += tt_delta;
      theta_raw += tr_delta;
      delay(10); 
    }
}

void test_encoder()
{
  init_encoder();
  Serial.println("TESTING ENCODER");
  while(true)
  {
    update_encoder();
  }
}

void test_ultrasonic()
{
  init_ultrasonic();
  while(true)
  {
    update_ultrasonic();
  }
}
