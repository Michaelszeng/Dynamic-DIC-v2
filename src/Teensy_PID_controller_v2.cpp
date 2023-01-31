/* PID interrupt enabled controller for the Dynamic DIC platform
 *    version: v1.1
 *    Ahmad Mujtaba Jebran for Conformable Decoders Lab
 *    
 *  setup:
 *    - sensor input (I2C, SPI, ANALOG)
 *    - interrupt setups and service routines
 *    - motor control pins
 */

 // include libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include <Watchdog_t4.h>

// define input/output pin numbers
// left and right pwm output pins
#define FPWM 22
#define BPWM 23

// I2C communication lines 
#define SDApin 18
#define SCLpin 19

// distance sensors I/O pins
#define Sen1_interrupt 30
#define Sen1_shutdown 29
#define Sen1_add 0x54
#define Sen2_interrupt 32
#define Sen2_shutdown 31
#define Sen2_add 0x52

// create sensor instances
SFEVL53L1X distSen1(Wire, Sen1_shutdown, Sen1_interrupt);
SFEVL53L1X distSen2(Wire, Sen2_shutdown, Sen2_interrupt);

// enable watchdog timer 3: starting at 32ms with 15ms step size
WDT_T4<WDT3> wdt;

// variables to store distances
volatile int distance1 = 0;
volatile int distance2 = 0;

// variable to keep track of updated sensors
// 0: no updates, 1: sensor 1 updated, 2: sensor 2 updated, 3: both sensors updated
volatile int updatedsensors = 0;

// Discrete time variables: to keep error values, direction, and duty cycle values
double error = 0;          // mm
double prev_error = 0;     // mm
double prev_error2 = 0;    // mm

// Continuous time variables: 
//double error = 0;
//double prev_error = 0;
//double sum_error = 0;
//double d_error = 0;

// variable for dt between command updates
double dt = 0;
double dt_start = 0;
double dt_end = 0;

// variables for motor supply voltage, command voltage, and duty cycle
double duty_cycle = 0;
double prev_duty_cycle = 0; 
const int Vs = 10;       // [V]. motor supply voltage
float Vm = 0;           // [V]. motor voltage
float prev_Vm = 0;      // [V]. previous motor voltage

// desired location with respect ot the center of the gantry
int desiredlocation = 0; // mm
int err_buffer = 3; // mm

double Kp = 0.004;
double Ki = 0.003;
double Kd = 0.04;

double Kstatic = 19;  // Percent duty cycle to overcome static friction

// A, B, C coefficients
double A = 0.0;
double B = 0.0;
double C = 0.0;

// debug variables
int debug_print=true;


// Implementation of Arduino map() function but with floats
float fmap(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}


void setup_motor_PWM(){
  // a function to configure PWM pins for the left and right half bridges to drive the motor.
  // Use FlexPWM4.0 (23) and FlexPWM4.1 (22) at 25kHz PWM 
  // setup the FPWM and BPWM pins and their frequencies to 25kHz
  if (debug_print){
    Serial.println(F("setting up Motor PWM"));
    delayMicroseconds(5000);
  }
  
  pinMode(FPWM, OUTPUT);
  pinMode(BPWM, OUTPUT);
  analogWriteFrequency(FPWM, 25000);
  analogWriteFrequency(BPWM, 25000);

  // Set the PWM resolution to 11 bits (0-2047) to control the PWM signal with 3.3V/2048 = 1.6mV increments and
  // the motor voltage by Vmot/2048 or 20V/2048 = 10mV increments
  analogWriteResolution(11);

  //set duty cycles on both PWM signals to 0
  analogWrite(FPWM, 0);
  analogWrite(BPWM, 0);

  if (debug_print){
    Serial.println(F("Motor PWM ready!"));
    delayMicroseconds(5000);
  }
}


void setup_I2C_communication(){
  // a function to setup the I2C communication over SDA1(17) and SCL1(16) ports with the two distance sensors
  // set SDA and SCL pins
  if (debug_print){
    Serial.println(F("Setting up I2C communication"));
    delayMicroseconds(5000);
  }
  
  Wire.setSCL(SCLpin);
  Wire.setSDA(SDApin);

  // begin I2C in master mode
  Wire.begin();

  if (debug_print){
    Serial.println(F("I2C communication ready!"));
    delayMicroseconds(5000);
  }
}

void setup_distSensors(){
  // check if sensor1 is ready and responding
  // put sensor 2 in standby mode
  if (debug_print){
    Serial.println(F("setting up distance sensors"));
    delayMicroseconds(5000);
  }
  
  digitalWrite(Sen2_shutdown, LOW);
  
  // check if sensor 1 responds
  distSen1.begin();
  delayMicroseconds(1000);

  if (debug_print){
    Serial.println(F("waiting for sensor1 to respond"));
    delayMicroseconds(1000);
  }
  
  while (distSen1.checkBootState() == 0){
    // wait until sensor1 is ready
    digitalWrite(Sen1_shutdown, LOW);
    delayMicroseconds(1000);
    digitalWrite(Sen1_shutdown, HIGH);
    delayMicroseconds(1000);
  }

  if (debug_print){
//    sensor 1 is responding, change its address
    Serial.print(F("sen 1 is booted: "));
    Serial.println(distSen1.checkBootState());
    delayMicroseconds(1000);
    Serial.print("Sensor1 address: ");
    Serial.println(distSen1.getI2CAddress());
    delayMicroseconds(1000);
  }
  
  // change sensor 1 address
  distSen1.setI2CAddress(Sen1_add);
  delayMicroseconds(1000);
  
  // check if the address is changed
  if (debug_print){
    Serial.print(F("Sensor1 new address: "));
    Serial.println(distSen1.getI2CAddress());
    delayMicroseconds(1000);
    Serial.print("sen 1 is booted");
    Serial.println(distSen1.checkBootState());
    delayMicroseconds(1000);
  }
  
  // put sensor 1 in standby, and wake Sen 2
  digitalWrite(Sen2_shutdown, HIGH);
  delayMicroseconds(1000);
  distSen2.begin();
  delayMicroseconds(1000);

  if (debug_print){
    Serial.println(F("waiting for sensor2 to respond"));
    delayMicroseconds(1000);
  }
  
  while (distSen2.checkBootState() == 0){
    // wait until sensor2 is ready
    digitalWrite(Sen2_shutdown, LOW);
    delayMicroseconds(1000);
    digitalWrite(Sen2_shutdown, HIGH);
    delayMicroseconds(1000); 
  }

  // sensor 2 is responsive
  if (debug_print){
    Serial.print(F("sen 2 is booted: "));
    Serial.println(distSen2.checkBootState());
    delayMicroseconds(1000);
    Serial.print(F("Sensor2 address: "));
    Serial.println(distSen2.getI2CAddress());
    delayMicroseconds(1000);
  }
  
  // Change the distance mode to short for both sensors
  distSen1.setDistanceModeShort();
  delayMicroseconds(1000);
  distSen2.setDistanceModeShort();
  delayMicroseconds(1000);
  
  // set the detection range between 0 - 400 mm
//  DetectionConfig dc;
//  dc.distanceMode = DISTANCE_SHORT;
//  dc.thresholdHigh = 400;
//  dc.thresholdLow = 5;
//  dc.windowMode = WINDOW_IN;
//  dc.IntOnNoTarget = 1;
//
//  distSen1.setThresholdConfig(&dc);
  
//  check if the settings were set properly
  if (debug_print){
    Serial.println("Detection ranges: ");
    Serial.println(distSen1.getDistanceThresholdLow());
    Serial.println(distSen1.getDistanceThresholdHigh());
    Serial.println(distSen1.getDistanceThresholdWindow());
    Serial.println(distSen2.getDistanceThresholdLow());
    Serial.println(distSen2.getDistanceThresholdHigh());
    Serial.println(distSen2.getDistanceThresholdWindow());
    delayMicroseconds(1000);
  }
  
  // set intermeasurement period to budget period to get continuous distance reading
  // with (intermeasurement period - measurement period) = 0-5ms
  // this is the interval at which sensors return a reading (must be at least as large as budgeting time)
  distSen1.setIntermeasurementPeriod(50);
  delayMicroseconds(1000);
  distSen2.setIntermeasurementPeriod(50);
  delayMicroseconds(1000);

  if (debug_print){
    Serial.println(F("intermeasurement periods: "));
    Serial.println(distSen1.getIntermeasurementPeriod());
    Serial.println(distSen2.getIntermeasurementPeriod());
    delayMicroseconds(1000);
  }
  
  // set the timing budgets to min possible of 15ms or {20, 33, 50, 100, 200, 500}
  distSen1.setTimingBudgetInMs(33);
  delayMicroseconds(1000);
  distSen2.setTimingBudgetInMs(33);
  delayMicroseconds(1000);

  if (debug_print){
    Serial.println(F("Timing budgets: "));
    Serial.println(distSen1.getTimingBudgetInMs());
    delayMicroseconds(1000);
    Serial.println(distSen2.getTimingBudgetInMs());
  }
  
  // sensors are ready!
  if (debug_print){
    Serial.println("Sensors ready!");
    delayMicroseconds(1000);
  }
}


void capture_data1_ISR(){
  // cature the data from sensor 1 over I2C port
//  if (debug_print){
//    Serial.println("capturing data1");
//  }
  
  distance1 = distSen1.getDistance();
  Serial.println("distance1");

  // check the status of the data
//   byte rangestatus = distSen1.getRangeStatus();
//   if (rangestatus == 0){  // if the status is good, then update the distance
//     distance1 = distSen1.getDistance();
//     // updated the updatedsensors variable
//     if (updatedsensors == 2){
//       updatedsensors = 3;
//     }
//     else {
//       updatedsensors = 1;
//     }
//   }

//   // otherwise: print the error status and clear interrupt to capture next reading
//   else {
//     if (debug_print){
//       Serial.print(millis());
//       Serial.print("      Sensor1 status: ");
//       Serial.println(rangestatus);
//     }

//     if (updatedsensors == 1) {
//       updatedsensors = 0;
//     }
//     else if (updatedsensors == 3) {
//       updatedsensors = 2;
//     }
//     // distSen1.clearInterrupt();
//   }
}

void capture_data2_ISR(){
  distance2 = distSen2.getDistance();
  Serial.println("-------------distance1");

  // check the status of the data
  // byte rangestatus = distSen2.getRangeStatus();
  // if (rangestatus == 0){  // if the status is good, then update the distance
  //   distance2 = distSen2.getDistance();
    
  //   if (updatedsensors == 1){
  //     updatedsensors = 3;
  //   }
  //   else {
  //     updatedsensors = 2;
  //   }
  // }

  // // otherwise: print the error status and clear interrupt to capture next reading
  // else {
  //   if (debug_print){
  //     Serial.print(millis());
  //     Serial.print("      Sensor2 status: ");
  //     Serial.println(rangestatus);
  //   }

  //   if (updatedsensors == 2) {
  //     updatedsensors = 0;
  //   }
  //   else if (updatedsensors == 3) {
  //     updatedsensors = 1;
  //   }
  //   // distSen2.clearInterrupt();
  // }
}

//int check_sensor_status(int sensor){
//  /* check the status of the sensor ranging data. 
//   *  0 = Good
//   *  1 = Sigma fail
//   *  2 = Signal fail
//   *  3 = wrapped target fail
//   *  or unknown failure cases
//   *  
//   *  Output: 1 if the data is good. 0 otherwise
//   */
//
//   if (sensor == 1){
//    byte sensorstatus = distSen1.getRangeStatus();
//   }
//   else if (sensor == 2){
//    byte sensorstatus = distSen2.getRangeStatus();
//   }
//
//   // print the status on serial monitor
//   switch (sensorstatus){
//  case 0:
//    Serial.println("Good");
//    break;
//  case 1:
//    Serial.println("Sigma fail");
//    break;
//  case 2:
//    Serial.println("Signal fail");
//    break;
//  case 7:
//    Serial.println("Wrapped target fail");
//    break;
//  default:
//    Serial.print("Unknown: ");
//    Serial.println(sensorstatus);
//    break;
//  }
//
//  // return 1 if the range is usable. otherwise return 0
//  if (sensorstatus == 0){
//    return 1
//  }
//  else {
//    return 0
//  }
//}

//void setup_watchdogtimer(){
//  // configure the watchdog timer 1 for 200ms (about two data samples dt)
//  WDT_timings_t config;
//  config.trigger = 2;
//  config.timeout = 3;
//  config.callback = stop_motor();
//  wdt.begin(config);
//} 



void stop_motor(){
  analogWrite(FPWM, 0);
  analogWrite(BPWM, 0);
}





/* A PID command function to calculate the motor duty cycle based on the tracking distance error from the sensors
 *  - triggered by external sensor input trigger
 *  - output a duty cycle to timerxx to generate PWMx.x and PWMX.X
 */
void update_PID_command(){
  dt_end = millis();
  dt = dt_end - dt_start;
  
  // discrete time implementation
  prev_error2 = prev_error;
  prev_error = error;

  error = distance1 - distance2;

  // continuous time implementation
//  prev_error = error;
//  error = distance1 - distance2; 
  
  
  // find direction of needed motion to minimize error. 
  if (abs(error) <= err_buffer) {
    // if the error is smaller than a buffer value, keep motor off
    error = 0;
  }

  // if (debug_print){
  //   Serial.print("ERROR: ");
  //   Serial.println(error);
  // }

    // Discrete time: calculate A,B,C coefficients
    A = Kp + (Ki*dt/2) + (Kd/dt);
    B = (Ki*dt/2) - (Kp) - (2*Kd/dt);
    C = (Kd/dt);

    // PID command for updated motor voltage
    Vm = (A*error) + (B*prev_error) + (C*prev_error2) + prev_Vm;

    // Continuous time: calculate errors
//    sum_error = sum_error + (error*dt/2);
//    d_error = (error - prev_error)/dt;

    // calculate command Vm
//    Vm = (Kp * error) + (Ki * sum_error) + (Kd * d_error);
//    Serial.print("Vm: ");
//    Serial.println(Vm);

    // convert to duty cycle %
    duty_cycle = abs(Vm)*100/Vs;
    duty_cycle += Kstatic;

    // Trim the duty cycle between 0 - 90% (0-1850 for 11bit PWM)
    if (duty_cycle < 0){
      duty_cycle = 0;
    }
    else if(duty_cycle > 50){
//      duty_cycle = 1850;
//      duty_cycle = 1535;
        duty_cycle = 1024;
    }
    // if a valid duty cycle is calculated, map between 0-90% in 11bit digits
    else {
      duty_cycle = int(fmap(duty_cycle, 0, 50, 0, 1024));
    }
    // if (debug_print) {
    //   Serial.print("Duty cycle: ");
    //   Serial.println(duty_cycle);
    // }

    if (Vm > 0){
      // turn motor forward
      analogWrite(BPWM, 0);
      delayMicroseconds(5);
      analogWrite(FPWM, duty_cycle);
    }
    else if (Vm <= 0){
      // turn motor backwards
      analogWrite(FPWM, 0);
      delayMicroseconds(5);
      analogWrite(BPWM, duty_cycle);
    }

    // reset dt clock and feed the watchdog
//    dt_start = 0;
    dt_start = dt_end;
    wdt.feed();
}





void setup_external_interrupts(){
  // a function to configure external interrupt pins to be triggered by the distance sensor
  // select the interrupt input pin with pullup resistor. and configure for pin change
//  ICR1/ICR2 = ?
  if (debug_print){
    Serial.println(F("setting up external interrupt pins"));
    delayMicroseconds(5000);
  }
  
  pinMode(Sen1_interrupt, INPUT_PULLUP);
  pinMode(Sen2_interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Sen1_interrupt), capture_data1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(Sen2_interrupt), capture_data2_ISR, RISING);



  if (debug_print){
  Serial.println(F("External interrupts ready!"));
  delayMicroseconds(5000);
  }
}



// setup the microcontroller with 1ms timer enabled interrupt to sample an external signal and actuate the motor
void setup() {
  // start Serial communication over USB at 9600 baud rate
  if (debug_print){
    Serial.begin(9600);
    delayMicroseconds(5000);
    Serial.println(F("Setting up the system"));
    delayMicroseconds(5000);
  }
  
  // clear global interrrupts. Use cli() function or set Interrupt Status Register (ISR) to 1s to clear it
  // TODO: just clear necessary interrupts, not all (don't use cli())
  cli();

  // setup motor PWM and pins for 25kHz frequency
  setup_motor_PWM();


  // setup external interrupt to be triggered by the distance sensor
  setup_external_interrupts();

  // setup I2C communication
  setup_I2C_communication();

  // setup the distance sensors
  setup_distSensors();

  // setup the watchdog timer 3
  WDT_timings_t config;
  config.timeout = 600; // ms
  config.callback = stop_motor;
  
  // enable global interrupts
  // TODO: just enable the necessary interrupts, register level, not all (don't use sei())
  sei();

  // start distance measurement
  if (debug_print){
    Serial.println(F("system ready! measuring distances"));
    delayMicroseconds(5000);
  }

  // start dt tracker, distance ranging, and watchdog timer
  dt_start = millis();
  distSen1.startRanging();
  distSen2.startRanging();
  // wdt.begin(config);
}




void loop() {
  update_PID_command();
    
  distSen1.clearInterrupt();
  distSen2.clearInterrupt();

  delay(12);
}


// void loop() {
//   // Kstatic test
//   Kstatic += 0.5;
//   Serial.println(Kstatic);
//   delay(500);
//   duty_cycle = int(fmap(Kstatic, 0, 50, 0, 1024));
//   analogWrite(BPWM, 0);
//   delayMicroseconds(5);
//   analogWrite(FPWM, duty_cycle);
// }