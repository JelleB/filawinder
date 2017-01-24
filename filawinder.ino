  //Filament Winder by Ian Johnson

// Using QTR Sensor library from Pololu for line following 

#include "EEPROMex.h"
#include "EEPROMVar.h"
//Guide Servo

#include <Servo.h> 
Servo servo;  
int angle = 0;   // servo position in degrees 

//Filament Sensor
#include "QTRSensors.h"
#define NUM_SENSORS   4     // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     

// sensors 0 through 3 are connected to analog pins pins 3 through 0 , respectively
QTRSensorsAnalog qtra((unsigned char[]) {3, 2, 1, 0}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

int minimum_from_EEPROM[4];
int maximum_from_EEPROM[4];


//Spooler PID Setup
#include "PID_v1.h"
double Setpoint, Input, Output;                            //Define PID Variables
double Kp = 0.0020;
double Ki = 0;
double Kd = 0.0001;
PID pullPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);    //Specify the links and initial tuning parameters

//Digital Pins
int hall_a_Pin = 7;           // Hall sensor A
int sensor_setPin = 4;           //Button for setting the Pull PID setpoint
int motor_spoolerPin = 5;     //PWM pin for spooler motor
int guidePin = 6;             //Guide servo
int guide_maxPin = 3;        //Button for setting the Max Guide limits
int guide_minPin = 8;        //Button for setting the Min Guide limit
int toggle = 2;              //Auto / Manual toggle switch

//Analog Pins
int knob_Pin = 6;   // Pot that controls the fast puller speed to raise the loop
int sensor_1 = 0;         //Top filament sensor
int sensor_2 = 1;      //MIddle filament sensor
int sensor_3 = 2;     //Bottom filament sensor
int sensor_4 = 3;

//Variables for spool rotation
int hall_a_status = HIGH;    //The last reading from the Hall sensor
int hall_b_status = HIGH;    //The last reading from the other Hall sensor
int hall_a_mode = 0;          //Has Hall A been triggered?
int hall_b_mode = 0;          //Has Hall B been triggered?
int Revolutions = 0;          //Rotation counter
int Last_Revolution = 0;      //For checking if a new rotation hasbeen counted

//Variables for moving the guide
float guide_min = EEPROM.readFloat(100);           //Right limit for filament guide
float guide_max = EEPROM.readFloat(90);          //Left Limit for Filamnet guide
int guide_direction = 0;        //Direction the guide is moving
float guide_angle = 90;         //Start out in the middle
float last_position;

//Variables for smoothing the potentiometer reading
const int numReadings = 5;

int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int inputPin = knob_Pin;





//Variables for puller speed control via the photo sensors

unsigned int line_position; 
int spooler_speed = 200;
int puller_speed = 255;
int rotation_status = 0;
int puller_speed_old = 0;

bool logOn=false;

void setup ()
{
  Serial.begin (9600);
  pinMode(motor_spoolerPin, OUTPUT);
  pinMode(hall_a_Pin, INPUT);
  pinMode(3, INPUT);
  pinMode(8, INPUT);
  pinMode(guidePin, OUTPUT);
  pinMode(sensor_setPin, INPUT);
  pinMode(toggle, INPUT);
  pinMode(sensor_1, INPUT);
  pinMode(sensor_2, INPUT);
  pinMode(sensor_3, INPUT);
  pinMode(sensor_4, INPUT);
  pinMode(13, OUTPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  
  digitalWrite(hall_a_Pin, HIGH);        //Pullup resistor for Hall A
  digitalWrite(8, HIGH);      //Pullup resistor for guide setup switch
  digitalWrite(3, HIGH);      //Pullup resistor for guide setup switch
  digitalWrite(guidePin, HIGH);
  digitalWrite(sensor_setPin, HIGH);        //Pullup resistor for pid setup switch
  digitalWrite(toggle, HIGH);            //Pullup resistor for toggle
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
 
  servo.attach(guidePin); 
  servo.write(90 );

restore_sensor();                //Read sensor calibration values out of EEPROM into minimum_from_EEPROM and maximum_from_EEPROM arrays
qtra.calibrate();
qtra.calibratedMinimumOn[0] = minimum_from_EEPROM[0];
qtra.calibratedMinimumOn[1] = minimum_from_EEPROM[1];
qtra.calibratedMinimumOn[2] = minimum_from_EEPROM[2];
qtra.calibratedMinimumOn[3] = minimum_from_EEPROM[3];

qtra.calibratedMaximumOn[0] = maximum_from_EEPROM[0];
qtra.calibratedMaximumOn[1] = maximum_from_EEPROM[1];
qtra.calibratedMaximumOn[2] = maximum_from_EEPROM[2];
qtra.calibratedMaximumOn[3] = maximum_from_EEPROM[3];
 
Serial.println();
Serial.println();
Serial.print("EEPROM Minimum Values: ");
  for (int i = 0; i < NUM_SENSORS; i++){
 Serial.print(minimum_from_EEPROM[i]);
    Serial.print(' ');
 }
  Serial.println(' ');
  Serial.print("EEPROM Maximum Values: ");
   
  for (int i = 0; i < NUM_SENSORS; i++){
  Serial.print(maximum_from_EEPROM[i]);
    Serial.print(' ');
   }
   
  Serial.println();
  Serial.println();

Serial.print("QTR Minimum Values: ");
  for (int i = 0; i < NUM_SENSORS; i++){
 Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
 }
  Serial.println(' ');
  Serial.print("QTR Maximum Values: ");
   
  for (int i = 0; i < NUM_SENSORS; i++){
  Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
   }
   
  Serial.println();
  Serial.println("Kp: " + String(Kp , 5) + " Ki: " + String(Ki) + " Kd: " + String(Kd , 5));
  

  
  
  //Pull PID
    //initialize the variables
  Input = line_position; 
  Setpoint = 1500;                       //The value PID trys to maintain.  The number controls the amount of tension on the spool.
  pullPID.SetMode(AUTOMATIC);          //turn the PID on
  pullPID.SetControllerDirection(REVERSE);
  
    // initialize all the knob readings to 0: 
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;   

  pullPID.SetOutputLimits(-150, 150);
//  pullPID.SetSampleTime(500);
  
}
 
void loop ()
{
  

if (digitalRead(11) == 0) {
serial_output();   //Troubleshoot mode
}


if (digitalRead(9) == 0) {
  test();
}


//Guide setup switch

      
while (digitalRead(4) == 0 && digitalRead(toggle) == 0) {    //If middle button is pressed while in auto mode, set the current guide position
float last;
last = guide_position(last_position);
last_position = last;

if (digitalRead(11) == 0) {
serial_output();   //Troubleshoot mode
}


}

if (digitalRead(3) == 0 && digitalRead(8) == 1) {     //If guide max nutton is pressed go to calibrate_max
                                                      //calibrate_max will loop while the button is held down.
//calibrate_max();                                      //When the button is let up it will return here and write the
//EEPROM.writeFloat(90, guide_max);                           //new value to the EEPROM
logOn=true;
}

if (digitalRead(8) == 0 && digitalRead(3) ==1) {      //If guide min button is pressed go to guide min

//calibrate_min();
//EEPROM.writeFloat(100, guide_min); 
logOn=false;
}

// Calibrate   
if (digitalRead(4) == 0 && digitalRead(toggle) == 1 ){
 set_sensor();
}


guide_control();       //Go to the guide control function


if (digitalRead(toggle) == 1) {
manual_control();      //Go to the manual_control function for manual pull control
}

if (digitalRead(toggle) == 0) {
  
pull_control();       //Go to the spool_control function for auto control
}

}
   
   
   
   
void serial_output()
{
Serial.print("Hall ");
Serial.print(digitalRead(hall_a_Pin));
Serial.print(" Right ");
Serial.print(digitalRead(guide_maxPin));
Serial.print(" Left ");
Serial.print(digitalRead(guide_minPin));
Serial.print(" Center ");
Serial.print(digitalRead(sensor_setPin));
Serial.print(" Auto ");
Serial.print(digitalRead(toggle));
Serial.print(" Knob ");
Serial.print(analogRead(knob_Pin));
Serial.print(" Guide Pos ");
Serial.print(guide_angle);
Serial.print(" Mot Speed ");
Serial.print(puller_speed);
Serial.print(" Loop Pos ");
Serial.println(Input);
}
   
   
void manual_control()
{  

  int knob_reading = analogRead(knob_Pin);  // Get value from Puller Max Speed Knob
  puller_speed = knob_reading / 4.011;           //convert reading from pot to 0-255
  analogWrite(motor_spoolerPin,puller_speed);          //Set motor to the speed

  }

unsigned int previousLinePosition=0;

void pull_control()
{
  previousLinePosition=Input;
  qtra.readCalibrated(sensorValues);              
  unsigned int line_position = qtra.readLine(sensorValues, QTR_EMITTERS_OFF, 1);  
  

  Input = line_position;                         //Get line position from sensors


  if (!pullPID.Compute()) return;              //Run the PID 



//  int ScaledOutput = (Output * 1.7);             //Scale the Output from 0-150 to 0-255)       
//  if (ScaledOutput <= 0) {ScaledOutput = 1;}     //Limit the output to the range of 0-255) 
//  if (ScaledOutput >= 255){ScaledOutput = 255;}
//  puller_speed = ScaledOutput;
    
  int deltaspeed=(int)(Output+0.5);
//  if ( (puller_speed - puller_speed_old) > 25) puller_speed = puller_speed_old + 25;
//  if ( (puller_speed - puller_speed_old) < -25) puller_speed =  puller_speed_old - 25;


  puller_speed=puller_speed+deltaspeed;
  if (logOn)
  {
    Serial.println("Output=" + String(Output) + " prev_p=" + String(previousLinePosition) + " cur_p=" + String(line_position) + " d_speed=" + String(deltaspeed) 
                  + " speed=" + String(puller_speed) + " dInputbijdrage=" + String(Kp*(line_position-previousLinePosition)) + " time=" + String(millis()));
  }
  if (puller_speed<0) puller_speed=0;
  if (puller_speed>255) puller_speed=255;
  analogWrite(motor_spoolerPin,puller_speed);    //Set the spool speed to the PID result
  puller_speed_old = puller_speed;

}

   
void guide_control()
{
  //The hall sensor might get checked several times while the magnet is in range, but we don't want a rotation logged with every check
  //while the magnet passes by.  When a rotation is logged, rotation status gets set to 1 so it doesn't get logged again until after the hall has
  //switched off.
  
    if (digitalRead(hall_a_Pin) == 0) {         //Keep rotation status at 0 as long as hall isn't triggered
    rotation_status = 0;
    }

    
   // If Hall A has been triggered and rotation status is 0, log a rotation
  if (digitalRead(hall_a_Pin) == 1 && rotation_status == 0){    
   
   if (guide_angle < guide_min){              //If the guide angle passes minimum set direction to forward
     guide_angle = guide_min;  
     guide_direction = 0;}
 
    if (guide_angle > guide_max) {            //If the guide angle has reached maximum change direction to back
    guide_angle = guide_max;  
    guide_direction = 1;}
 
    if (guide_direction == 0) {               //If the current direction of the guide is forward
    if (digitalRead(12) == 1){                // If there is no jumper on Pin 12
    guide_angle = (guide_angle + 1.17); }     //Move the guide +1.17 degree for 1.75mm filament
    if (digitalRead(12) == 0){                //If there is a jumper on Pin 12
       guide_angle = (guide_angle + 1.90); }     //Move the guide 2 degrees for 3mm filament
    servo.write(guide_angle);}
    
   if (guide_direction == 1) {                //If the current direction of the guide is back
      if (digitalRead(12) == 1){              //If there is no jumper on Pin 12
    guide_angle = (guide_angle - 1.17); }     //Move the guide -1.17 degree for 1.75mm filament
     if (digitalRead(12) == 0){               //If there is a jumper on Pin 12
       guide_angle = (guide_angle - 1.90); }     //Move the guide -2 degrees for 3mm filament
     servo.write(guide_angle);}
   
     rotation_status = 1;                    //Remember that a rotation was counted 
  float last_position = guide_angle;   
   
  }
 
  // If Hall A is still being triggered after the rotation has been counted (rotation_status = 0) nothing will happen.
    
}

float guide_position(float last_pos){

smoothing();
  int reading = average;               // 0 to 1023
 float  cal_position = reading / 5;                        // 0 to 180-ish
  servo.write(cal_position);  //Move the guide to the knob position
  guide_angle = cal_position; 
    if (guide_angle > last_pos) {            //If the guide angle has reached maximum change direction to back  
    guide_direction = 0;} 
  
  if (guide_angle < last_pos){              //If the guide angle passes minimum set direction to forward
     guide_direction = 1;}
last_pos = guide_angle;
return last_pos;

}



void calibrate_max()  // Use the puller speed knob to set the guide limits
{
while (digitalRead(3) == 0 && digitalRead(8) == 1) {
smoothing();
  int reading = average;              // 0 to 1023
 int  cal_position = reading / 5;                        // 0 to 180-ish
  servo.write(cal_position);                        //Move the guide to the knob position
    guide_max = cal_position;                       //Make the current position the max limit

if (digitalRead(11) == 0) {
serial_output();   //Troubleshoot mode
}

}
 
  }


void calibrate_min()  // Use the puller speed knob to set the guide limits
{
while (digitalRead(8) == 0 && digitalRead(3) ==1) {
smoothing();
  int reading = average;                 // 0 to 1023
 int  cal_position = reading / 6;    // 0 to 180-ish
  servo.write(cal_position);                        //Move the guide to the knob position
    guide_min = cal_position;                       //Make the current position the max limit

if (digitalRead(11) == 0) {
serial_output();   //Troubleshoot mode
}
  
}
}


void set_sensor()                                    //Calibrate the reflectance sensors  Add LED indication that calibration is happening
{
  qtra.resetCalibration();
  delay(500);
 
  for (int i = 0; i < 200; i++)  // make the calibration take about 5 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  Serial.print("Minimum Values: ");
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    EEPROM.writeInt(i*2,qtra.calibratedMinimumOn[i]);     //Write the minimum values into EEPROM. Each takes 2 slots, 0-1,2-3,4-5,6-7
    Serial.print(' ');
  }
  Serial.println();
  Serial.print("Maximum Values: ");
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
  EEPROM.writeInt((i*2)+10,qtra.calibratedMaximumOn[i]);     //Write the minimum values into EEPROM. Each takes 2 slots, 10-11,12-13,14-15,16-17
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

}
 
 int smoothing(){
     // subtract the last reading:
  total= total - readings[index];         
  // read from the sensor:  
  readings[index] = analogRead(inputPin); 
  // add the reading to the total:
  total= total + readings[index];       
  // advance to the next position in the array:  
  index = index + 1;                    

  // if we're at the end of the array...
  if (index >= numReadings)              
    // ...wrap around to the beginning: 
    index = 0;                           

  // calculate the average:
  average = total / numReadings;         
  // send it to the computer as ASCII digits
   
  delay(1);        // delay in between reads for stability  
  //return average;
  
 }
 
 void test ()
{while (digitalRead(9) == 0) {
      analogWrite(motor_spoolerPin, 255);
      analogWrite(guidePin, 255);
      Serial.println("Testing");
  if (digitalRead(hall_a_Pin) == 0 && 
  digitalRead(sensor_setPin) == 0 && 
  digitalRead(guide_maxPin) == 0 && 
  digitalRead(guide_minPin) == 0 && 
  digitalRead(toggle) == 0 && 
  digitalRead(9) == 0 &&
  digitalRead(10) == 0 &&
  digitalRead(11) == 0 &&
  digitalRead(12) == 0 &&
  analogRead(knob_Pin) > 900 && 
  analogRead(sensor_1) > 900 && 
  analogRead(sensor_2) > 900 &&
  analogRead(sensor_3) > 900 &&
  analogRead(sensor_4) > 900 &&
  analogRead(4) > 900 &&
  analogRead(5) > 900 &&
  analogRead(7) > 900)
  {
    digitalWrite(13, HIGH);
    Serial.println("Verified");
  }
  else {digitalWrite(13, LOW);
  }
    } 
}
void restore_sensor(){
  
    for (int i = 0; i < NUM_SENSORS; i++)
  {minimum_from_EEPROM[i] = EEPROM.readInt(i*2);
  }

  for (int i = 0; i < NUM_SENSORS; i++)
  {maximum_from_EEPROM[i] = EEPROM.readInt((i*2)+10);
 
}
}

