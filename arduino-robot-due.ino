#include <Servo.h>

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
Servo myservo1;
Servo myservo2;


#define LEFT_SENSOR_PIN 9
#define RIGHT_SENSOR_PIN 8


short left_sensor;
short right_sensor;

void setup()
{
  Serial.begin(9600);
    
  myservo1.attach(2);
  myservo2.attach(3);
  pinMode(LEFT_SENSOR_PIN,INPUT_PULLUP);
  pinMode(RIGHT_SENSOR_PIN,INPUT_PULLUP);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  sensor.startContinuous();

  FORWARD();
}



void readSensors() {
    left_sensor = digitalRead(LEFT_SENSOR_PIN);
    right_sensor = digitalRead(RIGHT_SENSOR_PIN);

    Serial.print(left_sensor);
    Serial.print(" - ");
    Serial.print(right_sensor); 
    
 }


void loop() 
{

  Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  int mm = sensor.readRangeContinuousMillimeters();
  doObstacleLineFollower(mm);
  readSensors();
  if (left_sensor == 1 && right_sensor == 0){   
         RIGHT();
         Serial.print("RIGHT");
  } else if (left_sensor == 0 && right_sensor == 1)  {         
         LEFT();
         Serial.print("LEFT");
  }  
  Serial.println("");
}



void doObstacleLineFollower(int mm){
      if (mm < 60){
          STOP();
          BACK();
          delay(500);          
          STOP();          
          TURN();
          delay(800);
          while(true) {
            
            readSensors();
            
            if (left_sensor == 1) {
                break;
            }      
          }
    }
}


void FORWARD() {
  myservo1.write(180);   
  myservo2.write(0);
}

void BACK() {
  myservo1.write(0);
  myservo2.write(180);
}

void STOP() {
  myservo1.write(90);
  myservo2.write(90);
}

void RIGHT (void)
{ 
  myservo2.write(90);
  myservo1.write(180); 
}

void LEFT (void)
{
  myservo1.write(90);
  myservo2.write(0);
}

void TURN (void)
{
  myservo1.write(0);
  myservo2.write(0);
}