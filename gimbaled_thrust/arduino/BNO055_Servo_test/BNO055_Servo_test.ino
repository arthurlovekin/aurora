#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

#define SERVOPIN1 8
Servo myservo;  // create servo object to control a servo
int pos = 1;    // variable to store the servo position

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Servo and BNO055 Test"); Serial.println("");

  myservo.attach(SERVOPIN1);
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

int dir = 0;
void loop(void) 
{
  if(pos <= 0) {
    dir = 0;
  } else if (pos >=180) {
    dir = 1;
  }
  if(dir == 0) { 
    pos++;
  }
  else { 
    pos--;
  }
  myservo.write(pos); 
  
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  
  delay(15);
}
