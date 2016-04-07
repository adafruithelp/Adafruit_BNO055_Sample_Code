
/* This code reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/


#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
 
  while (!Serial) {
     // wait for serial port to connect. Needed for native USB port only
  }

  
  //Check to see if the Inertial Sensor is wired correctly and functioning normally
  if(!bno.begin()){
    Serial.println("Inertial Sensor failed, or not present");
  }else{
    Serial.println("Inertial Sensor present");
  }
  delay(1000);
  //Retreive data on the sensor like software version and any error codes
  Serial.println("Sensor details:");
  displaySensorDetails();
  Serial.println("Sensor status:");
  displaySensorStatus();
  
  
}

void loop() {
  //Uses both rawdata and the Adafruit sensor object to grab data from the sensor
  sensors_event_t event;
  bno.getEvent(&event);
  uint8_t Temperature = bno.getTemp(); //Temperature in degrees Celsius
  imu::Vector<3> Acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);  //Acceleration in meters per second squared
  imu::Vector<3> Linear_Acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);//Acceleration in meters per second squared
  imu::Vector<3> Rotational_Velocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); //Angular velocity in radians per second
  imu::Vector<3> Gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);  //Gravitational strength in meters per second squared 
  imu::Vector<3> Magnetic_Field = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER); //Magnetic field intensity in micro Tesla or uT
   
    
    
   // print to the serial port:
   Serial.println("Acceleration: "+getString(Acceleration));
   Serial.println("Linear Acceleration: " + getString(Linear_Acceleration));
   Serial.println("Rotational Velocity: "+getString(Rotational_Velocity));
   Serial.println("Gravity: "+getString(Gravity));
   Serial.println("Magnetic Field: " + getString(Magnetic_Field));  
   Serial.println("Temperature: "+String(Temperature));
   //Prints the adafruit sensor orientation, it isn't a vector like the rest, so this is just easier
   Serial.print("Orientation: <");
   Serial.print(event.orientation.x, 4);
   Serial.print(", ");
   Serial.print(event.orientation.y, 4);
   Serial.print(", ");
   Serial.print(event.orientation.z, 4);
   Serial.println(">");
   Serial.println("------------------------------------");
  
   delay(100);
}
void displaySensorDetails(void){
  sensor_t sensor;
  bno.getSensor(&sensor);
  
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void displaySensorStatus(void){
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  
  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("------------------------------------");
  
  delay(500);
}
/* Takes vector output and makes it a nice string to look at, may change it to a row vector but whatever
 * 
 */
String getString(imu::Vector<3> vec){
  String res = ("<" + String(vec.x()) + ", " + String(vec.y()) + ", " + String(vec.z())+">");
  //Serial.println(res);
  return res;
}












