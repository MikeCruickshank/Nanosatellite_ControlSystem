#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "StandardCplusplus.h"
#include <vector>
#include <algorithm>

using namespace std;
Adafruit_BNO055 bno = Adafruit_BNO055(55);


const int threshold = 2; // velocity threshold of when to fire nozzles, rpm
int num_readings = 2; // number of velocity readings that have to consistently be above threshold to fire
int pos_thrust_pin = 36;      // for now ccw
int neg_thrust_pin = 38;
int pos_thrust_pin_ground = 37;
int neg_thrust_pin_ground = 39;
int axis = 3; // x=1, y=2, z=3
int timestep = 100; // time between velocity readings, in milliseconds
int displayTime = 1;
int countNum = 0;


vector <vector<double> > history_velocity; // vector of vectors (# of readings x 3) of all velocity readings
vector <vector<bool> > history_nozzle;
bool  positive_thruster, negative_thruster;


double toRPM(double degPerSecond)
{
  return (degPerSecond*60/(360));
}

void controlNozzles(vector<vector<double> > &velocityVector, vector<vector<bool> > &nozzleVector, int axis)
{

  vector<bool> current_nozzle = nozzleVector.back();
  bool next_nozzle[2];
  
  vector<double> current_velocity;
  int state(1); // state = 0 for v < -thres, = 2 v > thresh
  
  int   N = velocityVector.size();
  for (int i=0; i < num_readings; ++i)
  {    
    current_velocity.push_back(velocityVector[N-1-i][axis-1]);
  }
    
  //vector<bool>::iterator it;
  //for(it = vb.begin(); it != vb.end(); it++)
  //cout << *it;
  //cout << endl;

  // Check to see if recent velocity is within threshold
  bool newState = true;
  for (vector<double>::iterator it = current_velocity.begin() ; it != current_velocity.end(); ++it)
  {
    if (*it < threshold & *it > -threshold){
      newState = false;
    }
  }

  // If recent velocity exceeds threshold, determine which direction
  if (newState)
  {
      bool aboveThreshold = false;
      for (vector<double>::iterator it = current_velocity.begin() ; it != current_velocity.end(); ++it)
      {
        if (*it > threshold){
          aboveThreshold = true;
        }
      }
      if (aboveThreshold){
        state = 2;
      }
      else{
        state = 0;
      }
  }
  else{
    state = 1;
  }

   
  // Control Nozzles
  if (state == 0)  // negative velocity, positive thruster
  {
    next_nozzle[0] = true;
  }
  else
  {
    next_nozzle[0] = false;
  }


  if (state == 2) // positive velocity, negative thruster
  {
    next_nozzle[1] = true;
  }
  else
  {
    next_nozzle[1] = false;
  }

  if (state == 1)
  {
    next_nozzle[0] = false;
    next_nozzle[1] = false;
  }

  vector<bool> next_nozzle_vector;
  next_nozzle_vector.push_back(next_nozzle[0]);
  next_nozzle_vector.push_back(next_nozzle[1]);
  nozzleVector.push_back(next_nozzle_vector);

  
  if (next_nozzle[0] & next_nozzle[1])
  {
    Serial.print("Warning: Both Nozzles are Open");
  }


    if(next_nozzle[0]){
      OpenPositiveNozzle();
    }

    if(next_nozzle[1]){ 
      OpenNegativeNozzle();
    }
    
}

void OpenPositiveNozzle(){
  Serial.println("       Opening Positive Nozzle");
  digitalWrite(pos_thrust_pin, HIGH);
  delay(100);
  digitalWrite(pos_thrust_pin, LOW);
  digitalWrite(pos_thrust_pin_ground, HIGH);
  delay(100);
  digitalWrite(pos_thrust_pin_ground, LOW);


}

void OpenNegativeNozzle(){
  Serial.println("       Opening Negative Nozzle");
  
  digitalWrite(neg_thrust_pin, HIGH);
  delay(100);
  digitalWrite(neg_thrust_pin, LOW);

  digitalWrite(neg_thrust_pin_ground, HIGH);
  delay(100);
  digitalWrite(neg_thrust_pin_ground, LOW);
}
//



void printVector(vector<vector<double> > &velocityVector, int n, int axis)
{
  int N = velocityVector.size();
  Serial.print("   Vector: ");
  for (int i=0; i < n; i++)
  {
   
     Serial.print(velocityVector[N-1-i][axis-1]);
     Serial.print(" RPM, ");
     delay(200);
  }
  Serial.println("");
}


void setup(void)
  {
  pinMode(pos_thrust_pin,OUTPUT);
  pinMode(neg_thrust_pin,OUTPUT);
  pinMode(pos_thrust_pin_ground,OUTPUT);
  pinMode(neg_thrust_pin_ground,OUTPUT);
  pinMode(22,OUTPUT);

  digitalWrite(22, HIGH);

  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
  /* There was a problem detecting the BNO055 ... check your connections */
  Serial.print("No BNO055 detected!");
  while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  vector<bool> current_nozzle;
  current_nozzle.push_back(false);
  current_nozzle.push_back(false);
  history_nozzle.push_back(current_nozzle);

  
  Serial.print("  Rotational Velocity Threshold: ");
  Serial.print(threshold);
  Serial.print(" RPM");
  Serial.println("");
}

void loop(void)
{
  vector <double> current_velocity;
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  current_velocity.push_back( (double)toRPM(euler.x()/16) );
  current_velocity.push_back( (double)toRPM(euler.y()/16) );
  current_velocity.push_back( (double)toRPM(euler.z()/16) );
  if (countNum > num_readings) {
    history_velocity.erase(history_velocity.begin());
    history_nozzle.erase(history_nozzle.begin());

  }
  history_velocity.push_back(current_velocity);
  controlNozzles(history_velocity,history_nozzle, axis); // pass by reference
  delay(timestep);

  if (countNum % (int(displayTime/(0.001*timestep))) == 0 || current_velocity[2] < -threshold || current_velocity[2] > threshold )
   {
    Serial.print(countNum);
    Serial.print("   X: ");
    Serial.print(current_velocity[0]);
    Serial.print(" RPM,  Y: ");
    Serial.print(current_velocity[1]);
    Serial.print(" RPM,  Z: ");
    Serial.print(current_velocity[2]);
    Serial.print(" RPM");
    Serial.println("");
   }
  
  countNum = countNum + 1;

}


