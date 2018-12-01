#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "StandardCplusplus.h"
#include <vector>
#include <algorithm>
#include <stdlib.h>

using namespace std;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Inputs
const int threshold = 15; // velocity threshold of when to fire nozzles, rpm
int num_readings = 2; // number of velocity readings that have to consistently be above threshold to fire

int axis = 3; // x=1, y=2, z=3
int timestep = 100; // time between velocity readings, in milliseconds
double displayTime = 0.1;
int countNum = 0;
//

// pins
int pos_thrust_pin = 4;      // for now ccw
int neg_thrust_pin = 2;
int pos_thrust_pin_ground = 5;
int neg_thrust_pin_ground = 3; 


// initializations
vector <vector<double> > history_velocity; // vector of vectors (# of readings x 3) of all velocity readings
vector <vector<bool> > history_nozzle;
bool  positive_thruster, negative_thruster;

double velocity_difference; // difference between average velocity and threshold
double average_velocity; // average velocity over num_readings
double sum_velocity; // sum of velocity over num_readings_for_integral
double delay_time; // duration nozzles are open for
//


double toRPM(double degPerSecond)
{
  return (degPerSecond*60/(360));
}

double toRPM_radians(double radPerSecond)
{
  return (radPerSecond*60/(2*3.14159));
}

double myAbs(double input){
  if (input < 0){
    return -input;
  }
  else{
    return input;
  }
}

void controlNozzles(vector<vector<double> > &velocityVector, vector<vector<bool> > &nozzleVector, int axis)
{

  vector<bool> current_nozzle = nozzleVector.back();
  bool next_nozzle[2];
  
  vector<double> current_velocity;
  int state(1); // state = 0 for v < -thres, = 2 v > thresh
  
  int   N = velocityVector.size();
  average_velocity = 0;
  for (int i=0; i < num_readings; ++i)
  {    
    current_velocity.push_back(velocityVector[N-1-i][axis-1]);
    average_velocity += velocityVector[N-1-i][axis-1];
  }
  average_velocity = average_velocity/num_readings;

  sum_velocity = 0;
  for (int i=0; i < num_readings_for_integral; ++i)
  {    
    current_velocity.push_back(velocityVector[N-1-i][axis-1]);
    sum_velocity += velocityVector[N-1-i][axis-1];
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
      velocity_difference = myAbs(average_velocity) - threshold;
      OpenPositiveNozzle(velocity_difference,myAbs(sum_velocity), axis);
    }

    if(next_nozzle[1]){ 
      velocity_difference = myAbs(average_velocity) - threshold;
      OpenNegativeNozzle(velocity_difference,myAbs(sum_velocity), axis);
    }
    
}

void OpenPositiveNozzle(double velocity_difference, double velocity_integral, int axis){
  delay_time = 50 + velocity_difference*K_p + velocity_integral*K_i;
  Serial.println("       Opening Positive Nozzle");
  digitalWrite(pos_thrust_pin, HIGH);
  delay(100);
  digitalWrite(pos_thrust_pin, LOW);

    vector <double> current_velocity_local; 
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  current_velocity_local.push_back( toRPM((double)euler.x()) );
  current_velocity_local.push_back( toRPM((double)euler.y()) );
  current_velocity_local.push_back( toRPM((double)euler.z()) );
  
 while (myAbs(current_velocity_local[axis-1]) > threshold)
 {
  delay(50);
 }
  digitalWrite(pos_thrust_pin_ground, HIGH);
  delay(100);
  digitalWrite(pos_thrust_pin_ground, LOW);  
}

void OpenNegativeNozzle(double velocity_difference, double velocity_integral, int axis){
  Serial.println("       Opening Negative Nozzle");
  delay_time = 50 + velocity_difference*K_p + velocity_integral*K_i;
  digitalWrite(neg_thrust_pin, HIGH);
  delay(delay_time);
  digitalWrite(neg_thrust_pin, LOW);

  vector <double> current_velocity_local; 
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  current_velocity_local.push_back( toRPM((double)euler.x()) );
  current_velocity_local.push_back( toRPM((double)euler.y()) );
  current_velocity_local.push_back( toRPM((double)euler.z()) );
  
 while (myAbs(current_velocity_local[axis-1]) > threshold){
  delay(50);
 }
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

  digitalWrite(neg_thrust_pin, LOW);
  digitalWrite(pos_thrust_pin, LOW);
  digitalWrite(neg_thrust_pin_ground, HIGH);
  digitalWrite(pos_thrust_pin_ground, HIGH);
//  delay(100);
//  digitalWrite(neg_thrust_pin_ground, LOW);
//  digitalWrite(pos_thrust_pin_ground, LOW);
  

  delay(10000);
  Serial.println("10 Seconds");
  delay(10000);

  
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
//  current_velocity.push_back( ((double)toRPM(euler.x())/16) );
//  current_velocity.push_back( ((double)toRPM(euler.y())/16) );
//  current_velocity.push_back( ((double)toRPM(euler.z())/16) );
//  current_velocity.push_back( toRPM_radians((double)euler.x()) );
//  current_velocity.push_back( toRPM_radians((double)euler.y()) );
//  current_velocity.push_back( toRPM_radians((double)euler.z()) );
  current_velocity.push_back( toRPM((double)euler.x()) );
  current_velocity.push_back( toRPM((double)euler.y()) );
  current_velocity.push_back( toRPM((double)euler.z()) );
  
  if (countNum > 2*num_readings+1) {
    history_velocity.erase(history_velocity.begin());
    history_nozzle.erase(history_nozzle.begin());

  }
  history_velocity.push_back(current_velocity);
  if (countNum > num_readings_for_integral+1) {
    controlNozzles(history_velocity,history_nozzle, axis); // pass by reference
  }
  delay(timestep);

//  if (countNum % (int(displayTime/(0.001*timestep))) == 0 || current_velocity[2] < -threshold || current_velocity[2] > threshold )
  if (countNum % (int(displayTime/(0.001*timestep))) == 0 )
   {
    //Serial.print(countNum);
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
//  Serial.print("Count:");
//  Serial.println(countNum);

}


