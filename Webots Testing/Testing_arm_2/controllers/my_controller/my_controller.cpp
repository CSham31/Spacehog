#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <bits/stdc++.h>

#define TIME_STEP 64
using namespace webots;
using namespace std;
int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Motor *wheels;

  wheels = robot->getMotor("motor");
  wheels->setPosition(INFINITY);
  wheels->setVelocity(0.0);
  cout<<"here"<<endl;
  int avoidObstacleCounter = 0;
  while (robot->step(TIME_STEP) != -1) {
    avoidObstacleCounter++;
    if (avoidObstacleCounter > 10) {
      break;
      }
      cout<<"done"<<endl;
    wheels->setVelocity(0.5);

  }
  wheels->setVelocity(0.0);
  cout<<"done"<<endl;
  
  
  delete robot;
  return 0;  // EXIT_SUCCESS
}