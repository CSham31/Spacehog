#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 64
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  //"left_motor", "right_motor"
  //"arm_servo", "box_servo"
  Motor *wheels[4];
  char wheels_names[4][15] = {"left_motor", "right_motor","arm_servo", "box_servo"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  int avoidObstacleCounter = 0;
  while (robot->step(TIME_STEP) != -1) {
  avoidObstacleCounter++;
    if (avoidObstacleCounter > 30) {
      break;
    } 

    wheels[0]->setVelocity(2);
    wheels[1]->setVelocity(2);

  }
   wheels[0]->setVelocity(0);
   wheels[1]->setVelocity(0);
  
  avoidObstacleCounter = 0;
  while (robot->step(TIME_STEP) != -1) {
  avoidObstacleCounter++;
    if (avoidObstacleCounter > 40) {
      break;
    } 

    wheels[2]->setVelocity(-0.5);

  }
  wheels[2]->setVelocity(0);
  
  
  avoidObstacleCounter = 0;
  while (robot->step(TIME_STEP) != -1) {
  avoidObstacleCounter++;
    if (avoidObstacleCounter > 65) {
      break;
    } 

    wheels[3]->setVelocity(-0.5);

  }
  wheels[3]->setVelocity(0);
  
  
  avoidObstacleCounter = 0;
  while (robot->step(TIME_STEP) != -1) {
  avoidObstacleCounter++;
    if (avoidObstacleCounter > 40) {
      break;
    } 

    wheels[2]->setVelocity(0.5);

  }
  wheels[2]->setVelocity(0);
    
  delete robot;
  return 0;  // EXIT_SUCCESS
}