#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 64
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  //"left_motor", "right_motor"
  Motor *wheels[2];
  char wheels_names[2][15] = {"arm_servo", "box_servo"};
  for (int i = 0; i < 2; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  int avoidObstacleCounter = 0;
  while (robot->step(TIME_STEP) != -1) {
  avoidObstacleCounter++;
    if (avoidObstacleCounter > 45) {
      break;
    } 

    wheels[0]->setVelocity(0.3);
    wheels[1]->setVelocity(0.3);

  }
    wheels[0]->setVelocity(0);
    wheels[1]->setVelocity(0);
  delete robot;
  return 0;  // EXIT_SUCCESS
}