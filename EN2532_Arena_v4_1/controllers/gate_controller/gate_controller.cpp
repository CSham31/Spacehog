#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 16
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Motor *wheels[4];
  char wheels_names[4][8] = {"gate_1", "gate_2", "gate_3", "gate_4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    // wheels[i]->setPosition(INFINITY);
    // wheels[i]->setVelocity(0.0);
  }
  //int avoidObstacleCounter = 0;
  while (robot->step(TIME_STEP) != -1) {
    // double leftSpeed = 1.0;
    // double rightSpeed = 1.0;
    // if (avoidObstacleCounter > 0) {
    //   avoidObstacleCounter--;
    //   leftSpeed = 1.0;
    //   rightSpeed = -1.0;
    // } else { // read sensors
    //   for (int i = 0; i < 2; i++) {
    //     if (ds[i]->getValue() < 950.0)
    //       avoidObstacleCounter = 100;
    //   }
    // }


    wheels[0]->setPosition(-1.57);
    wheels[1]->setPosition(-1.57);
    wheels[2]->setPosition(-1.57);
    wheels[3]->setPosition(-1.57);

    wheels[0]->setVelocity(0.5);
    wheels[1]->setVelocity(0.5);
    wheels[2]->setVelocity(0.5);
    wheels[3]->setVelocity(0.5);
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}