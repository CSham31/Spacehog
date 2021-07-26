#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 16
using namespace webots;
using namespace std;

float gateVelocity = 1.0;
float UP = -1.57;
float DOWN = 0;

int TIME_PERIOD = 20;
int currentTime = 0;
int unitTime = 0;

int offset = 1; //keep this below 7

Robot *robot = new Robot();
Motor *wheels[4];

void initialize_gates()
{
  char wheels_names[4][8] = {"gate_1", "gate_2", "gate_3", "gate_4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
  }
}

void first_gte(float dir)
{
  wheels[0]->setPosition(dir);
  wheels[2]->setPosition(dir);
  wheels[0]->setVelocity(gateVelocity);
  wheels[2]->setVelocity(gateVelocity);
}

void second_gte(float dir)
{
  wheels[1]->setPosition(dir);
  wheels[3]->setPosition(dir);
  wheels[1]->setVelocity(gateVelocity);
  wheels[3]->setVelocity(gateVelocity);
}

int main(int argc, char **argv) {

  initialize_gates();

  while (robot->step(TIME_STEP) != -1) {

    currentTime = robot->getTime();
    unitTime = currentTime % TIME_PERIOD;

    if (unitTime <3+offset){
      first_gte(UP);
      second_gte(DOWN);
    }else if (unitTime < 10+offset){
      first_gte(UP);
      second_gte(UP);
    }else if (unitTime < 13+offset){
      first_gte(DOWN);
      second_gte(UP);
    }else{
      first_gte(DOWN);
      second_gte(DOWN);
    }

  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}