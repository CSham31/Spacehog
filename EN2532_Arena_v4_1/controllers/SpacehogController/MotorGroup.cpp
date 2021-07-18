#include <webots/Motor.hpp>
#include "MotorGroup.h"
#include "LineFollower.h"
#include <math.h>
#include <bits/stdc++.h>

#define TIME_STEP 32
using namespace webots;
using namespace std;

MotorGroup::MotorGroup(){}

void MotorGroup::initialize(LineFollower* follower){
  follower = follower;
  init_motors(follower);
    
}

void MotorGroup::init_motors(LineFollower* follower)
{
    for (int i = 0; i < 4; i++)
    {
        wheels[i] = follower->getMotor(wheels_names[i]);
        wheels[i]->setPosition(INFINITY);
        wheels[i]->setVelocity(0.0);
    }
}


void MotorGroup::enable_motor_velocity_control()
{
    for (int i = 0; i < 4; i++)
    {
        wheels[i]->setPosition(INFINITY);
        wheels[i]->setVelocity(0.0);
    }
}

void MotorGroup::set_velocity(float leftSpeed, float rightSpeed)
{
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
    // wheels[2]->setVelocity(leftSpeed);
    // wheels[3]->setVelocity(rightSpeed);
}

void MotorGroup::set_position(double leftPosition, double rightPosition)
{
    wheels[0]->setPosition(leftPosition); //left
    wheels[1]->setPosition(rightPosition);
    // wheels[2]->setPosition(leftPosition); //left
    // wheels[3]->setPosition(rightPosition);
}

void MotorGroup::set_velocity_servo(int servo, float speed)
{
    wheels[servo]->setVelocity(speed);
}

void MotorGroup::set_position_servo(int servo, double position)
{
    wheels[servo]->setPosition(position);

}

void MotorGroup::set_control_pid(float val1, float val2,float val3){
    for (int i = 0; i < 4; i++)
    {
        wheels[i]->setControlPID(val1, val2, val3);
    }

}

