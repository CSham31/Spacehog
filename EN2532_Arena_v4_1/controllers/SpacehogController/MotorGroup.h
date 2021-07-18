#include <webots/Motor.hpp>


using namespace webots;

#ifndef MOTORGROUP_H
#define MOTORGROUP_H
class LineFollower;

class MotorGroup
{
public:
    MotorGroup();

    void initialize(LineFollower* follower);
    void init_motors(LineFollower* follower);

    void enable_motor_velocity_control();
    void set_velocity(float leftSpeed, float rightSpeed);
    void set_position(double leftPosition, double rightPosition);
    void set_control_pid(float val1, float val2,float val3);

    void set_velocity_servo(int servo, float speed);
    void set_position_servo(int servo, double position);

private:
    LineFollower *follower;
    Motor *wheels[4];
    char wheels_names[4][12] = {"motor_left", "motor_right", "arm_servo", "box_servo"};
};

#endif