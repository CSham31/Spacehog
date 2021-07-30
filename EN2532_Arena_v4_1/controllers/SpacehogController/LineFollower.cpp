#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include "LineFollower.h"
#include "SensorGroup.h"
#include "MotorGroup.h"
#include <math.h>
#include <cmath>
#include <bits/stdc++.h>

#define TIME_STEP 16
using namespace webots;
using namespace std;

LineFollower::LineFollower()
{
    sensorGroup = new SensorGroup();
    motorGroup = new MotorGroup();
    sensorGroup->initialize(this);
    motorGroup->initialize(this);
}

////////////////////////////////////////////////////////// delay /////////////////////////////////////////////////////////////////////////

void LineFollower::passive_wait(double targetLeft, double targetRight)
{
    const double DELTA = 0.1;
    double dif;
    double effectiveLeft, effectiveRight;
    do
    {
        if (step(TIME_STEP) == -1)
            exit(EXIT_SUCCESS);

        effectiveLeft = sensorGroup->get_encoder_val(LEFT);
        effectiveRight = sensorGroup->get_encoder_val(RIGHT);
        dif = min(fabs(effectiveLeft - targetLeft), fabs(effectiveRight - targetRight));

    } while (dif > DELTA);
}

void LineFollower::passive_wait_servo(int servo, double target)
{
    const double DELTA = 0.01;
    double dif;
    double effective;
    do
    {
        if (step(TIME_STEP) == -1)
            exit(EXIT_SUCCESS);

        effective = sensorGroup->get_encoder_val(servo);
        dif = fabs(effective - target);             //check here again
    } while (dif > DELTA);
}

void LineFollower::passive_wait_curve_path(double targetLeft, double targetRight)
{
    const double DELTA = 0.1;
    double dif;
    double effectiveLeft, effectiveRight;
    do
    {
        if (step(TIME_STEP) == -1)
            exit(EXIT_SUCCESS);

        if ((sensorGroup->is_box_detected(LEFT) == 1) && (box_detected == false)) //secondition is to ensure that same box dont get detected multiple times
        {
            nearBoxDetected = true;
            box_detected = true;
        }
        else if ((sensorGroup->is_box_detected(LEFT) == 2) && (box_detected == false))
        {
            farBoxDetected = true;
            box_detected = true;
        }


        effectiveLeft = sensorGroup->get_encoder_val(LEFT);
        effectiveRight = sensorGroup->get_encoder_val(RIGHT);
        dif = min(fabs(effectiveLeft - targetLeft), fabs(effectiveRight - targetRight));

    } while (dif > DELTA);
}

void LineFollower::delay(int time)
{
    int target = time / TIME_STEP;
    int count = 0;
    while (step(TIME_STEP) != -1){
        count += 1;
        if (count > target)
            break;

    }
}

////////////////////////////////////////////////////////// follow line methods /////////////////////////////////////////////////////////////////////////

void LineFollower::follow_line(float Kp, float Kd, float minSpd, float baseSpd, float maxSpd)
{
    int position = sensorGroup->qtr_read_line();
    int error = position - 3500;

    double controlValue = (error * Kp) + (error - wallFollowPreviousError) * Kd;

    lineFollowPreviousError = error;

    rightSpeed = baseSpd - controlValue;
    leftSpeed = baseSpd + controlValue;

    if (rightSpeed < minSpd)
        rightSpeed = minSpd;
    else if (rightSpeed > maxSpd)
        rightSpeed = maxSpd;

    if (leftSpeed < minSpd)
        leftSpeed = minSpd;
    else if (leftSpeed > maxSpd)
        leftSpeed = maxSpd;

    motorGroup->set_velocity(leftSpeed, rightSpeed);
}

void LineFollower::follow_line_until_junc_detect_fast()
{
    int count = DEACCELERATE_COUNT;
    while (step(TIME_STEP) != -1)
    {
        if(sensorGroup->is_junction_detected() == true)
            break;
        if (count>-1)
        {
            baseSpeed = CONST_BASE_SPEED - count;
            count-=0.5;
        }
        follow_line(0.001,0.0,7.0,baseSpeed,17.0);
    }

}

void LineFollower::follow_line_until_junc_detect_slow()
{
    while (step(TIME_STEP) != -1)
    {
        if(sensorGroup->is_junction_detected() == true)
            break;
        follow_line(0.01,0.0,2.5,5.0,7.5);
    }

}

void LineFollower::follow_line_until_segment_detect()
{
    while (step(TIME_STEP) != -1)
    {
        if(sensorGroup->is_line_segment_detected() == true)
            break;
        follow_line(0.01,0.0,2.5,5.0,7.5);
    }

}

void LineFollower::follow_line_and_count_pillars()
{
    bool pillarDetected = false;

    while (step(TIME_STEP) != -1)
    {
        if(sensorGroup->is_junction_detected() == true)
        {
            cout<<"PILLAR COUNT : "<<pillarCount<<endl;
            break;
        }

        if (sensorGroup->is_pillar_detected(rampDirection) == true)
        {
            pillarDetected = true;
            //cout<<pillarDetected<<endl;
        }

        if ((pillarDetected == true) and (sensorGroup->is_pillar_detected(rampDirection) == 0 ))
        {
            pillarCount += 1;
            pillarDetected = false;
            //cout<<"detected"<<endl;
        }
        
        follow_line(0.01,0.0,2.5,5.0,7.5);
    }

}

void LineFollower::follow_line_until_wall_detect()
{
    int count = DEACCELERATE_COUNT;
    while (step(TIME_STEP) != -1)
    {
        if(sensorGroup->is_wall_entrance() == true)
            break;
        if (count>-1)
        {
            baseSpeed = CONST_BASE_SPEED - count;
            count-=1;
        }
        follow_line(0.01,0.0,2.5,5.0,7.5);
    }
}

void LineFollower::follow_line_until_box_detect()
{
    int count = DEACCELERATE_COUNT;
    while (step(TIME_STEP) != -1)
    {
        if(sensorGroup->get_generic_value(DS_SENSOR_FRONT) < 55 )
            break;
        if (count>-1)
        {
            baseSpeed = CONST_BASE_SPEED - count;
            count-=1;
        }
        follow_line(0.001,0.0,7.0,12.0,17.0);
    }
    motorGroup->set_velocity(0, 0);
}

void LineFollower::follow_line_first_phase()
{
    while (step(TIME_STEP) != -1)
    {
        if ((sensorGroup->is_wall_entrance() == true) and (sensorGroup->is_deadend() == true))
            break;

        if(sensorGroup->is_junction_detected() == true)
        {
            //cout<<"junc detected"<<endl;
            complete_turn(sensorGroup->nextTurn);

        }
            
        follow_line(0.01,0.0,2.5,5.0,7.5);
    }
}

void LineFollower::follow_line_second_phase()
{
    int count = DEACCELERATE_COUNT;
    while (step(TIME_STEP) != -1)
    {
        if(sensorGroup->is_junction_detected() == true)
        {
            //cout<<"junc detected"<<endl;
            step(TIME_STEP);
            step(TIME_STEP);
            //cout<<"after one step"<<endl;
            if (sensorGroup->is_line_segment_detected() == true)
            {
                isCircleDetected = true;        //not used
                break;
            }
            go_forward_specific_distance(5.45);              //go forward value is different in here as robot go two steps forward
            complete_turn(sensorGroup->nextTurn,false);
            count = DEACCELERATE_COUNT;
        }
            
        if (count>-1)
        {
            baseSpeed = CONST_BASE_SPEED - count;
            count-=0.5;
        }
        follow_line(0.001,0.0,7.0,baseSpeed,17.0);
    }
}

////////////////////////////////////////////////////////// turns /////////////////////////////////////////////////////////////

void LineFollower::complete_turn(int dir, bool goForward)
{
    if (goForward == true)
    {
        go_forward_specific_distance(5.8);
    }

    sensorGroup->stabilize_encoder(this);

    int sign = 1;
    if (dir == LEFT)
        sign = -1;

    double initialLeftENcount = sensorGroup->get_encoder_val(LEFT);
    double initialRightENcount = sensorGroup->get_encoder_val(RIGHT);

    double leftCount = initialLeftENcount + (sign * TURN90_EN_COUNT);
    double rightCount = initialRightENcount - (sign * TURN90_EN_COUNT);

    if (dir == BACK)
    {
        leftCount = initialLeftENcount + TURN180_EN_COUNT;
        rightCount = initialRightENcount - TURN180_EN_COUNT;
    }

    motorGroup->set_control_pid(4.5, 0, 0);
    motorGroup->set_velocity(10, 10);

    motorGroup->set_position(leftCount, rightCount);

    passive_wait(leftCount, rightCount);

    motorGroup->enable_motor_velocity_control();
}

void LineFollower::go_forward_specific_distance(double distance)
{

    sensorGroup->stabilize_encoder(this);

    double initialLeftENcount = sensorGroup->get_encoder_val(LEFT);
    double initialRightENcount = sensorGroup->get_encoder_val(RIGHT);

    motorGroup->set_control_pid(8, 0, 0);
    motorGroup->set_velocity(7.5, 7.5);

    motorGroup->set_position(distance + initialLeftENcount, distance + initialRightENcount);
    passive_wait(distance + initialLeftENcount, distance + initialRightENcount);
    motorGroup->enable_motor_velocity_control();
}

void LineFollower::go_forward_specific_distance_curve(double distance, int dir)
{

    sensorGroup->stabilize_encoder(this);

    double initialLeftENcount = sensorGroup->get_encoder_val(LEFT);
    double initialRightENcount = sensorGroup->get_encoder_val(RIGHT);

    
    if (dir == LEFT)
    {
        motorGroup->set_control_pid(4.5, 0, 0);
        motorGroup->set_velocity(6.0, 7.5);
        motorGroup->set_position(distance-1.5 + initialLeftENcount, distance + initialRightENcount);
        passive_wait_curve_path(distance-1.5 + initialLeftENcount, distance + initialRightENcount);
    }
    else{
        motorGroup->set_control_pid(4.5, 0, 0);
        motorGroup->set_velocity(7.5, 6.0);
        motorGroup->set_position(distance + initialLeftENcount, distance-2.5 + initialRightENcount);
        passive_wait_curve_path(distance + initialLeftENcount, distance-2.5 + initialRightENcount);
    }
    motorGroup->enable_motor_velocity_control();
}

////////////////////////////////////////////////////////// wall follow /////////////////////////////////////////////////////////////

void LineFollower::follow_both_walls(float Kp, float Kd, float threshold)
{
    double error = 0;

    if (sensorGroup->is_wall(LEFT))
    {
        error = threshold - round(sensorGroup->get_distance_value(DS_SENSOR_LEFT));
        //cout<<"following left wall"<<endl;
    }
    else if (sensorGroup->is_wall(RIGHT))
    {
        error = round(sensorGroup->get_distance_value(DS_SENSOR_RIGHT)) - threshold;
    }
    else
    {
        error = 0;
    }

    double controlValue = (error * Kp) + (error - wallFollowPreviousError) * Kd;

    //cout<<controlValue<<endl;

    wallFollowPreviousError = error;

    // leftSpeed = WALL_MAX_VELOCITY;
    // rightSpeed = WALL_MAX_VELOCITY;

    rightSpeed = WALL_FOLLOW_BASE_SPEED - controlValue;
    leftSpeed = WALL_FOLLOW_BASE_SPEED + controlValue;

    if (rightSpeed < WALL_MIN_VELOCITY)
        rightSpeed = WALL_MIN_VELOCITY;
    else if (rightSpeed > WALL_MAX_VELOCITY)
        rightSpeed = WALL_MAX_VELOCITY;

    if (leftSpeed < WALL_MIN_VELOCITY)
        leftSpeed = WALL_MIN_VELOCITY;
    else if (leftSpeed > WALL_MAX_VELOCITY)
        leftSpeed = WALL_MAX_VELOCITY;

    //cout<<leftSpeed<<"  "<<rightSpeed<<endl;
    motorGroup->set_velocity(leftSpeed, rightSpeed);
}

void LineFollower::follow_wall_until_line_detect()
{
    while (step(8) != -1)
    {
        if (sensorGroup->is_wall_exit() == true)               //( sensorGroup->qtr_read_line() > 0 ||  sensorGroup->qtr_read_line() < 7000)
            break;
        follow_both_walls(0.023,0.06,100);
    }
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LineFollower::navigate_gates()
{
    //int initialTime = 0
    bool gateOpen = false;

    go_forward_specific_distance(4.0);  

    while (step(TIME_STEP) != -1)
    {
        if (sensorGroup->is_gate_detected(DS_SENSOR_FRONT) == true)
        {
            cout<<"first detected"<<endl;
            //initialTime = getTime();
            while (step(TIME_STEP) != -1)
            {
                // if ((getTime()- initialTime) > 3)
                //     break                   //means gate was closing
                if (sensorGroup->is_gate_detected(DS_SENSOR_BOX) == true)
                {
                    gateOpen = true;
                    cout<<"second detected"<<endl;
                    break;
                }
                    
            }
        }
        if (gateOpen == true)
        {
            gateOpen = false;
            break;
        }
    }
    delay(2000);
    follow_line_until_segment_detect();
    go_forward_specific_distance(3.0);  //to pass the line segment
    follow_line_until_junc_detect_slow();
    go_forward_specific_distance(7.5);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LineFollower::set_servo(int state, bool wait)
{
    if (state <3)       //arm servo
    {
        motorGroup->set_control_pid(6, 0, 0);
        motorGroup->set_velocity_servo(ARM, 2);
        motorGroup->set_position_servo(ARM, servoPosition[state]);
        if (wait == true)
            passive_wait_servo(ARM, servoPosition[state]);
    }
    else
    {
        motorGroup->set_control_pid(6, 0, 0);
        motorGroup->set_velocity_servo(BOX, 1.5);
        motorGroup->set_position_servo(BOX, servoPosition[state]);
        if (wait == true)
            passive_wait_servo(BOX, servoPosition[state]);
    }
}

//////////////////////////////////////////////////////////// main //////////////////////////////////////////////////////////////////////

void LineFollower::determine_direction()
{
    if ((abs(frontFaceColour - bottomFaceColour) % 2) == 1)
        rampDirection = RIGHT;
    else
        rampDirection = LEFT;

    cout<<"FORNT COLOUR : ";
    sensorGroup->print_color_patch(frontFaceColour);
    cout<<"BOTTOM COLOUR : ";
    sensorGroup->print_color_patch(bottomFaceColour);

    if (rampDirection == LEFT)
        cout<<"RAMP DIRECTION : LEFT"<<endl;
    else
        cout<<"RAMP DIRECTION : RIGHT"<<endl;
}

void LineFollower::grab_box_detect_color()
{
    set_servo(POS_ARM_DOWN);
    if (farBoxDetected == true){
        frontFaceColour = sensorGroup->get_colour(CAM_BACK);
    }
    set_servo(POS_BOX_DOWN);
    if (nearBoxDetected == true){
        frontFaceColour = sensorGroup->get_colour(CAM_FRONT);
    }

    set_servo(POS_ARM_UP);
    bottomFaceColour = sensorGroup->get_colour(CAM_ARM);
    sensorGroup->set_LED(LEFT,frontFaceColour);
    sensorGroup->set_LED(RIGHT,bottomFaceColour);
    determine_direction();
}

void LineFollower::circular_path_middle_task()
{
    complete_turn(LEFT,false);  // false to stop robot going specific distance forward
    set_servo(POS_BOX_UP,false);

    if (nearBoxDetected == true)
    {
        follow_line_until_box_detect();
        grab_box_detect_color();
    }

    follow_line_until_junc_detect_fast();
    go_forward_specific_distance(3.0); //to pass the middle cross
    
    if (farBoxDetected == true)
    {
        follow_line_until_box_detect();
        grab_box_detect_color();
    }

    follow_line_until_junc_detect_fast();
    go_forward_specific_distance(5.9);
    set_servo(POS_BOX_UP);          //release the box      
}

void LineFollower::circular_path_task()
{
    complete_turn(RIGHT);
    cout<<"QUADRANT NO. 1"<<endl;
    follow_line_until_junc_detect_slow();
    go_forward_specific_distance_curve(6.9,LEFT);
    if (box_detected == true)
    {
        circular_path_middle_task();
        complete_turn(LEFT,false);
        cout<<"QUADRANT NO. 2"<<endl;
        follow_line_until_junc_detect_slow();
        go_forward_specific_distance_curve(6.15,LEFT);        //change this to left
        complete_turn(RIGHT,false);
    }
    else                         //this case we assume that box is in the second cross line for sure
    {
        cout<<"QUADRANT NO. 4"<<endl;
        follow_line_until_junc_detect_slow();
        go_forward_specific_distance_curve(6.9,LEFT);
        circular_path_middle_task();
        complete_turn(RIGHT,false);
        cout<<"QUADRANT NO. 2"<<endl;
        follow_line_until_junc_detect_slow();
        go_forward_specific_distance_curve(6.15,RIGHT);        //change this to left
        complete_turn(LEFT,false);
    }
    cout<<"OUT OF THE CIRCLE"<<endl;
    set_servo(POS_BOX_DEFAULT,false); 
}

void LineFollower::task()
{
    // go_forward_specific_distance(3.0);
    // follow_line_until_junc_detect_fast();
    // complete_turn(sensorGroup->nextTurn);
    // follow_line_until_wall_detect(); 
    // follow_wall_until_line_detect();        //wall following
    // follow_line_until_junc_detect_fast();
    // complete_turn(sensorGroup->nextTurn);
    // follow_line_until_junc_detect_fast();
    // complete_turn(sensorGroup->nextTurn);
    // follow_line_until_junc_detect_fast();

    go_forward_specific_distance(3.0);
    follow_line_first_phase();
    follow_wall_until_line_detect();        //wall following
    follow_line_second_phase();
    circular_path_task();                   //entered to the circle
    follow_line_until_junc_detect_slow();
    complete_turn(rampDirection);    //on top of ramp
    follow_line_and_count_pillars();
    if (pillarCount % 2 != 0)
    {
        complete_turn(BACK,false);
        follow_line_until_junc_detect_fast();
        go_forward_specific_distance(3.0);
        follow_line_until_junc_detect_fast();
        if (rampDirection == LEFT)
            complete_turn(RIGHT);
        else
            complete_turn(LEFT);
    }else{
        complete_turn(rampDirection);
    }
    follow_line_until_segment_detect();
    navigate_gates();
    sensorGroup->set_LED(LEFT,0);
    sensorGroup->set_LED(RIGHT,0);
    cout<<"TASK COMPLETED"<<endl;

}

void LineFollower::test()
{
    //test - distance sensors
    //cout<<sensorGroup->get_distance_value(DS_SENSOR_LEFT)<<"    "<<sensorGroup->get_distance_value(DS_SENSOR_RIGHT)<<endl;

    //test - turns
    // follow_line_until_junc_detect_fast();
    // complete_turn(RIGHT);

    //test - gate navigate
    // follow_line_until_segment_detect();
    // navigate_gates();

    //test - line follow
    //follow_line(0.001,0.0,7.0,baseSpeed,17.0);

    //follow_line_second_phase();
    follow_line_first_phase();
    cout<<"out"<<endl;
}
