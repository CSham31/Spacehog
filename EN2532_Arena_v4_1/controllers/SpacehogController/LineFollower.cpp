#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include "LineFollower.h"
#include "SensorGroup.h"
#include "MotorGroup.h"
#include <math.h>
#include <bits/stdc++.h>

#define TIME_STEP 8
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
        //cout<<target<<endl;
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

        if ((sensorGroup->is_pillar_detected(LEFT) == 1) && (box_detected == false)) //secondition is to ensure that same box dont get detected multiple times
        {
            nearBoxDetected = true;
            box_detected = true;
        }
        else if ((sensorGroup->is_pillar_detected(LEFT) == 2) && (box_detected == false))
        {
            farBoxDetected = true;
            box_detected = true;
        }


        effectiveLeft = sensorGroup->get_encoder_val(LEFT);
        effectiveRight = sensorGroup->get_encoder_val(RIGHT);
        dif = min(fabs(effectiveLeft - targetLeft), fabs(effectiveRight - targetRight));

    } while (dif > DELTA);
}

////////////////////////////////////////////////////////// follow line methods /////////////////////////////////////////////////////////////////////////

void LineFollower::follow_line(float Kp, float Kd)
{
    // leftSpeed = 7.5;
    // rightSpeed = 7.5;

    int position = sensorGroup->qtr_read_line();
    int error = position - 3500;

    double controlValue = (error * Kp) + (error - wallFollowPreviousError) * Kd;

    //cout<<controlValue<<endl;

    lineFollowPreviousError = error;

    // leftSpeed = MAX_VELOCITY;
    // rightSpeed = MAX_VELOCITY;

    rightSpeed = WALL_FOLLOW_BASE_SPEED - controlValue;
    leftSpeed = WALL_FOLLOW_BASE_SPEED + controlValue;

    if (rightSpeed < MIN_VELOCITY)
        rightSpeed = MIN_VELOCITY;
    else if (rightSpeed > MAX_VELOCITY)
        rightSpeed = MAX_VELOCITY;

    if (leftSpeed < MIN_VELOCITY)
        leftSpeed = MIN_VELOCITY;
    else if (leftSpeed > MAX_VELOCITY)
        leftSpeed = MAX_VELOCITY;

    motorGroup->set_velocity(leftSpeed, rightSpeed);
}

void LineFollower::follow_line_until_junc_detect()
{
    while (step(8) != -1)
    {
        if(sensorGroup->is_junction_detected() == true)
            break;
        follow_line(0.05,0.0);
    }
}

void LineFollower::follow_line_until_wall_detect()
{
    while (step(8) != -1)
    {
        if(sensorGroup->is_wall_entrance() == true)
            break;
        follow_line(0.05,0.0);
    }
}

void LineFollower::follow_line_until_box_detect()
{
    while (step(8) != -1)
    {
        if(sensorGroup->get_distance_value(DS_SENSOR_FRONT) < 60 )
            break;
        follow_line(0.05,0.0);
    }
    motorGroup->set_velocity(0, 0);
}

void LineFollower::follow_line_striaght()
{
    leftSpeed = 7.5;
    rightSpeed = 7.5;

    leftIRVal = sensorGroup->get_ir_value(0);
    rightIRVal = sensorGroup->get_ir_value(1);

    if (sensorGroup->get_digital_value(0) == WHITE)
    {
        leftSpeed = 2.5;
        rightSpeed = 3.5;
    }
    else if (sensorGroup->get_digital_value(1) == WHITE)
    {
        leftSpeed = 3.5;
        rightSpeed = 2.5;
    }

    motorGroup->set_velocity(leftSpeed, rightSpeed);
}

void LineFollower::follow_line_initial_phase()
{
    follow_line_striaght();

    int color = sensorGroup->detect_color_patch();

    if (((colorPatch != -1 && color == 4) || sensorGroup->get_encoder_val(0) > 155) && !colorPrinted)
    {
        sensorGroup->print_color_patch();
        colorPrinted = true;
    }

    if (color < 4)
    {
        colorPatch = color;
        if (colorPatch != 1)
        {
            currentInst = 3;
        }
    }

    bool junctionAhead = sensorGroup->is_junction_detected();

    if (junctionAhead)
    {
        if (!colorPrinted)
        {
            sensorGroup->print_color_patch();
            colorPrinted = true;
        }
        update_state();
    }
}

void LineFollower::follow_line_middle_phase()
{
    follow_line_striaght();

    bool wallAhead = sensorGroup->is_wall_entrance();
    bool junctionAhead = sensorGroup->is_junction_detected();

    if (wallAhead || junctionAhead)
    {
        update_state();
        if (junctionAhead)
        {
            go_forward_specific_distance(3.0);
        }
    }
}

void LineFollower::follow_line_end_phase()
{
    follow_line_striaght();

    bool junctionAhead = sensorGroup->is_junction_detected();

    if (junctionAhead || sensorGroup->detect_color_patch() != colorPatch)
    {
        if (junctionAhead)
        {
            go_forward_specific_distance(6);
        }
        else
        {
            go_forward_specific_distance(0.4);
        }
        motorGroup->set_velocity(0, 0);
        update_state();
    }
}

////////////////////////////////////////////////////////// turns /////////////////////////////////////////////////////////////

void LineFollower::complete_turn(int dir, bool goForward)
{
    if (goForward == true)
    {
        go_forward_specific_distance(5.9);
        cout<<"forward"<<endl;
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
    motorGroup->set_velocity(7.5, 7.5);

    motorGroup->set_position(leftCount, rightCount);

    passive_wait(leftCount, rightCount);

    motorGroup->enable_motor_velocity_control();
}

void LineFollower::go_forward_specific_distance(double distance)
{

    sensorGroup->stabilize_encoder(this);

    double initialLeftENcount = sensorGroup->get_encoder_val(LEFT);
    double initialRightENcount = sensorGroup->get_encoder_val(RIGHT);

    motorGroup->set_control_pid(4.5, 0, 0);
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

    motorGroup->set_control_pid(4.5, 0, 0);
    motorGroup->set_velocity(7.5, 7.5);
    if (dir == LEFT)
    {
        motorGroup->set_position(distance-2.5 + initialLeftENcount, distance + initialRightENcount);
        passive_wait_curve_path(distance-2.5 + initialLeftENcount, distance + initialRightENcount);
    }
    else{
        motorGroup->set_position(distance + initialLeftENcount, distance-2.5 + initialRightENcount);
        passive_wait_curve_path(distance + initialLeftENcount, distance-2.5 + initialRightENcount);
    }
    motorGroup->enable_motor_velocity_control();
}

////////////////////////////////////////////////////////// wall follow /////////////////////////////////////////////////////////////

void LineFollower::navigate_wall_maze()
{
    bool skip = false;
    if (sensorGroup->is_wall_exit() == true)
    {
        skip = true;
        update_state(); 
    }
    if (!skip)
    {
        if ((sensorGroup->get_distance_value(DS_SENSOR_FRONT) < frontWallThreshold) == true)
        {
            wallJuncCount++;
            if (wallJuncCount % 2 == 1)
            {
                wallFollowErrorThreshold = WALL_FOLLOW_HORIZONTAL_SIDE_THRESHOLD;
                frontWallThreshold = WALL_FOLLOW_HORIZONTAL_FRONT_THRESHOLD;
            }
            else
            {
                wallFollowErrorThreshold = WALL_FOLLOW_VERTICAL_SIDE_THRESHOLD;
                frontWallThreshold = WALL_FOLLOW_VERTICAL_FRONT_THRESHOLD;
            }

            if (sensorGroup->is_wall(RIGHT) == false)
                complete_turn(RIGHT);
            else
                complete_turn(LEFT);
        }

        follow_both_walls(0.02, 1.0, wallFollowErrorThreshold);
    }
}

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

    double controlValue = (error * Kp) + (error - wallFollowPreviousError) * Kd;

    //cout<<controlValue<<endl;

    wallFollowPreviousError = error;

    leftSpeed = MAX_VELOCITY;
    rightSpeed = MAX_VELOCITY;

    rightSpeed = WALL_FOLLOW_BASE_SPEED - controlValue;
    leftSpeed = WALL_FOLLOW_BASE_SPEED + controlValue;

    if (rightSpeed < MIN_VELOCITY)
        rightSpeed = MIN_VELOCITY;
    else if (rightSpeed > MAX_VELOCITY)
        rightSpeed = MAX_VELOCITY;

    if (leftSpeed < MIN_VELOCITY)
        leftSpeed = MIN_VELOCITY;
    else if (leftSpeed > MAX_VELOCITY)
        leftSpeed = MAX_VELOCITY;

    motorGroup->set_velocity(leftSpeed, rightSpeed);
}

void LineFollower::follow_wall_until_line_detect()
{
    while (step(8) != -1)
    {
        if (sensorGroup->is_wall_exit() == true)               //( sensorGroup->qtr_read_line() > 0 ||  sensorGroup->qtr_read_line() < 7000)
            break;
        follow_both_walls(0.005,0.1,100);
    }
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LineFollower::set_servo(int state)
{
    if (state <3)       //arm servo
    {
        motorGroup->set_control_pid(4.5, 0, 0);
        motorGroup->set_velocity_servo(ARM, 0.5);
        motorGroup->set_position_servo(ARM, servoPosition[state]);
        passive_wait_servo(ARM, servoPosition[state]);
    }
    else
    {
        motorGroup->set_control_pid(4.5, 0, 0);
        motorGroup->set_velocity_servo(BOX, 0.5);
        motorGroup->set_position_servo(BOX, servoPosition[state]);
        passive_wait_servo(BOX, servoPosition[state]);
    }
}




////////////////////////////////////////////////////////// final stage methods /////////////////////////////////////////////////////////////////////////

void LineFollower::find_destination()
{
    sensorGroup->detect_color_patches();
    finalPosition = distance(sensorGroup->COLORS, find(sensorGroup->COLORS, sensorGroup->COLORS + 3, colorPatch));

    if (finalPosition % 2 == 0)
    {
        order[8 + finalPosition / 2] = TURN_LEFT;
        order[9 - finalPosition / 2] = TURN_RIGHT;
        order[10] = CAL_FACTORS;
        finalPhase = currentInst + 3;
        ++currentInst;
    }
    else
    {
        order[8] = CAL_FACTORS;
        finalPhase = currentInst;
    }

    update_state();
}

void LineFollower::find_factors(int n)
{
    if (!factorsFound)
    {
        printf("N: %d\n", n);

        vector<int> factors;
        int i = 1;
        while (i * i <= n)
        {
            if (n % i == 0)
            {
                factors.push_back(i);
                if (n / i != i)
                {
                    factors.push_back(n / i);
                }
            }
            i++;
        }
        sort(factors.begin(), factors.end());

        cout << "Factors: " << endl;
        
        for (auto i = factors.begin(); i != factors.end(); ++i)
        {
            std::cout << *i << endl;
        }
    }
    factorsFound = true;
    exit(0);
}

//////////////////////////////////////////////////////////// main //////////////////////////////////////////////////////////////////////

void LineFollower::update_state()
{
    if (currentInst != 0)
    {
        currentInst++;
        if (currentInst % 2 == 1)
        {
            if (currentInst < 4)
            {
                currentState = LINEFOLLOW_INITIAL;
            }
            else if (currentInst > finalPhase)
            {
                currentState = LINEFOLLOW_END;
            }
            else
            {
                currentState = LINEFOLLOW;
            }
        }
        else
        {
            currentState = order[currentInst / 2 - 1];
            if (currentState == TURN_RIGHT)
                sensorGroup->enable_wall_follow();
        }
    }
    else
    {
        currentInst = 1;
        currentState = LINEFOLLOW_INITIAL;
    }
}

//maze following
void LineFollower::travel_maze()
{
    switch (currentState)
    {
    case LINEFOLLOW:
        follow_line_middle_phase();
        break;

    case INITIALIZE:
        go_forward_specific_distance(3);
        update_state();
        break;

    case LINEFOLLOW_END:
        follow_line_end_phase();
        break;

    case LINEFOLLOW_INITIAL:
        follow_line_initial_phase();
        break;

    case SQUARE:
        go_forward_specific_distance(8.6);
        update_state();
        break;

    case WALLFOLLOW:
        navigate_wall_maze();
        break;

    case TURN_RIGHT:
        complete_turn(RIGHT);
        update_state();
        break;

    case TURN_LEFT:
        complete_turn(LEFT);
        update_state();
        break;

    case FIND_DESTINATION:
        find_destination();
        break;

    case CAL_FACTORS:
        find_factors((finalPosition + 1) * 4);
        break;

    default:
        // motorGroup->set_velocity("Default case");
        break;
    }
}

void LineFollower::grab_box_detect_color()
{
    set_servo(POS_ARM_DOWN);
    set_servo(POS_BOX_DOWN);
    set_servo(POS_ARM_UP);
    //color detection code
}

void LineFollower::circular_path_middle_task()
{
    complete_turn(LEFT,false);  // false to stop robot going specific distance forward

    if (nearBoxDetected == true)
    {
        follow_line_until_box_detect();
        grab_box_detect_color();
    }

    follow_line_until_junc_detect();
    go_forward_specific_distance(3.0); //to pass the middle cross
    
    if (farBoxDetected == true)
    {
        follow_line_until_box_detect();
        grab_box_detect_color();
    }

    follow_line_until_junc_detect();
    go_forward_specific_distance(5.9);
    set_servo(POS_BOX_UP);          //release the box
}

void LineFollower::circular_path_task()
{
    complete_turn(RIGHT);
    cout<<"after turn"<<endl;
    follow_line_until_junc_detect();
    go_forward_specific_distance_curve(7.5,LEFT);
    if (box_detected == true)
    {
        circular_path_middle_task();
        complete_turn(LEFT,false);
        follow_line_until_junc_detect();
        go_forward_specific_distance_curve(7.5,LEFT);        //change this to left
        complete_turn(RIGHT,false);
    }
    else                         //this case we assume that box is in the second cross line for sure
    {
        follow_line_until_junc_detect();
        go_forward_specific_distance_curve(7.5,LEFT);
        circular_path_middle_task();
        complete_turn(RIGHT,false);
        follow_line_until_junc_detect();
        go_forward_specific_distance_curve(7.5,RIGHT);        //change this to left
        complete_turn(LEFT,false);
    }
}

void LineFollower::task()
{
    go_forward_specific_distance(2.5);
    follow_line_until_junc_detect();
    complete_turn(LEFT);
    follow_line_until_wall_detect(); 
    follow_wall_until_line_detect();
    follow_line_until_junc_detect();
    complete_turn(RIGHT);
    follow_line_until_junc_detect();
    complete_turn(RIGHT);
    follow_line_until_junc_detect();
    circular_path_task();
    follow_line_until_junc_detect();
    //cout<<"done"<<endl;
    complete_turn(LEFT);
    follow_line_until_junc_detect();
    motorGroup->robot_stop();

    // complete_turn(RIGHT);

    // follow_line_until_junc_detect();
    // complete_turn(RIGHT);
    // follow_line_until_junc_detect();
    // go_forward_specific_distance_curve(7.5);

    // follow_line_until_junc_detect();
    // circular_path_task();

    //go_forward_specific_distance_curve(7.5);

    //go_forward_specific_distance(6.0);

}

void LineFollower::test()
{
    //sensorGroup->enable_wall_follow();
    //motorGroup->set_velocity(7.0, 7.0);
    cout<<sensorGroup->get_distance_value(DS_SENSOR_FRONT)<<endl;
    //cout<<sensorGroup->is_wall(RIGHT)<<endl;
    //cout<<sensorGroup->is_pillar_detected(RIGHT)<<endl;
    //follow_both_walls(0.005,0.1,100);
    //complete_turn(RIGHT);

    // set_servo(POS_ARM_DOWN);
    // set_servo(POS_BOX_DOWN);
    // set_servo(POS_ARM_UP);

    //follow_line(0.05,0.0);

    //cout<<sensorGroup->qtr_read_line()<<endl;
}
