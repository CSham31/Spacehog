#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include "SensorGroup.h"
#include "MotorGroup.h"

using namespace webots;

#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#ifdef __cplusplus
extern "C"
{
#endif

    enum State
    {
        INITIALIZE,
        LINEFOLLOW_INITIAL,
        LINEFOLLOW,
        LINEFOLLOW_END,
        WALLFOLLOW,
        SQUARE,
        TURN_RIGHT,
        TURN_LEFT,
        CAL_FACTORS,
        FIND_DESTINATION,
    };

    class LineFollower : public Robot
    {
    public:
        LineFollower();

        void update_state();
        void travel_maze();
        void task();
        void test();
        void circular_path_task();
        void grab_box_detect_color();
        void circular_path_middle_task();

        void passive_wait(double targetLeft, double targetRight);
        void passive_wait_servo(int servo, double target);
        void passive_wait_curve_path(double targetLeft, double targetRight);

        void follow_line(float Kp, float Kd);
        void follow_line_until_junc_detect();
        void follow_line_until_wall_detect();
        void follow_line_until_box_detect();
        void follow_line_striaght();
        void follow_line_initial_phase();
        void follow_line_end_phase();
        void follow_line_middle_phase();
        
        
        void complete_turn(int dir, bool goForward = true);
        void go_forward_specific_distance(double distance);
        void go_forward_specific_distance_curve(double distance, int dir);

        void navigate_wall_maze();
        void follow_both_walls(float Kp, float Kd, float threshold);
        void follow_wall_until_line_detect();

        void set_servo(int state);

        void find_destination();
        void find_factors(int n);



    private:
        SensorGroup *sensorGroup;
        MotorGroup *motorGroup;

        State currentState = INITIALIZE;
        int currentInst = 0;
        State order[12] = {SQUARE, SQUARE, TURN_RIGHT, TURN_LEFT, WALLFOLLOW, TURN_LEFT, TURN_RIGHT, FIND_DESTINATION};

        int BLACK = 0;
        int WHITE = 1;

        int LEFT = 0;
        int RIGHT = 1;
        int BACK = 2;
        int FRONT = 4;
        int ARM = 2;    //index of arm servo
        int BOX = 3;    //index of box servo

        float leftSpeed;
        float rightSpeed;


        float MAX_VELOCITY = 7.5;
        float MIN_VELOCITY = 2.5;
         
        int DS_SENSOR_FRONT = 2;
        int DS_SENSOR_RIGHT = 1;
        int DS_SENSOR_LEFT = 0;
        int TOF_RIGHT = 5;
        int TOF_LEFT = 4;

        float rightIRVal = 0;
        float leftIRVal = 0;

        float TURN90_EN_COUNT = 5.44;
        float TURN180_EN_COUNT = 10.4;


        float WALL_FOLLOW_VERTICAL_SIDE_THRESHOLD = 63.5;
        float WALL_FOLLOW_HORIZONTAL_SIDE_THRESHOLD = 80.0;
        float WALL_FOLLOW_BASE_SPEED = 5.0;
        float wallFollowErrorThreshold = WALL_FOLLOW_VERTICAL_SIDE_THRESHOLD;
        int wallJuncCount = 0;

        float WALL_FOLLOW_VERTICAL_FRONT_THRESHOLD = 75.0;
        float WALL_FOLLOW_HORIZONTAL_FRONT_THRESHOLD = 60.0;
        float frontWallThreshold = WALL_FOLLOW_VERTICAL_FRONT_THRESHOLD;

        float wallFollowPreviousError = 0.0;
        float lineFollowPreviousError = 0.0;

        int colorPatch = -1;
        int detectedSquares = 1;
        int finalPosition = -1;
        bool factorsFound = false;
        bool colorPrinted = false;
        int finalPhase = 50;

        int POS_ARM_UP = 0;
        int POS_ARM_DOWN = 1;
        int POS_ARM_DEFAULT = 2;
        int POS_BOX_UP = 3;
        int POS_BOX_DOWN = 4;
        int POS_BOX_DEFAULT = 5;
        float servoPosition[6] = {0.8,-0.59,0.0,1.6,-0.1,0.0};

        bool farBoxDetected = false;
        bool nearBoxDetected = false;
        bool box_detected = false;


    };
#ifdef __cplusplus
}
#endif

#endif

