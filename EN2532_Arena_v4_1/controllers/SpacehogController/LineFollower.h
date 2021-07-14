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
        void test();

        void passive_wait(double targetLeft, double targetRight);

        void follow_line_striaght();
        void follow_line_initial_phase();
        void follow_line_end_phase();
        void follow_line_middle_phase();
        
        
        void complete_turn(int dir);
        void go_forward_specific_distance(double distance);

        void navigate_wall_maze();
        void follow_both_walls(float Kp, float Kd, float threshold);

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

        float leftSpeed;
        float rightSpeed;


        float MAX_VELOCITY = 7.5;
        float MIN_VELOCITY = 2.5;
         
        int DS_SENSOR_FRONT = 4;
        int DS_SENSOR_RIGHT = 1;
        int DS_SENSOR_LEFT = 0;

        float rightIRVal = 0;
        float leftIRVal = 0;

        float TURN90_EN_COUNT = 5.8;
        float TURN180_EN_COUNT = 11.35;


        float WALL_FOLLOW_VERTICAL_SIDE_THRESHOLD = 63.5;
        float WALL_FOLLOW_HORIZONTAL_SIDE_THRESHOLD = 80.0;
        float WALL_FOLLOW_BASE_SPEED = 5.0;
        float wallFollowErrorThreshold = WALL_FOLLOW_VERTICAL_SIDE_THRESHOLD;
        int wallJuncCount = 0;

        float WALL_FOLLOW_VERTICAL_FRONT_THRESHOLD = 75.0;
        float WALL_FOLLOW_HORIZONTAL_FRONT_THRESHOLD = 60.0;
        float frontWallThreshold = WALL_FOLLOW_VERTICAL_FRONT_THRESHOLD;

        float wallFollowPreviousError = 0.0;

        int colorPatch = -1;
        int detectedSquares = 1;
        int finalPosition = -1;
        bool factorsFound = false;
        bool colorPrinted = false;
        int finalPhase = 50;
    };
#ifdef __cplusplus
}
#endif

#endif