#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include "SensorGroup.h"
#include "LineFollower.h"
#include <math.h>
#include <bits/stdc++.h>

#define TIME_STEP 8
using namespace webots;
using namespace std;

void SensorGroup::initialize(LineFollower *follower)
{
    follower = follower;
    init_distance_sensor(follower);
    init_encoders(follower);
    init_camera(follower);
    stabilize_ir_and_distance_sensors(follower);
}

void SensorGroup::init_distance_sensor(LineFollower *follower)
{
    for (int i = 0; i < 6; i++)
    {
        ds[i] = follower->getDistanceSensor(dsNames[i]);
        ds[i]->enable(TIME_STEP);
    }
}

void SensorGroup::init_encoders(LineFollower *follower)
{
    for (int i = 0; i < 2; i++)
    {
        encoder[i] = follower->getPositionSensor(encoder_name[i]);
        encoder[i]->enable(TIME_STEP);
    }
}

float SensorGroup::get_ir_value(int index)
{
    float val = ds[index]->getValue();
    return val;
}

void SensorGroup::init_camera(LineFollower *follower)
{
    camera = follower->getCamera(camera_name);
    camera->enable(TIME_STEP);
    WIDTH = camera->getWidth();
    HEIGHT = camera->getHeight();
}

float SensorGroup::get_distance_value(int index)
{
    float val = ds[index]->getValue();
    return round(val * REFLECTION_FACTOR);
    //return val;
}

double SensorGroup::get_encoder_val(int index)
{
    double val = encoder[index]->getValue();
    return val;
}

int SensorGroup::get_digital_value(int index)
{
    if (get_ir_value(index) > BLACK_WHITE_THRESHOLD)
        return BLACK;
    else
        return WHITE;
}

bool SensorGroup::is_junction_detected()
{
    if ((get_digital_value(IR_LEFT_1) == WHITE) or (get_digital_value(IR_RIGHT_1) == WHITE))
        return true;
    else
        return false;
}

void SensorGroup::enable_wall_follow()
{
    enableWallFollow = true;
}
bool SensorGroup::is_wall(int side)
{
    if ((side == RIGHT) and (get_distance_value(DS_SENSOR_RIGHT) < SIDE_WALL_THRESHOLD))
        return true && enableWallFollow;
    else if ((side == LEFT) and (get_distance_value(DS_SENSOR_LEFT) < SIDE_WALL_THRESHOLD))
        return true && enableWallFollow;
    else if ((side == FRONT) and (get_distance_value(DS_SENSOR_FRONT) < FRONT_WALL_THRESHOLD))
        return true && enableWallFollow;
    else
        return false && enableWallFollow;
}

bool SensorGroup::is_wall_entrance()
{
    if (is_wall(LEFT) == true or is_wall(RIGHT) == true)
        return true;
    else
        return false;
}

bool SensorGroup::is_wall_exit()
{
    if (is_wall(LEFT) == false and is_wall(RIGHT) == false)
        return true;
    else
        return false;
}

void SensorGroup::stabilize_encoder(LineFollower *follower)
{
    if (isnan(get_encoder_val(LEFT)) || isnan(get_encoder_val(RIGHT)))
    {
        while (follower->step(TIME_STEP) != -1)
        {
            if (!(isnan(get_encoder_val(LEFT)) && isnan(get_encoder_val(RIGHT))))
                break; //to make sure that encoder dosent return NaN
        }
    }
}

void SensorGroup::stabilize_ir_and_distance_sensors(LineFollower *follower)
{
    while (follower->step(TIME_STEP) != -1)
    {
        if (!(isnan(get_ir_value(IR_LEFT_0)) && isnan(get_ir_value(IR_RIGHT_0)) && isnan(get_ir_value(IR_LEFT_1)) && isnan(get_ir_value(IR_RIGHT_1)) 
        && isnan(get_ir_value(DS_SENSOR_FRONT)) && isnan(get_ir_value(DS_SENSOR_LEFT)) && isnan(get_ir_value(DS_SENSOR_RIGHT))))
            break; //to make sure that ir and distance sensors dosent return NaN
    }
}

int SensorGroup::detect_color_patch()
{
    const unsigned char *IMAGE = camera->getImage();

    int redpix = 0;
    int greenpix = 0;
    int bluepix = 0;

    int i, j;

    for (j = HEIGHT - 1; j >= 0; j--)
    {
        for (i = WIDTH / 4; i < 3 * WIDTH / 4; i++)
        {

            redpix = camera->imageGetRed(IMAGE, WIDTH, i, j);
            bluepix = camera->imageGetBlue(IMAGE, WIDTH, i, j);
            greenpix = camera->imageGetGreen(IMAGE, WIDTH, i, j);

            if ((redpix > 4 * greenpix) && (redpix > 4 * bluepix))
            {
                recentColor = RED;
                return RED;
            }
            else if ((greenpix > 4 * redpix) && (greenpix > 4 * bluepix))
            {
                recentColor = GREEN;
                return GREEN;
            }
            else if ((bluepix > 4 * redpix) && (bluepix > 4 * greenpix))
            {
                recentColor = BLUE;
                return BLUE;
            }
        }
    }
    return NO_PATCH;
}

void SensorGroup::detect_color_patches()
{
    const unsigned char *IMAGE = camera->getImage();

    int redpix = 0;
    int greenpix = 0;
    int bluepix = 0;

    int i, j;
    int ind = 0;
    bool red_detected = false;
    bool green_detected = false;
    bool blue_detected = false;

    for (i = 0; i < WIDTH; i++)
    {
        for (j = 0; j < HEIGHT; j++)
        {
            redpix = camera->imageGetRed(IMAGE, WIDTH, i, j);
            bluepix = camera->imageGetBlue(IMAGE, WIDTH, i, j);
            greenpix = camera->imageGetGreen(IMAGE, WIDTH, i, j);

            if (!red_detected && (redpix > 4 * greenpix) && (redpix > 4 * bluepix))
            {
                COLORS[ind] = RED;
                red_detected = true;
                ind += 1;
            }
            else if (!green_detected && (greenpix > 4 * redpix) && (greenpix > 4 * bluepix))
            {
                COLORS[ind] = GREEN;
                green_detected = true;
                ind += 1;
            }
            else if (!blue_detected && (bluepix > 4 * redpix) && (bluepix > 4 * greenpix))
            {
                COLORS[ind] = BLUE;
                blue_detected = true;
                ind += 1;
            }
        }
    }
    return;
}

void SensorGroup::print_color_patch()
{
    if (recentColor == RED)
    {
        cout << "RED" << endl;
    }
    else if (recentColor == GREEN)
    {
        cout << "GREEN" << endl;
    }
    else if(recentColor == BLUE)
    {
        cout << "BLUE" << endl;
    }
}