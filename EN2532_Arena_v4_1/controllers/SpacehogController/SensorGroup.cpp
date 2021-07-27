#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>
#include "SensorGroup.h"
#include "LineFollower.h"
#include <math.h>
#include <bits/stdc++.h>

#define TIME_STEP 16
using namespace webots;
using namespace std;

void SensorGroup::initialize(LineFollower *follower)
{
    follower = follower;
    init_LED(follower);
    init_distance_sensor(follower);
    init_qtr_sensor(follower);
    init_encoders(follower);
    init_camera(follower);
    stabilize_encoder(follower);
    stabilize_ir_and_distance_sensors(follower);
}

void SensorGroup::init_LED(LineFollower *follower)
{
  for (int i =0 ; i<2; i++){
    led[i] = follower->getLED(led_name[i]);
  }
}

void SensorGroup::init_distance_sensor(LineFollower *follower)
{
    for (int i = 0; i < 6; i++)
    {
        ds[i] = follower->getDistanceSensor(dsNames[i]);
        ds[i]->enable(TIME_STEP);
    }
}

void SensorGroup::init_qtr_sensor(LineFollower *follower)
{
    for (int i = 0; i < 10; i++)
    {
        qtr[i] = follower->getDistanceSensor(qtrNames[i]);
        qtr[i]->enable(TIME_STEP);
    }
}

void SensorGroup::init_encoders(LineFollower *follower)
{
    for (int i = 0; i < 4; i++)
    {
        encoder[i] = follower->getPositionSensor(encoder_name[i]);
        encoder[i]->enable(TIME_STEP);
    }
}

float SensorGroup::get_ir_value(int index)
{
    float val = qtr[index]->getValue();
    return val;
}

void SensorGroup::init_camera(LineFollower *follower)
{
    for (int i = 0; i < 3; i++)
    {
    camera[i] = follower->getCamera(camera_name[i]);
    camera[i]->enable(TIME_STEP);

    }
}

void SensorGroup::set_LED(int side,int colour)
{
    led[side]->set(colour);
}

float SensorGroup::get_ir_value_distance_sensors(int index)     //only for stabilization
{
    float val = ds[index]->getValue();
    return val;
}

float SensorGroup::get_distance_value(int index)
{
    float val = ds[index]->getValue();
    return round(val * REFLECTION_FACTOR);
    //return val;
}

float SensorGroup::get_generic_value(int index)     //for TOFs and front sensors
{
    return round(ds[index]->getValue());
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
    if ((get_digital_value(LINE_DETECT_LEFT) == WHITE) or (get_digital_value(LINE_DETECT_RIGHT) == WHITE))
        return true;
    else
        return false;
}

bool SensorGroup::is_line_segment_detected()
{
    if ((get_digital_value(QTR_0) == WHITE) and (get_digital_value(QTR_1) == WHITE) and (get_digital_value(QTR_2) == WHITE) and (get_digital_value(QTR_3) == WHITE) 
    and (get_digital_value(QTR_4) == WHITE) and (get_digital_value(QTR_5) == WHITE) and (get_digital_value(QTR_6) == WHITE) and (get_digital_value(QTR_7) == WHITE))
        return true;
    else
        return false;
}

int SensorGroup::qtr_read_line()
{
    int irValues[8];
    int numerator = 0;
    int denominator = 0;

    for (int i=0; i<8 ; i++){
        irValues[i] = get_digital_value(i);
    }
    for (int i=0; i<8 ; i++){
        numerator += i*1000*irValues[i];
        denominator += irValues[i];
    }
    //cout<<irValues[7]<<"  "<<irValues[6]<<"  "<<irValues[1]<<"  "<<irValues[0]<<endl;
    if ((numerator == 0) and (denominator == 0))    // to prevent 0/0 division form happening
    {
        if (previousQTR_7 == 1)
            return 7000;
        else if (previousQTR_0 == 1)
            return 0;
        else{
            //cout<<"undefined value at qtr read line"<<endl;
            return 3500;
        }
    }

    previousQTR_7 = irValues[QTR_7];
    previousQTR_0 = irValues[QTR_0];

    return (numerator/denominator);
}

bool SensorGroup::is_wall(int side)
{
    if ((side == RIGHT) and (get_distance_value(DS_SENSOR_RIGHT) < SIDE_WALL_THRESHOLD))
        return true;
    else if ((side == LEFT) and (get_distance_value(DS_SENSOR_LEFT) < SIDE_WALL_THRESHOLD))
        return true;
    else if ((side == FRONT) and (get_distance_value(DS_SENSOR_FRONT) < FRONT_WALL_THRESHOLD))
        return true;
    else
        return false;
}

bool SensorGroup::is_wall_entrance()
{
    if (is_wall(LEFT) == true || is_wall(RIGHT) == true)
        return true;
    else
        return false;
}

bool SensorGroup::is_wall_exit()
{
    if (is_wall(LEFT) == false && is_wall(RIGHT) == false)
        return true;
    else
        return false;
}

int SensorGroup::is_box_detected(int side)
{
    float distance;

    if (side == LEFT)
        distance = get_generic_value(TOF_LEFT); 
    else 
        distance = get_generic_value(TOF_RIGHT); 
    

    if (distance < NEAR_RANGE ) 
        return 1;
    else if ( distance < FAR_RANGE) 
        return 2;
    else
        return 0;
}

bool SensorGroup::is_pillar_detected(int side)
{
    float distance;

    if (side == LEFT)
        distance = get_generic_value(TOF_LEFT); 
    else 
        distance = get_generic_value(TOF_RIGHT); 
    

    if (distance < 15 ) 
        return 1;
    else
        return 0;
}

bool SensorGroup::is_gate_detected(int sensor)
{
    if (get_generic_value(sensor) < GATE_RANGE ) 
        return 1;
    else
        return 0;
}

void SensorGroup::stabilize_encoder(LineFollower *follower)
{
    if (isnan(get_encoder_val(LEFT)) || isnan(get_encoder_val(RIGHT)) || isnan(get_encoder_val(2)) || isnan(get_encoder_val(3)))    //2,3 for arm and box servos
    {
        while (follower->step(TIME_STEP) != -1)
        {
            if (!(isnan(get_encoder_val(LEFT)) && isnan(get_encoder_val(RIGHT)) && isnan(get_encoder_val(2)) && isnan(get_encoder_val(3))))
                break; //to make sure that encoder dosent return NaN
        }
    }
}

void SensorGroup::stabilize_ir_and_distance_sensors(LineFollower *follower)
{
    while (follower->step(TIME_STEP) != -1)
    {
        if (!(isnan(get_ir_value_distance_sensors(DS_SENSOR_FRONT)) && isnan(get_ir_value_distance_sensors(DS_SENSOR_LEFT)) && isnan(get_ir_value_distance_sensors(DS_SENSOR_RIGHT)) && isnan(get_ir_value_distance_sensors(DS_SENSOR_BOX)) 
        && isnan(get_ir_value_distance_sensors(TOF_LEFT)) && isnan(get_ir_value_distance_sensors(TOF_RIGHT)) && isnan(get_ir_value(QTR_0)) && isnan(get_ir_value(QTR_1)) && isnan(get_ir_value(QTR_2))
        && isnan(get_ir_value(QTR_3)) && isnan(get_ir_value(QTR_4)) && isnan(get_ir_value(QTR_5)) && isnan(get_ir_value(QTR_6)) && isnan(get_ir_value(QTR_7)) && isnan(get_ir_value(LINE_DETECT_LEFT))
        && isnan(get_ir_value(LINE_DETECT_RIGHT))))
            break; //to make sure that ir and distance sensors dosent return NaN
    }
}

int SensorGroup::get_colour(int cam)
{
    const unsigned char *IMAGE = camera[cam]->getImage();

    WIDTH = camera[cam]->getWidth();
    HEIGHT = camera[cam]->getHeight();

    int redpix = 0;
    int greenpix = 0;
    int bluepix = 0;

    int i, j;

    for (j = CAM_PIXEL_THRESH ; j < (HEIGHT-CAM_PIXEL_THRESH); j++)
    {
        for (i = CAM_PIXEL_THRESH; i < (WIDTH-CAM_PIXEL_THRESH); i++)
        {

            redpix += camera[cam]->imageGetRed(IMAGE, WIDTH, i, j);
            bluepix += camera[cam]->imageGetBlue(IMAGE, WIDTH, i, j);
            greenpix += camera[cam]->imageGetGreen(IMAGE, WIDTH, i, j);

            if ((redpix > greenpix) && (redpix >  bluepix))
            {
                recentColor = RED;
                return RED;
            }
            else if ((greenpix >  redpix) && (greenpix >  bluepix))
            {
                recentColor = GREEN;
                return GREEN;
            }
            else if ((bluepix >  redpix) && (bluepix >  greenpix))
            {
                recentColor = BLUE;
                return BLUE;
            }
        }
    }
    cout<<"no colour"<<endl;
    return NO_COLOR;
}

void SensorGroup::print_color_patch(int color)
{
    if (color == RED)
    {
        cout << "RED" << endl;
    }
    else if (color == GREEN)
    {
        cout << "GREEN" << endl;
    }
    else if(color == BLUE)
    {
        cout << "BLUE" << endl;
    }
}