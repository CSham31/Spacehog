#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>

using namespace webots;

#ifndef SENSORGROUP_H
#define SENSORGROUP_H

class LineFollower;
class SensorGroup
{
public:
    void initialize(LineFollower* follower);
    void init_distance_sensor(LineFollower* follower);
    void init_encoders(LineFollower* follower);
    void init_camera(LineFollower *follower);

    
    float get_ir_value(int index);
    float get_distance_value(int index);
    float get_generic_value(int index);
    double get_encoder_val(int index);
    int get_digital_value(int index);
    void stabilize_encoder(LineFollower* follower);
    void stabilize_ir_and_distance_sensors(LineFollower *follower);
    
    int detect_color_patch();
    void detect_color_patches();
    void print_color_patch();

    bool is_junction_detected();
    void enable_wall_follow();
    bool is_wall(int side);
    bool is_wall_entrance();
    bool is_wall_exit();

    int is_pillar_detected(int side);

    int COLORS[3];

private:
    LineFollower *follower;
    DistanceSensor *ds[6];
    char dsNames[6][12] = {"sharp_left", "sharp_right", "sharp_front", "sharp_box", "tof_left", "tof_right"};

    PositionSensor *encoder[4];
    char encoder_name[4][18] = {"encoder_left", "encoder_right","arm_servo_encoder","box_servo_encoder"};
    bool enableWallFollow = false;

    Camera *camera;
    char camera_name[8] = {"camera"};

    int BLACK = 0;
    int WHITE = 1;

    int LEFT = 0;
    int RIGHT = 1;
    int BACK = 2;
    int FRONT = 4;

    int IR_LEFT_0 = 0;
    int IR_RIGHT_0 = 1;
    int IR_LEFT_1 = 2;
    int IR_RIGHT_1 = 3;
    int DS_SENSOR_FRONT = 2;
    int DS_SENSOR_RIGHT = 1;
    int DS_SENSOR_LEFT = 0;
    int TOF_RIGHT = 5;
    int TOF_LEFT = 4;

    int WIDTH, HEIGHT;
    int RED = 1;
    int GREEN = 2;
    int BLUE = 3;
    int NO_PATCH = 4;
    int recentColor = -1;

    int SIDE_WALL_THRESHOLD = 150;
    int FRONT_WALL_THRESHOLD = 75;

    float IR_BLACK_VALUE = 10;
    float IR_WHITE_VALUE = 4.8;
    float BLACK_WHITE_THRESHOLD = (IR_BLACK_VALUE + IR_WHITE_VALUE) / 2;

    
    float RED_LEVEL = 0.666667;
    float ROUGHNESS = 0.9;
    float OCCLUSION = 0;
    float REFLECTION_FACTOR = 0.2 + 0.8*RED_LEVEL*(1 - 0.5*ROUGHNESS)*(1 - 0.5*OCCLUSION);

    int FAR_RANGE = 140;
    int NEAR_RANGE = 70;
    
};

#endif