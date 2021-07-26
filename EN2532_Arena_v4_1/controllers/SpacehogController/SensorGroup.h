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
    void init_qtr_sensor(LineFollower *follower);
    void init_encoders(LineFollower* follower);
    void init_camera(LineFollower *follower);

    
    float get_ir_value(int index);
    float get_distance_value(int index);
    float get_generic_value(int index);
    double get_encoder_val(int index);
    int get_digital_value(int index);
    void stabilize_encoder(LineFollower* follower);
    void stabilize_ir_and_distance_sensors(LineFollower *follower);
    
    int get_colour(int cam);
    void detect_color_patches();
    void print_color_patch(int color);

    bool is_junction_detected();
    bool is_line_segment_detected();
    void enable_wall_follow();
    bool is_wall(int side);
    bool is_wall_entrance();
    bool is_wall_exit();

    int is_pillar_detected(int side);
    bool is_gate_detected(int sensor);

    int qtr_read_line();

    int COLORS[3];

private:
    LineFollower *follower;
    DistanceSensor *ds[6];
    char dsNames[6][12] = {"sharp_left", "sharp_right", "sharp_front", "sharp_box", "tof_left", "tof_right"};

    DistanceSensor *qtr[10];
    char qtrNames[10][18] = {"qtr_0", "qtr_1", "qtr_2", "qtr_3", "qtr_4", "qtr_5","qtr_6","qtr_7","line_detect_left","line_detect_right"};

    PositionSensor *encoder[4];
    char encoder_name[4][18] = {"encoder_left", "encoder_right","arm_servo_encoder","box_servo_encoder"};
    bool enableWallFollow = false;

    Camera *camera[3];
    char camera_name[3][14] = {"camera","camera_back","camera_front"};

    int BLACK = 0;
    int WHITE = 1;

    int LEFT = 0;
    int RIGHT = 1;
    int BACK = 2;
    int FRONT = 4;

    int QTR_0 = 0;
    int QTR_1 = 1;
    int QTR_2 = 2;
    int QTR_3 = 3;
    int QTR_4 = 4;
    int QTR_5 = 5;
    int QTR_6 = 6;
    int QTR_7 = 7;
    int LINE_DETECT_LEFT = 8;
    int LINE_DETECT_RIGHT = 9;

    int previousQTR_7 = 0;
    int previousQTR_0 = 0;

    int DS_SENSOR_BOX = 3;
    int DS_SENSOR_FRONT = 2;
    int DS_SENSOR_RIGHT = 1;
    int DS_SENSOR_LEFT = 0;
    int TOF_RIGHT = 5;
    int TOF_LEFT = 4;

    int CAM_ARM = 0;
    int CAM_BACK = 1;
    int CAM_FRONT = 2;
    int CAM_PIXEL_THRESH = 20;

    int WIDTH, HEIGHT;
    int RED = 1;
    int GREEN = 2;
    int BLUE = 3;
    int NO_COLOR = 4;
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
    int GATE_RANGE = 300;
    
};

#endif