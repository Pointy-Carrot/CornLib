#include "CornLib/chassis.h"


Chassis::Chassis(vex::motor* left_motor, vex::motor* right_motor, vex::gyro* gyro, float track_width, float drive_rpm, cornlib::PID* lateral, cornlib::PID* angular) :
left_motor(left_motor), right_motor(right_motor), gyro(gyro), track_width(track_width), drive_rpm(drive_rpm), lateral(lateral), angular(angular) {};

void Chassis::turnToPoint(float x, float y, float timeout, bool async = false){}

void Chassis::turnToHeading(float heading, float timeout, bool async = false){}

void Chassis::moveToPoint(float x, float y, float timeout, bool async = false){}

void Chassis::moveDistance(float distance, float timeout, bool async = false){}

Pose Chassis::getPose(bool radians = false){
    if(radians){
        return pose_rad;
    } else{
        return pose_deg;
    }
}

void Chassis::setPose(float x, float y, float theta, bool radians = false){
    pose_deg.x = x;
    pose_rad.x = x;
    pose_deg.y = y;
    pose_rad.y = y;
    if(radians){
        pose_rad.theta = radians;
        pose_deg.theta = theta * (180 / M_PI);
    } else{
        pose_deg.theta = theta;
        pose_rad.theta = theta * (M_PI / 180);
    }
}

