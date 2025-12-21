#pragma once
#include "CornLib/robot_init.h"
#include "vex.h"

class Chassis {
    vex::motor* left_motor;
    vex::motor* right_motor;
    vex::gyro* gyro;
    float track_width;
    float drive_rpm;
    Pose pose_deg;
    Pose pose_rad;
    cornlib::PID* lateral;
    cornlib::PID* angular;

    public:
        /**
         * @brief chassis constructor
         * 
         * @param left_motors left side drive motor
         * @param right_motors right side drive motor
         * @param gyro gryo
         * @param track_width track width
         * @param drive_rpm rpm of the drivetrain 
         */
        Chassis(vex::motor* left_motor, vex::motor* right_motor, vex::gyro* gyro, float track_width, float drive_rpm, cornlib::PID* lateral, cornlib::PID* angular);

        /**
         * @brief turn to a point on the field
         * 
         * @param x x coordinate in inches
         * @param y y coordinate in inches
         * @param timeout maximum amount of time the robot can spend completing the move
         * @param async whether to run the movement asynchronously
         */
        void turnToPoint(float x, float y, float timeout, bool async = false);

        /**
         * @brief turn to a desired heading
         * 
         * @param theta heading in degrees
         * @param timeout maximum amount of time the robot can spend completing the move
         * @param async whether to run the movement asynchronously
         */
        void turnToHeading(float theta, float timeout, bool async = false);

        /**
         * @brief move to a point on the field
         * 
         * @param x x coordinate in inches
         * @param y y coordinate in inches
         * @param timeout maximum amount of time the robot can spend completing the move
         * @param async whether to run the movement asynchronously
         */
        void moveToPoint(float x, float y, float timeout, bool async = false);

        /**
         * @brief move a distance
         * 
         * @param distance distance to travel in inches
         * @param timeout maximum amount of time the robot can spend completing the move
         * @param async whether to run the movement asynchronously
         */
        void moveDistance(float distance, float timeout, bool async = false);

        /**
         * @brief returns a Pose object
         * 
         * @param radians whether to output the theta in radians
         * @return Pose object
         */
        Pose getPose(bool radians = false);

        /**
         * @brief update the robot's position
         * 
         * @param radians whether to update the theta in radians
         */
        void setPose(float x, float y, float theta, bool radians = false);
};  