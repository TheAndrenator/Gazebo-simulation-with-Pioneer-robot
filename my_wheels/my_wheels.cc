// File:          my_wheels.cc
// Date:          07/03/2024
// Description:   Gazebo plugin to control a robot
//                Optimized reactive navigation (shortest distance)
//                Bug 2-like algorithm
// Author:        Andrés Gracia Guillén

// Useful libraries
#include <iostream>
#include <limits>
#include <math.h>
#include <vector>
#include <numeric>
#include <chrono>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/RaySensor.hh>

// Namespace for "std"
using namespace std;

/// Useful constant values for the program
// Pi Number
#define PI 3.141592654
// Map dimensions
#define map_x 12
#define map_y 12
// Maximum velocity
#define MAX_SPEED 5.0


namespace gazebo
{
    class MyWheels : public ModelPlugin
    {
    public:
        // Function to load and configurate the plugin in the simulation
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            model = _model;

            if (!_sdf->HasElement("left_joint")) // Check if the plugin has the required elements
                gzerr << "MyWheels plugin missing <left_joint> element\n";

            if (!_sdf->HasElement("right_joint"))
                gzerr << "MyWheels plugin missing <right_joint> element\n";

            // Get the pointers to the joints
            leftJoint = _model->GetJoint(_sdf->GetElement("left_joint")->Get<string>());
            rightJoint = _model->GetJoint(_sdf->GetElement("right_joint")->Get<string>());

            if (!leftJoint) {
                gzerr << "Unable to find left joint[" << _sdf->GetElement("left_joint")->Get<string>() << "]\n";
            }
            if (!rightJoint) {
                gzerr << "Unable to find right joint[" << _sdf->GetElement("right_joint")->Get<string>() << "]\n";
            }

            // Listen to the update event. This event is broadcast every simulation iteration.
            updateConnectionWheels = event::Events::ConnectWorldUpdateBegin(bind(&MyWheels::OnUpdate, this));
            
            //////////////////////////////////////////
            // Code extracted from my_sensor_model_g9 plugin
            int sensor_num = _model->GetSensorCount(); // Get the number of sensors in the model

            sensor = sensors::get_sensor("my_sensor"); // Get the pointer to the sensor

            if (!sensor) { // Check if the sensor is a LaserSensor
                gzerr << "the plugin requires a LaserSensor.\n";
                return;
            }
            
            sensor->SetActive(true); // Activate the sensor

            gzdbg << "Opened " << sensor->ScopedName() << "\n";

            raySensor = dynamic_pointer_cast<sensors::RaySensor>(sensor); 
            if (!raySensor) {
                gzerr << "dynamic_pointer_cast to RaySensor failed!\n";
                return;
            }
            // This block of parameters is obtained from gazebo
            gzdbg << "AngleMax [deg] " << raySensor->AngleMax().Degree() << "\n";
            gzdbg << "AngleMin [deg] " << raySensor->AngleMin().Degree() << "\n";
            gzdbg << "RangeMax [m] " << raySensor->RangeMax() << "\n";
            gzdbg << "RangeMin [m] " << raySensor->RangeMin() << "\n";
            gzdbg << "AngleResolution [deg] " << raySensor->AngleResolution() * 180.0 / M_PI << "\n";
            gzdbg << "RangeCount " << raySensor->RangeCount() << "\n";
            gzdbg << "UpdateRate " << raySensor->UpdateRate() << "\n";

            // Listen to the update event. This event is broadcast every simulation iteration.
            updateConnectionSensor = event::Events::ConnectWorldUpdateBegin(bind(&MyWheels::OnUpdate, this));
        }

        double left_speed, right_speed; // Velocities when avoiding obstacles
        // Calculate final point
        double goalX = map_x - 1.5;
        double goalY = map_y - 1.5;
        int i = 0; // Flag
        int durationFlag = 0; // Simulation time flag

        // Called by the world update start event
        void OnUpdate()
        {
            
            static auto startTime = chrono::high_resolution_clock::now();

            // Get the current pose of the robot
            ignition::math::Pose3d pose = model->WorldPose();
            float x = pose.Pos().X();
            float y = pose.Pos().Y();
            printf("POS: (%f, %f)\n", x, y);

            // Calculate angle towards the goal
            double desiredAngle = atan2(goalY - y, goalX - x);
            double currentAngle = pose.Rot().Yaw();
            double angleError = desiredAngle - currentAngle;
            printf("[ROT] Goal: %f | Robot: %f | Error: %f\n", desiredAngle, currentAngle, angleError);

            // Normalize the angle error between -pi and pi
            while (angleError > PI) angleError -= 2.0 * PI;
            while (angleError < -PI) angleError += 2.0 * PI;

            // Calculate distance to the goal
            double distance = sqrt(pow(goalX - x, 2) + pow(goalY - y, 2));
            printf("Distance to goal: %f\n", distance);

            // Calculate simulation time
            auto currentTime = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();

            // If too close to the goal, stop the robot
            if (distance < 0.3) {
                left_speed = 0.0;
                right_speed = 0.0;
                printf("---------------------\n");
                printf("-------GOAL!!!-------\n");
                printf("---------------------\n");
                // Store simulation time
                if (i != 1) {
                    durationFlag = duration;
                }
                i = 1; // Update flag
                printf("Simulation time: %lld seconds\n", durationFlag);
                exit(0);
                return;
            }

            if (i == 0) {
                printf("Simulation time: %lld seconds\n", duration);
            }

            // Get the range values from the laser sensor
            vector<double> ranges;
            raySensor->Ranges(ranges);
            if (ranges.size() == 0) return;
            double rightRange, leftRange, midRange, midRightRange, midLeftRange;

            // Calculate distance from right side of the robot
            if (!isnan(ranges[15]) && !isnan(ranges[10]) && !isnan(ranges[5])) {
                rightRange = (ranges[15] + ranges[10] + ranges[5]) / 3;
            } else {
                return;
            }

            // Calculate distance from left side of the robot
            if (!isnan(ranges[720-1]) && !isnan(ranges[720-6]) && !isnan(ranges[720-11])) {
                leftRange = (ranges[720-1] + ranges[720-6] + ranges[720-11]) / 3;
            } else {
                return;
            }

            // Calculate distance from middle side of the robot
            if (!isnan(ranges[359]) && !isnan(ranges[358]) && !isnan(ranges[360])) {
                midRange = (ranges[359] + ranges[358] + ranges[360]) / 3;
            } else {
                return;
            }

            // Calculate distance from right-front side of the robot
            if (!isnan(ranges[179]) && !isnan(ranges[181]) && !isnan(ranges[180])) {
                midRightRange = (ranges[179] + ranges[181] + ranges[180]) / 3;
            } else {
                return;
            }

            // Calculate distance from left-front side of the robot
            if (!isnan(ranges[269]) && !isnan(ranges[271]) && !isnan(ranges[270])) {
                midLeftRange = (ranges[269] + ranges[271] + ranges[270]) / 3;
            } else {
                return;
            }
            
            // Print distance values
            printf("[RAY] Left: %f | Front-left: %f | Front: %f | Front-right: %f | Right: %f\n", leftRange, midLeftRange, midRange, midRightRange, rightRange);

            // Calculate the linear and angular velocities when there are no obstacles
            double linearVelocity = 1.0;
            double angularVelocity = 0.5 * angleError;
            double leftVelocity = linearVelocity - angularVelocity;
            double rightVelocity = linearVelocity + angularVelocity;

            // Control logic of the robot
            if ((midRange < 0.4) || (midLeftRange < 0.4) || (midRightRange < 0.4) || (leftRange < 0.4) || (rightRange < 0.4)) {
                // Obstacle avoidance
                if ((rightRange < 0.4) && (leftRange > 0.3) || (midRightRange < 0.7)) {
                    // If obstacle to the right side, turn left
                    left_speed = -1.0;
                    right_speed = 1.0;
                    printf("TURNING LEFT!!!!!\n");
                } else if ((rightRange > 0.3) && (leftRange < 0.4) || (midLeftRange < 0.7)) {
                    // If obstacle in the left side, turn right
                    left_speed = 1.0;
                    right_speed = -1.0;
                    printf("TURNING RIGHT!!!!!\n");
                } else {
                    // Consider other obstacle cases
                    if (rightRange < 0.5) {
                        // Turn left
                        left_speed =-1.0;
                        right_speed = 1.0;
                        printf("TURNING LEFT!!!!!\n");
                    } else if (leftRange < 0.5) {
                        // Turn right
                        left_speed = 1.0;
                        right_speed = -1.0;
                        printf("TURNING RIGHT!!!!!\n");
                    } else {
                        // Go backwards
                        left_speed = -1.0;
                        right_speed = -1.5;
                        printf("GOING BACKWARDS!!!!!\n");
                    }
                }
            } else {
                // Otherwise, move forward in the target direction
                left_speed = leftVelocity;
                right_speed = rightVelocity;
                printf("Going forward\n");
            }

            printf("\n");
            
            // Set velocities to the wheels
            leftJoint->SetVelocity(0, left_speed);
            rightJoint->SetVelocity(0, right_speed);
        }

    private:
        physics::ModelPtr model; // Pointer to the model
        physics::JointPtr leftJoint, rightJoint; // Pointers to the joints
        event::ConnectionPtr updateConnectionWheels; // Pointer to the update event connection
        
        sensors::RaySensorPtr raySensor; // Pointer to the sensor
        sensors::SensorPtr sensor; // Pointer to the sensor
        event::ConnectionPtr updateConnectionSensor; // Pointer to the update event connection
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MyWheels)
}
