#include "InverseKinematics.h"
#include "Geometry.h"
#include "BLA.h"

using namespace Geometry;
using namespace BLA;

void setup()
{
    Serial.begin(115200);

    // Let's start by declaring the kinematic chain described in the URDF above
    Link world;
    Link upper_arm(1.0, {0.01335, 0, 0, 0, 0.01335, 0, 0, 0, 0.0008}, {0, 0, 0.2}, {0, 0, 0});
    Link forearm(1.0, {0.01335, 0, 0, 0, 0.01335, 0, 0, 0, 0.0008}, {0, 0, 0.2}, {0, 0, 0});
    Link hand(1.0, {0.01335, 0, 0, 0, 0.01335, 0, 0, 0, 0.0008}, {0, 0, 0.2}, {0, 0, 0});

    // All these joints are revolute so their joint axes point in the positive y direction
    Joint shoulder(world, upper_arm, Matrix<6>(0, 1, 0, 0, 0, 0), {0, 0, 0.4}, {0, 0, 0});
    Joint elbow(upper_arm, forearm, Matrix<6>(0, 1, 0, 0, 0, 0), {0, 0, 0.4}, {0, 0, 0});
    Joint wrist(forearm, hand, Matrix<6>(0, 1, 0, 0, 0, 0), {0, 0, 0.4}, {0, 0, 0});

    // Set a target pose for the end-effector (hand)
    Transform target_pose;
    target_pose.translation = {0.8, 0.0, 0.8};
    target_pose.rotation = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    // Compute the inverse kinematics
    Matrix<3> joint_positions = inverse_kinematics(world, target_pose);

    // Set the computed joint positions
    shoulder.position = joint_positions(0);
    elbow.position = joint_positions(1);
    wrist.position = joint_positions(2);

    // Print out the joint positions
    Serial << "Inverse Kinematics joint positions:\n";
    Serial << "Shoulder: " << shoulder.position << "\n";
    Serial << "Elbow: " << elbow.position << "\n";
    Serial << "Wrist: " << wrist.position << "\n";

    // Verify the result by calculating the forward kinematics
    Transform calculated_pose = forward_kinematics(world, Matrix<3>(shoulder.position, elbow.position, wrist.position));
    Serial << "Forward Kinematics calculated pose:\n";
    Serial << "Translation: " << calculated_pose.translation(0) << ", " << calculated_pose.translation(1) << ", " << calculated_pose.translation(2) << "\n";
    Serial << "Rotation: \n";
    Serial << calculated_pose.rotation(0, 0) << ", " << calculated_pose.rotation(0, 1)
