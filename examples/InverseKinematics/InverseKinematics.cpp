#include "InverseKinematics.h"

using namespace Geometry;
using namespace BLA;

Link::Link(float mass, const BLA::Matrix<3, 3> &inertia, const Geometry::Translation &xyz,
           const Geometry::EulerAngles &rpy)
    : external_force(Zeros<6>())
{
    Pose inertia_pose(rpy.to_rotation_matrix(), xyz);
    spatial_inertia = Zeros<6, 6>();
    spatial_inertia.Submatrix<3, 3>(0, 0) = inertia;
    spatial_inertia.Submatrix<3, 3>(3, 3) = Identity<3, 3>() * mass;
    spatial_inertia = ~adjoint(inertia_pose.inverse()) * spatial_inertia * adjoint(inertia_pose.inverse());
}

Joint::Joint(Link &parent, Link &child, const Geometry::Twist &axis_, const Geometry::Translation &xyz,
             const Geometry::EulerAngles &rpy)
    : axis(axis_), offset(Pose(rpy.to_rotation_matrix(), xyz).inverse())
{
    child.joint = this;
    parent.child = &child;
    child.parent = &parent;
}

template <int n>
float norm(const Matrix<n>& vec) {
    float sum = 0;
    for (int i = 0; i < n; ++i) {
        sum += vec(i) * vec(i);
    }
    return sqrt(sum);
}

// Function to calculate the Jacobian of a serial robot manipulator with revolute joints
template <int NumJoints>
BLA::Matrix<6, NumJoints> jacobian(Link& end_effector) {
    // Initialize the 6xNumJoints Jacobian matrix
    BLA::Matrix<6, NumJoints> J;

    // Initialize the current pose with the end effector's pose in the last joint space
    Geometry::Pose current_pose = end_effector.pose;

    // Iterate through the robot manipulator's joints (starting from the first child link)
    Link* current_link = end_effector.parent;

    while (current_link)
    {
        
    }

    for (int i = 0; i < NumJoints; ++i) {
        const Joint& joint = *current_link->joint;

        // Calculate the joint axis in the world frame
        Geometry::Twist joint_axis_in_world = current_pose * joint.axis;

        // Compute the partial derivative of the end-effector's position with respect to the current joint variable
        // and store it in the first 3 rows of the corresponding column in the Jacobian matrix
        J.Submatrix<3, 1>(0, i) = Geometry::skew(joint_axis_in_world.Submatrix<3, 1>(0, 0)) * (current_link->pose.p - current_pose.p);
        
        // Compute the partial derivative of the end-effector's orientation with respect to the current joint variable
        // and store it in the last 3 rows of the corresponding column in the Jacobian matrix
        J.Submatrix<3, 1>(3, i) = joint_axis_in_world.Submatrix<3, 1>(0, 0);

        // Update the current pose by multiplying it with the current link's pose
        current_pose = current_pose * current_link->pose;

        // Move on to the next link in the chain
        current_link = current_link->child;
    }

    // Return the Jacobian matrix
    return J;
}

bool inverse_kinematics(Link& end_effector, const Geometry::Pose& target_pose, float step_size, int max_iterations, float tolerance)
{
    for (int i = 0; i < max_iterations; ++i)
    {
        // Calculate the current end effector pose
        Geometry::Pose current_pose = end_effector.pose;

        // Compute the error in translation and rotation
        Geometry::Translation position_error = target_pose.p - current_pose.p;

        // Check for convergence
        if (norm(position_error) < tolerance < tolerance)
        {
            return true;
        }

        // Calculate the Jacobian matrix
        BLA::Matrix<6, 6> J = jacobian(end_effector);

        // Compute the change in joint angles
        BLA::Matrix<6> delta_twist = BLA::Zeros<6>();
        delta_twist(0) = position_error(0);
        delta_twist(1) = position_error(1);
        delta_twist(2) = position_error(2);
        BLA::Matrix<6> delta_joint_angles = (~J) * delta_twist * step_size;

        // Update joint angles
        auto link = &end_effector;
        for (int j = 0; j < 6; ++j)
        {
            if (!link->joint) break;

            link->joint->position += delta_joint_angles(j);
            link = link->parent;
        }
    }
    return false;
}
