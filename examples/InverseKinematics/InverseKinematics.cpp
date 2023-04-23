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

// Function to compute the cross product of two 3D vectors
Matrix<3> cross_product(const Matrix<3>& a, const Matrix<3>& b) {
    Matrix<3> result;

    result(0) = a(1) * b(2) - a(2) * b(1);
    result(1) = a(2) * b(0) - a(0) * b(2);
    result(2) = a(0) * b(1) - a(1) * b(0);

    return result;
}

template <int n>
float Norm(const Matrix<n>& vec) {
    T sum = 0;
    for (int i = 0; i < n; ++i) {
        sum += vec(i) * vec(i);
    }
    return sqrt(sum);
}

bool inverse_kinematics(Link& end_effector, const Geometry::Pose& target_pose, float step_size, int max_iterations, float tolerance)
{
    for (int i = 0; i < max_iterations; ++i)
    {
        // Calculate the current end effector pose
        Geometry::Pose current_pose = end_effector.pose;

        // Compute the error in translation and rotation
        Geometry::Translation position_error = target_pose.p - current_pose.p;
        Geometry::AngularVelocity orientation_error = (cross_product(current_pose.R.Column(0), target_pose.R.Column(0)) +
                                                       cross_product(current_pose.R.Column(1), target_pose.R.Column(1)) +
                                                       cross_product(current_pose.R.Column(2), target_pose.R.Column(2)) ) * 0.5f;

        // Check for convergence
        if (position_error.Norm() < tolerance && orientation_error.Norm() < tolerance)
        {
            return true;
        }

        // Calculate the Jacobian matrix
        BLA::Matrix<6, 6> J = BLA::Zeros<6, 6>();
        Link* link = &end_effector;
        while (link->joint)
        {
            Geometry::Twist joint_axis = link->joint->axis;
            Geometry::Wrench joint_wrench = adjoint(link->pose.inverse()) * joint_axis;

            J = J - joint_wrench * (link->parent->twist * joint_axis).Norm();

            link = link->parent;
        }

        // Compute the change in joint angles
        BLA::Matrix<6> delta_twist = position_error;
        delta_twist.Append(orientation_error);
        BLA::Matrix<6> delta_joint_angles = step_size * J.Transpose() * delta_twist;

        // Update joint angles
        link = &end_effector;
        for (int j = 0; j < 6; ++j)
        {
            if (!link->joint) break;

            link->joint->position += delta_joint_angles(j);
            link = link->parent;
        }
    }

    return false;
}
