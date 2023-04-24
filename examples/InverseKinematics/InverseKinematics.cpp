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
        BLA::Matrix<6, 6> J = BLA::Zeros<6, 6>();

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
