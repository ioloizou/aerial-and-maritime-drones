#include "lab_auve/uavsim.h"
#include <eigen3/Eigen/Dense>
#include "matplotlibcpp.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// Plot stuff
namespace plt = matplotlibcpp;
std::vector<double> plot_data;
std::vector<double> plot_data2;
std::vector<double> plot_data3;
std::vector<double> plot_data4;

//Usefull variables
double la = 0.17;
double kt = 5.5e-6;
double kd = 3.299e-7;
double drone_mass = 1.0;
double gravity = 9.81;

double scalar_force;

Matrix4d mixer;
Matrix4d inv_mixer;

Vector4d squared_desired_motors;

void SetMixerMatrix(){
    mixer << kt, kt, kt, kt,
             -kt*la, kt*la, 0, 0,
             0, 0, -kt*la, kt*la,
             -kd, -kd, kd, kd;

    inv_mixer = mixer.inverse();
}

Vector3d orientationControl(const Quaterniond& desired_orientation, const Quaterniond& current_orientation, const Vector3d current_velocity)
{

    auto inverse_current_orientation = current_orientation.inverse();
    auto error_orientation = inverse_current_orientation*desired_orientation;

    // Define weights for the diagonal
    Eigen::VectorXd kp_weights(3);
    Eigen::VectorXd kd_weights(3);

    kp_weights << 2., 2., 0.1;
    kd_weights << 0.4, 0.4, 0.001;

    // Create diagonal matrices using asDiagonal()
    Eigen::MatrixXd Kp = kp_weights.asDiagonal();
    Eigen::MatrixXd Kd = kd_weights.asDiagonal();

    Vector3d tau = -Kd*current_velocity + Kp*sgn(error_orientation.w())*error_orientation.vec();

    return tau;
}

Quaterniond positionControl(const Vector3d& desired_position, const Vector3d& desired_velocity,
                             const Vector3d& current_position, const Vector3d& current_velocity,
                             double& scalar_force)
{
    // Define weights for the diagonal
    Eigen::VectorXd kp_weights(3);
    Eigen::VectorXd kd_weights(3);

    kp_weights << 2., 2., 2.;
    kd_weights << 3., 3., 3.;

    // Create diagonal matrices using asDiagonal()
    Eigen::MatrixXd Kp = kp_weights.asDiagonal();
    Eigen::MatrixXd Kd = kd_weights.asDiagonal();

    Eigen::Vector3d g;
    g(2) = -gravity;

    Vector3d f = Kd*(desired_velocity - current_velocity) + Kp*(desired_position - current_position) - drone_mass*g;
    scalar_force = f.norm();
    Vector3d z_body = f.normalized();

    Vector3d x_w = Vector3d::UnitX();
    Vector3d y_body = z_body.cross(x_w);
    Vector3d x_body = y_body.cross(z_body);

    Matrix3d R;
    R.col(0) = x_body;
    R.col(1) = y_body;
    R.col(2) = z_body;

    Quaterniond desired_orientation(R);

    return desired_orientation;
}

void computeTrajectory(double t, Vector3d& desired_position, Vector3d& desired_velocity) {
    // Trajectory parameters
    double a = 0.5;      // Semi-major axis (X direction) (meters)
    double b = 1.0;      // Semi-minor axis (Y direction) (meters)
    double omega = 1.5;  // Angular velocity (rad/s)
    double z0 = 1.0;     // Constant height (meters)

    // Compute desired position (ellipse in XY plane)
    desired_position.x() = a * cos(omega * t);
    desired_position.y() = b * sin(omega * t);
    desired_position.z() = z0;  // Constant height

    // Compute desired velocity (derivatives)
    desired_velocity.x() = -a * omega * sin(omega * t);
    desired_velocity.y() = b * omega * cos(omega * t);
    desired_velocity.z() = 0.0;  // No vertical movement
}

int main()
{
    plt::figure();
    int loop_count=0;

    UAVSim uav = UAVSim();
    double prev_clock = uav.getClock();

    sleep(1); // necessary for proper init before start control

    while (uav.isReadyToGo() and loop_count<5000) {
        if ((uav.getClock() - prev_clock) > 0.005) // Loop at 200Hz
        {
            prev_clock = uav.getClock();
            Vector4d desired_motors;
            loop_count = 0;

            SetMixerMatrix();

            Quaterniond current_orientation = uav.getOrientation();
            Vector3d current_angular_velocity = uav.getAngularVelocity();
            Vector3d current_position = uav.getPosition();
            Vector3d current_linear_velocity = uav.getLinearVelocity();

            Vector4d U;
            scalar_force = 0;

            Vector3d desired_position;
            Vector3d desired_velocity;

            computeTrajectory(uav.getClock(), desired_position, desired_velocity);

            auto desired_orientation = positionControl(desired_position, desired_velocity,
                                         current_position, current_linear_velocity,
                                         scalar_force);

            Vector3d tau = orientationControl(desired_orientation, current_orientation, current_angular_velocity);


            U << scalar_force, tau;
            Vector4d squared_desired_motors;
            squared_desired_motors = inv_mixer*U.array().matrix();

            desired_motors = squared_desired_motors.array().sqrt().cwiseMax(0).cwiseMin(1466);

            // Motor speed should be between 0 and 1466 rpm
            uav.setMotors(desired_motors);

//            // A basic example of a scope plot
//            // Indeed you need to adapt this to your needs
//            // It uses a C++ library based on python matplotlib with similar calls
//            plt::clf();
//            // You can create as many plots as you want
//            plot_data.push_back(desired_position.z());
//            plot_data2.push_back(uav.getPosition().z());
//            plt::plot(plot_data);
//            plt::plot(plot_data2);
//            plt::plot(plot_data3);
//            plt::plot(plot_data4);
//            plt::pause(0.0001);
        }
        usleep(1);
        loop_count ++;
    }
    // To plot for you report you can add some code here and use plt::show so the code will wait that you close the figures to stop.
}
