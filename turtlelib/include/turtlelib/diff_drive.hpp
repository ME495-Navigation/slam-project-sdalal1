#ifndef TURTLELIB_DIFFDRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Diff-Drive calculations for two wheels


#include<iosfwd> // contains forward definitions for iostream objects

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"

namespace turtlelib
{
    /// \brief the wheel angle 
    struct wheel_positions{

        /// \brief the left wheel orientation
        double left;

        /// \brief the right wheel orientation
        double right;
    };

    /// \brief The diff drive kinematics
    class DiffDrive{
        public:

        /// @brief the constructor ro se the track width and radius
        /// @param track The track width of the robot
        /// @param radius the radius of the wheels
        DiffDrive(double track, double radius);

        /// @brief Constructor with the additon of the orientaiton
        /// @param t The transformation of the orientation
        /// @param track track width of the robot
        /// @param radius radius of wheels
        DiffDrive(Transform2D t, double track, double radius);

        /// @brief get the wheen angle of the memeber
        /// @return the wheel angle
        wheel_positions get_wheel_angle();

        /// @brief to get current transformation of the robot
        /// @return the current transformation of the member
        Transform2D get_transformation();

        /// @brief Get the wheel radius
        /// @return returns a wheel radius
        double get_wheel_radius() const;

        /// @brief get the track width
        /// @return the track width of the member
        double get_wheel_track() const;

        /// @brief get the twist
        /// @return Teh body twist
        Twist2D get_twist() const;

        /// @brief calculation fo the forward kinematics after a timestep
        /// @param left_prime The left wheel orienattion after time delta t
        /// @param right_prime The right wheel orientation after time delta t
        void compute_fk(double left_prime, double right_prime);

        /// @brief Computing the inverse kinematiocs of the bot
        /// @param twi The twist to be achieved
        /// @return The wheel velocities of bot
        wheel_positions compute_ik(Twist2D twi);

        private:
        /// @brief The wheel position of the robot
        wheel_positions psi;
        /// @brief The orienatation and position of the bot
        Transform2D trans;
        /// \brief the track distacne of between wheels
        double wheel_track;
        /// \brief the radius of the wheel 
        double wheel_radius;
        /// \brief the body twist
        turtlelib::Twist2D twist;
    };

}
#endif