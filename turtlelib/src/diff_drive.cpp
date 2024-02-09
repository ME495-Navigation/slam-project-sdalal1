#include "turtlelib/diff_drive.hpp"
#include <iostream>

namespace turtlelib{

DiffDrive::DiffDrive(double track, double radius){
    wheel_track = track;
    wheel_radius = radius;
    trans=Transform2D();
    psi = wheel_positions{0.0,0.0};
    twist = Twist2D{0.0,0.0,0.0};
}

DiffDrive::DiffDrive(Transform2D t, double track, double radius){
    trans = t;
    wheel_track = track;
    wheel_radius = radius;
    psi = wheel_positions{0.0,0.0};
    twist = Twist2D{0.0,0.0,0.0};
}

wheel_positions DiffDrive::get_wheel_angle(){
    return psi;
}

Transform2D DiffDrive::get_transformation(){
    return trans;
}

double DiffDrive::get_wheel_radius() const{
    return wheel_radius;
}

double DiffDrive::get_wheel_track() const{
    return wheel_track;
}

Twist2D DiffDrive::get_twist() const{
    return twist;
}

void DiffDrive::change_transform(Transform2D tr){
    trans = tr;
}

void DiffDrive::compute_fk(double l_prime, double r_prime)
{
    //part of formula taken from Modern robotics 13.15
    auto phi = (wheel_radius/wheel_track) * (r_prime - l_prime); //Equation 3 from Kinematics.pdf
    auto x_dot = (wheel_radius/2) * (l_prime+r_prime); //Equation 4 from kinematics.pdf
    
    auto y_dot = 0.0;

    psi.left = psi.left+ l_prime;
    psi.right = psi.right + r_prime;

    twist.omega = phi;
    twist.x = x_dot;
    twist.y = y_dot;

    auto t_new = integrate_twist(twist); //Converting the twist to tranform

    trans *= t_new; //changing it back to body frame
}

wheel_positions DiffDrive::compute_ik(Twist2D twi){
    if(almost_equal(twi.y,0)){
        psi.left = (((twi.x)-(0.5*wheel_track*twi.omega))/wheel_radius);  //Equation 1 from Kinematics.pdf
        psi.right = (((twi.x)+(0.5*wheel_track*twi.omega))/wheel_radius); //Equation 2 from Kinematics.pdf
        
        return wheel_positions{psi.left, psi.right};
    }
    else{
        throw std::logic_error("The y velocity is not zero, check again");
    }
}
}