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

void DiffDrive::compute_fk(double l_prime, double r_prime)
{
    auto phi = (wheel_radius/wheel_track) * (r_prime - l_prime);
    auto x_dot = (wheel_radius/2) * cos(trans.rotation()) * (l_prime+r_prime);
    auto y_dot = (wheel_radius/2) * sin(trans.rotation()) * (l_prime+r_prime);

    psi.left += l_prime;
    psi.right += r_prime;

    auto new_phi = trans.rotation() + phi ;
    auto new_x = trans.translation().x + x_dot ; 
    auto new_y = trans.translation().y + y_dot ;

    twist.omega = new_phi;
    twist.x = new_x;
    twist.y = new_y;

    trans = turtlelib::integrate_twist(twist);
}

wheel_positions DiffDrive::compute_ik(Twist2D twi){
    if(almost_equal(twi.y,0)){
        psi.left = (((twi.x)-(0.5*wheel_track*twi.omega))/wheel_radius);
        psi.right = (((twi.x)+(0.5*wheel_track*twi.omega))/wheel_radius);
        
        return wheel_positions{psi.left, psi.right};
    }
    else{
        throw std::logic_error("The y velocity is not zero, check again");
    }
}
}