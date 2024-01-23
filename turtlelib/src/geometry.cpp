#include "turtlelib/geometry2d.hpp"
#include <iostream>

// #define PI 3.14159265358979323846
namespace turtlelib
{
double normalize_angle(double rad){
    while(rad < -PI){
        rad += 2*PI;
    }
    while(rad > PI){
        rad-=2*PI;
    } 
    return rad;
}


std::ostream & operator<<(std::ostream & os, const turtlelib::Point2D & p){
    return os<<"["<<p.x<<" "<<p.y<<"]"; 
}

std::istream & operator>>(std::istream & is, turtlelib::Point2D & p){
    char c1 = is.get();
    if (c1 == '['){
        is >> p.x >> p.y >> c1;
    }
    else{
        is.putback(c1);
        is >> p.x >> p.y;
    }
    return is;
}

Vector2D normalize(Vector2D v){
        double dim = std::sqrt(pow(v.x,2) + pow(v.y,2));
        return {v.x/dim, v.y/dim};
}

Vector2D operator-(const Point2D & head, const Point2D & tail){
    Vector2D coord;
    coord.x = head.x - tail.x;
    coord.y = head.y - tail.y;
    return coord;
}

Point2D operator+(const Point2D & tail, const Vector2D & disp){
    Point2D disp_point;
    disp_point.x = disp.x+tail.x;
    disp_point.y = disp.y+tail.y;
    return disp_point;
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v){
    return os<<"["<<v.x<<" "<<v.y<<"]";
}

std::istream & operator>>(std::istream & is, Vector2D & v){
    char c1 = is.get();
    if (c1 == '['){
        is >> v.x >> v.y >> c1;
    }
    else{
        is.putback(c1);
        is >> v.x >> v.y;
    }
    return is;
}

}