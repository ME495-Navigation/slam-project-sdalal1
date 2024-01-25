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

Vector2D& Vector2D::operator+=(Vector2D add){
    x += add.x;
    y += add.y;
    return *this;
}

Vector2D& Vector2D::operator-=(Vector2D sub){
    x-=sub.x;
    y-=sub.y;
    return *this;
}

Vector2D& Vector2D::operator*=(double s){
    x*=s;
    y*=s;
    return *this;
}

Vector2D operator+(Vector2D a, Vector2D b){
    a+=b;
    return a;
}

Vector2D operator-(Vector2D a, Vector2D b){
    a-=b;
    return a;
}

Vector2D operator*(double s, Vector2D a){
    a*=s;
    return a;
}

Vector2D operator*(Vector2D a, double s){
    a*=s;
    return a;
}

double dot(Vector2D a, Vector2D b){
    double pro;
    pro = (a.x*b.x)+(a.y*b.y);
    return pro;
}

double magnitude(Vector2D a){
    double mag;
    mag = std::sqrt(pow(a.x,2) + pow(a.y,2));
    return mag;
}

double angle(Vector2D a, Vector2D b){
    double ang;
    ang = acos(dot(a,b)/(magnitude(a)*magnitude(b)));
    return ang;
}

Vector2D normalize(Vector2D v){
        // double dim = std::sqrt(pow(v.x,2) + pow(v.y,2));
        return {v.x/magnitude(v), v.y/magnitude(v)};
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