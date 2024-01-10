#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <iostream>
#include <sstream>

//https://stackoverflow.com/questions/64121152/how-to-compare-floating-point-in-catch2

namespace turtlelib{
TEST_CASE("Normalizing angle", "[normalize]"){

REQUIRE_THAT(normalize_angle(PI), 
        Catch::Matchers::WithinAbs(PI, 0.1));

REQUIRE_THAT(normalize_angle(-PI), 
        Catch::Matchers::WithinAbs(-PI, 0.1));

REQUIRE_THAT(normalize_angle(0.0), 
        Catch::Matchers::WithinAbs(0.0, 0.1));

REQUIRE_THAT(normalize_angle(PI/4), 
        Catch::Matchers::WithinAbs(PI/4, 0.1));

REQUIRE_THAT(normalize_angle(-PI/4), 
        Catch::Matchers::WithinAbs(-PI/4, 0.1));

CHECK_THAT(normalize_angle(3*PI/2), 
        Catch::Matchers::WithinAbs(-PI/2, 0.1));

CHECK_THAT(normalize_angle(-5*PI/2), 
        Catch::Matchers::WithinAbs(-PI/2, 0.1));
}

TEST_CASE("testing - operator", "-"){
Point2D head{5.0,10.0};
Point2D tail{2.0,2.0};
Vector2D vec;
vec = operator-(head,tail);

REQUIRE(vec.x == head.x - tail.x);
REQUIRE(vec.y == head.y - tail.y);

}

TEST_CASE("testing + operator", "+"){
Point2D a{5.0,10.0};
Vector2D b{2.0,2.0};
Point2D pt;
pt = operator+(a,b);

REQUIRE(pt.x == b.x + a.x);
REQUIRE(pt.y == b.y + a.y);

}

TEST_CASE("testing << operator for a pt, point brackets", "<<"){
    Point2D pt;
    std::istringstream is("[4.5 7.2]");
    is >> pt;
    REQUIRE(pt.x == 4.5);
    REQUIRE(pt.y == 7.2);
}

TEST_CASE("testing << operator for a pt, point no brackets", "<<"){
    Point2D pt;
    std::istringstream is("4.8 7.7");
    is >> pt;
    REQUIRE(pt.x == 4.8);
    REQUIRE(pt.y == 7.7);
}

TEST_CASE("testing >> operator for a point", ">>"){
    Point2D pt{2.0,3.2};
    std::stringstream os;
    os<<pt;
    REQUIRE(os.str()=="[2.0 3.2]");
}

TEST_CASE("testing << operator for a vec, vector brackets", "<<"){
    Vector2D vec;
    std::istringstream is("[4.5 7.2]");
    is >> vec;
    REQUIRE(vec.x == 4.5);
    REQUIRE(vec.y == 7.2);
}

TEST_CASE("testing << operator for a vec, vector no brackets", "<<"){
    Vector2D vec;
    std::istringstream is("4.8 7.7");
    is >> vec;
    REQUIRE(vec.x == 4.8);
    REQUIRE(vec.y == 7.7);
}

TEST_CASE("testing >> operator for a vector", ">>"){
    Vector2D vec{2.0,3.2};
    std::stringstream os;
    os<<vec;
    REQUIRE(os.str()=="[2.0 3.2]");
}
}