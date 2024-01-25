#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <iostream>
#include <sstream>

//https://stackoverflow.com/questions/64121152/how-to-compare-floating-point-in-catch2

namespace turtlelib{
TEST_CASE("Normalizing angle", "[normalize]"){

        REQUIRE_THAT(normalize_angle(PI), 
                Catch::Matchers::WithinAbs(PI, 0.01));

        REQUIRE_THAT(normalize_angle(-PI), 
                Catch::Matchers::WithinAbs(-PI, 0.01));

        REQUIRE_THAT(normalize_angle(0.0), 
                Catch::Matchers::WithinAbs(0.0, 0.01));

        REQUIRE_THAT(normalize_angle(PI/4), 
                Catch::Matchers::WithinAbs(PI/4, 0.01));

        REQUIRE_THAT(normalize_angle(-PI/4), 
                Catch::Matchers::WithinAbs(-PI/4, 0.01));

        CHECK_THAT(normalize_angle(3*PI/2), 
                Catch::Matchers::WithinAbs(-PI/2, 0.01));

        CHECK_THAT(normalize_angle(-5*PI/2), 
                Catch::Matchers::WithinAbs(-PI/2, 0.01));
}

TEST_CASE("Vector+= funct","[Vec+=]"){
        Vector2D vec{1,1};
        Vector2D add{2,1};
        vec+=add;
        REQUIRE_THAT(vec.x, 
                Catch::Matchers::WithinAbs(3, 0.01));
        REQUIRE_THAT(vec.y, 
                Catch::Matchers::WithinAbs(2, 0.01));
}

TEST_CASE("Vector-= funct","[Vec-=]"){
        Vector2D vec{1,2};
        Vector2D sub{2,1};
        vec-=sub;
        REQUIRE_THAT(vec.x, 
                Catch::Matchers::WithinAbs(-1, 0.01));
        REQUIRE_THAT(vec.y, 
                Catch::Matchers::WithinAbs(1, 0.01));
}

TEST_CASE("Vector*= funct","[Vec*=]"){
        Vector2D vec{1,2};
        double mul{2};
        vec*=mul;
        REQUIRE_THAT(vec.x, 
                Catch::Matchers::WithinAbs(2, 0.01));
        REQUIRE_THAT(vec.y, 
                Catch::Matchers::WithinAbs(4, 0.01));
}

TEST_CASE("Vector+ funct","[Vec+]"){
        Vector2D a{1,1};
        Vector2D b{2,1};
        Vector2D c = a+b;
        REQUIRE_THAT(c.x, 
                Catch::Matchers::WithinAbs(3, 0.01));
        REQUIRE_THAT(c.y, 
                Catch::Matchers::WithinAbs(2, 0.01));
}

TEST_CASE("Vector- funct","[Vec-]"){
        Vector2D a{1,2};
        Vector2D b{2,1};
        Vector2D c = a-b;
        REQUIRE_THAT(c.x, 
                Catch::Matchers::WithinAbs(-1, 0.01));
        REQUIRE_THAT(c.y, 
                Catch::Matchers::WithinAbs(1, 0.01));
}

TEST_CASE("Vector* funct","[Vec*]"){
        Vector2D a{1,2};
        double b{2};
        Vector2D c = a*b;
        Vector2D d = b*a;
        REQUIRE_THAT(c.x,
                Catch::Matchers::WithinAbs(2, 0.01));
        REQUIRE_THAT(c.y, 
                Catch::Matchers::WithinAbs(4, 0.01));
        REQUIRE_THAT(d.x,
                Catch::Matchers::WithinAbs(2, 0.01));
        REQUIRE_THAT(d.y, 
                Catch::Matchers::WithinAbs(4, 0.01));
}

TEST_CASE("dot product testing","[dot_product]"){
        Vector2D a{1,2};
        Vector2D b{2,3};
        double c = dot(a,b);
        REQUIRE_THAT(c,
                Catch::Matchers::WithinAbs(8, 0.01));
}

TEST_CASE("magnitude testing","[vector_mag]"){
        Vector2D a{1,2};
        double c = magnitude(a);
        REQUIRE_THAT(c,
                Catch::Matchers::WithinAbs(sqrt(5), 0.01));
}

TEST_CASE("angle testing","[vector_angle]"){
        Vector2D a{1,2};
        Vector2D b{2,1};
        double c = angle(a,b);
        REQUIRE_THAT(c,
                Catch::Matchers::WithinAbs(acos(0.8), 0.01));
}

TEST_CASE("testing normalize function", "[normalize]"){
        Vector2D vec{1,1};
        vec = normalize(vec);

        REQUIRE_THAT(vec.x, 
                Catch::Matchers::WithinAbs(0.7071, 0.01));
        REQUIRE_THAT(vec.y, 
                Catch::Matchers::WithinAbs(0.7071, 0.01));

}

TEST_CASE("testing - operator", "-"){
        Point2D head{5.0,10.0};
        Point2D tail{2.0,2.0};
        Vector2D vec;
        vec = operator-(head,tail);

        REQUIRE_THAT(vec.x, 
                Catch::Matchers::WithinAbs(head.x-tail.x, 0.01));
        REQUIRE_THAT(vec.y, 
                Catch::Matchers::WithinAbs(head.y-tail.y, 0.01));
}

TEST_CASE("testing + operator", "+"){
        Point2D a{5.0,10.0};
        Vector2D b{2.0,2.0};
        Point2D pt;
        pt = operator+(a,b);

        REQUIRE_THAT(pt.x, 
                Catch::Matchers::WithinAbs(b.x + a.x, 0.01));
        REQUIRE_THAT(pt.y, 
                Catch::Matchers::WithinAbs(b.y + a.y, 0.01));
}

TEST_CASE("testing >> operator for a pt, point brackets", ">>"){
        Point2D pt;
        std::istringstream is("[4.5 7.2]");
        is >> pt;
        REQUIRE_THAT(pt.x, 
                Catch::Matchers::WithinAbs(4.5, 0.01));
        REQUIRE_THAT(pt.y, 
                Catch::Matchers::WithinAbs(7.2, 0.01));
}

TEST_CASE("testing >> operator for a pt, point no brackets", ">>"){
        Point2D pt;
        std::istringstream is("4.8 7.7");
        is >> pt;
        REQUIRE_THAT(pt.x, 
                Catch::Matchers::WithinAbs(4.8, 0.01));
        REQUIRE_THAT(pt.y, 
                Catch::Matchers::WithinAbs(7.7, 0.01));
}

TEST_CASE("testing << operator for a point", "<<"){
        Point2D pt{2.2,3.2};
        std::stringstream os;
        os<<pt;
        REQUIRE(os.str()=="[2.2 3.2]");
}

TEST_CASE("testing >> operator for a vec, vector brackets", ">>"){
        Vector2D vec;
        std::istringstream is("[4.5 7.2]");
        is >> vec;
        REQUIRE_THAT(vec.x, 
                Catch::Matchers::WithinAbs(4.5, 0.01));
        REQUIRE_THAT(vec.y, 
                Catch::Matchers::WithinAbs(7.2, 0.01));
}

TEST_CASE("testing >> operator for a vec, vector no brackets", ">>"){
        Vector2D vec;
        std::istringstream is("4.8 7.7");
        is >> vec;
        REQUIRE_THAT(vec.x, 
                Catch::Matchers::WithinAbs(4.8, 0.01));
        REQUIRE_THAT(vec.y, 
                Catch::Matchers::WithinAbs(7.7, 0.01));
}

TEST_CASE("testing << operator for a vector", "<<"){
    Vector2D vec{2.2,3.2};
    std::stringstream os;
    os<<vec;
    REQUIRE(os.str()=="[2.2 3.2]");
}
}