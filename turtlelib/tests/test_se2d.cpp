#include <catch2/catch_test_macros.hpp>
#include "turtlelib/se2d.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <iostream>
#include <sstream>

namespace turtlelib{
    TEST_CASE("testing << operator for a twist", "[outstr<<]"){
        Twist2D twi{1.57, 3.1, 3.5};
        std::stringstream os;
        os<<twi;
        REQUIRE(os.str()=="[1.57 3.1 3.5]");
    }

    TEST_CASE("testing >> operator for a twist, twist no brackets", "[instr>>]"){
        Twist2D twi;
        std::istringstream is("1.4 3.5 3.5");
        is >> twi;
        REQUIRE_THAT(twi.omega, 
            Catch::Matchers::WithinAbs(1.4, 0.01));
        REQUIRE_THAT(twi.x, 
            Catch::Matchers::WithinAbs(3.5, 0.01));
        REQUIRE_THAT(twi.y, 
            Catch::Matchers::WithinAbs(3.5, 0.01));
    }

    TEST_CASE("testing >> operator for a twist, twist brackets", "[instr>>]"){
        Twist2D twi;
        std::istringstream is("[1.4 3.5 3.5]");
        is >> twi;
        REQUIRE_THAT(twi.omega, 
            Catch::Matchers::WithinAbs(1.4, 0.01));
        REQUIRE_THAT(twi.x, 
            Catch::Matchers::WithinAbs(3.5, 0.01));
        REQUIRE_THAT(twi.y, 
            Catch::Matchers::WithinAbs(3.5, 0.01));
    }

    TEST_CASE("Testing identity Transformation", "[transform]"){
        Transform2D trans;
        REQUIRE_THAT(trans.rotation(), 
            Catch::Matchers::WithinAbs(0.0, 0.01));
        REQUIRE_THAT(trans.translation().x , 
            Catch::Matchers::WithinAbs(0.0, 0.01));
        REQUIRE_THAT(trans.translation().y, 
            Catch::Matchers::WithinAbs(0.0, 0.01));
    }

    TEST_CASE("testing translation", "[translation]"){
        Vector2D vec_trans{5.1, 3.2};
        Transform2D trans(vec_trans);
        REQUIRE_THAT(trans.rotation(), 
            Catch::Matchers::WithinAbs(0.0, 0.01));
        REQUIRE_THAT(trans.translation().x , 
            Catch::Matchers::WithinAbs(5.1, 0.01));
        REQUIRE_THAT(trans.translation().y, 
            Catch::Matchers::WithinAbs(3.2, 0.01));   
    }

    TEST_CASE("testing rotation", "[rotation]"){
        double vec_rot {2.5};
        Transform2D trans(vec_rot);
        REQUIRE_THAT(trans.rotation(), 
            Catch::Matchers::WithinAbs(2.5, 0.01));
        REQUIRE_THAT(trans.translation().x , 
            Catch::Matchers::WithinAbs(0.0, 0.01));
        REQUIRE_THAT(trans.translation().y, 
            Catch::Matchers::WithinAbs(0.0, 0.01));       
    }

    TEST_CASE("testing rotation and translation", "[rotation & translation]"){
        Vector2D vec_trans{5.1, 3.2};
        double vec_rot {2.5};
        Transform2D trans(vec_trans,vec_rot);
        REQUIRE_THAT(trans.rotation(), 
            Catch::Matchers::WithinAbs(2.5, 0.01));
        REQUIRE_THAT(trans.translation().x , 
            Catch::Matchers::WithinAbs(5.1, 0.01));
        REQUIRE_THAT(trans.translation().y, 
            Catch::Matchers::WithinAbs(3.2, 0.01));       
    }    

    TEST_CASE("testing () operator for point", "[pt()]"){
        Point2D pt{1.0, 0.0};
        Transform2D trans{Vector2D{2.2,3.3}, PI/2};
        pt = trans.operator()(pt);

        REQUIRE_THAT(pt.x, 
        Catch::Matchers::WithinAbs(2.2, 0.01));
        REQUIRE_THAT(pt.y, 
        Catch::Matchers::WithinAbs(4.3, 0.01));
    }

    TEST_CASE("testing () operator for vector", "[vec()]"){
        Vector2D vec{1.4, 1.1};
        Transform2D trans{Vector2D{2.2,3.3}, PI/2};
        vec = trans.operator()(vec);

        REQUIRE_THAT(vec.x, 
        Catch::Matchers::WithinAbs(-1.1, 0.01));
        REQUIRE_THAT(vec.y, 
        Catch::Matchers::WithinAbs(1.4, 0.01));
    }

    TEST_CASE("testing () operator for twist", "[twist()]"){
        Twist2D twi{PI/2, 1.4, 1.1};
        Transform2D trans{Vector2D{2.2,3.3}, PI/2};
        twi = trans.operator()(twi);
        REQUIRE_THAT(twi.omega,
        Catch::Matchers::WithinAbs(PI/2, 0.01));
        REQUIRE_THAT(twi.x, 
        Catch::Matchers::WithinAbs(4.081, 0.01));
        REQUIRE_THAT(twi.y, 
        Catch::Matchers::WithinAbs(-2.057, 0.01));
    }
    TEST_CASE("tesing the inv operator","[inverse]"){
        Transform2D trans{Vector2D{2.2,3.3}, PI/2};
        Transform2D inv = trans.inv();
        REQUIRE_THAT(inv.rotation(),
        Catch::Matchers::WithinAbs(-PI/2, 0.01));
        REQUIRE_THAT(inv.translation().x,
        Catch::Matchers::WithinAbs(-3.3, 0.01));
        REQUIRE_THAT(inv.translation().y,
        Catch::Matchers::WithinAbs(2.2, 0.01));
    }

    TEST_CASE("testing the *= operator","[multiplication*=]"){
        Transform2D trans{Vector2D{2.4,4.4}, 0.0};
        Transform2D rhs{Vector2D{2.1,1.1}, 0.0};
        trans.operator*=(rhs);
        REQUIRE_THAT(trans.rotation(),
        Catch::Matchers::WithinAbs(0.0, 0.01));
        REQUIRE_THAT(trans.translation().x,
        Catch::Matchers::WithinAbs(4.5, 0.01));
        REQUIRE_THAT(trans.translation().y,
        Catch::Matchers::WithinAbs(5.5, 0.01));
    } 

    TEST_CASE("Testing the Rotation return", "[return_rotation]"){
        Transform2D trans{Vector2D{2.4,4.4}, 2.5};
        double rad = trans.rotation();
        REQUIRE_THAT(rad,
        Catch::Matchers::WithinAbs(2.5,0.01));
    }

    TEST_CASE("Testing the translation return", "[return_translation]"){
        Transform2D trans{Vector2D{2.4,4.4}, 2.5};
        Vector2D vec = trans.translation();
        REQUIRE_THAT(vec.x,
        Catch::Matchers::WithinAbs(2.4,0.01));
        REQUIRE_THAT(vec.y,
        Catch::Matchers::WithinAbs(4.4,0.01));
    }

    TEST_CASE("Testing Twist integrate pure rotation", "[twist_rotation]"){
        Twist2D t{0.0,1.2,3.3};
        auto t_rot = integrate_twist(t);
        REQUIRE_THAT(t_rot.rotation(),
        Catch::Matchers::WithinAbs(0.0, 0.01));
        REQUIRE_THAT(t_rot.translation().x,
        Catch::Matchers::WithinAbs(1.2, 0.01));
        REQUIRE_THAT(t_rot.translation().y,
        Catch::Matchers::WithinAbs(3.3, 0.01));
    }

    TEST_CASE("Testing Twist integrate pure translation", "[twist_translation]"){
        Twist2D t{1.2,0.0,0.0};
        auto t_rot = integrate_twist(t);
        REQUIRE_THAT(t_rot.rotation(),
        Catch::Matchers::WithinAbs(1.2, 0.01));
        REQUIRE_THAT(t_rot.translation().x,
        Catch::Matchers::WithinAbs(0.0, 0.01));
        REQUIRE_THAT(t_rot.translation().y,
        Catch::Matchers::WithinAbs(0.0, 0.01));
    }

    TEST_CASE("Testing Twist integrate translation & rotation", "[twist_translation_rotation]"){
        Twist2D t{1.57,2.2,1.33};
        auto t_rot = integrate_twist(t);
        REQUIRE_THAT(t_rot.rotation(),
        Catch::Matchers::WithinAbs(1.57, 0.01));
        REQUIRE_THAT(t_rot.translation().x,
        Catch::Matchers::WithinAbs(0.55, 0.01));
        REQUIRE_THAT(t_rot.translation().y,
        Catch::Matchers::WithinAbs(2.247, 0.01));
    }

    TEST_CASE("integrate twist","[twist]") // Abhishek,Sankar
    {
    Twist2D T_a{0.0, 2.0, 1.0};
    Twist2D T_b{PI/2.0, 0.0, 0.0};
    Twist2D T_c{2*PI, 5.0, 0.0};

    Transform2D T_A = integrate_twist(T_a);
    Transform2D T_B = integrate_twist(T_b);
    Transform2D T_C = integrate_twist(T_c);

    REQUIRE_THAT(T_A.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(T_A.translation().x, Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(T_A.translation().y, Catch::Matchers::WithinAbs(1.0, 1e-5));

    REQUIRE_THAT(T_B.rotation(), Catch::Matchers::WithinAbs(PI/2.0, 1e-5));
    REQUIRE_THAT(T_B.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(T_B.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));

    REQUIRE_THAT(T_C.rotation(), Catch::Matchers::WithinAbs(2*PI, 1e-5));
    REQUIRE_THAT(T_C.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(T_C.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
    }

    TEST_CASE("testing << with trabsform2d","[Transform2D<<]"){
        Transform2D trans{Vector2D{2.4,4.4}, 2.5};
        std::stringstream os;
        os<<trans;
        REQUIRE(os.str()=="deg: 143.239 x: 2.4 y: 4.4");
    }

    TEST_CASE("testing >> operator for a transform2D, transform2d no brackets", "[intrans>>]"){
        Transform2D trans;
        std::istringstream is("1.4 3.5 3.5");
        is >> trans;
        REQUIRE_THAT(trans.rotation(), Catch::Matchers::WithinAbs(turtlelib::deg2rad(1.4),0.01));
        REQUIRE_THAT(trans.translation().x,
            Catch::Matchers::WithinAbs(3.5,0.01));
        REQUIRE_THAT(trans.translation().y,
            Catch::Matchers::WithinAbs(3.5,0.01));
    }

    TEST_CASE("testing >> operator for a transform2D, transform2d terms", "[intrans_brack>>]"){
        Transform2D trans;
        std::istringstream is("deg: 90 x: 3.5 y: 3.5");
        is >> trans;
        REQUIRE_THAT(trans.rotation(), Catch::Matchers::WithinAbs(turtlelib::deg2rad(90),0.01));
        REQUIRE_THAT(trans.translation().x,
            Catch::Matchers::WithinAbs(3.5,0.01));
        REQUIRE_THAT(trans.translation().y,
            Catch::Matchers::WithinAbs(3.5,0.01));
    }

    TEST_CASE("testing * operator for two transform2D objects", "[multiplication*]"){
        Transform2D lhs{Vector2D{3.0,5.2}, PI/2};
        Transform2D rhs{Vector2D{3.7,6.3}, PI/4};
        lhs = operator*(lhs,rhs);

        REQUIRE_THAT(lhs.rotation(),
            Catch::Matchers::WithinAbs(3*(PI/4),0.01));
        REQUIRE_THAT(lhs.translation().x,
            Catch::Matchers::WithinAbs(-3.3,0.01));
        REQUIRE_THAT(lhs.translation().y,
            Catch::Matchers::WithinAbs(8.9,0.01));
    }

}