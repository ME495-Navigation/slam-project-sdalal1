#include <catch2/catch_test_macros.hpp>
#include "turtlelib/diff_drive.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <iostream>
#include <sstream>

namespace turtlelib{
    TEST_CASE("Constructor with only track and radius", "[DiffDrive_]"){
        DiffDrive d(3.5, 4.2);
        REQUIRE_THAT(d.get_wheel_track(), 
            Catch::Matchers::WithinAbs(3.5, 1e-5));
        REQUIRE_THAT(d.get_wheel_radius(), 
            Catch::Matchers::WithinAbs(4.2, 1e-5));
        REQUIRE_THAT(d.get_transformation().rotation(), 
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(d.get_transformation().translation().x, 
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(d.get_transformation().translation().y, 
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(d.get_wheel_angle().left,
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(d.get_wheel_angle().right, 
            Catch::Matchers::WithinAbs(0.0, 1e-5));
    }

    TEST_CASE("Constructor with only track and radius", "[DiffDrive_trans]"){
        Transform2D t(Vector2D{3.3, 2.2}, 1.57);
        DiffDrive d(t, 3.5, 4.2);
        REQUIRE_THAT(d.get_wheel_track(), 
            Catch::Matchers::WithinAbs(3.5, 1e-5));
        REQUIRE_THAT(d.get_wheel_radius(), 
            Catch::Matchers::WithinAbs(4.2, 1e-5));
        REQUIRE_THAT(d.get_transformation().rotation(), 
            Catch::Matchers::WithinAbs(1.57, 1e-5));
        REQUIRE_THAT(d.get_transformation().translation().x, 
            Catch::Matchers::WithinAbs(3.3, 1e-5));
        REQUIRE_THAT(d.get_transformation().translation().y, 
            Catch::Matchers::WithinAbs(2.2, 1e-5));
        REQUIRE_THAT(d.get_wheel_angle().left,
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(d.get_wheel_angle().right, 
            Catch::Matchers::WithinAbs(0.0, 1e-5));
    }

    TEST_CASE("forward motion tetsing", "[forward]"){
        auto dif = DiffDrive(1.3, 0.5);
        auto wheel = dif.compute_ik(Twist2D{0.0, 0.2 , 0.0});
        REQUIRE_THAT(wheel.left, 
            Catch::Matchers::WithinAbs(0.4, 1e-5));
        REQUIRE_THAT(wheel.right, 
            Catch::Matchers::WithinAbs(0.4, 1e-5));
        
        dif.compute_fk(wheel.left,wheel.right);

        REQUIRE_THAT(dif.get_transformation().rotation(), 
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().x, 
            Catch::Matchers::WithinAbs(0.2, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().y, 
            Catch::Matchers::WithinAbs(0.0, 1e-5)); 
    }

    TEST_CASE("rotation motion tetsing", "[rotation]"){
        auto dif = DiffDrive(1.3, 0.5);
        auto wheel = dif.compute_ik(Twist2D{1.57, 0.0 , 0.0});
        REQUIRE_THAT(wheel.left, 
            Catch::Matchers::WithinAbs(-2.041, 1e-5));
        REQUIRE_THAT(wheel.right, 
            Catch::Matchers::WithinAbs(2.041, 1e-5));
        
        dif.compute_fk(wheel.left,wheel.right);

        REQUIRE_THAT(dif.get_transformation().rotation(), 
            Catch::Matchers::WithinAbs(1.57, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().x, 
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().y, 
            Catch::Matchers::WithinAbs(0.0, 1e-5)); 
    }

    TEST_CASE("rotation motion circle", "[circle]"){
        auto dif = DiffDrive(1.3, 0.5);
        auto wheel = dif.compute_ik(Twist2D{1.57, 0.2 , 0.0});
        REQUIRE_THAT(wheel.left, 
            Catch::Matchers::WithinAbs(-1.641, 1e-5));
        REQUIRE_THAT(wheel.right, 
            Catch::Matchers::WithinAbs(2.441, 1e-5));
        
        dif.compute_fk(wheel.left,wheel.right);

        REQUIRE_THAT(dif.get_transformation().rotation(), 
            Catch::Matchers::WithinAbs(1.57, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().x, 
            Catch::Matchers::WithinAbs(0.2, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().y, 
            Catch::Matchers::WithinAbs(0.0, 1e-5)); 
    }

    TEST_CASE("rotation motion exception", "[exception]"){
        auto dif = DiffDrive(1.3, 0.5);
        CHECK_THROWS(dif.compute_ik(Twist2D{1.57, 0.2 , 1.2}));
    }

}