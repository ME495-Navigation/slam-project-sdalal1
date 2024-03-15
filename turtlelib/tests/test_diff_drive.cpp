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

    TEST_CASE("forward ik motion testing", "[forward_ik]"){
        // auto dif = DiffDrive(1.3, 0.5);
        // auto wheel = dif.compute_ik(Twist2D{0.0, 0.2 , 0.0});
        // REQUIRE_THAT(wheel.left, 
        //     Catch::Matchers::WithinAbs(0.4, 1e-5));
        // REQUIRE_THAT(wheel.right, 
        //     Catch::Matchers::WithinAbs(0.4, 1e-5));
        auto dif = DiffDrive(0.8,1.0);
        auto wheel = dif.compute_ik(Twist2D{0.0, 1.0 , 0.0});
        REQUIRE_THAT(wheel.left, 
            Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(wheel.right, 
            Catch::Matchers::WithinAbs(1.0, 1e-5));
    }

     TEST_CASE("forward fk motion testing", "[forward_fk]"){
        auto dif = DiffDrive(1.3, 0.5);
        turtlelib::wheel_positions wheel{0.4,0.4};

        dif.compute_fk(wheel.left,wheel.right);

        REQUIRE_THAT(dif.get_transformation().rotation(), 
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().x, 
            Catch::Matchers::WithinAbs(0.2, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().y, 
            Catch::Matchers::WithinAbs(0.0, 1e-5)); 
    }

    TEST_CASE("rotation ik motion testing", "[rotation_ik]"){
        auto dif = DiffDrive(1.3, 0.5);
        auto wheel = dif.compute_ik(Twist2D{1.57, 0.0 , 0.0});
        REQUIRE_THAT(wheel.left, 
            Catch::Matchers::WithinAbs(-2.041, 1e-5));
        REQUIRE_THAT(wheel.right, 
            Catch::Matchers::WithinAbs(2.041, 1e-5));
    }
    TEST_CASE("rotation ik motion testing", "[rotation_fk]"){   
        auto dif = DiffDrive(1.3, 0.5);
        turtlelib::wheel_positions wheel{-2.041,2.041};
        dif.compute_fk(wheel.left,wheel.right);

        REQUIRE_THAT(dif.get_transformation().rotation(), 
            Catch::Matchers::WithinAbs(1.57, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().x, 
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().y, 
            Catch::Matchers::WithinAbs(0.0, 1e-5)); 
    }

    TEST_CASE("rotation ik motion circle", "[circle_ik]"){
        auto dif = DiffDrive(1.3, 0.5);
        auto wheel = dif.compute_ik(Twist2D{1.57, 0.2 , 0.0});
        REQUIRE_THAT(wheel.left, 
            Catch::Matchers::WithinAbs(-1.641, 1e-5));
        REQUIRE_THAT(wheel.right, 
            Catch::Matchers::WithinAbs(2.441, 1e-5));
    }
    TEST_CASE("rotation fk motion circle", "[circle_fk]"){
        auto dif = DiffDrive(1.3, 0.5);
        turtlelib::wheel_positions wheel{-1.641,2.441};
        dif.compute_fk(wheel.left,wheel.right);
        REQUIRE_THAT(dif.get_transformation().rotation(), 
            Catch::Matchers::WithinAbs(1.57, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().x, 
            Catch::Matchers::WithinAbs(0.1273884946, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().y, 
            Catch::Matchers::WithinAbs(0.1272870921, 1e-5));
    }

    TEST_CASE("rotation motion exception", "[exception]"){
        auto dif = DiffDrive(1.3, 0.5);
        CHECK_THROWS(dif.compute_ik(Twist2D{1.57, 0.2 , 1.2}));
    }

    TEST_CASE("changing trans", "[change_transform]"){
        auto dif = DiffDrive(1.3,0.5);
        Transform2D tr{Vector2D{1.0,2.0}, 1.57};
        dif.change_transform(tr);
        REQUIRE_THAT(dif.get_transformation().rotation(), 
            Catch::Matchers::WithinAbs(1.57, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().x, 
            Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().y, 
            Catch::Matchers::WithinAbs(2.0, 1e-5));
    }
    
    TEST_CASE("all get testing", "[get_testing]"){
        Transform2D tr{Vector2D{1.0,2.0}, 1.57};
        auto dif = DiffDrive(tr,1.3,0.5);
        REQUIRE_THAT(dif.get_transformation().rotation(), 
            Catch::Matchers::WithinAbs(1.57, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().x, 
            Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(dif.get_transformation().translation().y, 
            Catch::Matchers::WithinAbs(2.0, 1e-5));
        REQUIRE_THAT(dif.get_wheel_angle().left,
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dif.get_wheel_angle().right,
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dif.get_wheel_radius(),
            Catch::Matchers::WithinAbs(0.5, 1e-5));
        REQUIRE_THAT(dif.get_wheel_track(),
            Catch::Matchers::WithinAbs(1.3, 1e-5));
        REQUIRE_THAT(dif.get_twist().omega,
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dif.get_twist().x,
            Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(dif.get_twist().y,
            Catch::Matchers::WithinAbs(0.0, 1e-5));
    }

    
}