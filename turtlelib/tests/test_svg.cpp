#include <iostream>
#include "turtlelib/svg.hpp"
#include "turtlelib/geometry2d.hpp"

int main() {
    // std::cout << "Hello, World!" << std::endl;
    // return 0;
    turtlelib::Svg testing;
    // turtlelib::Point2D pt{504.2 , 403.5};
    turtlelib::Point2D pt{2.1 , 2.2};
    turtlelib::Point2D tail{1.2 , 3.5};
    turtlelib::Point2D head{2.2 , 4.5};
    turtlelib::Point2D origin{0.00 , 0.00};
    turtlelib::Point2D origin1{1.00 , 1.00};
    turtlelib::Point2D origin2{-1.00 , -1.00};
    double angle = 0.00;
    double angle1 = turtlelib::PI/4;

    std::ofstream svgFile;

    testing.openSVGFile(svgFile, "output.svg");
    testing.create_arrow_def(svgFile);
    testing.draw_point(svgFile, pt, "red", "red");
    testing.draw_vector(svgFile, tail,head, "blue");
    testing.draw_frame(svgFile,origin, angle, "red", "purple","a");
    testing.draw_frame(svgFile,origin1, angle1, "blue", "green","b");
    testing.draw_frame(svgFile,origin2, angle1, "blue", "green","b");
    testing.closeSVGFile(svgFile);

}