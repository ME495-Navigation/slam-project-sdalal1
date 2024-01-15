#include <iostream>
#include "turtlelib/svg.hpp"
#include "turtlelib/geometry2d.hpp"

int main() {
    // std::cout << "Hello, World!" << std::endl;
    // return 0;
    turtlelib::Svg testing;
    turtlelib::Point2D pt{504.2 , 403.5};
    turtlelib::Point2D tail{504.2 , 403.5};
    turtlelib::Point2D head{504.2 , 483.5};
    turtlelib::Point2D origin{408.00 , 528.00};
    double xaxis = 504.00;
    double yaxis = 432.00;
    std::ofstream svgFile;

    testing.openSVGFile(svgFile, "output.svg");
    testing.create_arrow_def(svgFile);
    testing.draw_point(svgFile, pt, "red", "red");
    testing.draw_vector(svgFile, tail,head, "blue");
    testing.draw_frame(svgFile,origin, xaxis, yaxis, "red", "purple","a");
    testing.closeSVGFile(svgFile);

}