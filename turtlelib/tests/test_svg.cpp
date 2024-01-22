#include <catch2/catch_test_macros.hpp>
#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <iostream>
#include <sstream>

// int main() {
//     // std::cout << "Hello, World!" << std::endl;
//     // return 0;
//     turtlelib::Svg testing;
//     // turtlelib::Point2D pt{504.2 , 403.5};
//     turtlelib::Point2D pt{2.1 , 2.2};
//     turtlelib::Point2D tail{1.2 , 3.5};
//     turtlelib::Point2D head{2.2 , 4.5};
//     turtlelib::Point2D origin{0.00 , 0.00};
//     turtlelib::Point2D origin1{1.00 , 1.00};
//     turtlelib::Point2D origin2{-1.00 , -1.00};
//     double angle = 0.00;
//     double angle1 = turtlelib::PI/4;

//     std::ofstream svgFile;

//     testing.openSVGFile(svgFile, "output.svg");
//     testing.create_arrow_def(svgFile);
//     testing.draw_point(svgFile, pt, "red", "red");
//     testing.draw_vector(svgFile, tail,head, "blue");
//     testing.draw_frame(svgFile,origin, angle, "red", "purple","a");
//     testing.draw_frame(svgFile,origin1, angle1, "blue", "green","b");
//     testing.draw_frame(svgFile,origin2, angle1, "blue", "green","b");
//     testing.closeSVGFile(svgFile);

// }
namespace turtlelib{
TEST_CASE("Svg DrawPoint", "[Svg]") 
{
    turtlelib::Svg testing;
    turtlelib::Point2D pt{2.1 , 2.2};
    turtlelib::Point2D tail{1.2 , 3.5};
    turtlelib::Point2D head{2.2 , 4.5};
    turtlelib::Point2D origin{0.00 , 0.00};
    double angle = 0.00;

    std::ofstream svgFile;

    testing.openSVGFile(svgFile, "test.svg");
    testing.create_arrow_def(svgFile);
    testing.draw_point(svgFile, pt, "red", "red");
    testing.draw_vector(svgFile, tail,head, "blue");
    testing.draw_frame(svgFile,origin, angle, "red", "purple","a");
    testing.closeSVGFile(svgFile);
    std::stringstream os;


    os << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    os << "<defs>\n";
    os << "<marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\"> <path transform=\"scale(0.2) translate(6,0)\" style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"/> </marker> </defs>\n";
    os << "<circle cx=\"609.6\" cy=\"316.8\" r=\"3\" stroke=\"red\" fill=\"red\" stroke-width=\"1\"/>\n";
    os << "<line x1=\"619.2\" x2=\"523.2\" y1=\"96\" y2=\"192\" stroke=\"blue\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
    os << "<g>\n";
    os << "<line x1=\"504\" x2=\"408\" y1=\"528\" y2=\"528\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
    os << "<line x1=\"408\" x2=\"408\" y1=\"432\" y2=\"528\" stroke=\"purple\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
    os << "<text x=\"408\" y=\"529\"> {a}</text>\n";
    os << "</g>\n";
    os << "</svg>\n";


    REQUIRE(testing.svgstring("test.svg") == os.str());
}
}