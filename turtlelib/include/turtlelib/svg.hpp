#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Svg sclass for visualisation

#include<iosfwd>
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include <iostream>
#include <fstream>

namespace turtlelib{
    class Svg
    {
    public:
        void openSVGFile(std::ofstream &svgFile, const std::string &filename);
        void create_arrow_def(std::ofstream &svgFile);
        void draw_point(std::ofstream &svgFile, Point2D &p, const std::string &stroke_color, const std::string fill_color);
        void draw_vector(std::ofstream &svgFile, Point2D &tail, Vector2D &head, const std::string &stroke_color);
        void draw_frame(std::ofstream &svgFile, Point2D &origin, double &xaxis, double &yaxis, const std::string &stroke_color_x,  const std::string &stroke_color_y, const std::string &frame_coord);
        void closeSVGFile(std::ofstream &svgFile);
    };
}


#endif