#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Svg sclass for visualisation

#include <iosfwd>
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include <string>
#include <iostream>
#include <fstream>

namespace turtlelib{
    /// \brief Class to create a SVG file
    class Svg
    {
    public:
        /// \brief Opens the svg file
        /// \param svgFile the file stream to open and edit the file
        /// \param filename the name of the file to be used
        /// \returns nothing
        void openSVGFile(std::ofstream &svgFile, const std::string &filename);

        /// \brief creates the definition of the arrow at thhe end of a vector
        /// \param svgFile the file stream to open and edit the file
        /// \returns nothing
        void create_arrow_def(std::ofstream &svgFile);
        
        /// \brief creates points in respect to the center of the screen
        /// \param svgFile the file stream to open and edit the file
        /// \param p [a point2d object holding the x and y position
        /// \param stroke_color the stroke color of the point
        /// \param fill_color the fill color of the point
        /// \returns nothing
        void draw_point(std::ofstream &svgFile, Point2D &p, const std::string &stroke_color, const std::string fill_color); // & fill_color
        
        /// \brief creates vector component wrt to the center of the screen
        /// \param svgFile the file stream to open and edit the file
        /// \param tail Point2d object holding x and y of the tail of the vector
        /// \param head Point2d object holding x and y of the head of the vector
        /// \param stroke_color the stroke color of the vector
        /// \returns nothing
        void draw_vector(std::ofstream &svgFile, Point2D &tail, Point2D &head, const std::string &stroke_color);
        
        /// \brief creates frame component wrt to the center of the screen
        /// \param svgFile the file stream to open and edit the file
        /// \param origin Point2d object holding x and y of the tail of the vector
        /// \param angle Point2d object holding x and y of the head of the vector
        /// \param stroke_color_x the stroke color of the x axis
        /// \param stroke_color_y the stroke color of the y axis
        /// \param frame_coord the label for the coordinate axis
        /// \returns nothing
        void draw_frame(std::ofstream &svgFile, Point2D &origin, double &angle, const std::string &stroke_color_x,  const std::string &stroke_color_y, const std::string &frame_coord);
        
        /// \brief closes the svg file with the tag
        /// \param svgFile the file stream to open and edit the file
        /// \returns nothing
        void closeSVGFile(std::ofstream &svgFile); // is the file in a valid state after this is called?
        
        /// \brief Creates a string
        /// \param filename to open the right file
        /// \returns a string for comaprision of function
        std::string svgstring(const std::string &filename);
    };
};


#endif
