#include "turtlelib/svg.hpp"
#include <iostream>
#include <sstream>

namespace turtlelib
{
    Point2D center{408.00,528.00};
    double scale {96.0};
    void Svg::openSVGFile(std::ofstream &svgFile, const std::string &filename){
        svgFile.open(filename);

        if (!svgFile.is_open()){
            std::cerr << "Error opening file!" << std::endl;
        }

        svgFile<<"<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";  
    }

    void Svg::create_arrow_def(std::ofstream &svgFile){
        svgFile<<"<defs>\n";
        svgFile<<"<marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\"> <path transform=\"scale(0.2) translate(6,0)\" style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"/> </marker> </defs>\n";
    }

    void Svg::draw_point(std::ofstream &svgFile, Point2D &p, const std::string &stroke_color, const std::string fill_color){
        Point2D pt;
        pt.x = center.x+(p.x*scale);
        pt.y = center.y-(p.y*scale);
        svgFile<<"<circle cx=\""<<pt.x<<"\" cy=\""<<pt.y<<"\" r=\"3\" stroke=\""<<stroke_color<<"\" fill=\""<<fill_color<<"\" stroke-width=\"1\"/>\n";
    }

    void Svg::draw_vector(std::ofstream &svgFile, Point2D &t, Point2D &h, const std::string &stroke_color){
        Point2D tail;
        Vector2D head;
        tail.x = center.x+(t.x*scale);
        tail.y = center.y-(t.y*scale);
        head.x = center.x+(h.x*scale);
        head.y = center.y-(h.y*scale);
        svgFile<<"<line x1=\""<<head.x<<"\" x2=\""<<tail.x<<"\" y1=\""<<head.y<<"\" y2=\""<<tail.y<<"\" stroke=\""<<stroke_color<<"\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
    }
    void Svg::draw_frame(std::ofstream &svgFile, Point2D &origin, double &angle,  const std::string &stroke_color_x,  const std::string &stroke_color_y, const std::string &frame_coord){
        origin.x = center.x+(origin.x*scale);
        origin.y = center.y-(origin.y*scale);
        Point2D xaxis;
        Point2D yaxis;

        xaxis.x = origin.x +(cos(angle)*scale);
        xaxis.y = origin.y -(sin(angle)*scale);
        yaxis.x = origin.x -(sin(angle)*scale);
        yaxis.y = origin.y -(cos(angle)*scale);

        svgFile<<"<g>\n";
        svgFile<<"<line x1=\""<<xaxis.x<<"\" x2=\""<<origin.x<<"\" y1=\""<<xaxis.y<<"\" y2=\""<<origin.y<<"\" stroke=\""<<stroke_color_x<<"\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
        svgFile<<"<line x1=\""<<yaxis.x<<"\" x2=\""<<origin.x<<"\" y1=\""<<yaxis.y<<"\" y2=\""<<origin.y<<"\" stroke=\""<<stroke_color_y<<"\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
        svgFile<<"<text x=\""<<origin.x<<"\" y=\""<<(origin.y+1.0)<<"\"> {"<<frame_coord<<"}</text>\n";
        svgFile<<"</g>\n";
    }
    void Svg::closeSVGFile(std::ofstream &svgFile){
        svgFile<<"</svg>\n";
        svgFile.close();
    }

    std::string Svg::svgstring(const std::string &filename)
    {
        const std::ifstream input_stream(filename, std::ios_base::binary);

        std::stringstream buffer;
        buffer << input_stream.rdbuf();

        return buffer.str();
    }
}