#include <iostream>
#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

int main() {
    turtlelib::Transform2D T_ab, T_bc , T_ba, T_cb, T_ac, T_ca;
    turtlelib::Point2D p_a, p_b,p_c;
    turtlelib::Vector2D v_a,v_b,v_c,v_bhat;
    turtlelib::Twist2D V_a, V_b, V_c;

    std::cout<<"Enter transform T_{a,b}:"<<"\n";
    std::cin >> T_ab;
    std::cout<<"Enter transform T_{b,c}:"<<"\n";
    std::cin >> T_bc;

    T_ba = T_ab.inv();
    T_cb = T_bc.inv();
    T_ac = T_ab*T_bc;
    T_ca = T_ac.inv();    

    std::cout<<"T_{a,b}: "<< T_ab <<"\n";
    std::cout<<"T_{b,a}: "<< T_ba<<"\n";
    std::cout<<"T_{b,c}: "<< T_bc<<"\n";
    std::cout<<"T_{c,b}: "<< T_cb<<"\n";
    std::cout<<"T_{a,c}: "<< T_ac<<"\n";
    std::cout<<"T_{c,a}: "<< T_ca<<"\n";

    std::cout<<"Enter point p_a:"<<"\n";
    std::cin>>p_a;

    p_b = T_ba.operator()(p_a);
    p_c = T_ca.operator()(p_a);

    std::cout<<"p_a: "<<p_a<<"\n";
    std::cout<<"p_b: "<<p_b<<"\n";
    std::cout<<"p_c: "<<p_c<<"\n";

    std::cout<<"Enter vector v_b:"<<"\n";
    std::cin>>v_b;

    v_a = T_ab.operator()(v_b);
    v_c = T_cb.operator()(v_b);

    v_bhat.x = v_b.x/(std::sqrt(pow(v_b.x,2) + pow(v_b.y,2)));
    v_bhat.y = v_b.y/(std::sqrt(pow(v_b.x,2) + pow(v_b.y,2)));

    std::cout<<"v_bhat: "<<v_bhat<<"\n";
    std::cout<<"v_a: "<<v_a<<"\n";
    std::cout<<"v_b: "<<v_b<<"\n";
    std::cout<<"v_c: "<<v_c<<"\n";

    std::cout<<"Enter twist V_b:"<<"\n";
    std::cin>>V_b;

    V_a = T_ab.operator()(V_b);
    V_c = T_cb.operator()(V_b);

    std::cout<<"V_a: "<<V_a<<"\n";
    std::cout<<"V_b: "<<V_b<<"\n";
    std::cout<<"V_c: "<<V_c<<"\n";


    turtlelib::Svg testing;
    // turtlelib::Point2D pt{504.2 , 403.5};
    turtlelib::Point2D pta{p_a.x , p_a.y};
    turtlelib::Point2D ptc{T_ab.operator()(p_b).x , T_ab.operator()(p_b).y};
    turtlelib::Point2D ptb{T_ac.operator()(p_c).x , T_ac.operator()(p_c).y};
    turtlelib::Point2D vbhat{v_bhat.x , v_bhat.y};
    turtlelib::Point2D va{v_a.x, v_a.y};
    turtlelib::Point2D vc{v_c.x, v_c.y};
    
    

    turtlelib::Point2D tailb{T_ab.translation().x ,  T_ab.translation().y };
    turtlelib::Point2D taila{0.0,  0.0 };
    turtlelib::Point2D tailc{T_ac.translation().x,  T_ac.translation().y };

    
    turtlelib::Point2D headb{T_ab.operator()(vbhat).x , T_ab.operator()(vbhat).y};
    turtlelib::Point2D heada{va.x , va.y};
    turtlelib::Point2D headc{T_ac.operator()(vc).x , T_ac.operator()(vc).y};
    
    turtlelib::Point2D origina{0.00 , 0.00};
    turtlelib::Point2D originb{T_ab.translation().x, T_ab.translation().y };
    turtlelib::Point2D originc{T_ac.translation().x , T_ac.translation().y};
    double angle = 0.00;
    double angle1 = T_ab.rotation();
    double angle2 = T_ac.rotation();
    

    std::ofstream svgFile;

    testing.openSVGFile(svgFile, "/tmp/frames.svg");
    testing.create_arrow_def(svgFile);
    testing.draw_point(svgFile, pta, "purple", "purple");
    testing.draw_point(svgFile, ptb, "brown", "brown");
    testing.draw_point(svgFile, ptc, "orange", "orange");
    
    testing.draw_vector(svgFile, tailb,headb, "brown");
    testing.draw_vector(svgFile, taila,heada, "purple");
    testing.draw_vector(svgFile, tailc,headc, "orange");

    
    testing.draw_frame(svgFile,origina, angle, "red", "purple","a");
    testing.draw_frame(svgFile,originb, angle1, "blue", "green","b");
    testing.draw_frame(svgFile,originc, angle2, "black", "red","c");
    testing.closeSVGFile(svgFile);

}