#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives.


#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if(std::abs(d1-d2)<epsilon){
            return true;
        }
        else{
            return false;
        }
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return deg * PI / 180.0;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad * 180.0 / PI;
    }

    /// \brief wrap an angle to (-PI, PI]
    /// \param rad (angle in radians)
    /// \return an angle equivalent to rad but in the range (-PI, PI]
    double normalize_angle(double rad);

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief a 2-Dimensional Point
    struct Point2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;
    };

    /// \brief output a 2 dimensional point as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param p - the point to print
    std::ostream & operator<<(std::ostream & os, const Point2D & p);

    /// \brief input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param p [out] - output vector
    /// HINT: See operator>> for Vector2D
    std::istream & operator>>(std::istream & is, Point2D & p);

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief Add vector to the existing vector
        /// \param add vector to be added
        /// \return addition of two vectors
        Vector2D& operator+=(Vector2D add);

        /// \brief Subtracting vector to the existing vector
        /// \param sub vector to be added
        /// \return subtraction of two vectors
        Vector2D& operator-=(Vector2D sub);

        /// \brief Multiplicating vector to the existing double 
        /// \param scalar scalar to be multiplied 
        /// \return subtraction of two vectors
        Vector2D& operator*=(double scalar);
    };

    /// \brief Addition of two vectors
    /// \param a first vector
    /// \param b second vector 
    /// \return a vector after addition of both vectors
    Vector2D operator+(Vector2D a, Vector2D b);

    /// \brief Subtracting two vectors
    /// \param a first vector
    /// \param b second vector 
    /// \return a vector after subtraction of both vectors
    Vector2D operator-(Vector2D a, Vector2D b);
    
    /// \brief Multiplication of scalar to a vector
    /// \param scalar first scalar as param
    /// \param vec vector as a second param
    /// \return a vector after multiplicating a scalar with vector 
    Vector2D operator*(double scalar, Vector2D vec);

    /// \brief Multiplication of vector to a scalar
    /// \param vec vector as first param
    /// \param scalar scalar as second param
    /// \return a vector after multiplicating a vector with scalar
    Vector2D operator*(Vector2D vec, double scalar);

    /// \brief Dot product of two vector
    /// \param a vector one
    /// \param b vector two
    /// \return a scalar which is returned after dot product
    double dot(Vector2D a, Vector2D b);

    /// \brief Magnitude of a vector
    /// \param a vector one magnitude
    /// \return scalar magnitude of the vector
    double magnitude(Vector2D a);

    /// \brief An angle between two vectors
    /// \param a vector one
    /// \param b vector two
    /// \return Returns an angle in radians
    double angle(Vector2D a,Vector2D b);

    /// \brief Normalising a vector
    /// \param v A vector to be normalised
    /// \return a unit vector in the same direction as vector v
    Vector2D normalize(Vector2D v);

    /// \brief Subtracting one point from another yields a vector
    /// \param head point corresponding to the head of the vector
    /// \param tail point corresponding to the tail of the vector
    /// \return a vector that points from p1 to p2
    /// NOTE: this is not implemented in terms of -= because subtracting two Point2D yields a Vector2D
    Vector2D operator-(const Point2D & head, const Point2D & tail);

    /// \brief Adding a vector to a point yields a new point displaced by the vector
    /// \param tail The origin of the vector's tail
    /// \param disp The displacement vector
    /// \return the point reached by displacing by disp from tail
    /// NOTE: this is not implemented in terms of += because of the different types
    Point2D operator+(const Point2D & tail, const Vector2D & disp);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// We have lower level control however. For example:
    /// peek looks at the next unprocessed character in the buffer without removing it
    /// get removes the next unprocessed character from the buffer.
    std::istream & operator>>(std::istream & is, Vector2D & v);
}

#endif
