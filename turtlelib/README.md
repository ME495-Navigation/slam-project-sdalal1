# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input
- svg - creates visual representation of the frames and vector as a SVG file
- diff_drive - handles the motion kinematics of the diff drive robot including forward and inverse kinematics

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   - Which of the methods would you implement and why?

      - Create a free function which takes in a Vector as an object and return another Vector object which is normalised
         - Pros - This is the most sinple way to do it as the function looks self sufficient. It goes in accordance with c++ guideline F.10 due to the reusability. F.3 will also be satisfied as the function is concise
         - Cons - Needs input to be put in by value which can be messed up by the user 
      - Create a member function of the Vector2D (in struct) that changes the value internally of the object called on
         - Pros - Has direct access to the member data points
         - Cons - This can mean that the member itself is changed implicitly which is not what we require. C4 guideline is not met here as it doesnt need direct access to the members
      - Create an operator overload which converts a type Vector 2D to a normalized vector
         - Pros- Ease of use, can be called concisely.
         - Cons - Finding a specific operator to overload which foes with the C.160 guideline of mimic conventional usage will nto work here. The only operator which can closely relate is `^` which is used for bitwise operation but not conventional.

      - I would prefer using implementation 1 due to the independence from any other member and the reusability of the function.

2. What is the difference between a class and a struct in C++?
    - A struct is default public while tje class is default private in C++

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
   - Guideline C2 - The data members of the Vector2D can vary independtly and is a Struct
   - Guideline C8 - As the Transform2D consist of private memebers, it is a class

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
   - Guideine C46 - Declare single-argument constuctors as explicit. This removes the issue with unintended conversion. They two examples we have are Transform2D constructors taking in a translation or rotation object.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
   - Guideline Con4 - The value of an inverse doesnt change after being constructed as the transform2D object doesnt change. The *= operator changes the value of the after construction and modifies the values inside
