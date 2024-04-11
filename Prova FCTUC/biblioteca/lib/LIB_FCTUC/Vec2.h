#ifndef VEC2_H
#define VEC2_H

/*
 * - By Martim Pegueiro aka LittleNyanCat. 2023/2024
*/

enum Direction {Up, Right, Down, Left};

class Vec2{

public:
    short x, y;

    Vec2();
    Vec2(const short);
    Vec2(const short, const short);
    Vec2(Direction);

    int Dot(const Vec2) const;

    float Magnitude() const;
    
    Vec2 RightTo() const;   //vector that is to the right of this one

    Vec2 operator+(const Vec2) const;
    Vec2 operator+=(const Vec2);
    Vec2 operator-(const Vec2) const;
    Vec2 operator-=(const Vec2);
    Vec2 operator*(const Vec2) const;
    Vec2 operator*(const int) const;
    Vec2 operator*(const unsigned int) const;
    Vec2 operator*(const float) const;
    float operator^(const Vec2) const; //angle between two vectors, in radians

    bool operator==(const Vec2) const;
    bool operator!=(const Vec2) const;
};



#endif // VEC2_H
