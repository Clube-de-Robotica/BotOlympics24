#include "Vec2.h"
#include "math.h"

Vec2::Vec2(){
    x = 0;
    y = 0;
}

Vec2::Vec2(const short xc){
    x = xc;
    y = 0;
}

Vec2::Vec2(const short xc, const short yc){
    x = xc;
    y = yc;
}

/**
 * @brief Returns a vector that represents a cardinal direction.
 * @param dir Up, Down, Left, Right
 */
Vec2 :: Vec2(Direction dir){
    switch(dir){
    case Up:
        x = 0;
        y = -1;
        break;

    case Right:
        x = 1;
        y = 0;
        break;

    case Down:
        x = 0;
        y = 1;
        break;

    case Left:
        x = -1;
        y = 0;
        break;
    }
}

/**
 * @brief Returns the dot product between this vector and v.
 * @param v the other vector to perform the dot product.
 */
int Vec2 :: Dot(const Vec2 v) const{
    return x * v.x + y * v.y;
}

float Vec2 :: Magnitude() const{
    return sqrt(x*x + y*y);
}
/**
 * @brief Returns the vector pointing to the right of this one.
 */
Vec2 Vec2 :: RightTo() const{
    return Vec2(y,-x);
}

Vec2 Vec2 :: operator+(const Vec2 v) const {
    return Vec2(x + v.x, y + v.y);
}

Vec2 Vec2 :: operator+=(const Vec2 v){
    x += v.x;
    y += v.y;
    return *this;
}

Vec2 Vec2 :: operator-(const Vec2 v) const {
    return Vec2(x - v.x, y - v.y);
}

Vec2 Vec2 :: operator-=(const Vec2 v){
    x -= v.x;
    y -= v.y;
    return *this;
}

Vec2 Vec2 :: operator*(const Vec2 v) const {
    return Vec2(x * v.x, y * v.y);
}

Vec2 Vec2 :: operator*(const int i) const {
    return Vec2(x * i, y * i);
}

Vec2 Vec2 :: operator*(const unsigned int i) const {
    return Vec2(x * i, y * i);
}

Vec2 Vec2 :: operator*(const float i) const {
    return Vec2(x * i, y * i);
}

bool Vec2 :: operator==(const Vec2 v) const{
    return x == v.x && y == v.y;
}

bool Vec2 :: operator!=(const Vec2 v) const{
    return x != v.x || y != v.y;
}
