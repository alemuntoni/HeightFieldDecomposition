/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef MYCOMMON_H
#define MYCOMMON_H

#include "common/utils.h"
#include "dcel/dcel.h"
#include <set>
#include <Eigen/Core>
#include <memory>

#define BORDER_PAY 5
#define STD_PAY 0
#define MIN_PAY -10
#define MAX_PAY 800

#define FLIP_ANGLE 0//-0.2588190451
#define LENGTH_MULTIPLIER 7
//#define USE_2D_ONLY

#define ONE_ON_SQRT2 1/(sqrt(2))
#define ONE_ON_SQRT3 1/(sqrt(3))

#define RED_ID 0
#define GREEN_ID 1
#define BLUE_ID 2
#define YELLOW_ID 6
#define MAGENTA_ID 10
#define CYANO_ID 14
#define WHITE_ID 18

#define IS_RED(label) ((label == 0) || (label==3))
#define IS_GREEN(label) ((label == 1) || (label==4))
#define IS_BLUE(label) ((label == 2) || (label==5))
#define IS_YELLOW(label) ((label >= 6) && (label < 10))
#define IS_MAGENTA(label) ((label >= 10) && (label < 14))
#define IS_CYANO(label) ((label >= 14) && (label < 18))
#define IS_WHITE(label) ((label >= 18) && (label < 26))

extern int WINDOW_MANAGER_ID;
extern int DCEL_MANAGER_ID;
extern int ENGINE_MANAGER_ID;

static const std::vector<Vec3> XYZ = {
    Vec3( 1.0f,  0.0f,  0.0f),                         //    +X : label  0
    Vec3( 0.0f,  1.0f,  0.0f),                         //    +Y : label  1
    Vec3( 0.0f,  0.0f,  1.0f),                         //    +Z : label  2
    Vec3(-1.0f,  0.0f,  0.0f),                         //    -X : label  3
    Vec3( 0.0f, -1.0f,  0.0f),                         //    -Y : label  4
    Vec3( 0.0f,  0.0f, -1.0f),                         //    -Z : label  5
    Vec3( ONE_ON_SQRT2,  ONE_ON_SQRT2,  0.0f),         //   +XY : label  6
    Vec3(-ONE_ON_SQRT2,  ONE_ON_SQRT2,  0.0f),         //  -X+Y : label  7
    Vec3(-ONE_ON_SQRT2, -ONE_ON_SQRT2,  0.0f),         //   -XY : label  8
    Vec3( ONE_ON_SQRT2, -ONE_ON_SQRT2,  0.0f),         //  +X-Y : label  9
    Vec3( ONE_ON_SQRT2,  0.0f,  ONE_ON_SQRT2),         //   +XZ : label 10
    Vec3(-ONE_ON_SQRT2,  0.0f,  ONE_ON_SQRT2),         //  -X+Z : label 11
    Vec3(-ONE_ON_SQRT2,  0.0f, -ONE_ON_SQRT2),         //   -XZ : label 12
    Vec3( ONE_ON_SQRT2,  0.0f, -ONE_ON_SQRT2),         //  +X-Z : label 13
    Vec3( 0.0f,  ONE_ON_SQRT2,  ONE_ON_SQRT2),         //   +YZ : label 14
    Vec3( 0.0f, -ONE_ON_SQRT2,  ONE_ON_SQRT2),         //  -Y+Z : label 15
    Vec3( 0.0f, -ONE_ON_SQRT2, -ONE_ON_SQRT2),         //   -YZ : label 16
    Vec3( 0.0f,  ONE_ON_SQRT2, -ONE_ON_SQRT2),         //  +Y-Z : label 17
    Vec3( ONE_ON_SQRT3,  ONE_ON_SQRT3,  ONE_ON_SQRT3), //  +XYZ : label 18
    Vec3(-ONE_ON_SQRT3, -ONE_ON_SQRT3, -ONE_ON_SQRT3), //  -XYZ : label 19
    Vec3( ONE_ON_SQRT3, -ONE_ON_SQRT3, -ONE_ON_SQRT3), // +X-YZ : label 20
    Vec3(-ONE_ON_SQRT3,  ONE_ON_SQRT3,  ONE_ON_SQRT3), // -X+YZ : label 21
    Vec3( ONE_ON_SQRT3,  ONE_ON_SQRT3, -ONE_ON_SQRT3), // +XY-Z : label 22
    Vec3(-ONE_ON_SQRT3, -ONE_ON_SQRT3,  ONE_ON_SQRT3), // -XY+Z : label 23
    Vec3( ONE_ON_SQRT3, -ONE_ON_SQRT3,  ONE_ON_SQRT3), //+X-Y+Z : label 24
    Vec3(-ONE_ON_SQRT3,  ONE_ON_SQRT3, -ONE_ON_SQRT3)  //-X+Y-Z : label 25
};

static const std::vector<Color> colors = {
    Color(255, 0, 0),    //     Red: label 0
    Color(0, 255, 0),    //   Green: label 1
    Color(0, 0, 255),    //    Blue: label 2
    Color(255, 0, 0),    //     Red: label 3
    Color(0, 255, 0),    //   Green: label 4
    Color(0, 0, 255),    //    Blue: label 5
    Color(255, 255, 0),    //  Yellow: label 6
    Color(255, 255, 0),    //  Yellow: label 7
    Color(255, 255, 0),    //  Yellow: label 8
    Color(255, 255, 0),    //  Yellow: label 9
    Color(255, 0, 255),    // Magenta: label 10
    Color(255, 0, 255),    // Magenta: label 11
    Color(255, 0, 255),    // Magenta: label 12
    Color(255, 0, 255),    // Magenta: label 13
    Color(0, 255, 255),    //   Cyano: label 14
    Color(0, 255, 255),    //   Cyano: label 15
    Color(0, 255, 255),    //   Cyano: label 16
    Color(0, 255, 255),    //   Cyano: label 17
    Color(200, 200, 200),    //   White: label 18
    Color(200, 200, 200),    //   White: label 19
    Color(200, 200, 200),    //   White: label 20
    Color(200, 200, 200),    //   White: label 21
    Color(200, 200, 200),    //   White: label 22
    Color(200, 200, 200),    //   White: label 23
    Color(200, 200, 200),    //   White: label 24
    Color(200, 200, 200)     //   White: label 25
};

Color colorOfNormal(const Vec3 &normal);
Vec3 nearestNormal(const Vec3& normal);
Color colorOfNearestNormal(const Vec3& normal);
int indexOfNormal(const Vec3& v);

#endif // MYCOMMON_H
