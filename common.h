/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef MYCOMMON_H
#define MYCOMMON_H

#include "cg3/utilities/utils.h"
#include "cg3/meshes/dcel/dcel.h"
#include <set>
#include <Eigen/Core>
#include <memory>
#include <cg3/viewer/mainwindow.h>

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

extern int DCEL_MANAGER_ID;
extern int ENGINE_MANAGER_ID;

extern MainWindow* mw;

const int MAX_BFGS_ITERATIONS = 150;

static const std::vector<cg3::Vec3> XYZ = {
    cg3::Vec3( 1.0f,  0.0f,  0.0f),                         //    +X : label  0
    cg3::Vec3( 0.0f,  1.0f,  0.0f),                         //    +Y : label  1
    cg3::Vec3( 0.0f,  0.0f,  1.0f),                         //    +Z : label  2
    cg3::Vec3(-1.0f,  0.0f,  0.0f),                         //    -X : label  3
    cg3::Vec3( 0.0f, -1.0f,  0.0f),                         //    -Y : label  4
    cg3::Vec3( 0.0f,  0.0f, -1.0f),                         //    -Z : label  5
    cg3::Vec3( ONE_ON_SQRT2,  ONE_ON_SQRT2,  0.0f),         //   +XY : label  6
    cg3::Vec3(-ONE_ON_SQRT2,  ONE_ON_SQRT2,  0.0f),         //  -X+Y : label  7
    cg3::Vec3(-ONE_ON_SQRT2, -ONE_ON_SQRT2,  0.0f),         //   -XY : label  8
    cg3::Vec3( ONE_ON_SQRT2, -ONE_ON_SQRT2,  0.0f),         //  +X-Y : label  9
    cg3::Vec3( ONE_ON_SQRT2,  0.0f,  ONE_ON_SQRT2),         //   +XZ : label 10
    cg3::Vec3(-ONE_ON_SQRT2,  0.0f,  ONE_ON_SQRT2),         //  -X+Z : label 11
    cg3::Vec3(-ONE_ON_SQRT2,  0.0f, -ONE_ON_SQRT2),         //   -XZ : label 12
    cg3::Vec3( ONE_ON_SQRT2,  0.0f, -ONE_ON_SQRT2),         //  +X-Z : label 13
    cg3::Vec3( 0.0f,  ONE_ON_SQRT2,  ONE_ON_SQRT2),         //   +YZ : label 14
    cg3::Vec3( 0.0f, -ONE_ON_SQRT2,  ONE_ON_SQRT2),         //  -Y+Z : label 15
    cg3::Vec3( 0.0f, -ONE_ON_SQRT2, -ONE_ON_SQRT2),         //   -YZ : label 16
    cg3::Vec3( 0.0f,  ONE_ON_SQRT2, -ONE_ON_SQRT2),         //  +Y-Z : label 17
    cg3::Vec3( ONE_ON_SQRT3,  ONE_ON_SQRT3,  ONE_ON_SQRT3), //  +XYZ : label 18
    cg3::Vec3(-ONE_ON_SQRT3, -ONE_ON_SQRT3, -ONE_ON_SQRT3), //  -XYZ : label 19
    cg3::Vec3( ONE_ON_SQRT3, -ONE_ON_SQRT3, -ONE_ON_SQRT3), // +X-YZ : label 20
    cg3::Vec3(-ONE_ON_SQRT3,  ONE_ON_SQRT3,  ONE_ON_SQRT3), // -X+YZ : label 21
    cg3::Vec3( ONE_ON_SQRT3,  ONE_ON_SQRT3, -ONE_ON_SQRT3), // +XY-Z : label 22
    cg3::Vec3(-ONE_ON_SQRT3, -ONE_ON_SQRT3,  ONE_ON_SQRT3), // -XY+Z : label 23
    cg3::Vec3( ONE_ON_SQRT3, -ONE_ON_SQRT3,  ONE_ON_SQRT3), //+X-Y+Z : label 24
    cg3::Vec3(-ONE_ON_SQRT3,  ONE_ON_SQRT3, -ONE_ON_SQRT3)  //-X+Y-Z : label 25
};

static const std::vector<cg3::Color> colors = {
    cg3::Color(255, 0, 0),    //     Red: label 0
    cg3::Color(0, 255, 0),    //   Green: label 1
    cg3::Color(0, 0, 255),    //    Blue: label 2
    cg3::Color(255, 0, 0),    //     Red: label 3
    cg3::Color(0, 255, 0),    //   Green: label 4
    cg3::Color(0, 0, 255),    //    Blue: label 5
    cg3::Color(255, 255, 0),    //  Yellow: label 6
    cg3::Color(255, 255, 0),    //  Yellow: label 7
    cg3::Color(255, 255, 0),    //  Yellow: label 8
    cg3::Color(255, 255, 0),    //  Yellow: label 9
    cg3::Color(255, 0, 255),    // Magenta: label 10
    cg3::Color(255, 0, 255),    // Magenta: label 11
    cg3::Color(255, 0, 255),    // Magenta: label 12
    cg3::Color(255, 0, 255),    // Magenta: label 13
    cg3::Color(0, 255, 255),    //   Cyano: label 14
    cg3::Color(0, 255, 255),    //   Cyano: label 15
    cg3::Color(0, 255, 255),    //   Cyano: label 16
    cg3::Color(0, 255, 255),    //   Cyano: label 17
    cg3::Color(200, 200, 200),    //   White: label 18
    cg3::Color(200, 200, 200),    //   White: label 19
    cg3::Color(200, 200, 200),    //   White: label 20
    cg3::Color(200, 200, 200),    //   White: label 21
    cg3::Color(200, 200, 200),    //   White: label 22
    cg3::Color(200, 200, 200),    //   White: label 23
    cg3::Color(200, 200, 200),    //   White: label 24
    cg3::Color(200, 200, 200)     //   White: label 25
};

cg3::Color colorOfNormal(const cg3::Vec3 &normal);
cg3::Vec3 nearestNormal(const cg3::Vec3& normal);
cg3::Color colorOfNearestNormal(const cg3::Vec3& normal);
int indexOfNormal(const cg3::Vec3& v);

#endif // MYCOMMON_H
