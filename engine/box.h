#ifndef BOX_H
#define BOX_H

#include <cg3/viewer/interfaces/drawable_object.h>
#include <cg3/geometry/bounding_box3.h>
#include <cg3/viewer/opengl_objects/opengl_objects3.h>
#include <cg3/meshes/eigenmesh/eigenmesh.h>
#include <cg3/utilities/color.h>

class Box3D : public cg3::BoundingBox3, public cg3::DrawableObject{
    public:

        typedef enum {
            LOW =    0b0001,
            MIDDLE = 0b0010,
            HIGH =   0b0100,
            ALL =    0b1000
        } HeightSplit;

        typedef enum {
            ONE_CORNER =      0b000001,
            TWO_CORNERS =     0b000010,
            ONE_EDGE =        0b000100,
            TWO_EDGES =       0b001000,
            TOTALLY_INSIDE =  0b010000,
            TOTALLY_OUTSIDE = 0b100000
        } TypeSplit;

        struct Split {
                HeightSplit hs;
                TypeSplit ts;
        };

        Box3D();
		Box3D(const cg3::Point3d &_min, const cg3::Point3d &_max, const cg3::Point3d &c1 = cg3::Point3d(), const cg3::Point3d &c2 = cg3::Point3d(), const cg3::Point3d &c3 = cg3::Point3d(), const cg3::Color c = cg3::Color(0,0,0));
		Box3D(const cg3::Point3d &_min, const cg3::Point3d &_max, const cg3::Color c);

        void setColor(const cg3::Color& c);
        cg3::Color getColor() const;

		const cg3::Point3d& getConstraint1() const;
		const cg3::Point3d& getConstraint2() const;
		const cg3::Point3d& getConstraint3() const;
		void setConstraint1(const cg3::Point3d &p);
		void setConstraint2(const cg3::Point3d &p);
		void setConstraint3(const cg3::Point3d &p);
        void setRotationMatrix(const Eigen::Matrix3d &rot);
        const Eigen::Matrix3d& getRotationMatrix() const;

        double getVolume() const;
		void getRotatedExtremes(std::vector<cg3::Point3d> &v) const;

        void setW(double d);
        void setH(double d);
        void setD(double d);
        void moveX(double d);
        void moveY(double d);
        void moveZ(double d);
        cg3::SimpleEigenMesh calculateEigenMesh(double minimumEdge = -1) const;
        cg3::SimpleEigenMesh getEigenMesh() const;
        void setEigenMesh(const cg3::SimpleEigenMesh& piece);
        void generateEigenMesh(double minimumEdge = -1);

        // DrawableObject interface
        void draw() const;
		cg3::Point3d sceneCenter() const;
        double sceneRadius() const;

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        void deserialize(std::ifstream& binaryFile);

		const cg3::Vec3d& getTarget() const;
		void setTarget(const cg3::Vec3d& value);
		cg3::Vec3d getRotatedTarget() const;


        int getId() const;
        void setId(int value);

        int getNumberTrianglesCovered() const;

        bool operator <(const Box3D& other) const;

        bool isSplitted() const;
        void setSplitted(bool value);

        static std::string typeSplitToString(const Split& s);
        Split getSplit(const cg3::Box& other);

        const std::set<unsigned int>& getTrianglesCovered() const;
        void setTrianglesCovered(const std::set<unsigned int>& value);
        void addTrianglesCovered(const std::set<unsigned int>& value);

        double getBaseLevel() const;
        void setBaseLevel(double newBase);

    protected:
        //Pointd min, max;
		cg3::Point3d c1, c2, c3;
        cg3::Color color;
        bool visible;
		cg3::Vec3d target;
        Eigen::Matrix3d rotation;
        int id;
        cg3::SimpleEigenMesh piece;
        bool splitted;
        std::set<unsigned int> trianglesCovered;

		void drawLine(const cg3::Point3d& a, const cg3::Point3d& b, const cg3::Color& c) const;
        void drawCube() const;

        unsigned int getTargetIndex();
};

inline void Box3D::setColor(const cg3::Color& c) {
    color = c;
}

inline cg3::Color Box3D::getColor() const {
    return color;
}

/*inline const Pointd& Box3D::getMin() const {
    return min;
}

inline void Box3D::setMin(const Pointd& value) {
    min = value;
}

inline const Pointd& Box3D::getMax() const {
    return max;
}

inline void Box3D::setMax(const Pointd& value) {
    max = value;
}*/

inline const cg3::Point3d& Box3D::getConstraint1() const {
    return c1;
}

inline const cg3::Point3d& Box3D::getConstraint2() const {
    return c2;
}

inline const cg3::Point3d& Box3D::getConstraint3() const {
    return c3;
}

inline const Eigen::Matrix3d&Box3D::getRotationMatrix() const {
    return rotation;
}

inline double Box3D::getVolume() const {
	return (_max.x()-_min.x())*(_max.y()-_min.y())*(_max.z()-_min.z());
}

inline void Box3D::setW(double d) {
	_max.setX(_min.x()+d);
}

inline void Box3D::setH(double d) {
	_max.setY(_min.y()+d);
}

inline void Box3D::setD(double d) {
	_max.setZ(_min.z()+d);
}

inline void Box3D::moveX(double d) {
	_min.setX(_min.x() + d);
	_max.setX(_max.x() + d);
}

inline void Box3D::moveY(double d) {
	_min.setY(_min.y() + d);
	_max.setY(_max.y() + d);
}

inline void Box3D::moveZ(double d) {
	_min.setZ(_min.z() + d);
	_max.setZ(_max.z() + d);
}

inline int Box3D::getId() const {
    return id;
}

inline void Box3D::setId(int value) {
    id = value;
}

inline int Box3D::getNumberTrianglesCovered() const {
    return trianglesCovered.size();
}

#endif // BOX_H
