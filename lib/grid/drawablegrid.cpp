#include "drawablegrid.h"
#include <omp.h>

using namespace cg3;

DrawableGrid::DrawableGrid() : drawMode(DRAW_KERNEL), slice(NO_SLICE), sliceValue(0), stepDrawGrid(2){
}

DrawableGrid::DrawableGrid(const Grid& g) : Grid(g), drawMode(DRAW_KERNEL), slice(NO_SLICE), sliceValue(0), stepDrawGrid(2){
    minSignedDistance = signedDistances.min();
}

DrawableGrid::DrawableGrid(const Point3i& resolution, const Array3D<Point3d>& gridCoordinates, const Array3D<gridreal>& signedDistances, const Point3d& gMin, const Point3d& gMax) :
    Grid(resolution, gridCoordinates, signedDistances, gMin, gMax), drawMode(DRAW_KERNEL), slice(NO_SLICE), sliceValue(0), stepDrawGrid(2) {
    minSignedDistance = signedDistances.min();
}

DrawableGrid::~DrawableGrid(){
}

double DrawableGrid::getKernelDistance() const {
    return kernelDistance;
}

void DrawableGrid::setKernelDistance(double value) {
    assert(value >= 0 && value <= 1);
    value = 1 - value;
    value *= minSignedDistance;
    kernelDistance = std::abs(value);
}

void DrawableGrid::setDrawKernel() {
    drawMode = DRAW_KERNEL;
}

void DrawableGrid::setDrawBorders() {
    drawMode = DRAW_WEIGHTS;
}

void DrawableGrid::setSlice(int value) {
    slice = value;
    sliceValue = 0;
}

void DrawableGrid::setSliceValue(int value) {
    switch(slice){
        case X_SLICE:
            assert (value < (int)resX);
            break;
        case Y_SLICE:
            assert (value < (int)resY);
            break;
        case Z_SLICE:
            assert (value < (int)resZ);
            break;
    }
    sliceValue = value;
}

double DrawableGrid::getHsvHFactor(double w) const {
    if (w>MAX_PAY) w = MAX_PAY;
    if (w<MIN_PAY) w = MIN_PAY;
    double value;
    if (w<0)
        value =((w - MIN_PAY)/(0-MIN_PAY))/2;
    else
        value =((w - 0)/(MAX_PAY-0))/2 + 0.5;
    return 1 - value;
    return 1-((w - MIN_PAY)/(MAX_PAY-MIN_PAY));
}

double DrawableGrid::getHsvVFactor(double w) const {
    if (w > MAX_PAY) {
        return 1-(w - MAX_PAY)/(675-MAX_PAY);
    }
    else if (w < MIN_PAY) {
        return 1-(w+140)/(MIN_PAY+140);
    }
    else return 1;
}

void DrawableGrid::draw() const {
    double xi, yi, zi;
    switch (drawMode){
        case DRAW_KERNEL:
            switch (slice){
                case NO_SLICE:
					for (unsigned int i = 0; i < getResX(); ++i){
						for (unsigned int j = 0; j < getResY(); ++j){
							for (unsigned int k = 0; k < getResZ(); ++k){
                                if (getSignedDistance(i,j,k) < -kernelDistance){
                                    opengl::drawSphere(getPoint(i,j,k), 0.3, QColor(255,0,0));
                                }
                            }
                        }
                    }
                    break;
                case X_SLICE:
					for (unsigned int j = 0; j < getResY(); ++j){
						for (unsigned int k = 0; k < getResZ(); ++k){
                            if (getSignedDistance(sliceValue,j,k) < -kernelDistance){
                                opengl::drawSphere(getPoint(sliceValue,j,k), 0.3, QColor(255,0,0));
                            }
                        }
                    }
                    break;
                case Y_SLICE:
					for (unsigned int i = 0; i < getResX(); ++i){
						for (unsigned int k = 0; k < getResZ(); ++k){
                            if (getSignedDistance(i,sliceValue,k) < -kernelDistance){
                                opengl::drawSphere(getPoint(i,sliceValue,k), 0.3, QColor(255,0,0));
                            }
                        }
                    }
                    break;
                case Z_SLICE:
					for (unsigned int i = 0; i < getResX(); ++i){
						for (unsigned int j = 0; j < getResY(); ++j){
                            if (getSignedDistance(i,j,sliceValue) < -kernelDistance){
                                opengl::drawSphere(getPoint(i,j,sliceValue), 0.3, QColor(255,0,0));
                            }
                        }
                    }
                    break;
            }
            break;
        case DRAW_WEIGHTS:
            switch (slice){
                case NO_SLICE:
					for (unsigned int i = 0; i < getResX(); i+=2){
						for (unsigned int j = 0; j < getResY(); j+=2){
							for (unsigned int k = 0; k < getResZ(); k+=2){
                                double w = getWeight(i,j,k);
                                QColor c; c.setHsv(getHsvHFactor(w)*240,255,255);
                                opengl::drawSphere(getPoint(i,j,k), 0.3, c);
                            }
                        }
                    }
                    break;
                case X_SLICE:
					for (unsigned int j = 0; j < getResY(); ++j){
						for (unsigned int k = 0; k < getResZ(); ++k){
                            double w = getWeight(sliceValue,j,k);
                            QColor c; c.setHsv(getHsvHFactor(w)*240,255,255);
                            opengl::drawSphere(getPoint(sliceValue,j,k), 0.4, c);
                        }
                    }
                    xi = getPoint(sliceValue, 0, 0).x();
					for (yi = bb.minY(); yi <= bb.maxY(); yi+=stepDrawGrid){
						for (zi = bb.minZ(); zi <= bb.maxZ(); zi+=stepDrawGrid){
							double w = getValue(Point3d(xi,yi,zi));
                            QColor c;
                            c.setHsv(getHsvHFactor(w)*240,255,getHsvVFactor(w)*255);
							opengl::drawSphere(Point3d(xi,yi,zi), 0.2, c);
                        }
                    }
                    break;
                case Y_SLICE:
					for (unsigned int i = 0; i < getResX(); ++i){
						for (unsigned int k = 0; k < getResZ(); ++k){
                            double w = getWeight(i,sliceValue,k);
                            QColor c; c.setHsv(getHsvHFactor(w)*240,255,255);
                            opengl::drawSphere(getPoint(i,sliceValue,k), 0.4, c);
                        }
                    }
                    yi = getPoint(0,sliceValue,0).y();
					for (xi = bb.minX(); xi <= bb.maxX(); xi+=stepDrawGrid){
						for (zi = bb.minZ(); zi <= bb.maxZ(); zi+=stepDrawGrid){
							double w = getValue(Point3d(xi,yi,zi));
                            QColor c;
                            c.setHsv(getHsvHFactor(w)*240,255,getHsvVFactor(w)*255);
							opengl::drawSphere(Point3d(xi,yi,zi), 0.2, c);
                        }
                    }
                    break;
                case Z_SLICE:
					for (unsigned int i = 0; i < getResX(); ++i){
						for (unsigned int j = 0; j < getResY(); ++j){
                            double w = getWeight(i,j,sliceValue);
                            QColor c; c.setHsv(getHsvHFactor(w)*240,255,255);
                            opengl::drawSphere(getPoint(i,j,sliceValue), 0.4, c);
                        }
                    }
                    zi = getPoint(0,0,sliceValue).z();
					for (xi = bb.minX(); xi <= bb.maxX(); xi+=stepDrawGrid){
						for (yi = bb.minY(); yi <= bb.maxY(); yi+=stepDrawGrid){
							double w = getValue(Point3d(xi,yi,zi));
                            QColor c;
                            c.setHsv(getHsvHFactor(w)*240,255,getHsvVFactor(w)*255);
							opengl::drawSphere(Point3d(xi,yi,zi), 0.2, c);
                        }
                    }
                    break;
            }
            break;
        default:
            assert(0);
    }

    for (unsigned int i = 0; i < cubes.size(); ++i){
        drawCube(cubes[i]);
    }
}

Point3d DrawableGrid::sceneCenter() const {
    return bb.center();
}

double DrawableGrid::sceneRadius() const {
   return bb.diag() / 2;
}

void DrawableGrid::addCube(const BoundingBox3& bb) {
    cubes.push_back(bb);
}

void DrawableGrid::deleteCubes() {
    cubes.clear();
}

void DrawableGrid::drawLine(const Point3d &a, const Point3d &b) const {
    glBegin(GL_LINES);
    glColor3f(0.0, 0.0, 0.0);
    glVertex3f(a.x(), a.y(), a.z());
    glVertex3f(b.x(), b.y(), b.z());
    glEnd();
}

void DrawableGrid::drawCube(const BoundingBox3&b) const {
	Point3d to(b.minX(), b.minY(), b.maxZ());
	drawLine(b.min(), to);
	to.set(b.minX(), b.maxY(), b.minZ());
	drawLine(b.min(), to);
	to.set(b.maxX(), b.minY(), b.minZ());
	drawLine(b.min(), to);

	to.set(b.maxX(), b.maxY(), b.minZ());
	drawLine(b.max(), to);
	to.set(b.maxX(), b.minY(), b.maxZ());
	drawLine(b.max(), to);
	to.set(b.minX(), b.maxY(), b.maxZ());
	drawLine(b.max(), to);

	Point3d from(b.minX(), b.minY(), b.maxZ());
	to.set(b.minX(), b.maxY(), b.maxZ());
    drawLine(from, to);
	from.set(b.minX(), b.minY(), b.maxZ());
	to.set(b.maxX(), b.minY(), b.maxZ());
    drawLine(from, to);

	from.set(b.minX(), b.maxY(), b.minZ());
	to.set(b.minX(), b.maxY(), b.maxZ());
    drawLine(from, to);
	from.set(b.minX(), b.maxY(), b.minZ());
	to.set(b.maxX(), b.maxY(), b.minZ());
    drawLine(from, to);

	from.set(b.maxX(), b.minY(), b.minZ());
	to.set(b.maxX(), b.maxY(), b.minZ());
    drawLine(from, to);
	from.set(b.maxX(), b.minY(), b.minZ());
	to.set(b.maxX(), b.minY(), b.maxZ());
    drawLine(from, to);
}

void DrawableGrid::setStepDrawGrid(double value) {
    stepDrawGrid = value;
}

void DrawableGrid::updateMinSignedDistance() {
    minSignedDistance = signedDistances.min();
}
