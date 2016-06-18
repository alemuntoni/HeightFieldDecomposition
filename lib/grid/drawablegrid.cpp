#include "drawablegrid.h"
#include <omp.h>

DrawableGrid::DrawableGrid() : visible(true) {
}

DrawableGrid::DrawableGrid(const Eigen::RowVector3i& resolution, const Eigen::MatrixXd& gridCoordinates, const Eigen::VectorXd& signedDistances, const Eigen::RowVector3i& gMin, const Eigen::RowVector3i& gMax) :
    Grid(resolution, gridCoordinates, signedDistances, gMin, gMax), visible(true), drawMode(DRAW_KERNEL), slice(NO_SLICE), sliceValue(0) {
}

DrawableGrid::~DrawableGrid(){
}

double DrawableGrid::getKernelDistance() const {
    return kernelDistance;
}

void DrawableGrid::setKernelDistance(double value) {
    kernelDistance = value;
}

void DrawableGrid::setDrawKernel() {
    drawMode = DRAW_KERNEL;
}

void DrawableGrid::setDrawBorders() {
    drawMode = DRAW_BORDERS;
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

double DrawableGrid::getHsvFactor(double w) const {
    return 1-((w - MIN_PAY)/(MAX_PAY-MIN_PAY));
}

void DrawableGrid::draw() const {
    if (visible){
        switch (drawMode){
            case DRAW_KERNEL:
                switch (slice){
                    case NO_SLICE:
                        for (unsigned int i = 0; i < getResX(); ++i){
                            for (unsigned int j = 0; j < getResY(); ++j){
                                for (unsigned int k = 0; k < getResZ(); ++k){
                                    if (getSignedDistance(i,j,k) < -kernelDistance){
                                        sphere(getPoint(i,j,k), 0.3, QColor(255,0,0));
                                    }
                                }
                            }
                        }
                        break;
                    case X_SLICE:
                        for (unsigned int j = 0; j < getResY(); ++j){
                            for (unsigned int k = 0; k < getResZ(); ++k){
                                if (getSignedDistance(sliceValue,j,k) < -kernelDistance){
                                    sphere(getPoint(sliceValue,j,k), 0.3, QColor(255,0,0));
                                }
                            }
                        }
                        break;
                    case Y_SLICE:
                        for (unsigned int i = 0; i < getResX(); ++i){
                            for (unsigned int k = 0; k < getResZ(); ++k){
                                if (getSignedDistance(i,sliceValue,k) < -kernelDistance){
                                    sphere(getPoint(i,sliceValue,k), 0.3, QColor(255,0,0));
                                }
                            }
                        }
                        break;
                    case Z_SLICE:
                        for (unsigned int i = 0; i < getResX(); ++i){
                            for (unsigned int j = 0; j < getResY(); ++j){
                                if (getSignedDistance(i,j,sliceValue) < -kernelDistance){
                                    sphere(getPoint(i,j,sliceValue), 0.3, QColor(255,0,0));
                                }
                            }
                        }
                        break;
                }


                break;
            case DRAW_BORDERS:
                switch (slice){
                    case NO_SLICE:
                        for (unsigned int i = 0; i < getResX(); ++i){
                            for (unsigned int j = 0; j < getResY(); ++j){
                                for (unsigned int k = 0; k < getResZ(); ++k){
                                    double w = getWeight(i,j,k);
                                    QColor c; c.setHsv(getHsvFactor(w)*240,255,255);
                                    sphere(getPoint(i,j,k), 0.3, c);
                                }
                            }
                        }
                        break;
                    case X_SLICE:
                        for (unsigned int j = 0; j < getResY(); ++j){
                            for (unsigned int k = 0; k < getResZ(); ++k){
                                double w = getWeight(sliceValue,j,k);
                                QColor c; c.setHsv(getHsvFactor(w)*240,255,255);
                                sphere(getPoint(sliceValue,j,k), 0.3, c);
                            }
                        }
                        break;
                    case Y_SLICE:
                        for (unsigned int i = 0; i < getResX(); ++i){
                            for (unsigned int k = 0; k < getResZ(); ++k){
                                double w = getWeight(i,sliceValue,k);
                                QColor c; c.setHsv(getHsvFactor(w)*240,255,255);
                                sphere(getPoint(i,sliceValue,k), 0.3, c);
                            }
                        }
                        break;
                    case Z_SLICE:
                        for (unsigned int i = 0; i < getResX(); ++i){
                            for (unsigned int j = 0; j < getResY(); ++j){
                                double w = getWeight(i,j,sliceValue);
                                QColor c; c.setHsv(getHsvFactor(w)*240,255,255);
                                sphere(getPoint(i,j,sliceValue), 0.3, c);
                            }
                        }
                        break;
                }
                break;
            default:
                assert(0);
        }
    }
}

Pointd DrawableGrid::sceneCenter() const {
    return bb.center();
}

double DrawableGrid::sceneRadius() const {
   return bb.diag();
}

bool DrawableGrid::isVisible() const {
    return visible;
}

void DrawableGrid::setVisible(bool b) {
    visible = b;
}
