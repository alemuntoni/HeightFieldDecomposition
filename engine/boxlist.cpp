#include "boxlist.h"

BoxList::BoxList() : visible(true), visibleBox(-1), cylinder(true){

}

BoxList::BoxList(bool cylinders) : visible(true), visibleBox(-1), cylinder(cylinders){
}

void BoxList::addBox(const Box3D& b, int i) {
    if (i == -1)
        boxes.push_back(b);
    else {
        boxes.insert(boxes.begin()+i, b);
    }
}

void BoxList::clearBoxes() {
    boxes.clear();
}

unsigned int BoxList::getNumberBoxes() const{
    return boxes.size();
}

Box3D BoxList::getBox(unsigned int i) const {
    if (i < boxes.size())
        return (boxes[i]);
    return Box3D();
}

void BoxList::setBox(unsigned int i, const Box3D& b) {
    assert (i < boxes.size());
    boxes[i] = b;
}

void BoxList::insert(const BoxList& o) {
    boxes.insert(boxes.end(), o.boxes.begin(), o.boxes.end());
}

void BoxList::removeBox(unsigned int i) {
    assert (i < boxes.size());
    boxes.erase(boxes.begin()+i);
}

void BoxList::getSubBoxLists(std::vector<BoxList>& v, int nPerBoxList) {
    int nBoxLists = getNumberBoxes() / nPerBoxList;
    if (getNumberBoxes() % nPerBoxList != 0) v.reserve(nBoxLists+1);
    else v.reserve(nBoxLists);
    for (unsigned int i = 0; i < getNumberBoxes(); i++){
        if (i % nPerBoxList == 0) v.push_back(BoxList());
        unsigned int j = i / nPerBoxList;
        v[j].addBox(getBox(i));
    }
}

void BoxList::setIds() {
    for (unsigned int i = 0; i < boxes.size(); i++){
        boxes[i].setId(i);
    }
}

void BoxList::sort(const Array2D<int>& ordering) {
    struct cmp {
        Array2D<int> order;
        cmp(const Array2D<int>& ord){
            order = ord;
        }
        bool operator()(const Box3D &a, const Box3D &b) const {
            bool val = order(a.getId(),b.getId());
            return val;
        }
    };

    std::sort(boxes.begin(), boxes.end(), cmp(ordering));

}

void BoxList::generatePieces(double minimumDistance) {
    for (unsigned int i= 0; i < boxes.size(); i++){
        boxes[i].generatePiece(minimumDistance);
    }
}

void BoxList::serialize(std::ofstream& binaryFile) const {
    Serializer::serialize(boxes, binaryFile);
}

void BoxList::deserialize(std::ifstream& binaryFile) {
    Serializer::deserialize(boxes, binaryFile);
    visibleBox = 0;
    cylinder = false;
}

void BoxList::setVisibleBox(int i) {
    if (i>=-1 && i < (int)boxes.size())
        visibleBox = i;
}

void BoxList::setCylinders(bool b) {
    cylinder = b;
}

void BoxList::draw() const {
    #ifdef VIEWER_DEFINED
    if (visible){
        if (visibleBox < 0){
            if (!cylinder){
                for (unsigned int i = 0; i < boxes.size(); i++)
                    boxes[i].draw();
            }
            else {
                for (unsigned int i = 0; i < boxes.size(); i++){
                    drawCube(boxes[i], boxes[i].getColor());
                }
            }
        }
        else
            boxes[visibleBox].draw();
    }
    #endif
}

Pointd BoxList::sceneCenter() const {
    return std::move(Pointd());
}

double BoxList::sceneRadius() const {
    return -1;
}

bool BoxList::isVisible() const {
    return visible;
}

void BoxList::setVisible(bool b) {
    visible = b;
}

#ifdef VIEWER_DEFINED
void BoxList::drawLine(const Pointd &a, const Pointd &b, const QColor& c) const {
    glBegin(GL_LINES);
    glColor3f(c.redF(), c.greenF(), c.blueF());
    glLineWidth(3);
    glVertex3f(a.x(), a.y(), a.z());
    glVertex3f(b.x(), b.y(), b.z());
    glEnd();
}

void BoxList::drawCube(const Box3D& b, const QColor &c) const {
    std::vector<Pointd> p;
    b.getRotatedExtremes(p);
    drawLine(p[0], p[1], c);
    drawLine(p[1], p[2], c);
    drawLine(p[2], p[3], c);
    drawLine(p[0], p[3], c);

    drawLine(p[4], p[5], c);
    drawLine(p[5], p[6], c);
    drawLine(p[6], p[7], c);
    drawLine(p[4], p[7], c);

    drawLine(p[0], p[4], c);
    drawLine(p[1], p[5], c);
    drawLine(p[2], p[6], c);
    drawLine(p[3], p[7], c);
}
#endif
