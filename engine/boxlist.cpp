#include "boxlist.h"

BoxList::BoxList() : visible(true), n(-1){

}

void BoxList::addBox(const Box3D& b) {
    boxes.push_back(b);
}

void BoxList::setVisibleBox(int i) {
    if (i>=-1 && i < (int)boxes.size())
        n = i;
}

void BoxList::clearBoxes() {
    boxes.clear();
}

unsigned int BoxList::getNumberBoxes() {
    return boxes.size();
}

Box3D BoxList::getBox(unsigned int i) {
    if (i < boxes.size())
        return (boxes[i]);
    return Box3D();
}

void BoxList::serialize(std::ofstream& binaryFile) const {
    Serializer::serialize(boxes, binaryFile);
}

void BoxList::deserialize(std::ifstream& binaryFile) {
    Serializer::deserialize(boxes, binaryFile);
    n = 0;
}

void BoxList::draw() const {
    if (visible){
        if (n < 0){
            for (unsigned int i = 0; i < boxes.size(); i++)
                boxes[i].draw();
        }
        else
            boxes[n].draw();
    }
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
