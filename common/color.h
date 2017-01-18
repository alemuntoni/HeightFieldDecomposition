#ifndef COLOR_H
#define COLOR_H

#ifndef QT_GUI_LIB
#include <QColor>
typedef QColor Color;
#else

class Color {
    public:
        Color();
        Color(unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha = 255);
        int red() const;
        int green() const;
        int blue() const;
        int alpha() const;
        float redF() const;
        float greenF() const;
        float blueF() const;
        float alphaF() const;
        void setAlpha(unsigned char alpha);
        void setRed(unsigned char red);
        void setGreen(unsigned char green);
        void setBlue(unsigned char blue);
        void setRgb(unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha = 255);
        void setAlphaF(float alpha);
        void setRedF(float red);
        void setGreenF(float green);
        void setBlueF(float blue);
        void setRgbF(float red, float green, float blue, float alpha = 1.0);

    protected:
        int r, g, b;
        int a;
};
#endif

Color::Color() : r(0), g(0), b(0), a(0){
}

Color::Color(unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha) : r(red), g(green), b(blue), a(alpha){
}

int Color::red() const {
    return r;
}

int Color::green() const {
    return g;
}

int Color::blue() const {
    return b;
}

int Color::alpha() const {
    return a;
}

float Color::redF() const {
    return (float)r/255;
}

float Color::greenF() const {
    return (float)g/255;
}

float Color::blueF() const {
    return (float)b/255;
}

float Color::alphaF() const {
    return (float)a()/255;
}

void Color::setAlpha(unsigned char alpha) {
    a = alpha;
}

void Color::setRed(unsigned char red) {
    r = red;
}

void Color::setGreen(unsigned char green) {
    g = green;
}

void Color::setBlue(unsigned char blue) {
    b = blue;
}

void Color::setRgb(unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha) {
    r = red;
    g = green;
    b = blue;
    a = alpha;
}

void Color::setAlphaF(float alpha) {
    a = (unsigned char)(alpha*255);
}

void Color::setRedF(float red) {
    r = (unsigned char)(red*255);
}

void Color::setGreenF(float green) {
    g = (unsigned char)(green*255);
}

void Color::setBlueF(float blue) {
    b = (unsigned char)(blue*255);
}

void Color::setRgbF(float red, float green, float blue, float alpha) {
    a = (unsigned char)(alpha*255);
    r = (unsigned char)(red*255);
    g = (unsigned char)(green*255);
    b = (unsigned char)(blue*255);
}

#endif // COLOR_H
