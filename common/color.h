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
        //toimplement
        int hsvHue() const;
        int hsvSaturation() const;
        float hsvHueF() const;
        float hsvSaturationF() const;
        void setAlpha(unsigned char alpha);
        void setRed(unsigned char red);
        void setGreen(unsigned char green);
        void setBlue(unsigned char blue);
        void setRgb(unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha = 255);
        void setHsv(unsigned char h, unsigned char s, unsigned char v, unsigned char alpha = 255);
        void setAlphaF(float alpha);
        void setRedF(float red);
        void setGreenF(float green);
        void setBlueF(float blue);
        void setRgbF(float red, float green, float blue, float alpha = 1.0);
        void setHsvF(float hf, float sf, float vf, float alpha = 1.0);

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

void Color::setHsv(unsigned char h, unsigned char s, unsigned char v, unsigned char alpha) {
    a= alpha;
    if (s == 0) {
        r = v;
        g = v;
        b = v;
    }
    else {
        unsigned char region, remainder, p, q, t;
        region = h / 43;
        remainder = (h - (region * 43)) * 6;

        p = (v * (255 - s)) >> 8;
        q = (v * (255 - ((s * remainder) >> 8))) >> 8;
        t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

        switch (region) {
            case 0:
                r = v; g = t; b = p;
                break;
            case 1:
                r = q; g = v; b = p;
                break;
            case 2:
                r = p; g = v; b = t;
                break;
            case 3:
                r = p; g = q; b = v;
                break;
            case 4:
                r = t; g = p; b = v;
                break;
            default:
                r = v; g = p; b = q;
                break;
        }

    }
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

void Color::setHsvF(float hf, float sf, float vf, float alpha) {
    setHsv(hf*255, sf*255, vf*255, alpha*255);
}

#endif // COLOR_H
