#pragma once

#include <QString>

#include <time.h>
#include <iostream>

class Timer {
    public:
        Timer (QString _caption) : caption(_caption) {
            secs = 0.f;
            start();
        }

        inline void start(){
            //DGP::logger << "[start]\t" << caption.toStdString() << DGP::endl;
            _start = clock();
        }

        inline void stop_and_print() {
            _stop = clock();
            float cycles = _stop - _start;
            secs = ((float)cycles) / ((float)CLOCKS_PER_SEC);
            std::cout << "[" << secs << " secs]\t" << caption.toStdString() << std::endl;
        }

        inline float delay() {
            return secs;
        }

    private:
        QString caption;
        clock_t _start, _stop;
        float secs;
};
