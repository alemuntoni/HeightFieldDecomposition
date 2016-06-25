#pragma once

#include <QString>

#include <time.h>
#include <iostream>

class Timer {
    public:
        Timer (const std::string& _caption) : caption(_caption) {
            secs = 0.f;
            start();
        }

        inline void start(){
            //DGP::logger << "[start]\t" << caption.toStdString() << DGP::endl;
            _start = clock();
        }

        inline void stopAndPrint() {
            _stop = clock();
            float cycles = _stop - _start;
            secs = ((float)cycles) / ((float)CLOCKS_PER_SEC);
            std::cout << "[" << secs << " secs]\t" << caption << std::endl;
        }

        inline float delay() {
            float s = clock();
            float cycles = s - _start;
            s = ((float)cycles) / ((float)CLOCKS_PER_SEC);
            return s;
        }

    private:
        std::string caption;
        clock_t _start, _stop;
        float secs;
};
