#pragma once

#include <QString>

#include <time.h>
#include <iostream>
#include <sys/time.h>

/*class Timer {
    public:
        Timer (const std::string& _caption) : caption(_caption) {
            secs = 0.f;
            start();
        }

        inline void start(){
            //DGP::logger << "[start]\t" << caption.toStdString() << DGP::endl;
            start = clock();
        }

        inline void stopAndPrint() {
            stop = clock();
            float cycles = stop - start;
            secs = ((float)cycles) / ((float)CLOCKS_PER_SEC);
            std::cout << "[" << secs << " secs]\t" << caption << std::endl;
        }

        inline float delay() {
            float s = clock();
            float cycles = s - start;
            s = ((float)cycles) / ((float)CLOCKS_PER_SEC);
            return s;
        }

    private:
        std::string caption;
        clock_t start, stop;
        float secs;
};*/

class Timer {
    public:
        Timer (const std::string& _caption) : caption(_caption), isStopped(false) {
            start();
        }

        inline void start(){
            //DGP::logger << "[start]\t" << caption.toStdString() << DGP::endl;
            gettimeofday(&begin, NULL);
        }

        inline void stopAndPrint() {
            gettimeofday(&end, NULL);
            isStopped = true;
            double secs =
                (end.tv_sec - begin.tv_sec) +
                ((end.tv_usec - begin.tv_usec)/1000000.0);

            std::cout << "[" << secs << " secs]\t" << caption << std::endl;
        }

        inline void stop() {
            gettimeofday(&end, NULL);
            isStopped = true;
        }

        inline void print () {
            double secs;
            if (isStopped)
                secs =
                    (end.tv_sec - begin.tv_sec) +
                    ((end.tv_usec - begin.tv_usec)/1000000.0);
            else {
                timeval s;
                gettimeofday(&s, NULL);
                secs =
                    (s.tv_sec - begin.tv_sec) +
                    ((s.tv_usec - begin.tv_usec)/1000000.0);
            }

            std::cout << "[" << secs << " secs]\t" << caption << std::endl;
        }

        inline double delay() {
            double secs;
            if (isStopped)
                secs =
                    (end.tv_sec - begin.tv_sec) +
                    ((end.tv_usec - begin.tv_usec)/1000000.0);
            else {
                timeval s;
                gettimeofday(&s, NULL);
                secs =
                    (s.tv_sec - begin.tv_sec) +
                    ((s.tv_usec - begin.tv_usec)/1000000.0);
            }

            return secs;
        }

    private:
        std::string caption;
        timeval begin, end;
        bool isStopped;
};
