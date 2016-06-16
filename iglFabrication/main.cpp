#include "functions.h"


int main(int argc, char * argv[]) {
    bool b = false;
    if (argc > 1) {
        b = generateGridAndDistanceField(argv[1]);
    }

    return !b;
}
