#include "LineFollower.h"
#define TIME_STEP 8

int main(int argc, char **argv)
{

    LineFollower follower;


    while (follower.step(8) != -1)
    {
        follower.test();

    };

    return 0;
};


