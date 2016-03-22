#ifndef POKE_SKYE_H
#define POKE_SKYE_H

#include <skye_tests/action_skye.h>

class Poke_skye
{
public:
    Poke_skye();

    bool perform_action(Eigen::Vector3d force, double duration);
};

#endif // POKE_SKYE_H
