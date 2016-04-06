#ifndef SKYE_GEOMETRIC_CONTROLLER_H
#define SKYE_GEOMETRIC_CONTROLLER_H

class SkyeGeometricController
{
private:
    double mass_, k_x_, k_v_, k_R_, k_omega_;


public:
    SkyeGeometricController();

    void computeForce();
    void InitializeParams();

};

#endif // SKYE_GEOMETRIC_CONTROLLER_H
