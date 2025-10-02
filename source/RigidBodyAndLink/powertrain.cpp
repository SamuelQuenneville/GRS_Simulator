/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "powertrain.h"

float f(float n, float x, float y, float z, float phi, float psi, float thrustTarget)
{
    return (-x*phi*phi/(n*n) - y*phi/n + z) * psi*n*n - thrustTarget;
}

float df(float n, float x, float y, float z, float phi, float psi)
{
    return (2*x*phi*phi/(n*n*n) + y*phi/(n*n)) * psi*n*n + (-x*phi*phi/(n*n) - y*phi/n + z) * 2*psi*n;
}

/*
 * Convert a desired thrust (Newton) at a given airspeed (m/s) to a rpm command
 * for an APC 16x8E propeller
 */
double thrust2rpm(float airspeed, float thrustTarget)
{
    constexpr float rho = 1.225;      // air density in kg/m^2
    constexpr float D = 16*0.0254;    // propeller diameter in meter
    float phi = 60*airspeed/D;
    float psi = rho*D*D*D*D/3600.0f;

    // Define with wind tunnel testing
    constexpr float x = 0.1588;
    constexpr float y = 0.0106;
    constexpr float z = 0.0757;

    float x0 = 4000;        // initial guess about half range
    float x1 = 0;
    float res = 100;

    // Newton Raphson method
    int i = 0;
    while (res > 10 && i++ < MAX_ITER_RPM)
    {
        float f0 = f(x0,x,y,z,phi,psi,thrustTarget);
        float df0 = df(x0,x,y,z,phi,psi);

        x1 = x0 - f0/df0;

        res = f(x1,x,y,z,phi,psi,thrustTarget);
        x0 = x1;
    }

    // Saturate desired RPM
    const float rpm = std::clamp(x0, 0.0f, 9000.0f);

    // Convert RPM to command between 0 and 1
    return rpm / 9000.0f;
}

double rpm2thrust(const float airspeed, const float rpmTarget) {
    constexpr float rho = 1.225;      // air density in kg/m^2
    constexpr float D = 16*0.0254;    // propeller diameter in meter

    const float rps = rpmTarget / 60.0f;
    const float airspeedOverRpsD = airspeed / (rps*D);

    // Define with wind tunnel testing
    constexpr float x = 0.1588;
    constexpr float y = 0.0106;
    constexpr float z = 0.0757;

    return (-x * airspeedOverRpsD*airspeedOverRpsD - y * airspeedOverRpsD + z) * rho * rps*rps * D*D*D*D;
}

float maxThrust(const float airspeed)
{
    constexpr float rho = 1.225;      // air density in kg/m^2
    constexpr float D = 16*0.0254;    // propeller diameter in meter

    constexpr float maxRps = 9000.0f / 60.0f;
    const float airspeedOverRpsD = airspeed / (maxRps*D);

    // Define with wind tunnel testing
    constexpr float x = 0.1588;
    constexpr float y = 0.0106;
    constexpr float z = 0.0757;

    return (-x * airspeedOverRpsD*airspeedOverRpsD - y * airspeedOverRpsD + z) * rho * maxRps*maxRps * D*D*D*D;
}
