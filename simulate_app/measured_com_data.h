// COM measurements from simulate_mc static mode (Press 'F' to enable)
// These are ground-truth measurements from the MuJoCo simulation
// with proper constraint solving and ground contact.

#ifndef MEASURED_COM_DATA_H_
#define MEASURED_COM_DATA_H_

struct ComMeasurement {
    double hip_angle;       // radians
    double hip_wheel_dist;  // meters, ||hip-wheel|| distance
    double com_rel_x;       // meters, COM x relative to wheel contact
    double com_rel_z;       // meters, COM z relative to wheel contact
    double theta_eq;        // radians, equilibrium angle
};

// Measurements from static mode at each LUT angle
static const ComMeasurement COM_MEASUREMENTS[] = {
    {-0.2700, 0.2846, 0.048, 0.256, -10.69 * 0.017453},  // -10.69°
    {-0.1625, 0.3714, 0.044, 0.321,  -7.77 * 0.017453},  //  -7.77°
    {-0.0550, 0.4477, 0.039, 0.377,  -5.89 * 0.017453},  //  -5.89°
    { 0.0525, 0.5059, 0.034, 0.418,  -4.72 * 0.017453},  //  -4.72°
    { 0.1600, 0.5610, 0.029, 0.457,  -3.66 * 0.017453},  //  -3.66°
    { 0.2675, 0.6138, 0.022, 0.494,  -2.54 * 0.017453},  //  -2.54°
    { 0.3750, 0.6636, 0.010, 0.530,  -1.07 * 0.017453},  //  -1.07°
};

static const int NUM_COM_MEASUREMENTS = sizeof(COM_MEASUREMENTS) / sizeof(COM_MEASUREMENTS[0]);

#endif  // MEASURED_COM_DATA_H_
