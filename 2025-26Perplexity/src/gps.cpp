#include "pros/gps.h"
#include "gps.hpp"
#include "main.h"
#include "pros/gps.hpp"
#include "pros/rtos.hpp"
#include <array>
#include <cmath>
// ----- CONFIG -----
// Sensor offset (meters) from robot turn-center to the GPS camera
constexpr double kGpsXOffset_m = /* your value */ 0.08;
constexpr double kGpsYOffset_m = /* your value */ -0.05;

// Your rough estimate of start pose (in *your canonical quadrant*), cm/deg
double kBaseStartX_cm;
double kBaseStartY_cm;
double kStartHeading_deg;

// Mirror signs describing field quadrant (+1 or -1)
int mx; // e.g., top-left uses -1
int my; // e.g., top-left uses +1

// Tuning thresholds
constexpr double kLockPosTol_m = 0.07; // 7 cm GPS reported error (if available)
constexpr double kLockWinPosStd_m = 0.015;  // 1.5 cm position stddev (fallback)
constexpr double kLockWinHeadStd_deg = 1.2; // heading stddev (fallback)
constexpr int kWindow = 20;                 // ~20 samples @ 20ms ≈ 400ms window
constexpr int kMinStableCount = 15;         // need majority of good samples
constexpr int kLoopDelayMs = 20;            // 50 Hz loop

// ----- UTIL -----
static inline double wrapDeg(double d) {
    while (d < 0)
        d += 360.0;
    while (d >= 360)
        d -= 360.0;
    return d;
}

template<size_t N>
struct Ring {
    std::array<double, N> xs{}, ys{}, hs{};
    int idx = 0, count = 0;
    void push(double x, double y, double h) {
        xs[idx] = x;
        ys[idx] = y;
        hs[idx] = h;
        idx = (idx + 1) % (int)N;
        if (count < (int)N) count++;
    }
    int size() const { return count; }
};

// Compute simple stddev (degrees for heading, handling wrap)
template<size_t N>
static void stats(const Ring<N>& r, double& xStd, double& yStd, double& hStd) {
    const int n = r.size();
    if (n < 2) {
        xStd = yStd = hStd = 1e9;
        return;
    }

    // Mean x,y
    double mx = 0, my = 0;
    for (int i = 0; i < n; i++) {
        mx += r.xs[i];
        my += r.ys[i];
    }
    mx /= n;
    my /= n;

    // Heading: compute mean via unit vectors
    double cx = 0, sy = 0;
    for (int i = 0; i < n; i++) {
        double rad = r.hs[i] * M_PI / 180.0;
        cx += std::cos(rad);
        sy += std::sin(rad);
    }
    cx /= n;
    sy /= n;
    const double hMean_deg = wrapDeg(std::atan2(sy, cx) * 180.0 / M_PI);

    // Std x,y
    double vx = 0, vy = 0;
    for (int i = 0; i < n; i++) {
        vx += (r.xs[i] - mx) * (r.xs[i] - mx);
        vy += (r.ys[i] - my) * (r.ys[i] - my);
    }
    vx /= (n - 1);
    vy /= (n - 1);

    // Heading std: angular distance to mean
    double vh = 0;
    for (int i = 0; i < n; i++) {
        double d = std::fabs(r.hs[i] - hMean_deg);
        if (d > 180.0) d = 360.0 - d;
        vh += d * d;
    }
    vh /= (n - 1);

    xStd = std::sqrt(vx);
    yStd = std::sqrt(vy);
    hStd = std::sqrt(vh);
}

// ----- SEED + REFINE PIPELINE -----
void seedGpsWithRoughEstimate(pros::Gps& gps) {
    gps.set_offset(kGpsXOffset_m, kGpsYOffset_m);

    // Mirror your *canonical* rough estimate by mx,my
    const double x_m = (kBaseStartX_cm) / 100.0;
    const double y_m = (kBaseStartY_cm) / 100.0;

    gps.set_position(x_m, y_m, kStartHeading_deg);
}

// Background task that waits for GPS to stabilize, then pushes a precise pose
Ring<kWindow> ring;
bool locked = false;
void gpsRefinerTask(pros::Gps& gps) {

    // Read GPS (meters, degrees)
    const double x_m = gps.get_position_x();
    const double y_m = gps.get_position_y();
    const double h_d = wrapDeg(gps.get_heading());

    ring.push(x_m, y_m, h_d);

    // Prefer GPS-reported accuracy if available
    bool accuracyOK = false;
#ifdef PROS_GPS_HAS_ERROR // define this yourself if your PROS has get_error()
    {
        const double err = gps.get_error(); // meters RMS (varies by firmware)
        accuracyOK = (err > 0.0 && err < kLockPosTol_m);
    }
#endif

    bool stableOK = false;
    {
        double sx, sy, sh;
        stats(ring, sx, sy, sh);
        stableOK = (ring.size() >= kMinStableCount) &&
                   (sx < kLockWinPosStd_m) && (sy < kLockWinPosStd_m) &&
                   (sh < kLockWinHeadStd_deg);
    }

    // Only accept/update when robot is not moving to avoid fighting local
    // odom
    if (!chassis.isInMotion()) {
        if (!locked && (accuracyOK || stableOK)) {
            // FIRST precise lock: push into your odometry (e.g., LemLib)
            chassis.setPose(x_m * 100.0 / 2.54, y_m * 100.0 / 2.54, h_d);
            std::cout << "Set Pose W/ GPS" << std::endl;

            locked = true;
        } else if (locked && (accuracyOK || stableOK)) {
            // Optional gentle nudge while stationary (keeps global pose
            // tight)
            chassis.setPose(x_m * 100.0 / 2.54, y_m * 100.0 / 2.54, h_d);
        }
    }

    pros::delay(kLoopDelayMs);
}

void gps_initialize(int _mx,
                    int _my,
                    double startX,
                    double startY,
                    double _heading,
                    pros::Gps& _gps) {
    mx = _mx;
    my = _my;
    kBaseStartX_cm = startX;
    kBaseStartY_cm = startY;
    kStartHeading_deg = _heading;
    seedGpsWithRoughEstimate(_gps);
    std::cout << "Initialized GPS" << std::endl;
    pros::Task(
        [&_gps] {
            while (true) {
                gpsRefinerTask(_gps);
            }
        },
        "gps_refiner");
}
