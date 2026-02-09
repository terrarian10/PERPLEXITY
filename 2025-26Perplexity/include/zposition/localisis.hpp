#include "lemlib/pose.hpp"
#include "pros/distance.hpp"
#include "zcommands/math.hpp"
#include "zposition/random.hpp"
#include <numbers>
#include <vector>
struct particle {
    lemlib::Pose pos;
};
struct weightedParticle {
    particle p;
    float weight;
};
struct simDistanceSensor {
    lemlib::Pose offset;
    float distance = 0;
};

inline void initalPopulate(std::vector<particle>& particles,
                           lemlib::Pose estStart) {
    particles.clear();
    for (int i = 0; i < 100; ++i) { // NumParticles
        particles.push_back(
            { lemlib::Pose{ randNormal(estStart.x, 6.0),
                            randNormal(estStart.y, 6.0),
                            randNormal(estStart.theta, 6.0) } });
    }
}

inline void moveParticles(std::vector<particle>& particles,
                          lemlib::Pose movement) {
    for (auto& p : particles) {
        p.pos.x += movement.x + (randNormal(0.0, 0.05) * (movement.x + 0.5));
        p.pos.y += movement.y + (randNormal(0.0, 0.05) * (movement.y + 0.5));
        p.pos.theta +=
            movement.theta + (randNormal(0.0, 0.05) * (movement.theta + 0.5));
    }
}

inline std::vector<weightedParticle> weightParticles(
    const std::vector<particle>& particles,
    const std::vector<simDistanceSensor>& sensors,
    lemlib::Pose chassisPose,
    std::vector<float> expectedDistances,
    float sigma) {
    std::vector<weightedParticle> ret;
    const float invSigma2 = 1.0f / (2 * sigma * sigma);
    for (auto& particle : particles) {
        float logw = 0;
        for (int i = 0; i < sensors.size(); i++) {
            float e = (vexRaycast(particle.pos + sensors[i].offset) -
                       expectedDistances[i]);
            logw += -(e * e) * invSigma2;
        }
        float w = std::exp(logw);
        ret.push_back({ particle, w });
    }

    return ret;
}

inline lemlib::Pose getBestPose(
    const std::vector<weightedParticle>& particles) {
    float overallWeight = 0;
    lemlib::Pose bestPose = { 0, 0, 0 };
    float s = 0, c = 0;
    for (auto& p : particles) {
        bestPose.x += p.p.pos.x * p.weight;
        bestPose.y += p.p.pos.y * p.weight;
        s += std::sin(p.p.pos.theta * (std::numbers::pi / 180)) * p.weight;
        c += std::cos(p.p.pos.theta * (std::numbers::pi / 180)) * p.weight;
        overallWeight += p.weight;
    }
    if (overallWeight < 1e-12f) return particles[0].p.pos;

    bestPose.x /= overallWeight;
    bestPose.y /= overallWeight;
    bestPose.theta = std::atan2(s, c) * (180 / std::numbers::pi);
    return bestPose;
}
inline void normalizeWeights(std::vector<weightedParticle>& wps) {
    float sum = 0.0f;
    for (auto& wp : wps)
        sum += wp.weight;

    if (sum < 1e-12f) {
        float u = 1.0f / std::max<size_t>(1, wps.size());
        for (auto& wp : wps)
            wp.weight = u;
        return;
    }
    for (auto& wp : wps)
        wp.weight /= sum;
}

inline std::vector<particle> resampleParticles(
    std::vector<weightedParticle>& wps,
    int N,
    float jitterPosStd,
    float jitterThetaStd) {
    normalizeWeights(wps);

    // find best particle (for fallback filling)
    int bestIdx = 0;
    for (int i = 1; i < (int)wps.size(); i++) {
        if (wps[i].weight > wps[bestIdx].weight) bestIdx = i;
    }

    std::vector<particle> out;
    out.reserve(N);

    // copy-count step
    for (auto& wp : wps) {
        int copies = (int)std::round(wp.weight * N);
        for (int k = 0; k < copies && (int)out.size() < N; k++) {
            particle p = wp.p;
            p.pos.x += randNormal(0.0f, jitterPosStd);
            p.pos.y += randNormal(0.0f, jitterPosStd);
            p.pos.theta += randNormal(0.0f, jitterThetaStd);
            out.push_back(p);
        }
    }

    // if rounding gave too few, fill with best particle
    while ((int)out.size() < N) {
        particle p = wps[bestIdx].p;
        p.pos.x += randNormal(0.0f, jitterPosStd);
        p.pos.y += randNormal(0.0f, jitterPosStd);
        p.pos.theta += randNormal(0.0f, jitterThetaStd);
        out.push_back(p);
    }

    return out;
}
// Hole shise its globl localization
// Cant wait to watch this fail
inline lemlib::Pose iterateLocal(std::vector<particle>& particles,
                                 std::vector<simDistanceSensor>& sensors,
                                 const std::vector<float> expectedDistances,
                                 lemlib::Pose chassisPose,
                                 lemlib::Pose oldChassisPose,
                                 float sigma) {
    moveParticles(particles, chassisPose - oldChassisPose);
    std::vector<weightedParticle> weightedParticles = weightParticles(
        particles, sensors, chassisPose, expectedDistances, sigma);
    lemlib::Pose bestPose = getBestPose(weightedParticles);
    particles =
        resampleParticles(weightedParticles, particles.size(), 1.0, 2.0);
    return bestPose;
}