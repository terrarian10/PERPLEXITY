#include "autons.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "zcommands/math.hpp"
#include "zcommands/structs.hpp"
#include <cstddef>
#include <cstdlib>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <variant>
#include <vector>
inline std::optional<float> raycastIfPresent(
    const std::optional<simDistSensor>& sensor,
    const lemlib::Pose& pose) {
    if (!sensor) return std::nullopt;

    return raycast(sensor->offset + pose,
                   box{ -182.88, -182.88, 182.88, 182.88 });
}

inline distanceData buildParticleData(const botDistanceSensors& sensors,
                                      const lemlib::Pose& pose) {
    return {
        .forward1 = raycastIfPresent(sensors.forward1, pose),
        .forward2 = raycastIfPresent(sensors.forward2, pose),
        .left1 = raycastIfPresent(sensors.left1, pose),
        .left2 = raycastIfPresent(sensors.left2, pose),
        .right1 = raycastIfPresent(sensors.right1, pose),
        .right2 = raycastIfPresent(sensors.right2, pose),
        .back1 = raycastIfPresent(sensors.back1, pose),
        .back2 = raycastIfPresent(sensors.back2, pose),
    };
}

inline particle createParticle(lemlib::Pose botPose,
                               botDistanceSensors distanceSensors) {

    return particle{ .pose = botPose,
                     .sensorData = {
                         buildParticleData(distanceSensors, botPose) } };
};
// So apparently theres this thing called gaussian distribution that is good at
// being random but good random and not bad random
inline void updateParticles(std::vector<particle>& particles,
                            const lemlib::Pose& movement,
                            float linStd,
                            float angStd,
                            float linStdPerIn = 0,
                            float angStdPerDeg = 0) {
    static thread_local std::mt19937 rng(std::random_device{}());

    float linSigma = linStd + linStdPerIn * std::hypot(movement.x, movement.y);
    float angSigma = angStd + angStdPerDeg * std::abs(movement.theta);

    std::normal_distribution<float> linN(0.0f, linSigma);
    std::normal_distribution<float> angN(0.0f, angSigma);

    for (auto& p : particles) {
        p.pose.x += movement.x + linN(rng);
        p.pose.y += movement.y + linN(rng);
        p.pose.theta = wrapDeg(p.pose.theta + movement.theta + angN(rng));
    }
}

inline float scoreOne(const std::optional<float>& a,
                      const std::optional<float>& b) {
    if (!a || !b) return 0.0f; // or return a penalty if you want
    return std::abs(*a - *b);
}

inline float getParticleScore(const particle& p,
                              const particle& chassis,
                              const sensorWeights& w,
                              float largeErr) {
    std::vector<float> diffs;
    diffs.reserve(8);

    auto addDiff = [&](const std::optional<float>& a,
                       const std::optional<float>& b) {
        if (a && b) diffs.push_back(std::abs(*a - *b));
    };

    addDiff(p.sensorData.distSenseData.forward1,
            chassis.sensorData.distSenseData.forward1);
    addDiff(p.sensorData.distSenseData.forward2,
            chassis.sensorData.distSenseData.forward2);
    addDiff(p.sensorData.distSenseData.left1,
            chassis.sensorData.distSenseData.left1);
    addDiff(p.sensorData.distSenseData.left2,
            chassis.sensorData.distSenseData.left2);
    addDiff(p.sensorData.distSenseData.right1,
            chassis.sensorData.distSenseData.right1);
    addDiff(p.sensorData.distSenseData.right2,
            chassis.sensorData.distSenseData.right2);
    addDiff(p.sensorData.distSenseData.back1,
            chassis.sensorData.distSenseData.back1);
    addDiff(p.sensorData.distSenseData.back2,
            chassis.sensorData.distSenseData.back2);

    if (diffs.empty()) return 0.0f;

    // Unweighted mean (for outlier detection)
    float mean =
        std::accumulate(diffs.begin(), diffs.end(), 0.0f) / diffs.size();

    float weightedSum = 0.0f;
    float weightSum = 0.0f;

    for (float diff : diffs) {
        float weight = (std::abs(diff - mean) > largeErr) ? w.obsWeight : 1.0f;

        weightedSum += diff * weight;
        weightSum += weight;
    }

    return weightedSum / weightSum;
}

inline std::vector<particle> NKVD(const std::vector<particle>& particles,
                                  const particle& chassis,
                                  const sensorWeights& w,
                                  float largeErr,
                                  float threshold) {
    if (particles.empty()) return {};

    std::vector<float> scores;
    scores.reserve(particles.size());

    float sum = 0.0f;
    for (const auto& p : particles) {
        float s = getParticleScore(p, chassis, w, largeErr);
        scores.push_back(s);
        sum += s;
    }

    float scoreMean = sum / scores.size();

    std::vector<particle> final;
    final.reserve(particles.size());
    for (size_t i = 0; i < particles.size(); ++i) {
        if (std::abs(scores[i] - scoreMean) < threshold) {
            final.push_back(particles[i]);
        }
    }
    return final;
}

inline std::vector<particle> populateParticles(
    const std::vector<particle>& initParticles,
    float spreadXY,    // stddev for x/y noise (same units as pose.x/y)
    float spreadTheta, // stddev for theta noise
    int totalParticles) {
    std::vector<particle> finalParticles;
    if (totalParticles <= 0 || initParticles.empty()) return finalParticles;

    finalParticles.reserve(static_cast<size_t>(totalParticles));

    // One RNG for the whole function
    static thread_local std::mt19937 rng{ std::random_device{}() };

    std::normal_distribution<float> nXY(0.0f, spreadXY);
    std::normal_distribution<float> nTh(0.0f, spreadTheta);

    for (int i = 0; i < totalParticles; ++i) {
        particle p =
            initParticles[static_cast<size_t>(i) % initParticles.size()];

        p.pose.x += nXY(rng);
        p.pose.y += nXY(rng);
        p.pose.theta = wrapDeg(p.pose.theta + nTh(rng));

        // If your theta is degrees, wrap to a stable range (optional but
        // recommended)

        finalParticles.push_back(p);
    }

    return finalParticles;
}

inline std::vector<particle> populateInitialParticles(
    lemlib::Chassis& chassis,
    botDistanceSensors& sensors,
    int numParticles,
    float linearTrust,
    float headingTrust) {
    std::vector<particle> particles;

    // Populate particles based on the chassis state and trust levels
    for (int i = 0; i < numParticles; ++i) {
        particle p{ chassis.getPose() };
        // One RNG for the whole function
        static thread_local std::mt19937 rng{ std::random_device{}() };

        std::normal_distribution<float> nXY(0.0f, linearTrust);
        std::normal_distribution<float> nTh(0.0f, headingTrust);
        // Add noise based on trust levels
        p.pose.x += nXY(rng);
        p.pose.y += nXY(rng);
        p.pose.theta = wrapDeg(p.pose.theta + nTh(rng));

        particles.push_back(p);
    }
    for (auto& p : particles) {
        p.sensorData.distSenseData = buildParticleData(sensors, p.pose);
    }
    return particles;
}