#include "autons.hpp"
#include "lemlib/pose.hpp"
#include "zcommands/math.hpp"
#include "zcommands/structs.hpp"
#include <cstddef>
#include <cstdlib>
#include <map>
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
        p.pose.theta += movement.theta + angN(rng);
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
    // One-line-per-sensor, but no repeated boilerplate logic.
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
    float score = std::accumulate(diffs.begin(), diffs.end(), 0.0f);
    float mean = score / diffs.size();
    auto checkDiff = [&](float diff, float mean, float threshhold) -> bool {
        return std::abs(diff - mean) < threshhold;
    };

    for (auto& diff : diffs) {
        if (!checkDiff(diff, mean, largeErr)) {
            diff *= w.obsWeight; // Lower the influence of outliers
        }
    }
    score = std::accumulate(diffs.begin(), diffs.end(), 0.0f);

    return score;
}

inline std::vector<particle> NKVD(const std::vector<particle>& particles,
                                  const particle& chassis,
                                  const sensorWeights& w,
                                  float largeErr,
                                  float mean) {
    std::map<particle, float> filtered;
    for (const auto& p : particles) {
        filtered[p] = getParticleScore(p, chassis, w, largeErr);
    }
    float scoreMean = std::accumulate(filtered.begin(),
                                      filtered.end(),
                                      0.0f,
                                      [](float sum, const auto& pair) {
                                          return sum + pair.second;
                                      }) /
                      filtered.size();
    for (auto& pair : filtered) {
        if (std::abs(pair.second - mean) > largeErr) {
            pair.second *= w.obsWeight;
        }
    }
    return std::vector<particle>{};
}