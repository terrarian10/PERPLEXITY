#include "commandHandling.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/distance.hpp"
#include "zposition/localisis.hpp"
#include <vector>

struct monteConfig {
    std::vector<particle>& particles;
    lemlib::Chassis& chassis;
    lemlib::Pose& oldPose;
    std::vector<pros::Distance>& distance;
    std::vector<lemlib::Pose>& offsets;
};

class tickMCL_c : public command {
  public:
    explicit tickMCL_c(monteConfig& config, float sigma)
        : particles(config.particles)
        , chassis(config.chassis)
        , oldPose(config.oldPose)
        , distance(config.distance)
        , offsets(config.offsets)
        , sigma(sigma) {};

    bool run() override {
        std::vector<simDistanceSensor> simDistanceSensor{};
        simDistanceSensor.reserve(offsets.size());
        for (const auto& offset : offsets) {
            simDistanceSensor.emplace_back(offset);
        }
        std::vector<float> expectedDistances{};
        for (auto& sensor : distance) {
            expectedDistances.emplace_back(sensor.get_distance());
        }
        iterateLocal(particles,
                     simDistanceSensor,
                     expectedDistances,
                     chassis.getPose(),
                     oldPose,
                     sigma);

        return true;
    };

  private:
    lemlib::Chassis& chassis;
    lemlib::Pose& oldPose;
    std::vector<pros::Distance> distance;
    float sigma;
    std::vector<particle>& particles;
    std::vector<lemlib::Pose> offsets;
};
class poseMCL_c : public command {
  public:
    explicit poseMCL_c(monteConfig& config, float sigma)
        : particles(config.particles)
        , chassis(config.chassis)
        , oldPose(config.oldPose)
        , distance(config.distance)
        , offsets(config.offsets)
        , sigma(sigma) {};

    bool run() override {
        std::vector<simDistanceSensor> simDistanceSensor{};
        simDistanceSensor.reserve(offsets.size());
        for (const auto& offset : offsets) {
            simDistanceSensor.emplace_back(offset);
        }
        std::vector<float> expectedDistances{};
        for (auto& sensor : distance) {

            expectedDistances.emplace_back(sensor.get_distance());
        }
        lemlib::Pose newPose = iterateLocal(particles,
                                            simDistanceSensor,
                                            expectedDistances,
                                            chassis.getPose(),
                                            oldPose,
                                            sigma);
        chassis.setPose(newPose);
        oldPose = newPose;
        return true;
    };

  private:
    lemlib::Chassis& chassis;
    lemlib::Pose& oldPose;
    std::vector<pros::Distance> distance;
    float sigma;
    std::vector<particle>& particles;
    std::vector<lemlib::Pose> offsets;
};

class populateMCL_c : public command {
  public:
    explicit populateMCL_c(monteConfig& config, lemlib::Pose estStart)
        : particles(config.particles)
        , start(estStart) {};

    bool run() override {
        initalPopulate(particles, start);
        return true;
    };

  private:
    lemlib::Pose start;
    std::vector<particle>& particles;
};
