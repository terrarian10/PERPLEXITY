#include "commandHandling.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/distance.hpp"
#include "zposition/localisis.hpp"
#include <iterator>
#include <vector>

struct monteConfig {
    std::vector<particle>& particles;
    lemlib::Chassis& chassis;
    lemlib::Pose& oldPose;
    std::array<pros::Distance, 4>& distance;
    std::vector<simDistanceSensor>& offsets;
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

        std::vector<float> expectedDistances{};
        std::vector<simDistanceSensor> validOffsets{};
        int i = 0;
        for (auto& sensor : distance) {
            if (sensor.get_distance() == errno) {
                continue;
            } else {
                expectedDistances.emplace_back(sensor.get_distance());
                validOffsets.emplace_back(offsets.at(i));
            }
            i++;
        }
        iterateLocal(particles,
                     validOffsets,
                     expectedDistances,
                     chassis.getPose(),
                     oldPose,
                     sigma);
        oldPose = chassis.getPose();

        return true;
    };

  private:
    lemlib::Chassis& chassis;
    lemlib::Pose& oldPose;
    std::array<pros::Distance, 4>& distance;
    float sigma;
    std::vector<particle>& particles;
    std::vector<simDistanceSensor>& offsets;
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

        std::vector<float> expectedDistances{};
        std::vector<simDistanceSensor> validOffsets{};
        int i = 0;
        for (auto& sensor : distance) {
            if (sensor.get_distance() == errno) {
                continue;
            } else {
                expectedDistances.emplace_back(sensor.get_distance());
                validOffsets.emplace_back(offsets.at(i));
            }
            i++;
        }
        lemlib::Pose newPose = iterateLocal(particles,
                                            validOffsets,
                                            expectedDistances,
                                            chassis.getPose(),
                                            oldPose,
                                            sigma);
        chassis.setPose(newPose);
        oldPose = newPose;
        std::cout << newPose.x << " " << newPose.y << " " << newPose.theta
                  << std::endl;
        return true;
    };

  private:
    lemlib::Chassis& chassis;
    lemlib::Pose& oldPose;
    std::array<pros::Distance, 4>& distance;
    float sigma;
    std::vector<particle>& particles;
    std::vector<simDistanceSensor>& offsets;
};

class populateMCL_c : public command {
  public:
    explicit populateMCL_c(monteConfig& config, lemlib::Pose estStart)
        : particles(config.particles)
        , start(estStart) {};

    bool run() override {

        initalPopulate(particles, start);
        std::cout << "Populated MCL with " << particles.size() << " particles"
                  << particles.at(0).pos.x << " " << particles.at(0).pos.y
                  << " " << particles.at(0).pos.theta << std::endl;

        return true;
    };

  private:
    lemlib::Pose start;
    std::vector<particle>& particles;
};
