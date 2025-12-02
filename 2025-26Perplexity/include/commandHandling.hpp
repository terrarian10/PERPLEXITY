#pragma once

#include "Outtake.hpp"
#include "airCylinder.hpp"
#include "autons.hpp"
#include "consts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

class command {
  public:
    virtual ~command() = default;
    virtual bool run() { return true; }
    virtual void quit() {}
    virtual void force_quit() {}
};

class outtake_c : public command {
  public:
    explicit outtake_c(Outt_States state, Outtake& outtake)
        : state_(state)
        , outtake_(outtake) {};
    bool run() override {
        // return true;
        outtake_.move(state_);
        if (outtake_.get_state() == state_) {
            return true;
        } else {
            return false;
        }
    }
    void quit() override {}
    void force_quit() override {}

  private:
    Outtake& outtake_;
    Outt_States state_;
};

class togglePneu_c : public command {
  public:
    explicit togglePneu_c(AirCylinder& air)
        : air(air) {};
    bool run() override {
        air.toggle();
        return true;
    }
    void quit() override {}
    void force_quit() override {}

  private:
    AirCylinder& air;
};

class setPose_c : public command {
  public:
    explicit setPose_c(lemlib::Pose pose)
        : pose(pose) {};
    bool run() override {
        pose.theta > 9000 ? pose.theta = chassis.getPose().theta : pose.theta;
        chassis.setPose(pose);
        std::cout << chassis.getPose().x;
        return true;
    }
    void quit() override {}
    void force_quit() override {}

  private:
    lemlib::Pose pose;
};
class move_point_c : public command {
  public:
    explicit move_point_c(float x,
                          float y,
                          lemlib::Chassis& chassis,
                          poseCFG config = { false, 40, 127, 5000 })
        : chassis(chassis)
        , x(x)
        , y(y)
        , config(config)
        , started(false) {};
    bool run() override {
        if (!started) {
            chassis.moveToPoint(x,
                                y,
                                config.timeout,
                                {
                                    .forwards = !config.reversed,
                                    .maxSpeed = float(config.maxSpeed),

                                    .minSpeed = float(config.minSpeed),
                                    .earlyExitRange = config.earlyExitRange,

                                });
            started = true;
            return false;
        }
        return !chassis.isInMotion();
        // std::cout << chassis.getPose().x << " " << chassis.getPose().y
        //           << "--POSITION" << "\n";
    }

    void force_quit() override { chassis.cancelMotion(); }

  private:
    lemlib::Chassis& chassis;
    float x;
    float y;
    poseCFG config;
    bool started;
};
class move_pose_c : public command {
  public:
    explicit move_pose_c(lemlib::Pose pose,
                         lemlib::Chassis& chassis,
                         poseCFG config = { false, 40, 127, 5000 })
        : chassis(chassis)
        , pose(pose)
        , config(config)
        , started(false) {};
    bool run() override {
        if (!started) {
            chassis.moveToPose(pose.x,
                               pose.y,
                               pose.theta,
                               config.timeout,
                               {
                                   .forwards = !config.reversed,
                                   .horizontalDrift = 8,
                                   .maxSpeed = float(config.maxSpeed),

                                   .minSpeed = float(config.minSpeed),
                                   .earlyExitRange = config.earlyExitRange,

                               });
            started = true;
            std::cout << "GOAL X: " << pose.x << " Y: " << pose.y
                      << "ROT:" << pose.theta << "\n";

            return false;
        }
        return !chassis.isInMotion();
        // std::cout << chassis.getPose().x << " " << chassis.getPose().y
        //           << "--POSITION" << "\n";
    }

    void force_quit() override { chassis.cancelMotion(); }

  private:
    lemlib::Chassis& chassis;
    lemlib::Pose pose;
    poseCFG config;
    bool started;
};

class turn_heading_c : public command {
  public:
    explicit turn_heading_c(int dir,
                            lemlib::Chassis& chassis,
                            poseCFG config = { false, 40, 127, 5000 })
        : chassis(chassis)
        , dir(dir)
        , config(config)
        , started(false) {};
    bool run() override {
        if (!started) {
            chassis.turnToHeading(dir,
                                  config.timeout,
                                  {
                                      .maxSpeed = config.maxSpeed,
                                      .minSpeed = config.minSpeed,
                                      .earlyExitRange = config.earlyExitRange,
                                  });
            started = true;
            return false;
        }
        return !chassis.isInMotion();
        // std::cout << chassis.getPose().x << " " << chassis.getPose().y
        //           << "--POSITION" << "\n";
    }

    void force_quit() override { chassis.cancelMotion(); }

  private:
    lemlib::Chassis& chassis;
    int dir;
    poseCFG config;
    bool started;
};

class wait_c : public command {
  public:
    explicit wait_c(int msec)
        : time(msec)
        , started(false)
        , start_time(0) {};
    bool run() override {
        if (!started) {
            started = true;

            start_time = pros::millis();
        }
        return (pros::millis() - start_time) >= time;
    }

  private:
    bool started;
    int time;
    int start_time;
};

class brake_c : public command {
  public:
    explicit brake_c(lemlib::Chassis& chassis, pros::motor_brake_mode_e brake)
        : chassis(chassis)
        , brake(brake) {};
    bool run() override {
        chassis.setBrakeMode(brake);
        return true;
    }

  private:
    lemlib::Chassis& chassis;
    pros::motor_brake_mode_e brake;
};
class repeat_cmd : public command {
  public:
    template<typename T>
    explicit repeat_cmd(T&& inner_cmd)
        : inner(std::make_unique<std::decay_t<T>>(std::forward<T>(inner_cmd))) {
    }

    void stop() { stopped = true; }

    bool run() override {
        if (stopped) return true; // Tell scheduler to remove me
        inner->run();             // Run the wrapped command
        return false;             // Keep repeating
    }

  private:
    std::unique_ptr<command> inner;
    bool stopped = false;
};
class group_repeat_cmd : public command {
  public:
    template<typename... Ts>
    explicit group_repeat_cmd(Ts&&... cmds) {
        // Fold expression over the argument pack
        (add_one(std::forward<Ts>(cmds)), ...);
    }

    void stop() { stopped = true; }

    bool run() override {
        if (stopped) return true; // tell scheduler to remove this whole group

        for (auto& c : commands) {
            c->run(); // ignore inner return values; we control lifetime
        }
        return false; // keep repeating
    }

  private:
    std::vector<std::unique_ptr<command>> commands;
    bool stopped = false;

    template<typename T>
    void add_one(T&& cmd) {
        using U = std::decay_t<T>;
        static_assert(
            std::is_base_of_v<command, U>,
            "group_repeat_cmd can only contain types derived from command");

        // Create a unique_ptr<command> that owns a U*
        commands.emplace_back(
            std::unique_ptr<command>(new U(std::forward<T>(cmd))));
    }
};

class Scheduler {
  public:
    template<typename T, class... Args>
    T& enqueue(Args&&... args) {
        static_assert(std::is_base_of<command, T>::value,
                      "Cmd must derive from command");
        auto ptr = std::make_unique<T>(std::forward<Args>(args)...);
        T& ref = *ptr;
        schedule.emplace_back(
            std::move(ptr)); // commands is vector<unique_ptr<command>>
        return ref;
    }

    bool tick() {
        if (schedule.empty()) { return false; }
        command* cmd = schedule.front().get();
        // std::cout << "ticking" << "\n";
        // std::cout << schedule.size() << "\n";

        const bool done = cmd->run();

        if (done) {
            cmd->quit();
            schedule.pop_front();
            // gpsRefinerTask(gps);
            std::cout << "Removed. At Size: " << schedule.size()
                      << " RESULT AT: " << chassis.getPose().x * 2.54 << " "
                      << chassis.getPose().y * 2.54 << " "
                      << chassis.getPose().theta << "Degrees" << std::endl
                      << outtake.log();
            tick();
        }
        return !schedule.empty();
    };

    void move_to_back() {
        if (schedule.empty()) return;
        schedule.emplace_back(std::move(schedule.front()));
        schedule.front()->quit();
        schedule.pop_front();
    }
    void move_back(int amount) {

        schedule.emplace(schedule.begin() + amount,
                         std::move(schedule.front()));
        schedule.front()->quit();

        schedule.pop_front();
    }
    void move_to_front() {
        schedule.emplace_front(std::move(schedule.back()));
        schedule.back()->quit();

        schedule.pop_back();
    }
    void force_quit_current() {
        if (schedule.empty()) return;
        schedule.front()->force_quit();
        schedule.pop_front();
    }

    bool empty() const { return schedule.empty(); }
    void clear() {
        while (!schedule.empty())
            force_quit_current();
    }

  private:
    std::deque<std::unique_ptr<command>> schedule;
};
