#pragma once

#include "Outtake.hpp"
#include "consts.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
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
        outtake_.run_at_state(state_);
        return true;
    }
    void quit() override {}
    void force_quit() override {}

  private:
    Outtake& outtake_;
    Outt_States state_;
};

class move_pose_c : public command {
  public:
    explicit move_pose_c(lemlib::Pose pose, lemlib::Chassis& chassis)
        : chassis(chassis)
        , pose(pose) {};
    bool run() override {
        if (!started) {
            chassis.moveToPose(pose.x, pose.y, pose.theta, 5000);
            started = true;
        }
        return !chassis.isInMotion();
    }

    void force_quit() override { chassis.cancelMotion(); }

  private:
    lemlib::Chassis& chassis;
    lemlib::Pose pose;
    bool started;
};

class Scheduler {
  public:
    template<class Cmd, class... Args>
    void enqueue(Args&&... args) {
        static_assert(std::is_base_of<command, Cmd>::value,
                      "Cmd must derive from command");
        schedule.emplace_back(
            std::make_unique<Cmd>(std::forward<Args>(args)...));
    }

    bool tick() {
        if (schedule.empty()) { return false; }
        command* cmd = schedule.front().get();
        const bool done = cmd->run();
        if (done) {
            cmd->quit();
            schedule.pop_front();
        }
        return !schedule.empty();
    };

    void move_to_back() {
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