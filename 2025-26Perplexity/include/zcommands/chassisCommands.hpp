#include "commandHandling.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "virtualController.hpp"

class tank_c : command {
  public:
    explicit tank_c(lemlib::Chassis& chassis, virtualController& vc)
        : chassis(chassis)
        , vc(vc) {};
    bool run() override;

  private:
    lemlib::Chassis& chassis;
    virtualController& vc;
};