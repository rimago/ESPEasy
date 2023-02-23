#ifndef PLUGINSTRUCTS_P149_DATA_STRUCT_H
#define PLUGINSTRUCTS_P149_DATA_STRUCT_H

#include "../../_Plugin_Helper.h"
#ifdef USES_P149

#define P149_DIRECTION_FWD 1
#define P149_DIRECTION_REV -1

struct P149_GPIO_config {
  byte high() const {
    return inverted ? 0 : 1;
  }

  byte low() const {
    return inverted ? 1 : 0;
  }

  // Don't call this from ISR functions.
  bool readState() const {
    const bool state = digitalRead(gpio) != 0;

    return inverted ? !state : state;
  }

  int      gpio     = -1;
  bool     inverted = false;
};
struct P149_config_struct {
  P149_GPIO_config motorFwd;
  P149_GPIO_config motorRev;

  uint64_t movementDuration;
  uint64_t limitOvershootDuration;

  uint32 positionMax;
};

struct P149_data_struct : public PluginTaskData_base {
  enum class State {
    Idle,
    RunFwd,
    RunRev,
    StopPosReached
  };

  P149_data_struct(const P149_config_struct& config);
  P149_data_struct() = delete;

  bool begin(double pos);

  // Perform regular loop
  void loop();

  // Run the motor length forward
  bool moveForward(double length);

  // Run the motor length in revere
  bool moveReverse(double length);

  // Move to relative position.
  bool moveToPos(double pos);

  // Stop the motor idle.
  void stop();

  void timeChanged();

  double  getPosition() const;

  State state       = State::Idle;

private:
  bool  initialized = false;

  const P149_config_struct         _config;
  volatile double                  position = 0;
  volatile uint64_t                lastMovementTime_us = 0;
  double                           pos_dest = 0;

  void        resetOvershoot();
  void        startMoving();

  void        updatePosition();
  void        checkLimit();

  void setPinState(const P149_GPIO_config& gpio_config,
                          int8_t                  state);

  static bool setPinMode(const P149_GPIO_config& gpio_config);

};

#endif // ifdef USES_P149
#endif // ifndef PLUGINSTRUCTS_P149_DATA_STRUCT_H
