#include "../PluginStructs/P149_data_struct.h"

#ifdef USES_P149

# include "../ESPEasyCore/ESPEasyGPIO.h"

# include "../Commands/GPIO.h"

# include "../Helpers/Hardware.h"

# define GPIO_PLUGIN_ID  1

P149_data_struct::P149_data_struct(const P149_config_struct& config) : _config(config) {}

bool P149_data_struct::begin(double pos)
{
  if (!initialized) {
    initialized = true;
    position = pos;
    lastMovementTime_us = 0;

    stop();
  }

  return true;
}

void P149_data_struct::loop()
{
  const State old_state(state);

  switch (state) {
    case P149_data_struct::State::Idle:
      return true;
    case P149_data_struct::State::RunFwd:
    case P149_data_struct::State::RunRev:
    {
      updatePosition();
      checkLimit();
      break;
    }
    case P149_data_struct::State::StopPosReached:
      return false;
  }

  return old_state == state;
}

void P149_data_struct::timeChanged()
{
  switch (state) {
    case P149_data_struct::State::RunFwd:
    case P149_data_struct::State::RunRev:
    {
      lastMovementTime_us = getMicros64();
    }
  }
}

bool P149_data_struct::moveForward(double length)
{
  if (length > 0 && length <= 1) {
    updatePosition();
    pos_dest = position + length;
    startMoving();
    return true;
  }

  return false;
}

bool P149_data_struct::moveReverse(double length)
{
  if (length > 0 && length <= 1) {
    updatePosition();
    pos_dest = position - length;
    startMoving();
    return true;
  }

  return false;
}

bool P149_data_struct::moveToPos(double pos)
{
  if (pos >= 0 && pos <= 1) {
    updatePosition();
    pos_dest = pos;
    startMoving();
    return true;
  }

  return false;
}

void P149_data_struct::stop()
{
  if(lastMovementTime_us > 0) {
    updatePosition();
  }

  setPinState(_config.motorFwd, 0);
  setPinState(_config.motorRev, 0);

  lastMovementTime_us = 0;

  state = P149_data_struct::State::Idle;
}

double P149_data_struct::getPosition() const
{
  // correct for overshoot
  if(position > 1) {
    return 1;
  } else if(position < 0) {
    return 0;
  }

  return position;
}

void P149_data_struct::startMoving()
{
  resetOvershoot();
  
  if (pos_dest > position) {
    state = P149_data_struct::State::RunFwd;
    setPinState(_config.motorRev, 0);
    setPinState(_config.motorFwd, 1);

    if(pos_dest == 1) {
      // add overshoot
      pos_dest += ((double)_config.limitOvershootDuration)/_config.movementDuration;
    }
  } else {
    state = P149_data_struct::State::RunRev;
    setPinState(_config.motorFwd, 0);
    setPinState(_config.motorRev, 1);

    if(pos_dest == 0) {
      // add overshoot
      pos_dest -= ((double)_config.limitOvershootDuration)/_config.movementDuration;
    }
  }
  
  lastMovementTime_us = getMicros64();
}


void P149_data_struct::resetOvershoot() {
  if(position < 0) {
    position = 0;
  } else if(position > 1) {
    position = 1;
  }
}

void P149_data_struct::updatePosition() {
  int direction = 0;
  switch (state) {
    case P149_data_struct::State::RunFwd:
    {
      direction = P149_DIRECTION_FWD;
      break;
    }
    case P149_data_struct::State::RunRev:
    {
      direction = P149_DIRECTION_REV;
      break;
    }
    default:
      return;
  }

  uint64_t time = getMicros64();
  double moveDur = (double)(time-lastMovementTime_us)/1000;
  position += moveDur/_config.movementDuration*direction;

  lastMovementTime_us = time;
}


void P149_data_struct::checkLimit() {
  int direction = 0;
  switch (state) {
    case P149_data_struct::State::RunFwd:
    {
      direction = P149_DIRECTION_FWD;
      break;
    }
    case P149_data_struct::State::RunRev:
    {
      direction = P149_DIRECTION_REV;
      break;
    }
    default:
      return;
  }

  if(direction*(position-pos_dest) >= 0) {
    stop();
    resetOvershoot();
    state = P149_data_struct::State::StopPosReached;
  }
}

void P149_data_struct::setPinState(const P149_GPIO_config& gpio_config, int8_t state)
{
  // FIXME TD-er: Must move this code to the ESPEasy core code.
  uint8_t mode = PIN_MODE_OUTPUT;

  state = state == 0 ? gpio_config.low() : gpio_config.high();
  uint32_t key = createKey(GPIO_PLUGIN_ID, gpio_config.gpio);

  if (globalMapPortStatus[key].mode != PIN_MODE_OFFLINE)
  {
    int8_t currentState;
    GPIO_Read(GPIO_PLUGIN_ID, gpio_config.gpio, currentState);

    if (currentState == -1) {
      mode  = PIN_MODE_OFFLINE;
      state = -1;
    }

    if (mode == PIN_MODE_OUTPUT)  {
      createAndSetPortStatus_Mode_State(key, mode, state);
      GPIO_Write(
        GPIO_PLUGIN_ID,
        gpio_config.gpio,
        state,
        mode);
    }
  }
}

bool P149_data_struct::setPinMode(const P149_GPIO_config& gpio_config)
{
  if (checkValidPortRange(GPIO_PLUGIN_ID, gpio_config.gpio)) {
    pinMode(gpio_config.gpio, INPUT);
    return true;
  }
  return false;
}

#endif // ifdef USES_P149
