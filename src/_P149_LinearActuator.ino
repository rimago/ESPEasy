#include "_Plugin_Helper.h"
#ifdef USES_P149

// #######################################################################################################
// ######################## Plugin 149 Linear Actuator  ########################
// #######################################################################################################


# include "src/PluginStructs/P149_data_struct.h"

# define PLUGIN_149
# define PLUGIN_ID_149         149
# define PLUGIN_NAME_149       "Linear Actuator"
# define PLUGIN_VALUENAME1_149 "Position"

# define P149_FLAGS                     PCONFIG_LONG(0)
# define P149_POSITION_MAX              PCONFIG(0)
# define P149_MOVEMENT_DURATION         PCONFIG(1)
# define P149_LIMIT_OVERSHOOT_DURATION  PCONFIG(2)

# define P149_FLAGBIT_MOTOR_FWD_INVERTED   0
# define P149_FLAGBIT_MOTOR_REV_INVERTED   1


boolean Plugin_149(uint8_t function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
    {
      Device[++deviceCount].Number           = PLUGIN_ID_149;
      Device[deviceCount].Type               = DEVICE_TYPE_CUSTOM0;
      Device[deviceCount].VType              = Sensor_VType::SENSOR_TYPE_DIMMER;
      Device[deviceCount].Ports              = 0;
      Device[deviceCount].PullUpOption       = false;
      Device[deviceCount].InverseLogicOption = false;
      Device[deviceCount].FormulaOption      = false;
      Device[deviceCount].ValueCount         = 1;
      Device[deviceCount].SendDataOption     = true;
      Device[deviceCount].TimerOption        = true;
      Device[deviceCount].TimerOptional      = true;
      break;
    }

    case PLUGIN_GET_DEVICENAME:
    {
      string = F(PLUGIN_NAME_149);
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES:
    {
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_149));
      break;
    }

    case PLUGIN_WEBFORM_SHOW_GPIO_DESCR:
    {
      const __FlashStringHelper *labels[] = {
        F("M Fwd"),
        F("M Rev")
      };
      int values[] = {
        CONFIG_PIN1,
        CONFIG_PIN2
      };
      constexpr size_t nrElements = sizeof(values) / sizeof(values[0]);

      for (size_t i = 0; i < nrElements; ++i) {
        if (i != 0) { addHtml(event->String1); }
        addHtml(labels[i]);
        addHtml(F(":&nbsp;"));
        addHtml(formatGpioLabel(values[i], true));
      }

      success = true;
      break;
    }
    case PLUGIN_SET_DEFAULTS:
    {
      P149_FLAGS              = 0;
      P149_POSITION_MAX       = 100;
      P149_MOVEMENT_DURATION   = 0;
      P149_LIMIT_OVERSHOOT_DURATION = 0;

      break;
    }

    case PLUGIN_WEBFORM_LOAD:
    {
      addFormSubHeader(F("Motor Control"));

      addFormPinSelect(PinSelectPurpose::Generic_output,
                       formatGpioName_output(F("Motor Fwd")),
                       F("taskdevicepin1"),
                       CONFIG_PIN1);
      addFormCheckBox(F("Motor Fwd Inverted"), F("fwd_inv"), bitRead(P149_FLAGS, P149_FLAGBIT_MOTOR_FWD_INVERTED));

      addFormSeparator(2);

      addFormPinSelect(PinSelectPurpose::Generic_output,
                       formatGpioName_output(F("Motor Rev")),
                       F("taskdevicepin2"),
                       CONFIG_PIN2);
      addFormCheckBox(F("Motor Rev Inverted"), F("rev_inv"), bitRead(P149_FLAGS, P149_FLAGBIT_MOTOR_REV_INVERTED));

      addFormSeparator(2);

      addFormNumericBox(F("Position Max"), F("position_max"), P149_POSITION_MAX, 0, 2147483647);

      addFormNumericBox(F("Motion Duration"), F("motion_duration"), P149_MOVEMENT_DURATION, 0, 2147483647);
      addUnit(F("ms"));

      addFormNumericBox(F("Limit Overshoot Duration"), F("limit_overshoot_duration"), P149_LIMIT_OVERSHOOT_DURATION, 0, 2147483647);
      addUnit(F("ms"));

      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE:
    {
      CONFIG_PIN1 = getFormItemInt(F("taskdevicepin1"));
      CONFIG_PIN2 = getFormItemInt(F("taskdevicepin2"));

      P149_POSITION_MAX          = getFormItemInt(F("position_max"));
      P149_MOVEMENT_DURATION          = getFormItemInt(F("motion_duration"));
      P149_LIMIT_OVERSHOOT_DURATION   = getFormItemInt(F("limit_overshoot_duration"));

      P149_FLAGS = 0;

      if (isFormItemChecked(F("fwd_inv"))) { bitSet(P149_FLAGS, P149_FLAGBIT_MOTOR_FWD_INVERTED); }

      if (isFormItemChecked(F("rev_inv"))) { bitSet(P149_FLAGS, P149_FLAGBIT_MOTOR_REV_INVERTED); }

      success = true;
      break;
    }

    case PLUGIN_INIT:
    {
      P149_config_struct config;
      config.motorFwd.gpio    = CONFIG_PIN1;
      config.motorRev.gpio    = CONFIG_PIN2;

      config.motorFwd.inverted = bitRead(P149_FLAGS, P149_FLAGBIT_MOTOR_FWD_INVERTED);
      config.motorRev.inverted = bitRead(P149_FLAGS, P149_FLAGBIT_MOTOR_REV_INVERTED);

      config.positionMax = P149_POSITION_MAX;
      config.movementDuration = P149_MOVEMENT_DURATION;
      config.limitOvershootDuration = P149_LIMIT_OVERSHOOT_DURATION;

      initPluginTaskData(event->TaskIndex, new (std::nothrow) P149_data_struct(config));
      P149_data_struct *P149_data =
        static_cast<P149_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (nullptr != P149_data) {
        // Restore the last known values on warm boot.
        P149_data->begin(((double)UserVar[event->BaseVarIndex + 0])/P149_POSITION_MAX);
        success = true;
      }
      break;
    }

    case PLUGIN_FIFTY_PER_SECOND:
    {
      P149_data_struct *P149_data =
        static_cast<P149_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (nullptr != P149_data) {

        P149_data->loop();

        if(P149_data->state == P149_data_struct::State::StopPosReached) 
          {
            if (Settings.UseRules) {
              String RuleEvent = getTaskDeviceName(event->TaskIndex);
              RuleEvent += '#';

              eventQueue.addMove(String(RuleEvent + F("positionReached")));
            }
            P149_data->state = P149_data_struct::State::Idle;
          }
        }

        UserVar[event->BaseVarIndex + 0] = (float)P149_data->getPosition()*P149_POSITION_MAX;
        success = true;
      }
      break;
    }

    case PLUGIN_READ:
    {
      P149_data_struct *P149_data =
        static_cast<P149_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (nullptr != P149_data) {
        UserVar[event->BaseVarIndex + 0] = P149_data->getPosition()*P149_POSITION_MAX;
        success = true;
      }
      break;
    }

    case PLUGIN_TIME_CHANGE:
    {
      P149_data_struct *P149_data =
        static_cast<P149_data_struct *>(getPluginTaskData(event->TaskIndex));

      P149_data->timeChanged();
      break;
    }

    case PLUGIN_WRITE:
    {
      P149_data_struct *P149_data =
        static_cast<P149_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (nullptr != P149_data) {
        const String command = parseString(string, 1);

        if (equals(command, F("forward"))) {
          addLog(LOG_LEVEL_INFO, "forward received");
          success = P149_data->moveForward(((double)event->Par1)/P149_POSITION_MAX);
        } else if (equals(command, F("reverse"))) {
          addLog(LOG_LEVEL_INFO, "reverse received");
          success = P149_data->moveReverse(((double)event->Par1)/P149_POSITION_MAX);
        } else if (equals(command, F("stop"))) {
          addLog(LOG_LEVEL_INFO, "stop received");
          P149_data->stop();
          success = true;
        } else if (equals(command, F("movetopos"))) {
          addLog(LOG_LEVEL_INFO, "movetopos received");
          success = P149_data->moveToPos(((double)event->Par1)/P149_POSITION_MAX);
        }
      }

      break;
    }
  }
  return success;
}

#endif // USES_P149
