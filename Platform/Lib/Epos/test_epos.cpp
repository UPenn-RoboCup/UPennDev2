#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include "co_master.h"
#include "epos_slave.h"

#define NODE_ID 1
#define TIMEOUT 1

co_sdo_setting sdo_settings[] = {
  {EPOS_CAN_BITRATE, 0x00, 0x02}, /* change to 1Mbps ? */
  {EPOS_SENSOR_CONFIGURATION, 0x01, 1000},
  {EPOS_SENSOR_CONFIGURATION, 0x02, 1},
  {EPOS_SENSOR_CONFIGURATION, 0x04, 0},
  {EPOS_CONFIGURATION_OF_DIGITAL_INPUTS, 0x04, 0x03},
  {EPOS_CONFIGURATION_OF_DIGITAL_INPUTS, 0x05, 0x04},
  {EPOS_CONFIGURATION_OF_DIGITAL_INPUTS, 0x06, 0x05},
  {EPOS_CONFIGURATION_OF_DIGITAL_INPUTS, 0x01, 0x02},
  {EPOS_CONFIGURATION_OF_DIGITAL_INPUTS, 0x02, 0x00},
  {EPOS_CONFIGURATION_OF_DIGITAL_INPUTS, 0x03, 0x01},
  {EPOS_DIGITAL_INPUT_FUNCTIONALITIES, 0x02, 0xE3C7},
  {EPOS_DIGITAL_INPUT_FUNCTIONALITIES, 0x03, 0x0000},
  {EPOS_DIGITAL_INPUT_FUNCTIONALITIES, 0x04, 0x000B},
  {EPOS_DIGITAL_OUTPUT_FUNCTIONALITIES, 0x01, 0x0000},
  {EPOS_DIGITAL_OUTPUT_FUNCTIONALITIES, 0x02, 0x1800},
  {EPOS_DIGITAL_OUTPUT_FUNCTIONALITIES, 0x03, 0x0000},
  {EPOS_CONFIGURATION_OF_DIGITAL_OUTPUTS, 0x01, 0x0F},
  {EPOS_CONFIGURATION_OF_DIGITAL_OUTPUTS, 0x02, 0x0E},
  {EPOS_CONFIGURATION_OF_DIGITAL_OUTPUTS, 0x03, 0x0D},
  {EPOS_CONFIGURATION_OF_DIGITAL_OUTPUTS, 0x04, 0x0C},
  {EPOS_CONFIGURATION_OF_ANALOG_INPUTS, 0x01, 0x000F},
  {EPOS_CONFIGURATION_OF_ANALOG_INPUTS, 0x02, 0x0000},
  {EPOS_ANALOG_INPUT_FUNCTIONALITIES_EXECUTION_MASK, 0x00, 0x0000},
  {EPOS_ANALOG_OUTPUT_1, 0x00, 0},
  {EPOS_CURRENT_MODE_SETTING_VALUE, 0x00, 0},
  {EPOS_VELOCITY_MODE_SETTING_VALUE, 0x00, 0},
  {EPOS_POSITION_MODE_SETTING_VALUE, 0x00, 0},
  {DSP402_CONTROLWORD, 0x00, 0},
  {DSP402_MODES_OF_OPERATION, 0x00, 1},
  {DSP402_FOLLOWING_ERROR_WINDOW, 0x00, 10000},
  {DSP402_POSITION_WINDOW, 0x00, 1e9},
  {DSP402_POSITION_WINDOW_TIME, 0x00, 0},
  {DSP402_TARGET_POSITION, 0x00, 0},
  {DSP402_TARGET_VELOCITY, 0x00, 0},
  {DSP402_MOTION_PROFILE_TYPE, 0x00, 0},
  {DSP402_PROFILE_VELOCITY, 0x00, 12000},
  {DSP402_PROFILE_ACCELERATION, 0x00, 50000},
  {DSP402_PROFILE_DECELERATION, 0x00, 50000},
  {DSP402_QUICK_STOP_DECELERATION, 0x00, 10000},
  {DSP402_MAX_PROFILE_VELOCITY, 0x00, 12000},
  {DSP402_MAX_ACCELERATION, 0x00, 100000},
  {DSP402_TORQUE_CONTROL_PARAMETER_SET, 0x01, 300},
  {DSP402_TORQUE_CONTROL_PARAMETER_SET, 0x02, 100},
  {DSP402_VELOCITY_CONTROL_PARAMETER_SET, 0x01, 1200},
  {DSP402_VELOCITY_CONTROL_PARAMETER_SET, 0x02, 40},
  {DSP402_VELOCITY_CONTROL_PARAMETER_SET, 0x03, 0},
  {DSP402_POSITION_CONTROL_PARAMETER_SET, 0x01, 70},
  {DSP402_POSITION_CONTROL_PARAMETER_SET, 0x02, 70},
  {DSP402_POSITION_CONTROL_PARAMETER_SET, 0x03, 145},
  {DSP402_POSITION_CONTROL_PARAMETER_SET, 0x04, 0},
  {DSP402_POSITION_CONTROL_PARAMETER_SET, 0x05, 0},
  {DSP402_MOTOR_TYPE, 0x00, 10},
  {DSP402_MOTOR_DATA, 0x01, 3000},
  {DSP402_MOTOR_DATA, 0x02, 6000},
  {DSP402_MOTOR_DATA, 0x03, 2},
  {DSP402_MOTOR_DATA, 0x04, 12500},
  {DSP402_MOTOR_DATA, 0x05, 40},
  /* actuator specific settings */
  {EPOS_HOME_POSITION, 0x00, 0},
  {EPOS_CURRENT_THRESHOLD_FOR_HOMING_MODE, 0x00, 500},
  {DSP402_HOME_OFFSET, 0x00, 0},
  {DSP402_HOMING_METHOD, 0x00, 23},
  {DSP402_HOMING_SPEEDS, 0x01, 100},
  {DSP402_HOMING_SPEEDS, 0x02, 20},
  {DSP402_HOMING_ACCELERATION, 0x00, 1000},
  {DSP402_SOFTWARE_POSITION_LIMIT, 0x01, -50000},
  {DSP402_SOFTWARE_POSITION_LIMIT, 0x02, 50000},
  {CO_SENTINEL}
};

co_pdo_parameter pdo_parameters[] = {
  {CO_RPDO1, CO_SYNCHRONOUS + 1, 0, 0},
  {CO_RPDO2, CO_SYNCHRONOUS + 1, 0, 0},
  {CO_RPDO3, CO_SYNCHRONOUS + 1, 0, 0},
  {CO_RPDO4, CO_SYNCHRONOUS + 1, 0, 0},
  {CO_TPDO1, CO_SYNCHRONOUS + 1, 0, 0},
  {CO_TPDO2, CO_ASYNCHRONOUS, 1000, 0},
  {CO_TPDO3, CO_ASYNCHRONOUS, 0, 0},
  {CO_TPDO4 | CO_INVALID, CO_RTR_UPDATE, 0, 0},
  {CO_SENTINEL}
};

co_pdo_mapping pdo_mappings[] = {
  {CO_RPDO1, 3,
    {CO_ENTRY(DSP402_CONTROLWORD, 0x00),
     CO_ENTRY(EPOS_POSITION_MODE_SETTING_VALUE, 0x00),
     CO_ENTRY(EPOS_CURRENT_MODE_SETTING_VALUE, 0x00)}
  },
  {CO_RPDO2, 2,
    {CO_ENTRY(DSP402_POSITION_CONTROL_PARAMETER_SET, 0x01), 
     CO_ENTRY(DSP402_POSITION_CONTROL_PARAMETER_SET, 0x02)}
  },
  {CO_RPDO3, 2,
    {CO_ENTRY(DSP402_TORQUE_CONTROL_PARAMETER_SET, 0x01), 
     CO_ENTRY(DSP402_TORQUE_CONTROL_PARAMETER_SET, 0x02)}
  },
  {CO_RPDO4, 1,
    {CO_ENTRY(DSP402_MODES_OF_OPERATION, 0x00)}
  },
  {CO_TPDO1, 3,
    {CO_ENTRY(DSP402_STATUSWORD, 0x00),
     CO_ENTRY(DSP402_POSITION_ACTUAL_VALUE, 0x00),
     CO_ENTRY(EPOS_ANALOG_INPUTS, 0x01)}
  },
  {CO_TPDO2, 1,
    {CO_ENTRY(DSP402_CURRENT_ACTUAL_VALUE, 0x00)}
  },
  {CO_TPDO3, 1,
    {CO_ENTRY(DSP402_MODES_OF_OPERATION_DISPLAY, 0x00)}
  },
  {CO_TPDO4, 0},
  {CO_SENTINEL}
};

static inline double get_time() 
{ 
  /* return monotonic system time in seconds */
  timespec ts;
#ifdef CLOCK_MONOTONIC_RAW
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
#else
  clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
  return ts.tv_sec + ts.tv_nsec*1e-9;
}

int main()
{
  /* initialize can bus */
  can_channel can0;
  if (can0.open_channel("can0") < 0)
    fprintf(stderr, "Error: unable to open can interface\n");

  /* initialize slave */
  epos_slave slave(NODE_ID);
  slave.register_sdo_settings(sdo_settings);
  slave.register_pdo_parameters(pdo_parameters);
  slave.register_pdo_mappings(pdo_mappings);

  /* initialize master */
  co_master can0_master(&can0); 
  can0_master.enable_debug_print(true);
  can0_master.register_slave(&slave);

  /* initialize controller */
  can0_master.enter_preoperational_state(NODE_ID);
  slave.shutdown();
  can0_master.sdo_download(NODE_ID, DSP402_CONTROLWORD, 0x00);

  if (!can0_master.configure_sdo_settings(NODE_ID, sdo_settings, TIMEOUT))
    fprintf(stderr, "Error: sdo configuration failed\n");
  if (!can0_master.configure_pdo_settings(NODE_ID, TIMEOUT))
    fprintf(stderr, "Error: pdo configuration failed\n");

  /* set modes of operation */
  slave.set_value(DSP402_MODES_OF_OPERATION, 0x00, -1); /* position mode */
//slave.set_value(DSP402_MODES_OF_OPERATION, 0x00, -3); /* current mode */
  can0_master.sdo_download(NODE_ID, DSP402_MODES_OF_OPERATION, 0x00);

  /* enable controller */
  slave.enable_operation();
  can0_master.sdo_download(NODE_ID, DSP402_CONTROLWORD, 0x00);
  can0_master.start_remote_node(NODE_ID);
  can0_master.sdo_upload(NODE_ID, DSP402_POSITION_ACTUAL_VALUE, 0x00);

  /* start control loop */ 
  double t0 = get_time();
  double p0 = slave.get_value(DSP402_POSITION_ACTUAL_VALUE, 0X00); 
  co_can_id receive_ids[] = {CO_TPDO1 | NODE_ID};
  while (1)
  {
    /* get position setpoint */
    double t = get_time();
    int current_setpoint = 0;
    int position_setpoint = p0 + 2000*sin(2*(t - t0));
    /* write dictionary values */
    slave.set_controlbit_new_setpoint(1);
    slave.set_controlbit_change_set_immediately(1);
    slave.set_value(EPOS_CURRENT_MODE_SETTING_VALUE, 0x00, current_setpoint);
    slave.set_value(EPOS_POSITION_MODE_SETTING_VALUE, 0x00, position_setpoint);
    can0_master.send_rpdo(NODE_ID, 1);
    can0_master.send_sync(0);
    /* read dictionary values */
    can0_master.receive(receive_ids, 1, TIMEOUT);
    can0_master.flush_can_channel();
    fprintf(stderr, "Statusword : %d\n",
      (int)slave.get_value(DSP402_STATUSWORD, 0x00));
    fprintf(stderr, "Modes of operation display : %d\n",
      (int)slave.get_value(DSP402_MODES_OF_OPERATION, 0x00));
    fprintf(stderr, "Analog input 1 : %d\n",
      (int)slave.get_value(EPOS_ANALOG_INPUTS, 0x01));
    fprintf(stderr, "Position mode setting value : %d\n",
      position_setpoint);
    fprintf(stderr, "Current mode setting value : %d\n",
      current_setpoint);
    fprintf(stderr, "Position actual value : %d\n",
      (int)slave.get_value(DSP402_POSITION_ACTUAL_VALUE, 0x00));
    fprintf(stderr, "Current actual value : %d\n",
      (int)slave.get_value(DSP402_CURRENT_ACTUAL_VALUE, 0x00));
    usleep(1000);
    system("clear");
  }
 
  can0.close_channel();
  return 0;
}
