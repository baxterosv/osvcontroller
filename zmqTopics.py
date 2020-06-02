##### Controller Published #####
ZMQ_GRAPH_DATA_TOPIC = "ipc:///tmp/graph_data.pipe"
ZMQ_MEASURED_VALUES = "ipc:///tmp/measured_values.pipe"
ZMQ_CURRENT_SET_CONTROLS = "ipc:///tmp/current_set_controls.pipe"

# Passes a string followed by tuple of rgb values
# e.g.: "Display this text", (255,255,255)
ZMQ_OSV_STATUS = "ipc:///tmp/osv_status.pipe"

# Passes an enum of current alarm state
# e.g.: Alarm.TRIGGERED
ZMQ_TRIGGERED_ALARMS = "ipc:///tmp/triggered_alarms.pipe"

ZMQ_CONTROLLER_SETTINGS_ECHO = "ipc:///tmp/controller_settings_echo"


##### GUI Published #####
ZMQ_ALARM_SETPOINTS = "ipc:///tmp/alarm_setpoints.pipe"
ZMQ_CONTROL_SETPOINTS = "ipc:///tmp/control_setpoints.pipe"
ZMQ_START_STOP_COMMANDS = "ipc:///tmp/start_stop_commands.pipe"

# Passes a boolean: True for mute pressed, False for unpresed
# e.g.: True
ZMQ_MUTE_ALARMS = "ipc:///tmp/mute_alarms.pipe"


ZMQ_CONTROL_MODE = "ipc:///tmp/control_mode.pipe"

ZMQ_CONTROLLER_SETTINGS = "ipc:///tmp/controller_settings"