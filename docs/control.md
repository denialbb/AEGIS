Control¶
class Control¶
Used to manipulate the controls of a vessel. This includes adjusting the throttle, enabling/disabling systems such as SAS and RCS, or altering the direction in which the vessel is pointing. Obtained by calling Vessel.control.

Note

Control inputs (such as pitch, yaw and roll) are zeroed when all clients that have set one or more of these inputs are no longer connected.

source¶
The source of the vessels control, for example by a kerbal or a probe core.

Attribute
:
Read-only, cannot be set

Return type
:
ControlSource

Game Scenes
:
Flight

state¶
The control state of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
ControlState

Game Scenes
:
Flight

sas¶
The state of SAS.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

Note

Equivalent to AutoPilot.sas

sas_mode¶
The current SASMode. These modes are equivalent to the mode buttons to the left of the navball that appear when SAS is enabled.

Attribute
:
Can be read or written

Return type
:
SASMode

Game Scenes
:
Flight

Note

Equivalent to AutoPilot.sas_mode

speed_mode¶
The current SpeedMode of the navball. This is the mode displayed next to the speed at the top of the navball.

Attribute
:
Can be read or written

Return type
:
SpeedMode

Game Scenes
:
Flight

rcs¶
The state of RCS.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

reaction_wheels¶
Returns whether all reactive wheels on the vessel are active, and sets the active state of all reaction wheels. See ReactionWheel.active.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

gear¶
The state of the landing gear/legs.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

legs¶
Returns whether all landing legs on the vessel are deployed, and sets the deployment state of all landing legs. Does not include wheels (for example landing gear). See Leg.deployed.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

wheels¶
Returns whether all wheels on the vessel are deployed, and sets the deployment state of all wheels. Does not include landing legs. See Wheel.deployed.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

lights¶
The state of the lights.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

brakes¶
The state of the wheel brakes.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

antennas¶
Returns whether all antennas on the vessel are deployed, and sets the deployment state of all antennas. See Antenna.deployed.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

cargo_bays¶
Returns whether any of the cargo bays on the vessel are open, and sets the open state of all cargo bays. See CargoBay.open.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

intakes¶
Returns whether all of the air intakes on the vessel are open, and sets the open state of all air intakes. See Intake.open.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

parachutes¶
Returns whether all parachutes on the vessel are deployed, and sets the deployment state of all parachutes. Cannot be set to False. See Parachute.deployed.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

radiators¶
Returns whether all radiators on the vessel are deployed, and sets the deployment state of all radiators. See Radiator.deployed.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

resource_harvesters¶
Returns whether all of the resource harvesters on the vessel are deployed, and sets the deployment state of all resource harvesters. See ResourceHarvester.deployed.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

resource_harvesters_active¶
Returns whether any of the resource harvesters on the vessel are active, and sets the active state of all resource harvesters. See ResourceHarvester.active.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

solar_panels¶
Returns whether all solar panels on the vessel are deployed, and sets the deployment state of all solar panels. See SolarPanel.deployed.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

abort¶
The state of the abort action group.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

throttle¶
The state of the throttle. A value between 0 and 1.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

input_mode¶
Sets the behavior of the pitch, yaw, roll and translation control inputs. When set to additive, these inputs are added to the vessels current inputs. This mode is the default. When set to override, these inputs (if non-zero) override the vessels inputs. This mode prevents keyboard control, or SAS, from interfering with the controls when they are set.

Attribute
:
Can be read or written

Return type
:
ControlInputMode

Game Scenes
:
Flight

pitch¶
The state of the pitch control. A value between -1 and 1. Equivalent to the w and s keys.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

yaw¶
The state of the yaw control. A value between -1 and 1. Equivalent to the a and d keys.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

roll¶
The state of the roll control. A value between -1 and 1. Equivalent to the q and e keys.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

forward¶
The state of the forward translational control. A value between -1 and 1. Equivalent to the h and n keys.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

up¶
The state of the up translational control. A value between -1 and 1. Equivalent to the i and k keys.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

right¶
The state of the right translational control. A value between -1 and 1. Equivalent to the j and l keys.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

wheel_throttle¶
The state of the wheel throttle. A value between -1 and 1. A value of 1 rotates the wheels forwards, a value of -1 rotates the wheels backwards.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

wheel_steering¶
The state of the wheel steering. A value between -1 and 1. A value of 1 steers to the left, and a value of -1 steers to the right.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

custom_axis01¶
The state of CustomAxis01. A value between -1 and 1.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

custom_axis02¶
The state of CustomAxis02. A value between -1 and 1.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

custom_axis03¶
The state of CustomAxis03. A value between -1 and 1.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

custom_axis04¶
The state of CustomAxis04. A value between -1 and 1.

Attribute
:
Can be read or written

Return type
:
float

Game Scenes
:
Flight

current_stage¶
The current stage of the vessel. Corresponds to the stage number in the in-game UI.

Attribute
:
Read-only, cannot be set

Return type
:
int

Game Scenes
:
Flight

activate_next_stage()¶
Activates the next stage. Equivalent to pressing the space bar in-game.

Returns
:
A list of vessel objects that are jettisoned from the active vessel.

Return type
:
list(Vessel)

Game Scenes
:
Flight

Note

When called, the active vessel may change. It is therefore possible that, after calling this function, the object(s) returned by previous call(s) to active_vessel no longer refer to the active vessel. Throws an exception if staging is locked.

stage_lock¶
Whether staging is locked on the vessel.

Attribute
:
Can be read or written

Return type
:
bool

Game Scenes
:
Flight

Note

This is equivalent to locking the staging using Alt+L

get_action_group(group)¶
Returns True if the given action group is enabled.

Parameters
:
group (int) – A number between 0 and 9 inclusive, or between 0 and 250 inclusive when the Extended Action Groups mod is installed.

Return type
:
bool

Game Scenes
:
Flight

set_action_group(group, state)¶
Sets the state of the given action group.

Parameters
:
group (int) –

A number between 0 and 9 inclusive, or between 0 and 250 inclusive when the Extended Action Groups mod is installed.

state (bool) –

Game Scenes
:
Flight

toggle_action_group(group)¶
Toggles the state of the given action group.

Parameters
:
group (int) –

A number between 0 and 9 inclusive, or between 0 and 250 inclusive when the Extended Action Groups mod is installed.

Game Scenes
:
Flight

add_node(ut[, prograde = 0.0][, normal = 0.0][, radial = 0.0])¶
Creates a maneuver node at the given universal time, and returns a Node object that can be used to modify it. Optionally sets the magnitude of the delta-v for the maneuver node in the prograde, normal and radial directions.

Parameters
:
ut (float) – Universal time of the maneuver node.

prograde (float) – Delta-v in the prograde direction.

normal (float) – Delta-v in the normal direction.

radial (float) – Delta-v in the radial direction.

Return type
:
Node

Game Scenes
:
Flight

nodes¶
Returns a list of all existing maneuver nodes, ordered by time from first to last.

Attribute
:
Read-only, cannot be set

Return type
:
list(Node)

Game Scenes
:
Flight

remove_nodes()¶
Remove all maneuver nodes.

Game Scenes
:
Flight

class ControlState¶
The control state of a vessel. See Control.state.

full¶
Full controllable.

partial¶
Partially controllable.

none¶
Not controllable.

class ControlSource¶
The control source of a vessel. See Control.source.

kerbal¶
Vessel is controlled by a Kerbal.

probe¶
Vessel is controlled by a probe core.

none¶
Vessel is not controlled.

class SASMode¶
The behavior of the SAS auto-pilot. See AutoPilot.sas_mode.

stability_assist¶
Stability assist mode. Dampen out any rotation.

maneuver¶
Point in the burn direction of the next maneuver node.

prograde¶
Point in the prograde direction.

retrograde¶
Point in the retrograde direction.

normal¶
Point in the orbit normal direction.

anti_normal¶
Point in the orbit anti-normal direction.

radial¶
Point in the orbit radial direction.

anti_radial¶
Point in the orbit anti-radial direction.

target¶
Point in the direction of the current target.

anti_target¶
Point away from the current target.

class SpeedMode¶
The mode of the speed reported in the navball. See Control.speed_mode.

orbit¶
Speed is relative to the vessel’s orbit.

surface¶
Speed is relative to the surface of the body being orbited.

target¶
Speed is relative to the current target.

class ControlInputMode¶
See Control.input_mode.

additive¶
Control inputs are added to the vessels current control inputs.

override¶
Control inputs (when they are non-zero) override the vessels current control inputs.
