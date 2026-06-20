Vessel¶
class Vessel¶
These objects are used to interact with vessels in KSP. This includes getting orbital and flight data, manipulating control inputs and managing resources. Created using active_vessel or vessels.

name¶
The name of the vessel.

Attribute
:
Can be read or written

Return type
:
str

type¶
The type of the vessel.

Attribute
:
Can be read or written

Return type
:
VesselType

situation¶
The situation the vessel is in.

Attribute
:
Read-only, cannot be set

Return type
:
VesselSituation

recoverable¶
Whether the vessel is recoverable.

Attribute
:
Read-only, cannot be set

Return type
:
bool

recover()¶
Recover the vessel.

met¶
The mission elapsed time in seconds.

Attribute
:
Read-only, cannot be set

Return type
:
float

biome¶
The name of the biome the vessel is currently in.

Attribute
:
Read-only, cannot be set

Return type
:
str

flight([reference_frame = None])¶
Returns a Flight object that can be used to get flight telemetry for the vessel, in the specified reference frame.

Parameters
:
reference_frame (ReferenceFrame) – Reference frame. Defaults to the vessel’s surface reference frame (Vessel.surface_reference_frame).

Return type
:
Flight

Game Scenes
:
Flight

Note

When this is called with no arguments, the vessel’s surface reference frame is used. This reference frame moves with the vessel, therefore velocities and speeds returned by the flight object will be zero. See the reference frames tutorial for examples of getting the orbital and surface speeds of a vessel.

orbit¶
The current orbit of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
Orbit

control¶
Returns a Control object that can be used to manipulate the vessel’s control inputs. For example, its pitch/yaw/roll controls, RCS and thrust.

Attribute
:
Read-only, cannot be set

Return type
:
Control

Game Scenes
:
Flight

comms¶
Returns a Comms object that can be used to interact with CommNet for this vessel.

Attribute
:
Read-only, cannot be set

Return type
:
Comms

Game Scenes
:
Flight

auto_pilot¶
An AutoPilot object, that can be used to perform simple auto-piloting of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
AutoPilot

Game Scenes
:
Flight

crew_capacity¶
The number of crew that can occupy the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
int

crew_count¶
The number of crew that are occupying the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
int

crew¶
The crew in the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
list(CrewMember)

resources¶
A Resources object, that can used to get information about resources stored in the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
Resources

Game Scenes
:
Flight

resources_in_decouple_stage(stage[, cumulative = True])¶
Returns a Resources object, that can used to get information about resources stored in a given stage.

Parameters
:
stage (int) – Get resources for parts that are decoupled in this stage.

cumulative (bool) – When False, returns the resources for parts decoupled in just the given stage. When True returns the resources decoupled in the given stage and all subsequent stages combined.

Return type
:
Resources

Game Scenes
:
Flight

Note

For details on stage numbering, see the discussion on Staging.

parts¶
A Parts object, that can used to interact with the parts that make up this vessel.

Attribute
:
Read-only, cannot be set

Return type
:
Parts

Game Scenes
:
Flight

mass¶
The total mass of the vessel, including resources, in kg.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

dry_mass¶
The total mass of the vessel, excluding resources, in kg.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

thrust¶
The total thrust currently being produced by the vessel’s engines, in Newtons. This is computed by summing Engine.thrust for every engine in the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

available_thrust¶
Gets the total available thrust that can be produced by the vessel’s active engines, in Newtons. This is computed by summing Engine.available_thrust for every active engine in the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

available_thrust_at(pressure)¶
Gets the total available thrust that can be produced by the vessel’s active engines, in Newtons. This is computed by summing Engine.available_thrust_at() for every active engine in the vessel. Takes the given pressure into account.

Parameters
:
pressure (float) – Atmospheric pressure in atmospheres

Return type
:
float

Game Scenes
:
Flight

max_thrust¶
The total maximum thrust that can be produced by the vessel’s active engines, in Newtons. This is computed by summing Engine.max_thrust for every active engine.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

max_thrust_at(pressure)¶
The total maximum thrust that can be produced by the vessel’s active engines, in Newtons. This is computed by summing Engine.max_thrust_at() for every active engine. Takes the given pressure into account.

Parameters
:
pressure (float) – Atmospheric pressure in atmospheres

Return type
:
float

Game Scenes
:
Flight

max_vacuum_thrust¶
The total maximum thrust that can be produced by the vessel’s active engines when the vessel is in a vacuum, in Newtons. This is computed by summing Engine.max_vacuum_thrust for every active engine.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

specific_impulse¶
The combined specific impulse of all active engines, in seconds. This is computed using the formula described here.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

specific_impulse_at(pressure)¶
The combined specific impulse of all active engines, in seconds. This is computed using the formula described here. Takes the given pressure into account.

Parameters
:
pressure (float) – Atmospheric pressure in atmospheres

Return type
:
float

Game Scenes
:
Flight

vacuum_specific_impulse¶
The combined vacuum specific impulse of all active engines, in seconds. This is computed using the formula described here.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

kerbin_sea_level_specific_impulse¶
The combined specific impulse of all active engines at sea level on Kerbin, in seconds. This is computed using the formula described here.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

moment_of_inertia¶
The moment of inertia of the vessel around its center of mass in
. The inertia values in the returned 3-tuple are around the pitch, roll and yaw directions respectively. This corresponds to the vessels reference frame (ReferenceFrame).

Attribute
:
Read-only, cannot be set

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

inertia_tensor¶
The inertia tensor of the vessel around its center of mass, in the vessels reference frame (ReferenceFrame). Returns the 3x3 matrix as a list of elements, in row-major order.

Attribute
:
Read-only, cannot be set

Return type
:
list(float)

available_torque¶
The maximum torque that the vessel generates. Includes contributions from reaction wheels, RCS, gimballed engines and aerodynamic control surfaces. Returns the torques in
around each of the coordinate axes of the vessels reference frame (ReferenceFrame). These axes are equivalent to the pitch, roll and yaw axes of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
tuple(tuple(float, float, float), tuple(float, float, float))

Game Scenes
:
Flight

available_reaction_wheel_torque¶
The maximum torque that the currently active and powered reaction wheels can generate. Returns the torques in
around each of the coordinate axes of the vessels reference frame (ReferenceFrame). These axes are equivalent to the pitch, roll and yaw axes of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
tuple(tuple(float, float, float), tuple(float, float, float))

Game Scenes
:
Flight

available_rcs_torque¶
The maximum torque that the currently active RCS thrusters can generate. Returns the torques in
around each of the coordinate axes of the vessels reference frame (ReferenceFrame). These axes are equivalent to the pitch, roll and yaw axes of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
tuple(tuple(float, float, float), tuple(float, float, float))

Game Scenes
:
Flight

available_rcs_force¶
The maximum force that the currently active RCS thrusters can generate. Returns the forces in
along each of the coordinate axes of the vessels reference frame (ReferenceFrame). These axes are equivalent to the right, forward and bottom directions of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
tuple(tuple(float, float, float), tuple(float, float, float))

Game Scenes
:
Flight

available_engine_torque¶
The maximum torque that the currently active and gimballed engines can generate. Returns the torques in
around each of the coordinate axes of the vessels reference frame (ReferenceFrame). These axes are equivalent to the pitch, roll and yaw axes of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
tuple(tuple(float, float, float), tuple(float, float, float))

Game Scenes
:
Flight

available_control_surface_torque¶
The maximum torque that the aerodynamic control surfaces can generate. Returns the torques in
around each of the coordinate axes of the vessels reference frame (ReferenceFrame). These axes are equivalent to the pitch, roll and yaw axes of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
tuple(tuple(float, float, float), tuple(float, float, float))

Game Scenes
:
Flight

available_other_torque¶
The maximum torque that parts (excluding reaction wheels, gimballed engines, RCS and control surfaces) can generate. Returns the torques in
around each of the coordinate axes of the vessels reference frame (ReferenceFrame). These axes are equivalent to the pitch, roll and yaw axes of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
tuple(tuple(float, float, float), tuple(float, float, float))

Game Scenes
:
Flight

reference_frame¶
The reference frame that is fixed relative to the vessel, and orientated with the vessel.

The origin is at the center of mass of the vessel.

The axes rotate with the vessel.

The x-axis points out to the right of the vessel.

The y-axis points in the forward direction of the vessel.

The z-axis points out of the bottom off the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
ReferenceFrame

Game Scenes
:
Flight

../../../\_images/vessel-aircraft.png
Vessel reference frame origin and axes for the Aeris 3A aircraft¶

../../../\_images/vessel-rocket.png
Vessel reference frame origin and axes for the Kerbal-X rocket¶

orbital_reference_frame¶
The reference frame that is fixed relative to the vessel, and orientated with the vessels orbital prograde/normal/radial directions.

The origin is at the center of mass of the vessel.

The axes rotate with the orbital prograde/normal/radial directions.

The x-axis points in the orbital anti-radial direction.

The y-axis points in the orbital prograde direction.

The z-axis points in the orbital normal direction.

Attribute
:
Read-only, cannot be set

Return type
:
ReferenceFrame

Game Scenes
:
Flight

Note

Be careful not to confuse this with ‘orbit’ mode on the navball.

../../../\_images/vessel-orbital.png
Vessel orbital reference frame origin and axes¶

surface_reference_frame¶
The reference frame that is fixed relative to the vessel, and orientated with the surface of the body being orbited.

The origin is at the center of mass of the vessel.

The axes rotate with the north and up directions on the surface of the body.

The x-axis points in the zenith direction (upwards, normal to the body being orbited, from the center of the body towards the center of mass of the vessel).

The y-axis points northwards towards the astronomical horizon (north, and tangential to the surface of the body – the direction in which a compass would point when on the surface).

The z-axis points eastwards towards the astronomical horizon (east, and tangential to the surface of the body – east on a compass when on the surface).

Attribute
:
Read-only, cannot be set

Return type
:
ReferenceFrame

Game Scenes
:
Flight

Note

Be careful not to confuse this with ‘surface’ mode on the navball.

../../../\_images/vessel-surface.png
Vessel surface reference frame origin and axes¶

surface_velocity_reference_frame¶
The reference frame that is fixed relative to the vessel, and orientated with the velocity vector of the vessel relative to the surface of the body being orbited.

The origin is at the center of mass of the vessel.

The axes rotate with the vessel’s velocity vector.

The y-axis points in the direction of the vessel’s velocity vector, relative to the surface of the body being orbited.

The z-axis is in the plane of the astronomical horizon.

The x-axis is orthogonal to the other two axes.

Attribute
:
Read-only, cannot be set

Return type
:
ReferenceFrame

Game Scenes
:
Flight

../../../\_images/vessel-surface-velocity.png
Vessel surface velocity reference frame origin and axes¶

position(reference_frame)¶
The position of the center of mass of the vessel, in the given reference frame.

Parameters
:
reference_frame (ReferenceFrame) – The reference frame that the returned position vector is in.

Returns
:
The position as a vector.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

bounding_box(reference_frame)¶
The axis-aligned bounding box of the vessel in the given reference frame.

Parameters
:
reference_frame (ReferenceFrame) – The reference frame that the returned position vectors are in.

Returns
:
The positions of the minimum and maximum vertices of the box, as position vectors.

Return type
:
tuple(tuple(float, float, float), tuple(float, float, float))

Game Scenes
:
Flight

velocity(reference_frame)¶
The velocity of the center of mass of the vessel, in the given reference frame.

Parameters
:
reference_frame (ReferenceFrame) – The reference frame that the returned velocity vector is in.

Returns
:
The velocity as a vector. The vector points in the direction of travel, and its magnitude is the speed of the body in meters per second.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

rotation(reference_frame)¶
The rotation of the vessel, in the given reference frame.

Parameters
:
reference_frame (ReferenceFrame) – The reference frame that the returned rotation is in.

Returns
:
The rotation as a quaternion of the form
.

Return type
:
tuple(float, float, float, float)

Game Scenes
:
Flight

direction(reference_frame)¶
The direction in which the vessel is pointing, in the given reference frame.

Parameters
:
reference_frame (ReferenceFrame) – The reference frame that the returned direction is in.

Returns
:
The direction as a unit vector.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

angular_velocity(reference_frame)¶
The angular velocity of the vessel, in the given reference frame.

Parameters
:
reference_frame (ReferenceFrame) – The reference frame the returned angular velocity is in.

Returns
:
The angular velocity as a vector. The magnitude of the vector is the rotational speed of the vessel, in radians per second. The direction of the vector indicates the axis of rotation, using the right-hand rule.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight
