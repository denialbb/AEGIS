Flight¶
class Flight¶
Used to get flight telemetry for a vessel, by calling Vessel.flight(). All of the information returned by this class is given in the reference frame passed to that method. Obtained by calling Vessel.flight().

Note

To get orbital information, such as the apoapsis or inclination, see Orbit.

g_force¶
The current G force acting on the vessel in
.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

mean_altitude¶
The altitude above sea level, in meters. Measured from the center of mass of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

surface_altitude¶
The altitude above the surface of the body or sea level, whichever is closer, in meters. Measured from the center of mass of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

bedrock_altitude¶
The altitude above the surface of the body, in meters. When over water, this is the altitude above the sea floor. Measured from the center of mass of the vessel.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

elevation¶
The elevation of the terrain under the vessel, in meters. This is the height of the terrain above sea level, and is negative when the vessel is over the sea.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

latitude¶
The latitude of the vessel for the body being orbited, in degrees.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

longitude¶
The longitude of the vessel for the body being orbited, in degrees.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

velocity¶
The velocity of the vessel, in the reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Returns
:
The velocity as a vector. The vector points in the direction of travel, and its magnitude is the speed of the vessel in meters per second.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

speed¶
The speed of the vessel in meters per second, in the reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

horizontal_speed¶
The horizontal speed of the vessel in meters per second, in the reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

vertical_speed¶
The vertical speed of the vessel in meters per second, in the reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

center_of_mass¶
The position of the center of mass of the vessel, in the reference frame ReferenceFrame

Attribute
:
Read-only, cannot be set

Returns
:
The position as a vector.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

rotation¶
The rotation of the vessel, in the reference frame ReferenceFrame

Attribute
:
Read-only, cannot be set

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

direction¶
The direction that the vessel is pointing in, in the reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Returns
:
The direction as a unit vector.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

pitch¶
The pitch of the vessel relative to the horizon, in degrees. A value between -90° and +90°.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

heading¶
The heading of the vessel (its angle relative to north), in degrees. A value between 0° and 360°.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

roll¶
The roll of the vessel relative to the horizon, in degrees. A value between -180° and +180°.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

prograde¶
The prograde direction of the vessels orbit, in the reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Returns
:
The direction as a unit vector.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

retrograde¶
The retrograde direction of the vessels orbit, in the reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Returns
:
The direction as a unit vector.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

normal¶
The direction normal to the vessels orbit, in the reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Returns
:
The direction as a unit vector.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

anti_normal¶
The direction opposite to the normal of the vessels orbit, in the reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Returns
:
The direction as a unit vector.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

radial¶
The radial direction of the vessels orbit, in the reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Returns
:
The direction as a unit vector.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

anti_radial¶
The direction opposite to the radial direction of the vessels orbit, in the reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Returns
:
The direction as a unit vector.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

atmosphere_density¶
The current density of the atmosphere around the vessel, in
.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

dynamic_pressure¶
The dynamic pressure acting on the vessel, in Pascals. This is a measure of the strength of the aerodynamic forces. It is equal to

. It is commonly denoted
.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

static_pressure¶
The static atmospheric pressure acting on the vessel, in Pascals.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

static_pressure_at_msl¶
The static atmospheric pressure at mean sea level, in Pascals.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

aerodynamic_force¶
The total aerodynamic forces acting on the vessel, in reference frame ReferenceFrame.

Attribute
:
Read-only, cannot be set

Returns
:
A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

simulate_aerodynamic_force_at(body, position, velocity)¶
Simulate and return the total aerodynamic forces acting on the vessel, if it where to be traveling with the given velocity at the given position in the atmosphere of the given celestial body.

Parameters
:
body (CelestialBody) –

position (tuple) –

velocity (tuple) –

Returns
:
A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

lift¶
The aerodynamic lift currently acting on the vessel.

Attribute
:
Read-only, cannot be set

Returns
:
A vector pointing in the direction that the force acts, with its magnitude equal to the strength of the force in Newtons.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

drag¶
The aerodynamic drag currently acting on the vessel.

Attribute
:
Read-only, cannot be set

Returns
:
A vector pointing in the direction of the force, with its magnitude equal to the strength of the force in Newtons.

Return type
:
tuple(float, float, float)

Game Scenes
:
Flight

speed_of_sound¶
The speed of sound, in the atmosphere around the vessel, in
.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

mach¶
The speed of the vessel, in multiples of the speed of sound.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

reynolds_number¶
The vessels Reynolds number.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

Note

Requires Ferram Aerospace Research.

true_air_speed¶
The true air speed of the vessel, in meters per second.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

equivalent_air_speed¶
The equivalent air speed of the vessel, in meters per second.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

terminal_velocity¶
An estimate of the current terminal velocity of the vessel, in meters per second. This is the speed at which the drag forces cancel out the force of gravity.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

angle_of_attack¶
The pitch angle between the orientation of the vessel and its velocity vector, in degrees.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

sideslip_angle¶
The yaw angle between the orientation of the vessel and its velocity vector, in degrees.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

total_air_temperature¶
The total air temperature of the atmosphere around the vessel, in Kelvin. This includes the Flight.static_air_temperature and the vessel’s kinetic energy.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

static_air_temperature¶
The static (ambient) temperature of the atmosphere around the vessel, in Kelvin.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

stall_fraction¶
The current amount of stall, between 0 and 1. A value greater than 0.005 indicates a minor stall and a value greater than 0.5 indicates a large-scale stall.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

Note

Requires Ferram Aerospace Research.

drag_coefficient¶
The coefficient of drag. This is the amount of drag produced by the vessel. It depends on air speed, air density and wing area.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

Note

Requires Ferram Aerospace Research.

lift_coefficient¶
The coefficient of lift. This is the amount of lift produced by the vessel, and depends on air speed, air density and wing area.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

Note

Requires Ferram Aerospace Research.

ballistic_coefficient¶
The ballistic coefficient.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

Note

Requires Ferram Aerospace Research.

thrust_specific_fuel_consumption¶
The thrust specific fuel consumption for the jet engines on the vessel. This is a measure of the efficiency of the engines, with a lower value indicating a more efficient vessel. This value is the number of Newtons of fuel that are burned, per hour, to produce one newton of thrust.

Attribute
:
Read-only, cannot be set

Return type
:
float

Game Scenes
:
Flight

Note

Requires Ferram Aerospace Research.
