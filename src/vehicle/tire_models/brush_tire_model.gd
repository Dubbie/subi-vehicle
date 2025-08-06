class_name BrushTireModel
extends BaseTireModel

@export var coefficient_of_friction: float = 0.9
@export var longitudinal_stiffness_factor: float = 18.0
@export var lateral_stiffness_factor: float = 17.0
@export var relaxation_length: float = 0.3

# Bristle deflection state - stored per wheel instance
var longitudinal_deflection: float = 0.0
var lateral_deflection: float = 0.0

func calculate_forces(params: TireParams) -> Vector2:
	# Unpack parameters
	var vertical_load = params.vertical_load
	var lat_slip_rad = params.lat_slip_angle_rad
	var muk = params.surface_friction
	var v_lon = params.longitudinal_velocity
	var delta = params.delta
	var wheel_angular_velocity = params.wheel_angular_velocity # rad/s
	var wheel_radius = params.wheel_radius # meters

	# Early exit for invalid conditions
	if vertical_load <= 0.0 or delta <= 0.0:
		longitudinal_deflection = 0.0
		lateral_deflection = 0.0
		return Vector2.ZERO

	# Calculate wheel rim speed and slip velocities directly
	var wheel_rim_speed = wheel_angular_velocity * wheel_radius # m/s

	# This is the actual slip velocity - the velocity difference at the contact patch
	var slip_velocity_lon = wheel_rim_speed - v_lon

	# For lateral slip, the wheel moves at the wheel rim speed
	var slip_velocity_lat = abs(v_lon) * tan(lat_slip_rad)

	# Load-dependent stiffness (force per unit deflection)
	var effective_lon_stiffness = vertical_load * longitudinal_stiffness_factor
	var effective_lat_stiffness = vertical_load * lateral_stiffness_factor

	# Update bristle deflection using relaxation model
	var relaxation_factor = 0.0
	if relaxation_length > 0:
		# Relaxation is based on how fast the tire is moving through the contact patch
		# Use the maximum of wheel rim speed and vehicle speed
		var contact_speed = max(wheel_rim_speed, abs(v_lon))
		if contact_speed > 0.01: # Avoid division by very small numbers
			relaxation_factor = contact_speed / relaxation_length

	# Update deflections
	longitudinal_deflection += (slip_velocity_lon - relaxation_factor * longitudinal_deflection) * delta
	lateral_deflection += (slip_velocity_lat - relaxation_factor * lateral_deflection) * delta

	# Calculate forces from bristle deflection
	var force_lon = longitudinal_deflection * effective_lon_stiffness
	var force_lat = lateral_deflection * effective_lat_stiffness

	# Apply friction circle limitation
	var max_friction_force = vertical_load * coefficient_of_friction * muk
	var combined_force = sqrt(force_lon * force_lon + force_lat * force_lat)

	if combined_force > max_friction_force and combined_force > 0.0:
		var force_scale = max_friction_force / combined_force
		force_lon *= force_scale
		force_lat *= force_scale

		# Update deflections to match limited forces
		longitudinal_deflection = force_lon / effective_lon_stiffness
		lateral_deflection = force_lat / effective_lat_stiffness

	# Return forces: Vector2(lateral_force, longitudinal_force)
	return Vector2(force_lat, force_lon)
