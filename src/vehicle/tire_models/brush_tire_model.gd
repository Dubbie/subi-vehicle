class_name BrushTireModel
extends BaseTireModel

@export var coefficient_of_friction: float = 1.2
@export var lateral_stiffness_factor: float = 20.0
@export var longitudinal_stiffness_factor: float = 10.0
@export var relaxation_length: float = 0.3

# Bristle deflection state must be stored here!
# Each wheel will have its own instance of this model, with its own state.
var longitudinal_deflection: float = 0.0
var lateral_deflection: float = 0.0

func calculate_forces(params: TireParams) -> Vector2:
	# Unpack parameters for clarity
	var vertical_load = params.vertical_load
	var lon_slip = params.lon_slip_ratio
	var lat_slip_rad = params.lat_slip_angle_rad
	var muk = params.surface_friction
	var v_lon = params.longitudinal_velocity
	var delta = params.delta

	if vertical_load <= 0.0 or delta <= 0.0:
		longitudinal_deflection = 0.0
		lateral_deflection = 0.0
		return Vector2.ZERO

	# Calculate slip velocities
	var slip_velocity_lon = lon_slip * max(abs(v_lon), 0.1)
	var slip_velocity_lat = tan(lat_slip_rad) * abs(v_lon)

	# Load-dependent stiffness
	var effective_lat_stiffness = vertical_load * lateral_stiffness_factor
	var effective_lon_stiffness = vertical_load * longitudinal_stiffness_factor

	# Update bristle deflection
	var relaxation_factor = (abs(v_lon) / relaxation_length) if relaxation_length > 0 else 0
	longitudinal_deflection += (slip_velocity_lon - relaxation_factor * longitudinal_deflection) * delta
	lateral_deflection += (slip_velocity_lat - relaxation_factor * lateral_deflection) * delta

	# Calculate forces from bristle deflection
	var force_z = longitudinal_deflection * effective_lon_stiffness
	var force_x = lateral_deflection * effective_lat_stiffness

	# Combine forces and limit to friction circle
	var max_friction_force = vertical_load * (coefficient_of_friction * muk)
	var combined_force_sq = force_z * force_z + force_x * force_x

	if combined_force_sq > max_friction_force * max_friction_force:
		var force_scale = max_friction_force / sqrt(combined_force_sq)
		force_z *= force_scale
		force_x *= force_scale

		longitudinal_deflection = - force_z / effective_lon_stiffness
		lateral_deflection = - force_x / effective_lat_stiffness

	return Vector2(force_x, force_z)