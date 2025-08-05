class_name PacejkaTireModel
extends BaseTireModel

@export_group("General")
@export var coefficient_of_friction: float = 0.9
@export var force_smoothing_speed: float = 70.0

@export_group("Low Speed Damping")
@export var low_speed_damping_threshold: float = 1.0
@export var min_damping_factor: float = 0.01

@export_group("Lateral Force (Cornering)")
@export var lat_b: float = 10.0
@export var lat_c: float = 1.9
@export var lat_e: float = 0.97

@export_group("Longitudinal Force (Traction/Braking)")
@export var lon_b: float = 12.0
@export var lon_c: float = 1.7
@export var lon_e: float = 1.0

var last_applied_force: Vector2 = Vector2.ZERO

func _magic_formula(input_slip: float, b: float, c: float, d: float, e: float) -> float:
	if b == 0.0 or c == 0.0 or d == 0.0:
		return 0.0

	var slip_rad = b * input_slip
	var term_atan = atan(slip_rad)
	var result = d * sin(c * atan(slip_rad - e * (slip_rad - term_atan)))
	return result


func calculate_forces(params: TireParams) -> Vector2:
	# Unpack parameters
	var vertical_load = params.vertical_load
	var lon_slip_ratio = params.lon_slip_ratio
	var lat_slip_angle_rad = params.lat_slip_angle_rad
	var muk = params.surface_friction
	var v_lon = params.longitudinal_velocity
	var delta = params.delta

	if vertical_load <= 0.0 or delta <= 0.0:
		# When not in contact, reset the force memory
		last_applied_force = Vector2.ZERO
		return Vector2.ZERO

	# --- 1. Calculate the "Target" Force using Pacejka ---
	var peak_force_d = vertical_load * coefficient_of_friction * muk

	var lat_slip_deg = abs(rad_to_deg(lat_slip_angle_rad))
	var force_lat_raw = _magic_formula(lat_slip_deg, lat_b, lat_c, peak_force_d, lat_e)

	var lon_slip_scaled = abs(lon_slip_ratio * 100.0)
	var force_lon_raw = _magic_formula(lon_slip_scaled, lon_b, lon_c, peak_force_d, lon_e)

	force_lat_raw *= sign(lat_slip_angle_rad)
	force_lon_raw *= sign(lon_slip_ratio)

	var target_force = Vector2(force_lat_raw, force_lon_raw)

	if target_force.length_squared() > peak_force_d * peak_force_d:
		target_force = target_force.normalized() * peak_force_d

	# --- 2. Apply Low-Speed Damping to the Target Force ---
	if low_speed_damping_threshold > 0.0:
		var interpolation_value = clamp(abs(v_lon) / low_speed_damping_threshold, 0.0, 1.0)
		var damping_factor = lerp(min_damping_factor, 1.0, interpolation_value)
		target_force *= damping_factor

	# --- 3. FINAL STEP: Smooth the Force Output ---
	# Interpolate from the previously applied force towards the new target force.
	# This kills oscillations and simulates force buildup over time (relaxation).
	var final_force = last_applied_force.lerp(target_force, delta * force_smoothing_speed)

	# Update our state for the next frame
	last_applied_force = final_force

	return final_force
