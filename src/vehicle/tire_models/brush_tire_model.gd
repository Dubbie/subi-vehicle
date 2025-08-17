class_name RealisticBrushTireModel
extends BaseTireModel

# Core friction parameters
## Peak longitudinal friction coefficient
@export var peak_longitudinal_mu: float = 1.1
## Peak lateral friction coefficient
@export var peak_lateral_mu: float = 1.0
## Sliding longitudinal friction
@export var sliding_longitudinal_mu: float = 0.8
## Sliding lateral friction
@export var sliding_lateral_mu: float = 0.7

# Slip characteristics - where the tire peaks and how sharp the curves are
## ~12% slip for peak longitudinal force
@export var peak_lon_slip_ratio: float = 0.12
## 6 degrees for peak lateral force
@export var peak_lat_slip_angle_deg: float = 6.0
## How sharp the longitudinal curve is (1.0-3.0)
@export var lon_curve_sharpness: float = 1.5
## How sharp the lateral curve is (1.0-3.0)
@export var lat_curve_sharpness: float = 2.0

# Load sensitivity - realistic tire behavior with load
## Load where tire performs best (N)
@export var optimal_load: float = 4000.0
## How much friction changes with load (0.0-1.0)
@export var load_sensitivity: float = 0.3

# Combined slip behavior
## Shape of friction circle (1.0 = perfect circle)
@export var friction_circle_shape: float = 1.1
## How sharply lateral grip falls off with longitudinal slip.
## 1.0 = linear (aggressive), 0.5 = curved (more controllable drifts)
@export var combined_slip_falloff: float = 0.3

# Transient behavior - how quickly tire responds to slip changes
## Distance for tire to respond to slip change (m)
@export var relaxation_length: float = 0.15
## Minimum speed for relaxation calculation (m/s)
@export var min_relaxation_speed: float = 1.0

# Internal state for transient behavior
var target_lon_force: float = 0.0
var target_lat_force: float = 0.0
var current_lon_force: float = 0.0
var current_lat_force: float = 0.0

func calculate_forces(params: TireParams) -> Vector2:
	var vertical_load = params.vertical_load
	var lat_slip_angle_rad = params.lat_slip_angle_rad
	var lon_slip_ratio = params.lon_slip_ratio
	var surface_friction = params.surface_friction
	var longitudinal_velocity = params.longitudinal_velocity
	var wheel_angular_velocity = params.wheel_angular_velocity
	var wheel_radius = params.wheel_radius
	var delta = params.delta

	# Safety checks
	if vertical_load <= 0.0 or delta <= 0.0:
		_reset_forces()
		return Vector2.ZERO

	# Calculate load factor - realistic tire load sensitivity
	var load_factor = _calculate_realistic_load_factor(vertical_load)

	# Calculate effective friction coefficients
	var effective_peak_lon_mu = peak_longitudinal_mu * load_factor * surface_friction
	var effective_peak_lat_mu = peak_lateral_mu * load_factor * surface_friction
	var effective_sliding_lon_mu = sliding_longitudinal_mu * load_factor * surface_friction
	var effective_sliding_lat_mu = sliding_lateral_mu * load_factor * surface_friction

	# Calculate target forces using Pacejka-style curves
	target_lon_force = _calculate_longitudinal_force(
		lon_slip_ratio, vertical_load,
		effective_peak_lon_mu, effective_sliding_lon_mu
	)

	target_lat_force = _calculate_lateral_force(
		lat_slip_angle_rad, lon_slip_ratio, vertical_load,
		effective_peak_lat_mu, effective_sliding_lat_mu
	)

	# Apply friction circle limit
	# First, calculate the ACTUAL maximum grip available at this load
	var peak_forces = get_peak_forces_at_load(vertical_load, surface_friction)
	var max_available_grip = peak_forces.peak_longitudinal_force # Use longitudinal as the main reference

	# Now apply the limit using the CORRECT maximum grip
	var limited_forces = _apply_friction_circle(target_lon_force, target_lat_force, max_available_grip)
	target_lon_force = limited_forces.y
	target_lat_force = limited_forces.x

	# Apply relaxation (transient behavior)
	_apply_relaxation(longitudinal_velocity, wheel_angular_velocity, wheel_radius, delta)

	# Return Vector2(lateral_force, longitudinal_force)
	return Vector2(current_lat_force, current_lon_force)

## Realistic tire load sensitivity:
## - Friction increases with load up to optimal point
## - Then decreases due to tire saturation
## - This matches real tire behavior
func _calculate_realistic_load_factor(vertical_load: float) -> float:
	if optimal_load <= 0.0:
		return 1.0

	var load_ratio = vertical_load / optimal_load

	# Parabolic curve with peak at optimal load
	# At optimal load: factor = 1.0
	# Below optimal: slightly lower friction
	# Above optimal: friction drops off more significantly
	var base_factor: float

	if load_ratio <= 1.0:
		# Below optimal load - slight increase up to optimal
		base_factor = 0.85 + 0.15 * load_ratio
	else:
		# Above optimal load - diminishing returns then decrease
		# This creates realistic tire saturation
		var excess = load_ratio - 1.0
		base_factor = 1.0 - (excess * excess * 0.2)

	# Apply load sensitivity scaling
	var final_factor = lerp(1.0, base_factor, load_sensitivity)

	# Clamp to reasonable bounds
	return clamp(final_factor, 0.4, 1.3)

## Pacejka-style longitudinal force curve
func _calculate_longitudinal_force(slip_ratio: float, p_load: float, peak_mu: float, sliding_mu: float) -> float:
	var abs_slip = abs(slip_ratio)
	var sign_slip = sign(slip_ratio)

	if abs_slip < 0.001: # Very small slip - linear region
		var stiffness = p_load * peak_mu / peak_lon_slip_ratio
		return slip_ratio * stiffness

	# Normalized slip (1.0 = peak slip)
	var normalized_slip = abs_slip / peak_lon_slip_ratio

	var force_coefficient: float

	if normalized_slip <= 1.0:
		# Build-up to peak - smooth S-curve
		var x = normalized_slip
		force_coefficient = peak_mu * x * (2.0 - x * (1.0 - 0.3 / lon_curve_sharpness))
	else:
		# Post-peak sliding region
		var excess_slip = normalized_slip - 1.0
		var slide_factor = exp(-lon_curve_sharpness * excess_slip)
		force_coefficient = sliding_mu + (peak_mu - sliding_mu) * slide_factor

	return sign_slip * p_load * force_coefficient

## Pacejka-style lateral force curve
func _calculate_lateral_force(slip_angle_rad: float, lon_slip_ratio: float, p_load: float, peak_mu: float, sliding_mu: float) -> float:
	# Combined Slip Reduction Factor:
	# Reduce lateral force capability based on how much longitudinal slip is happening.
	var normalized_lon_slip = abs(lon_slip_ratio / peak_lon_slip_ratio)

	# More realistic combined slip model that allows drifting
	# Uses a curved falloff instead of linear, and never goes to zero
	var combined_slip_factor: float
	if normalized_lon_slip < 1.0:
		# Before peak longitudinal slip - minimal lateral reduction
		combined_slip_factor = 1.0 - (normalized_lon_slip * normalized_lon_slip * 0.1)
	else:
		# After peak longitudinal slip - gradual reduction but never to zero
		var excess_slip = normalized_lon_slip - 1.0
		# Exponential decay that asymptotes to 0.3 (30% lateral grip remains)
		combined_slip_factor = 0.3 + 0.7 * exp(-excess_slip * combined_slip_falloff)

	# Ensure we never completely lose lateral grip
	combined_slip_factor = max(combined_slip_factor, 0.25)

	var abs_angle = abs(slip_angle_rad)
	var sign_angle = sign(slip_angle_rad)

	if abs_angle < 0.001:
		var stiffness = p_load * peak_mu / deg_to_rad(peak_lat_slip_angle_deg)
		return slip_angle_rad * stiffness * combined_slip_factor

	var normalized_angle = abs_angle / deg_to_rad(peak_lat_slip_angle_deg)
	var force_coefficient: float

	if normalized_angle <= 1.0:
		var x = normalized_angle
		force_coefficient = peak_mu * sin(x * PI / 2.0) * (1.0 + 0.2 * (1.0 - x))
	else:
		var excess_angle = normalized_angle - 1.0
		var slide_factor = exp(-lat_curve_sharpness * excess_angle)
		force_coefficient = sliding_mu + (peak_mu - sliding_mu) * slide_factor

	# --- AND MODIFY THIS FINAL LINE ---
	return sign_angle * p_load * force_coefficient * combined_slip_factor

## Apply friction circle/ellipse limitation
func _apply_friction_circle(force_lon: float, force_lat: float, max_friction_force: float) -> Vector2:
	if max_friction_force <= 0.0:
		return Vector2(force_lat, force_lon)

	# Use the standard, unweighted magnitude to check if we're outside the circle
	var force_magnitude = sqrt(force_lon * force_lon + force_lat * force_lat)

	if force_magnitude > max_friction_force:
		# If we are outside, scale the forces back to the edge of the circle
		var scale_factor = max_friction_force / force_magnitude
		force_lon *= scale_factor
		force_lat *= scale_factor

	return Vector2(force_lat, force_lon)

## Apply tire relaxation - how quickly the tire responds to slip changes.
## Simulates the physical delay in tire deformation
func _apply_relaxation(longitudinal_velocity: float, wheel_angular_velocity: float, wheel_radius: float, delta: float) -> void:
	# Calculate effective speed for relaxation
	var wheel_speed = abs(wheel_angular_velocity * wheel_radius)
	var vehicle_speed = abs(longitudinal_velocity)
	var effective_speed = max(wheel_speed, vehicle_speed, min_relaxation_speed)

	# Relaxation rate based on how fast we're moving through the contact patch
	var relaxation_rate = effective_speed / relaxation_length

	# Time constant for exponential approach to target
	var tau = 1.0 / max(relaxation_rate, 0.1) # Prevent division by zero
	var alpha = 1.0 - exp(-delta / tau)

	# Smooth approach to target forces
	current_lon_force = lerp(current_lon_force, target_lon_force, alpha)
	current_lat_force = lerp(current_lat_force, target_lat_force, alpha)

func _reset_forces() -> void:
	target_lon_force = 0.0
	target_lat_force = 0.0
	current_lon_force = 0.0
	current_lat_force = 0.0

# Utility functions for tuning and debugging

## Calculate what the peak forces should be at a given load - Useful for tuning and understanding tire behavior
func get_peak_forces_at_load(p_load: float, surface_mu: float = 1.0) -> Dictionary:
	var load_factor = _calculate_realistic_load_factor(p_load)

	return {
		"peak_longitudinal_force": p_load * peak_longitudinal_mu * load_factor * surface_mu,
		"peak_lateral_force": p_load * peak_lateral_mu * load_factor * surface_mu,
		"load_factor": load_factor
	}

## Calculate forces at specific slip values - useful for debugging
# func get_force_at_slip(slip_ratio: float, slip_angle_deg: float, p_load: float, surface_mu: float = 1.0) -> Dictionary:
# 	var load_factor = _calculate_realistic_load_factor(p_load)
# 	var effective_peak_lon_mu = peak_longitudinal_mu * load_factor * surface_mu
# 	var effective_sliding_lon_mu = sliding_longitudinal_mu * load_factor * surface_mu
# 	var effective_peak_lat_mu = peak_lateral_mu * load_factor * surface_mu
# 	var effective_sliding_lat_mu = sliding_lateral_mu * load_factor * surface_mu

# 	var lon_force = _calculate_longitudinal_force(slip_ratio, p_load, effective_peak_lon_mu, effective_sliding_lon_mu)
# 	var lat_force = _calculate_lateral_force(deg_to_rad(slip_angle_deg), , p_load, effective_peak_lat_mu, effective_sliding_lat_mu)

# 	var limited = _apply_friction_circle(lon_force, lat_force, p_load * surface_mu)

# 	return {
# 		"longitudinal_force": limited.y,
# 		"lateral_force": limited.x,
# 		"raw_longitudinal_force": lon_force,
# 		"raw_lateral_force": lat_force,
# 		"combined_limited": (limited.y != lon_force) or (limited.x != lat_force)
# 	}

# Generate data for plotting slip curves - useful for tuning
# func generate_slip_curve_data(p_load: float = 4000.0, surface_mu: float = 1.0) -> Dictionary:
# 	var lon_curve = []
# 	var lat_curve = []

# 	# Longitudinal curve
# 	for i in range(201):
# 		var slip = (i - 100) * 0.005 # -0.5 to +0.5 slip ratio
# 		var result = get_force_at_slip(slip, 0.0, p_load, surface_mu)
# 		lon_curve.append(Vector2(slip, result.longitudinal_force))

# 	# Lateral curve
# 	for i in range(181):
# 		var angle = (i - 90) * 0.5 # -45 to +45 degrees
# 		var result = get_force_at_slip(0.0, angle, p_load, surface_mu)
# 		lat_curve.append(Vector2(angle, result.lateral_force))

# 	return {
# 		"longitudinal_curve": lon_curve,
# 		"lateral_curve": lat_curve,
# 		"peak_info": get_peak_forces_at_load(p_load, surface_mu)
# 	}
