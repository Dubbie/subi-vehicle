class_name SteeringController
extends Node

#region Configuration
@export_group("Input Rates")
## How quickly the steering moves towards the player's input.
@export var steer_rate: float = 3.0
## How quickly the steering returns to center when there is no input.
@export var center_rate: float = 6.0

@export_group("Grip-Aware Steering")
## Enable grip-aware steering limitation
@export var grip_aware_enabled: bool = true
## Minimum speed threshold for grip limiting (m/s). Below this, full steering is allowed.
@export var min_speed_threshold: float = 3.0
## Maximum allowed slip angle before grip limiting kicks in (degrees)
@export var max_slip_angle_deg: float = 12.0
## Counter-steering detection sensitivity. Higher values make counter-steering easier to detect.
@export var counter_steer_sensitivity: float = 0.2
## How aggressively to limit steering when approaching grip limit (0-1)
@export var grip_limit_factor: float = 0.6
## Smoothing factor for grip calculations (0-1). Higher = more responsive but less stable.
@export var grip_smoothing: float = 0.25
## Rate limiter for steering changes (per second)
@export var max_steer_change_rate: float = 4.0
#endregion

#region State Variables
# Internal steering state
var steer_value: float = 0.0
var previous_steer_input: float = 0.0
var previous_steer_value: float = 0.0
var grip_limited_steer_input: float = 0.0
var raw_steer_input: float = 0.0

# Grip analysis state
var current_front_slip_angle: float = 0.0
var smoothed_front_slip_angle: float = 0.0
var is_counter_steering: bool = false
var grip_utilization: float = 0.0
var slip_angle_trend: float = 0.0 #
#endregion

# Vehicle reference (set by vehicle controller)
var vehicle_controller: VehicleController

#region Public API
func initialize(p_vehicle_controller: VehicleController):
	vehicle_controller = p_vehicle_controller

func process_inputs(steer_input: float, delta: float):
	# Store raw input
	raw_steer_input = steer_input

	# Store previous values for trend analysis
	var prev_slip = smoothed_front_slip_angle
	previous_steer_value = steer_value

	if grip_aware_enabled and vehicle_controller:
		# Update grip analysis first
		_update_grip_analysis()

		# Calculate slip angle trend
		slip_angle_trend = (smoothed_front_slip_angle - prev_slip) / delta if delta > 0 else 0.0

		# Apply grip-aware steering limitation
		grip_limited_steer_input = _apply_grip_aware_limiting(steer_input, delta)
		_process_steering_movement(grip_limited_steer_input, delta)
	else:
		# Standard steering without grip limiting
		_process_steering_movement(steer_input, delta)

	# Store for next frame
	previous_steer_input = steer_input

func get_steer_value() -> float:
	return steer_value

func get_grip_utilization() -> float:
	return grip_utilization

func get_front_slip_angle() -> float:
	return smoothed_front_slip_angle

func is_grip_limiting_active() -> bool:
	return grip_aware_enabled and grip_utilization > grip_limit_factor and vehicle_controller.get_vehicle_speed() > min_speed_threshold

func is_currently_counter_steering() -> bool:
	return is_counter_steering

func get_steering_limit() -> float:
	if not grip_aware_enabled:
		return 1.0
	return _calculate_grip_limit()

func get_debug_info() -> Dictionary:
	return {
		"steer_value": steer_value,
		"raw_input": raw_steer_input,
		"grip_limited_input": grip_limited_steer_input,
		"front_slip_angle": smoothed_front_slip_angle,
		"grip_utilization": grip_utilization,
		"is_counter_steering": is_counter_steering,
		"grip_limiting_active": is_grip_limiting_active(),
		"steering_limit": get_steering_limit(),
		"slip_angle_trend": slip_angle_trend,
		"vehicle_speed": vehicle_controller.get_vehicle_speed() if vehicle_controller else 0.0
	}

#region Private API
func _apply_grip_aware_limiting(raw_input: float, delta: float) -> float:
	var vehicle_speed = vehicle_controller.get_vehicle_speed()

	# Don't limit at low speeds or when stationary
	if vehicle_speed < min_speed_threshold:
		return raw_input

	# Detect counter-steering first
	is_counter_steering = _detect_counter_steering(raw_input)

	# If counter-steering, allow more aggressive input but still some limiting
	if is_counter_steering:
		var counter_steer_multiplier = 1.2 # Allow 20% more steering when counter-steering
		return clamp(raw_input * counter_steer_multiplier, -1.0, 1.0)

	# Calculate grip-based steering limit
	var grip_limit = _calculate_grip_limit()

	# Apply rate limiting to prevent sudden steering changes
	var max_change = max_steer_change_rate * delta
	var rate_limited_input = clamp(raw_input,
		grip_limited_steer_input - max_change,
		grip_limited_steer_input + max_change)

	# Apply the grip limit
	return clamp(rate_limited_input, -grip_limit, grip_limit)

func _update_grip_analysis():
	# Get front axle (assuming index 0 is front)
	if vehicle_controller.axles.size() == 0:
		return

	var front_axle = vehicle_controller.axles[0]

	# Check if wheels have contact
	if not front_axle.left_wheel.has_contact and not front_axle.right_wheel.has_contact:
		current_front_slip_angle = 0.0
		smoothed_front_slip_angle = 0.0
		grip_utilization = 0.0
		return

	# --- FIX: Average the SIGNED slip angles first to preserve direction ---
	var total_slip_rad = 0.0
	var wheel_count = 0

	if front_axle.left_wheel.has_contact:
		total_slip_rad += front_axle.left_wheel.lat_slip
		wheel_count += 1

	if front_axle.right_wheel.has_contact:
		total_slip_rad += front_axle.right_wheel.lat_slip
		wheel_count += 1

	# Calculate the average signed slip angle
	var average_slip_rad = 0.0
	if wheel_count > 0:
		average_slip_rad = total_slip_rad / wheel_count

	# Store the smoothed, signed slip angle in degrees for use in detection logic
	current_front_slip_angle = rad_to_deg(average_slip_rad)
	smoothed_front_slip_angle = lerp(smoothed_front_slip_angle, current_front_slip_angle, grip_smoothing)

	# --- Now, calculate grip utilization using the MAGNITUDE (abs) of the slip ---
	grip_utilization = abs(smoothed_front_slip_angle) / max_slip_angle_deg
	# Apply a curve to make the transition more gradual
	grip_utilization = grip_utilization * grip_utilization

func _detect_counter_steering(current_input: float) -> bool:
	# Need minimum slip angle and input to detect counter-steering
	# Use the magnitude of the slip angle for the threshold check
	if abs(smoothed_front_slip_angle) < 2.0 or abs(current_input) < counter_steer_sensitivity:
		return false

	# --- FIX: Use the smoothed, signed slip angle calculated in the analysis step ---
	# A positive slip angle means the front is sliding right.
	# A positive input means steering right.
	var slip_direction = sign(smoothed_front_slip_angle)
	var input_direction = sign(current_input)

	# --- FIX: Counter-steering is when you steer IN THE SAME DIRECTION as the slide ---
	var is_correcting = (slip_direction != 0 and slip_direction == input_direction)

	return is_correcting

func _calculate_grip_limit() -> float:
	# More progressive grip limiting
	if grip_utilization <= grip_limit_factor:
		return 1.0 # Full steering available

	# Calculate how much we're over the grip limit threshold
	var over_limit_ratio = (grip_utilization - grip_limit_factor) / (1.0 - grip_limit_factor)

	# Use a smoother curve for steering reduction
	var steering_reduction = smoothstep(0.0, 1.0, over_limit_ratio)

	# Never go below 15% steering, and make the reduction more gradual
	var min_steering = 0.15
	var available_steering = lerp(1.0, min_steering, steering_reduction * 0.8)

	# If slip angle is decreasing, allow more steering
	if slip_angle_trend < -0.1:
		available_steering = min(1.0, available_steering * 1.3)

	return available_steering

func _process_steering_movement(target_input: float, delta: float):
	# Choose appropriate rate based on whether we're moving toward input or centering
	var rate_to_use: float

	if target_input != 0.0:
		# When grip limiting is active, use slower rates to prevent oscillation
		var grip_factor = 1.0
		if is_grip_limiting_active():
			grip_factor = 0.7 # Reduce rate by 30% when grip limiting

		rate_to_use = steer_rate * grip_factor
		steer_value = move_toward(steer_value, target_input, rate_to_use * delta)
	else:
		rate_to_use = center_rate
		steer_value = move_toward(steer_value, 0.0, rate_to_use * delta)

	# Ensure we stay within bounds
	steer_value = clamp(steer_value, -1.0, 1.0)