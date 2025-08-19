class_name SteeringController
extends Node

#region Configuration
@export_group("Input Rates")
## How quickly the steering moves towards the player's input.
@export var steer_rate: float = 5.0
## How quickly the steering returns to center when there is no input.
@export var center_rate: float = 12.0

@export_group("Grip-Aware Steering")
## Enable grip-aware steering limitation
@export var grip_aware_enabled: bool = true
## Minimum speed threshold for grip limiting (m/s). Below this, full steering is allowed.
@export var min_speed_threshold: float = 3.0
## Speed range over which grip limiting fades in
@export var grip_limit_fade_range: float = 2.0
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
## Smoothing rate for grip limit transitions (higher = smoother)
@export var grip_limit_smoothing_rate: float = 8.0

@export_group("Steering Assistance")
## Enable automatic counter-steering assistance.
@export var steering_assist_enabled: bool = true
## The amount of assistance to apply (0 = none, 1 = full).
@export var steering_assist_level: float = 0.5:
	set(value):
		steering_assist_level = clamp(value, 0.0, 1.0)
## Slip angle (in degrees) required before the assist becomes active.
@export var assist_slip_threshold: float = 1.0
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
var slip_angle_trend: float = 0.0
var current_steering_limit: float = 1.0
#endregion

# Vehicle reference (set by vehicle controller)
var vehicle_controller: VehicleController

#region Public API
func initialize(p_vehicle_controller: VehicleController):
	vehicle_controller = p_vehicle_controller


func process_inputs(steer_input: float, delta: float):
	raw_steer_input = steer_input
	var prev_slip = smoothed_front_slip_angle
	previous_steer_value = steer_value

	if grip_aware_enabled and vehicle_controller:
		# This updates slip angles AND smoothly updates the steering limit
		_update_grip_and_limits(delta)

		# Calculate slip angle trend (needs to happen after grip update)
		slip_angle_trend = (smoothed_front_slip_angle - prev_slip) / delta if delta > 0 else 0.0

		var final_input = steer_input
		if steering_assist_enabled and steering_assist_level > 0.0:
			var assist_target = _calculate_steering_assist()
			final_input = lerp(steer_input, assist_target, steering_assist_level)

		grip_limited_steer_input = _apply_grip_aware_limiting(final_input, delta)
		_process_steering_movement(grip_limited_steer_input, delta)
	else:
		current_steering_limit = 1.0 # Ensure limit is reset when disabled
		_process_steering_movement(steer_input, delta)

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
		"vehicle_speed": vehicle_controller.get_vehicle_speed() if vehicle_controller else 0.0,
		"assist_target": _calculate_steering_assist() if steering_assist_enabled else 0.0
	}

#region Private API
func _update_grip_and_limits(delta: float) -> void:
	# Perform the slip angle analysis.
	_update_grip_analysis()
	# Calculate the TARGET steering limit for this frame.
	var target_steering_limit = _calculate_target_grip_limit()
	# Smoothly move our actual steering limit towards the target. This is the key fix.
	current_steering_limit = move_toward(current_steering_limit, target_steering_limit, grip_limit_smoothing_rate * delta)

func _apply_grip_aware_limiting(raw_input: float, delta: float) -> float:
	var vehicle_speed = vehicle_controller.get_vehicle_speed()
	is_counter_steering = _detect_counter_steering(raw_input)

	# Calculate the input limited by grip and rate
	var grip_limited_input: float
	if is_counter_steering:
		grip_limited_input = clamp(raw_input * 1.5, -1.0, 1.0)
	else:
		var max_change = max_steer_change_rate * delta
		var rate_limited_input = clamp(raw_input, grip_limited_steer_input - max_change, grip_limited_steer_input + max_change)
		# Apply the SMOOTHED grip limit
		grip_limited_input = clamp(rate_limited_input, -current_steering_limit, current_steering_limit)

	# Apply the smoothed speed-based fade-in using smoothstep
	var start_speed = min_speed_threshold
	var end_speed = min_speed_threshold + grip_limit_fade_range
	var speed_transition_factor = 0.0
	if end_speed > start_speed:
		var raw_factor = (vehicle_speed - start_speed) / (end_speed - start_speed)
		speed_transition_factor = smoothstep(0.0, 1.0, raw_factor)

	# Interpolate between raw input and the fully processed (grip + rate limited) input
	return lerp(raw_input, grip_limited_input, speed_transition_factor)

func _calculate_target_grip_limit() -> float:
	if grip_utilization <= grip_limit_factor:
		return 1.0

	var over_limit_ratio = (grip_utilization - grip_limit_factor) / (1.0 - grip_limit_factor)
	var steering_reduction = smoothstep(0.0, 1.0, over_limit_ratio)
	var min_steering = 0.15
	var available_steering = lerp(1.0, min_steering, steering_reduction * 0.8)

	if slip_angle_trend < -0.1:
		available_steering = min(1.0, available_steering * 1.3)
	return available_steering

func _process_steering_movement(target_input: float, delta: float):
	var rate_to_use: float
	if target_input != 0.0:
		# This remaps its range to provide a gentle reduction factor for the steer rate.
		var grip_rate_factor = lerp(0.7, 1.0, current_steering_limit)
		rate_to_use = steer_rate * grip_rate_factor
		steer_value = move_toward(steer_value, target_input, rate_to_use * delta)
	else:
		rate_to_use = center_rate
		steer_value = move_toward(steer_value, 0.0, rate_to_use * delta)

	steer_value = clamp(steer_value, -1.0, 1.0)

func _calculate_steering_assist() -> float:
	var vehicle_speed = vehicle_controller.get_vehicle_speed()

	# Only activate above a minimum speed and slip angle to avoid twitching
	if vehicle_speed < min_speed_threshold or abs(smoothed_front_slip_angle) < assist_slip_threshold:
		return 0.0 # No assistance needed

	# Don't interfere if the player is intentionally steering into the slide
	var slip_direction = sign(smoothed_front_slip_angle)
	var input_direction = sign(raw_steer_input)
	if input_direction != 0 and input_direction == slip_direction:
		# Player is likely inducing or holding a drift, so the assist backs off.
		return raw_steer_input # Return the player's own input to avoid a sudden change

	# Calculate the ideal counter-steer input.
	var assist_target = - smoothed_front_slip_angle / max_slip_angle_deg

	# Return the clamped correction value
	return clamp(assist_target, -1.0, 1.0)

func _update_grip_analysis():
	# Get front axle
	if vehicle_controller.axles.size() == 0:
		return

	var front_axle = vehicle_controller.axles[0]

	# Check if wheels have contact
	if not front_axle.left_wheel.has_contact and not front_axle.right_wheel.has_contact:
		current_front_slip_angle = 0.0
		smoothed_front_slip_angle = 0.0
		grip_utilization = 0.0
		return

	# Average the SIGNED slip angles first to preserve direction
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

	# Now, calculate grip utilization using the MAGNITUDE (abs) of the slip
	grip_utilization = abs(smoothed_front_slip_angle) / max_slip_angle_deg
	# Apply a curve to make the transition more gradual
	grip_utilization = grip_utilization * grip_utilization

func _detect_counter_steering(current_input: float) -> bool:
	# Need minimum slip angle and input to detect counter-steering
	# Use the magnitude of the slip angle for the threshold check
	if abs(smoothed_front_slip_angle) < 2.0 or abs(current_input) < counter_steer_sensitivity:
		return false

	# A positive slip angle means the front is sliding right.
	# A positive input means steering right.
	var slip_direction = sign(smoothed_front_slip_angle)
	var input_direction = sign(current_input)

	var is_correcting = (slip_direction != 0 and slip_direction != input_direction)

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
