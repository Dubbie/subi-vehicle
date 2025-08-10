class_name TransmissionController
extends Node

signal gear_changed(old_gear: int, new_gear: int)
signal shift_requested(target_gear: int)

# Transmission configuration
@export_group("Gear Ratios")
## Gear ratios are defined in the order they appear in the array.
## R, N, 1, 2, 3, 4, 5...
@export var gear_ratios: Array[float] = [-2.9, 0.0, 2.66, 1.78, 1.3, 0.9]

@export_group("Automatic Transmission")
@export var auto_shift_enabled: bool = true
## RPM at which to upshift
@export var upshift_rpm: float = 6000.0
## RPM at which to downshift
@export var downshift_rpm: float = 3000.0
## Minimum time between shifts (prevents hunting)
@export var shift_delay: float = 0.5
## Enable kickdown for aggressive downshifts
@export var kickdown_enabled: bool = true
@export var kickdown_throttle_threshold: float = 0.8

@export_group("Manual Override")
## Allow manual shifting even in auto mode
@export var manual_override_enabled: bool = true
## Time to return to auto mode after manual input
@export var manual_override_timeout: float = 3.0

@export_group("Shift Logic")
## Minimum speed to engage first gear (prevents stalling)
@export var first_gear_min_speed: float = 0.1
## Maximum speed for reverse gear
@export var reverse_max_speed: float = 5.0

# Internal state
var current_gear_index: int = 1 # Start in neutral
var target_gear_index: int = 1
var last_shift_time: float = 0.0
var manual_override_time: float = 0.0
var is_manual_override: bool = false

# Calculated shift speeds for each gear
var upshift_speeds: Array[float] = []
var downshift_speeds: Array[float] = []

# References (set by vehicle controller)
var wheel_radius: float = 0.3 # Default, should be set by vehicle
var differential_ratio: float = 3.9 # Default, should be set by vehicle

func initialize(wheel_r: float, diff_ratio: float):
	wheel_radius = wheel_r
	differential_ratio = diff_ratio
	_calculate_shift_speeds()

func _calculate_shift_speeds():
	upshift_speeds.clear()
	downshift_speeds.clear()

	print("--- Calculating Shift Speeds ---")
	print("Wheel radius: ", wheel_radius, " m, Differential ratio: ", differential_ratio)

	# Calculate speeds for each forward gear
	for i in range(2, gear_ratios.size()): # Skip reverse (0) and neutral (1)
		var gear_ratio = gear_ratios[i]
		if gear_ratio > 0.0:
			# Calculate vehicle speed at upshift RPM for this gear
			var wheel_rpm_at_upshift = upshift_rpm / (gear_ratio * differential_ratio)
			var wheel_angular_velocity = wheel_rpm_at_upshift * (2.0 * PI / 60.0)
			var upshift_speed = wheel_angular_velocity * wheel_radius * 0.85
			upshift_speeds.append(upshift_speed)

			# Calculate vehicle speed at downshift RPM for this gear
			var wheel_rpm_at_downshift = downshift_rpm / (gear_ratio * differential_ratio)
			wheel_angular_velocity = wheel_rpm_at_downshift * (2.0 * PI / 60.0)
			var downshift_speed = wheel_angular_velocity * wheel_radius * 0.85
			downshift_speeds.append(downshift_speed)
		else:
			upshift_speeds.append(0.0)
			downshift_speeds.append(0.0)

	print("--- Transmission Setup ---")
	print("Upshift speeds: ", upshift_speeds)
	print("Downshift speeds: ", downshift_speeds)
	print("------------------------")

func update_transmission(
	vehicle_speed: float,
	throttle_pedal: float,
	brake_pedal: float,
	is_grounded: bool,
	delta: float
) -> int:
	# Update timers
	last_shift_time += delta
	if is_manual_override:
		manual_override_time += delta
		if manual_override_time > manual_override_timeout:
			is_manual_override = false
			manual_override_time = 0.0

	# Process manual shift inputs if enabled
	if manual_override_enabled:
		_process_manual_inputs()

	# Automatic shifting logic (only if not in manual override)
	if auto_shift_enabled and not is_manual_override and is_grounded:
		_process_automatic_shifting(vehicle_speed, throttle_pedal, brake_pedal)

	return current_gear_index

func _process_manual_inputs():
	var shift_up = Input.is_action_just_pressed("shift_up")
	var shift_down = Input.is_action_just_pressed("shift_down")

	if shift_up or shift_down:
		is_manual_override = true
		manual_override_time = 0.0

		if shift_up:
			shift_to_gear(current_gear_index + 1)
		elif shift_down:
			shift_to_gear(current_gear_index - 1)

func _process_automatic_shifting(vehicle_speed: float, throttle_pedal: float, brake_pedal: float):
	# Don't shift too frequently
	if last_shift_time < shift_delay:
		return

	var current_forward_gear = current_gear_index - 2 # Convert to 0-based forward gear index

	# Upshift logic
	if current_gear_index >= 2 and current_gear_index < gear_ratios.size() - 1: # In forward gear, not top gear
		var upshift_speed = upshift_speeds[current_forward_gear] if current_forward_gear < upshift_speeds.size() else 999.0

		# Normal upshift based on speed
		if vehicle_speed > upshift_speed:
			shift_to_gear(current_gear_index + 1)
			return

	# Downshift logic
	if current_gear_index > 2: # In forward gear above 1st
		var downshift_speed = downshift_speeds[current_forward_gear] if current_forward_gear < downshift_speeds.size() else 0.0

		# Normal downshift based on speed
		if vehicle_speed < downshift_speed:
			shift_to_gear(current_gear_index - 1)
			return

		# Kickdown logic - aggressive downshift for more power
		if kickdown_enabled and throttle_pedal > kickdown_throttle_threshold:
			# Check if a lower gear would keep us under redline
			var target_gear_forward = current_forward_gear - 1
			if target_gear_forward >= 0:
				var target_gear_ratio = gear_ratios[target_gear_forward + 2]
				var projected_rpm = (vehicle_speed / wheel_radius) * 60.0 / (2.0 * PI) * target_gear_ratio * differential_ratio

				# Only kickdown if we won't exceed safe RPM
				if projected_rpm < (upshift_rpm - 500.0):
					shift_to_gear(current_gear_index - 1)
					return

	# Automatic gear selection based on vehicle state
	_select_appropriate_gear(vehicle_speed, brake_pedal)

func _select_appropriate_gear(vehicle_speed: float, brake_pedal: float):
	# Neutral selection
	if vehicle_speed < 0.1 and brake_pedal > 0.1:
		# If stopped and braking, go to neutral
		if current_gear_index != 1:
			shift_to_gear(1) # Neutral
		return

	# Reverse selection (would need additional input logic)
	# This is typically handled by a separate reverse input or gear selector

	# First gear selection
	if current_gear_index == 1 and vehicle_speed > first_gear_min_speed:
		# Engage first gear when starting to move
		shift_to_gear(2) # First gear
		return

func shift_to_gear(target_gear: int):
	# Validate gear range
	target_gear = clampi(target_gear, 0, gear_ratios.size() - 1)

	if target_gear == current_gear_index:
		return

	# Don't allow shifting to reverse at speed
	if target_gear == 0 and get_vehicle_speed() > reverse_max_speed:
		return

	var old_gear = current_gear_index
	current_gear_index = target_gear
	last_shift_time = 0.0

	emit_signal("gear_changed", old_gear, current_gear_index)
	emit_signal("shift_requested", current_gear_index)

	print("Shifted from gear %d to gear %d" % [old_gear, current_gear_index])

func get_current_gear() -> int:
	return current_gear_index

func get_current_gear_ratio() -> float:
	if current_gear_index < gear_ratios.size():
		return gear_ratios[current_gear_index]
	return 0.0

func get_gear_name() -> String:
	match current_gear_index:
		0: return "R"
		1: return "N"
		_: return str(current_gear_index - 1)

func force_gear(gear: int):
	"""Force a specific gear (for debugging or special cases)"""
	shift_to_gear(gear)

func get_vehicle_speed() -> float:
	# This should be set by the vehicle controller
	return 0.0

func set_auto_shift_enabled(enabled: bool):
	auto_shift_enabled = enabled

func set_manual_override(enabled: bool):
	is_manual_override = enabled
	if enabled:
		manual_override_time = 0.0

func get_transmission_info() -> Dictionary:
	var current_forward_gear = current_gear_index - 2
	var next_upshift_speed = 0.0
	var next_downshift_speed = 0.0

	if current_forward_gear >= 0 and current_forward_gear < upshift_speeds.size():
		next_upshift_speed = upshift_speeds[current_forward_gear]
	if current_forward_gear > 0 and current_forward_gear < downshift_speeds.size():
		next_downshift_speed = downshift_speeds[current_forward_gear]

	return {
		"current_gear": current_gear_index,
		"gear_name": get_gear_name(),
		"gear_ratio": get_current_gear_ratio(),
		"auto_shift_enabled": auto_shift_enabled,
		"manual_override": is_manual_override,
		"next_upshift_speed": next_upshift_speed,
		"next_downshift_speed": next_downshift_speed,
		"time_since_shift": last_shift_time
	}