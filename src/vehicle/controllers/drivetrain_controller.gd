class_name DrivetrainController
extends Node

signal gear_changed(old_gear: int, new_gear: int)
signal engine_stalled()
signal clutch_overheated()

#region Configuration
@export_group("Engine")
## Defines the engine's torque output (in Nm) at different engine speeds (in RPM).
@export var torque_curve: Curve
## Rotational inertia of the engine. Determines how quickly the engine can change its speed. (kg⋅m²)
@export var engine_inertia: float = 0.3
## Engine braking torque when throttle is released. (Nm)
@export var engine_brake_torque: float = 30.0
## Frictional torque that resists engine rotation, proportional to its angular velocity. (Nm⋅s/rad)
@export var engine_friction_coefficient: float = 0.006
## The engine's idle speed. The engine will attempt to stay at this RPM when not under load. (RPM)
@export var engine_min_rpm: float = 800.0
## The engine's redline. The simulation will not allow the engine to exceed this speed. (RPM)
@export var engine_max_rpm: float = 7000.0

@export_group("Transmission")
## Ratios for each gear [-R, N, 1, 2, 3...]. Determines torque multiplication. (Dimensionless)
@export var gear_ratios: Array[float] = [-3.382, 0.0, 3.321, 1.902, 1.308, 1.0, 0.838]
## Enables or disables automatic gear shifting.
@export var auto_shift_enabled: bool = true
## The engine RPM at which the automatic transmission will initiate an upshift. (RPM)
@export var upshift_rpm: float = 6200.0
## The engine RPM below which the automatic transmission will initiate a downshift. (RPM)
@export var downshift_rpm: float = 3500.0
## The duration of a gear shift, during which torque is reduced. (Seconds)
@export var shift_delay: float = 0.3

@export_group("Clutch")
## Determines how aggressively the clutch locks. Higher values mean a sharper, more "grabby" clutch. (Nm per (rad/s))
@export var clutch_stiffness: float = 250.0
## Multiplier on the engine's max torque to determine the clutch's torque capacity. (e.g., 1.2 = 120% of engine torque)
@export var clutch_capacity_multiplier: float = 1.2
## Smoothing factor for clutch torque application to prevent sudden jolts. (0-1)
@export var clutch_smoothing_factor: float = 0.05
## The amount of energy the clutch can absorb before its temperature increases by 1°C. (Joules/°C)
@export var clutch_heat_capacity: float = 600.0
## The rate at which the clutch cools down towards ambient temperature. (Factor/second)
@export var clutch_cooling_rate: float = 0.6
## The temperature at which the clutch begins to fade and lose its ability to transmit torque. (°C)
@export var max_operating_temperature: float = 300.0

@export_group("Smart Assists")
## Enables or disables the system that automatically disengages the clutch to prevent stalling.
@export var anti_stall_enabled: bool = true
## Enables or disables launch assist, which helps manage clutch slip for optimal acceleration.
@export var launch_assist_enabled: bool = true
## Enables or disables a fully automatic clutch, removing the need for a clutch pedal.
@export var auto_clutch_enabled: bool = false
## The engine speed below which it is considered stalled and will shut off. (RPM)
@export var stall_rpm_threshold: float = 400.0
## The engine speed below which the anti-stall system starts to disengage the clutch. (RPM)
@export var clutch_out_rpm: float = 1200.0
## The speed at which the clutch is engaged/disengaged when anti-stall is active. (Engagement factor/second)
@export var anti_stall_engagement_speed: float = 2.0
## The minimum engine RPM required to activate launch assist. (RPM)
@export var launch_assist_rpm_threshold: float = 2000.0
## The target slip RPM that launch assist will try to maintain for a good start. (RPM)
@export var launch_assist_slip_target: float = 200.0

@export_group("Auto Clutch")
## The engine RPM threshold for the auto-clutch to start engaging. (RPM)
@export var auto_clutch_engagement_threshold: float = 1200.0
## The speed at which the auto-clutch engages. (Engagement factor/second)
@export var auto_clutch_speed: float = 3.0
## The speed at which the auto-clutch disengages. (Engagement factor/second)
@export var auto_clutch_disengage_speed: float = 2.4
## The minimum engagement speed for the auto-clutch. (Engagement factor/second)
@export var auto_clutch_min_engagement_speed: float = 3.0

@export_group("Auto Shift")
## Allow auto-shift from Neutral into first gear when throttle is applied.
@export var auto_shift_to_first_enabled: bool = true
## The vehicle speed below which the transmission can automatically shift into first gear. (m/s)
@export var first_gear_engage_speed: float = 2.0
#endregion

#region Constants
const RADS_TO_RPM = 60.0 / (2.0 * PI)
const RPM_TO_RADS = (2.0 * PI) / 60.0
#endregion

#region State Variables
# Engine state
var engine_rpm: float = 0.0
var engine_angular_velocity: float = 0.0
var engine_torque: float = 0.0

# Clutch state
var clutch_engagement: float = 0.0
var clutch_torque: float = 0.0
var clutch_temperature: float = 20.0
var clutch_wear: float = 0.0
var clutch_slip_rpm: float = 0.0

# Transmission state
var current_gear_index: int = 1 # Start in neutral
var gear_shift_progress: float = 0.0
var is_shifting: bool = false
var last_shift_time: float = 0.0

# Calculated values
var gearbox_rpm: float = 0.0
var gearbox_angular_velocity: float = 0.0
var output_torque: float = 0.0

# Internal state
var _anti_stall_active: bool = false
var _launch_assist_active: bool = false
var _shift_speeds_calculated: bool = false
var _upshift_speeds: Array[float] = []
var _downshift_speeds: Array[float] = []

# Drivetrain configuration (set by vehicle)
var wheel_radius: float = 0.3
var differential_ratio: float = 3.9
#endregion

func _ready():
	engine_rpm = engine_min_rpm
	engine_angular_velocity = engine_rpm * RPM_TO_RADS
	clutch_temperature = 20.0

## Initialize drivetrain with vehicle-specific parameters
func initialize_drivetrain(wheel_r: float, diff_ratio: float) -> void:
	wheel_radius = wheel_r
	differential_ratio = diff_ratio
	_calculate_shift_speeds()

## Main update function - processes all drivetrain components together
## Returns: { torque: float, gear: int, rpm: float, diagnostics: Dictionary }
func update_drivetrain(
	throttle_input: float,
	clutch_pedal: float,
	brake_pedal: float,
	handbrake_pull: float,
	vehicle_speed: float,
	wheel_angular_velocities: Array[float],
	is_grounded: bool,
	delta: float
) -> Dictionary:
	# Update timers
	last_shift_time += delta
	if is_shifting and last_shift_time > shift_delay:
		_complete_gear_change()

	# Calculate gearbox state from wheel speeds
	_update_gearbox_state(wheel_angular_velocities)

	# Process gear changes - FIXED: Process shifting before clutch updates
	if auto_shift_enabled and not is_shifting and is_grounded:
		_process_automatic_shifting(vehicle_speed, throttle_input)

	# Update clutch engagement
	_update_clutch_engagement(
		throttle_input, clutch_pedal, brake_pedal, handbrake_pull, vehicle_speed, is_grounded, delta
	)

	# Calculate clutch slip and torque
	var slip_velocity = engine_angular_velocity - gearbox_angular_velocity
	clutch_slip_rpm = abs(slip_velocity * RADS_TO_RPM)

	# Calculate clutch torque capacity
	var max_clutch_torque = _get_max_clutch_torque()
	var target_clutch_torque = clampf(
		slip_velocity * clutch_stiffness * clutch_engagement,
		- max_clutch_torque,
		max_clutch_torque
	)

	# Smooth clutch torque
	clutch_torque = lerp(clutch_torque, target_clutch_torque, clutch_smoothing_factor)

	# Update engine with clutch reaction torque
	_update_engine(throttle_input, clutch_torque, delta)

	# Calculate final output torque
	var current_gear_ratio = get_current_gear_ratio()
	if is_shifting:
		# Reduce torque during shifts
		output_torque = clutch_torque * current_gear_ratio * 0.3
	else:
		output_torque = clutch_torque * current_gear_ratio

	# Update thermal and wear simulation
	_update_clutch_condition(delta)

	# Return comprehensive state
	return {
		"torque": output_torque,
		"gear": current_gear_index,
		"rpm": engine_rpm,
		"diagnostics": _get_diagnostics()
	}

## Update engine physics
func _update_engine(throttle_input: float, clutch_reaction_torque: float, delta: float) -> void:
	# Calculate engine torque
	var throttle_torque = 0.0
	if torque_curve:
		throttle_torque = torque_curve.sample_baked(engine_rpm) * clampf(throttle_input, 0.0, 1.0)

	var friction_torque = engine_friction_coefficient * engine_angular_velocity
	var brake_torque = engine_brake_torque * (1.0 - clampf(throttle_input, 0.0, 1.0))

	# Net torque calculation
	var net_torque = throttle_torque - friction_torque - brake_torque - clutch_reaction_torque

	# Update angular velocity
	var angular_acceleration = net_torque / engine_inertia
	engine_angular_velocity += angular_acceleration * delta

	# Apply RPM limits
	var min_angular_velocity = engine_min_rpm * RPM_TO_RADS
	var max_angular_velocity = engine_max_rpm * RPM_TO_RADS
	engine_angular_velocity = clampf(engine_angular_velocity, min_angular_velocity, max_angular_velocity)

	# Update RPM
	engine_rpm = engine_angular_velocity * RADS_TO_RPM

	# Store current engine torque output
	engine_torque = throttle_torque - friction_torque - brake_torque

	# Proper stall detection - only when engine drops below actual stall RPM
	if engine_rpm <= stall_rpm_threshold:
		engine_stalled.emit()
		# Force engine to minimum idle to simulate restart needed
		engine_rpm = engine_min_rpm
		engine_angular_velocity = engine_rpm * RPM_TO_RADS

## Calculate gearbox RPM from wheel speeds
func _update_gearbox_state(wheel_angular_velocities: Array[float]) -> void:
	if wheel_angular_velocities.is_empty():
		gearbox_rpm = 0.0
		gearbox_angular_velocity = 0.0
		return

	# Average wheel speed
	var avg_wheel_angular_velocity = 0.0
	for wheel_vel in wheel_angular_velocities:
		avg_wheel_angular_velocity += wheel_vel
	avg_wheel_angular_velocity /= wheel_angular_velocities.size()

	# Convert to gearbox speed through differential and gear ratio
	var current_gear_ratio = get_current_gear_ratio()
	if abs(current_gear_ratio) > 0.001:
		gearbox_angular_velocity = avg_wheel_angular_velocity * differential_ratio * current_gear_ratio
		gearbox_rpm = gearbox_angular_velocity * RADS_TO_RPM
	else:
		gearbox_rpm = 0.0
		gearbox_angular_velocity = 0.0

## Update clutch engagement based on input and assists
func _update_clutch_engagement(
	throttle_input: float,
	clutch_pedal: float,
	brake_pedal: float,
	handbrake_pull: float,
	vehicle_speed: float,
	is_grounded: bool,
	delta: float
) -> void:
	var target_engagement = 1.0 - clutch_pedal # Inverted
	_anti_stall_active = false
	_launch_assist_active = false

	# Don't force disengagement in first gear, only neutral and reverse
	if current_gear_index == 0 or current_gear_index == 1: # Only reverse and neutral
		target_engagement = 0.0
	# Auto-clutch logic
	elif auto_clutch_enabled and current_gear_index > 1:
		target_engagement = _calculate_auto_clutch_engagement(
			throttle_input, brake_pedal, handbrake_pull, vehicle_speed, is_grounded
		)
	# Manual clutch with assists
	elif current_gear_index > 1:
		target_engagement = _apply_clutch_assists(
			target_engagement, throttle_input, brake_pedal, handbrake_pull, vehicle_speed, is_grounded
		)

	# Smooth engagement changes
	var engagement_speed = _get_clutch_engagement_speed(target_engagement)
	if target_engagement > clutch_engagement:
		clutch_engagement = min(target_engagement, clutch_engagement + engagement_speed * delta)
	else:
		clutch_engagement = max(target_engagement, clutch_engagement - engagement_speed * delta)

	clutch_engagement = clampf(clutch_engagement, 0.0, 1.0)

## Calculate clutch engagement for auto-clutch mode
func _calculate_auto_clutch_engagement(
	throttle_input: float,
	brake_input: float,
	handbrake_input: float,
	vehicle_speed: float,
	is_grounded: bool
) -> float:
	# No engagement if not grounded or no throttle
	if not is_grounded:
		return 0.0

	# Disengage clutch if pressing brakes or handbrake
	if brake_input > 0.0 or handbrake_input > 0.0:
		return 0.0

	# Progressive engagement based on RPM and throttle
	var base_engagement: float = 0.0
	if engine_rpm < clutch_out_rpm:
		# Below clutch out RPM, only engage if throttle is applied
		if throttle_input < 0.1:
			return 0.0
		# Progressive engagement based on RPM above idle
		var rpm_factor = clampf((engine_rpm - engine_min_rpm) / (clutch_out_rpm - engine_min_rpm), 0.0, 1.0)
		var throttle_factor = clampf(throttle_input, 0.0, 1.0)
		base_engagement = rpm_factor * throttle_factor * 0.8
	else:
		# Above clutch out RPM, engage more aggressively
		var rpm_factor = clampf((engine_rpm - clutch_out_rpm) / 1000.0, 0.0, 1.0)
		base_engagement = 0.3 + rpm_factor * 0.7 # Start at 30% engagement

	# Apply launch assist if enabled
	if launch_assist_enabled and vehicle_speed < 5.0:
		base_engagement = _apply_launch_assist(base_engagement)

	return clampf(base_engagement, 0.0, 1.0)

## Apply anti-stall and launch assist to manual clutch
func _apply_clutch_assists(
	base_engagement: float,
	throttle_input: float,
	brake_input: float,
	handbrake_input: float,
	vehicle_speed: float,
	is_grounded: bool
) -> float:
	var assisted_engagement = base_engagement

	# Anti-stall logic with three RPM zones
	if anti_stall_enabled and is_grounded and current_gear_index > 1:
		# Zone 1: Above clutch_out_rpm - normal operation
		# Zone 2: Between clutch_out_rpm and idle - progressive clutch reduction
		# Zone 3: Below idle - emergency anti-stall
		if engine_rpm < clutch_out_rpm and throttle_input < 0.1:
			if engine_rpm >= engine_min_rpm:
				# Progressive clutch disengagement between clutch_out_rpm and idle
				var disengagement_factor = (engine_rpm - engine_min_rpm) / (clutch_out_rpm - engine_min_rpm)
				assisted_engagement = min(assisted_engagement, disengagement_factor)
				_anti_stall_active = true
			else:
				# Emergency anti-stall below idle RPM
				var emergency_factor = max(0.0, (engine_rpm - stall_rpm_threshold) / (engine_min_rpm - stall_rpm_threshold))
				assisted_engagement = min(assisted_engagement, emergency_factor * 0.5)
				_anti_stall_active = true

	# Launch assist
	if launch_assist_enabled and vehicle_speed < 5.0 and engine_rpm > launch_assist_rpm_threshold and throttle_input > 0.3:
		assisted_engagement = _apply_launch_assist(assisted_engagement)

	# Disengage clutch if pressing brakes or handbrake
	if brake_input > 0.0 or handbrake_input > 0.0:
		assisted_engagement = 0.0

	return assisted_engagement

## Apply launch assist logic
func _apply_launch_assist(base_engagement: float) -> float:
	_launch_assist_active = true

	if clutch_slip_rpm < launch_assist_slip_target:
		return base_engagement * 0.85 # Reduce engagement for more slip
	elif clutch_slip_rpm > launch_assist_slip_target * 2.0:
		return min(1.0, base_engagement * 1.1) # Increase engagement to reduce slip

	return base_engagement

## Get appropriate clutch engagement speed
func _get_clutch_engagement_speed(target_engagement: float) -> float:
	if auto_clutch_enabled:
		if current_gear_index == 2: # First gear
			return 1.5 if target_engagement > clutch_engagement else 0.8
		else:
			return auto_clutch_speed if target_engagement > clutch_engagement else auto_clutch_disengage_speed
	elif _anti_stall_active or _launch_assist_active:
		return anti_stall_engagement_speed
	else:
		return 10.0 # Fast manual response

## Calculate maximum clutch torque capacity
func _get_max_clutch_torque() -> float:
	var max_engine_torque = 0.0
	if torque_curve:
		max_engine_torque = torque_curve.sample_baked(engine_rpm)

	var max_torque = max_engine_torque * clutch_capacity_multiplier * clutch_engagement

	# Apply wear effects
	max_torque *= (1.0 - clutch_wear * 0.3)

	# Apply temperature effects
	if clutch_temperature > max_operating_temperature:
		max_torque *= 0.7
		if clutch_temperature > max_operating_temperature + 50.0:
			clutch_overheated.emit()

	return max_torque

## Update clutch temperature and wear
func _update_clutch_condition(delta: float) -> void:
	# Heat generation from slip
	var power_dissipated = (clutch_slip_rpm * PI / 30.0) * abs(clutch_torque)
	var heat_generated = power_dissipated * delta
	var temperature_increase = heat_generated / clutch_heat_capacity
	clutch_temperature += temperature_increase

	# Cooling
	var cooling = (clutch_temperature - 20.0) * clutch_cooling_rate * delta
	clutch_temperature = max(20.0, clutch_temperature - cooling)

	# Wear calculation
	var wear_rate = (clutch_slip_rpm / 1000.0) * (abs(clutch_torque) / 1000.0) * delta * 0.001
	if clutch_temperature > 200.0:
		wear_rate *= 1.0 + ((clutch_temperature - 200.0) / 100.0)
	clutch_wear = min(1.0, clutch_wear + wear_rate)

## Pre-calculate shift points for each gear
func _calculate_shift_speeds() -> void:
	if _shift_speeds_calculated:
		return

	_upshift_speeds.clear()
	_downshift_speeds.clear()

	for i in range(2, gear_ratios.size()):
		var gear_ratio = gear_ratios[i]
		if gear_ratio > 0.0:
			var upshift_wheel_rpm = upshift_rpm / (gear_ratio * differential_ratio)
			var upshift_speed = (upshift_wheel_rpm * 2.0 * PI / 60.0) * wheel_radius
			_upshift_speeds.append(upshift_speed * 0.85)

			var downshift_wheel_rpm = downshift_rpm / (gear_ratio * differential_ratio)
			var downshift_speed = (downshift_wheel_rpm * 2.0 * PI / 60.0) * wheel_radius
			_downshift_speeds.append(downshift_speed * 0.85)
		else:
			_upshift_speeds.append(999.0)
			_downshift_speeds.append(0.0)

	_shift_speeds_calculated = true

## Handle automatic transmission shifting
func _process_automatic_shifting(vehicle_speed: float, throttle_input: float) -> void:
	if not _shift_speeds_calculated:
		_calculate_shift_speeds()

	# Auto-shift from neutral to first gear
	if current_gear_index == 1 and auto_shift_to_first_enabled: # In neutral
		if throttle_input > 0.2 and vehicle_speed < first_gear_engage_speed:
			_initiate_gear_change(2) # Shift to first gear (index 2)
			return

	var current_forward_gear = current_gear_index - 2

	# Upshift check
	if current_gear_index >= 2 and current_gear_index < gear_ratios.size() - 1:
		if current_forward_gear < _upshift_speeds.size():
			var upshift_speed = _upshift_speeds[current_forward_gear]
			if vehicle_speed > upshift_speed and last_shift_time > shift_delay:
				_initiate_gear_change(current_gear_index + 1)

	# Downshift check
	if current_gear_index > 2:
		if current_forward_gear > 0 and current_forward_gear - 1 < _downshift_speeds.size():
			var downshift_speed = _downshift_speeds[current_forward_gear - 1]
			if vehicle_speed < downshift_speed and last_shift_time > shift_delay and throttle_input < 0.8:
				_initiate_gear_change(current_gear_index - 1)

## Start a gear change
func _initiate_gear_change(target_gear: int) -> void:
	target_gear = clampi(target_gear, 0, gear_ratios.size() - 1)
	if target_gear == current_gear_index:
		return

	var old_gear = current_gear_index
	current_gear_index = target_gear
	is_shifting = true
	last_shift_time = 0.0
	gear_shift_progress = 0.0

	# Emit with proper gear numbers (0-based for reverse, 1-based for forward)
	var old_display_gear = old_gear if old_gear <= 1 else old_gear - 1
	var new_display_gear = target_gear if target_gear <= 1 else target_gear - 1
	gear_changed.emit(old_display_gear, new_display_gear)

## Complete the gear change process
func _complete_gear_change() -> void:
	is_shifting = false
	gear_shift_progress = 1.0

## Get comprehensive diagnostic information
func _get_diagnostics() -> Dictionary:
	var engine_max_torque = 0.0
	if torque_curve:
		engine_max_torque = torque_curve.sample_baked(engine_rpm)

	return {
		"engine": {
			"rpm": engine_rpm,
			"torque": engine_torque,
			"load_percent": (engine_torque / engine_max_torque) * 100.0 if engine_max_torque > 0 else 0.0,
			"is_idle": engine_rpm <= engine_min_rpm + 50,
			"is_redline": engine_rpm >= engine_max_rpm - 50
		},
		"clutch": {
			"engagement_percent": clutch_engagement * 100.0,
			"torque": clutch_torque,
			"slip_rpm": clutch_slip_rpm,
			"temperature": clutch_temperature,
			"wear_percent": clutch_wear * 100.0,
			"anti_stall_active": _anti_stall_active,
			"launch_assist_active": _launch_assist_active
		},
		"transmission": {
			"gear": current_gear_index,
			"gear_name": get_gear_name(),
			"gear_ratio": get_current_gear_ratio(),
			"is_shifting": is_shifting,
			"shift_progress": gear_shift_progress
		}
	}

#region Public API
func get_current_gear_ratio() -> float:
	if current_gear_index < gear_ratios.size():
		return gear_ratios[current_gear_index]
	return 0.0

func get_gear_name() -> String:
	match current_gear_index:
		0: return "R"
		1: return "N"
		_: return str(current_gear_index - 1)

## Manual upshift request
func manual_shift_up() -> void:
	if current_gear_index < gear_ratios.size() - 1:
		_initiate_gear_change(current_gear_index + 1)

##  Manual downshift request
func manual_shift_down() -> void:
	if current_gear_index > 0:
		_initiate_gear_change(current_gear_index - 1)

## Force specific gear (for debugging)
func force_gear(gear_index: int) -> void:
	_initiate_gear_change(clampi(gear_index, 0, gear_ratios.size() - 1))

func get_engine_rpm() -> float:
	return engine_rpm

func get_engine_load() -> float:
	if not torque_curve:
		return 0.0
	var max_torque = torque_curve.sample_baked(engine_rpm)
	return (engine_torque / max_torque * 100.0) if max_torque > 0 else 0.0

## Force engine stall
func stall_engine() -> void:
	engine_rpm = engine_min_rpm
	engine_angular_velocity = engine_rpm * RPM_TO_RADS
	engine_stalled.emit()

## Reset clutch wear and temperature
func reset_clutch_condition() -> void:
	clutch_wear = 0.0
	clutch_temperature = 20.0
#endregion