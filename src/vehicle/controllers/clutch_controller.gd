class_name ClutchController
extends Node

# Clutch configuration
var clutch_stiffness: float = 250.0
var clutch_capacity_multiplier: float = 1.2
var clutch_smoothing_factor: float = 0.05

# Clutch state
var clutch_engagement: float = 0.0 # 0.0 = fully disengaged, 1.0 = fully engaged
var clutch_torque: float = 0.0
var last_slip_velocity: float = 0.0
var clutch_temperature: float = 20.0 # Celsius
var clutch_wear: float = 0.0 # 0.0 = new, 1.0 = worn out

# Smart assists configuration
var anti_stall_enabled: bool = true
var launch_assist_enabled: bool = true
var auto_clutch_enabled: bool = false

# Anti-stall parameters
var stall_rpm_threshold: float = 600.0
var anti_stall_engagement_speed: float = 2.0
var anti_stall_rpm_target: float = 1200.0

# Launch assist parameters
var launch_assist_rpm_threshold: float = 2000.0
var launch_assist_slip_target: float = 200.0 # RPM
var launch_assist_engagement_rate: float = 1.5

# Auto-clutch parameters (for beginners)
var auto_clutch_engagement_threshold: float = 1500.0
## Auto clutch engage speed when higher than first gear.
var auto_clutch_speed: float = 3.0
## Auto clutch disengage speed when higher than first gear.
var auto_clutch_disengage_speed: float = 2.4
## m/s - minimum speed to stay engaged
var auto_clutch_min_engagement_speed: float = 3.0
## Auto clutch engage speed in first gear
var auto_clutch_first_gear_engagement_speed: float = 1.5
## Auto clutch disengage speed in first gear
var auto_clutch_first_gear_disengage_speed: float = 0.8

# Thermal simulation
var clutch_heat_capacity: float = 500.0 # J/K
var clutch_cooling_rate: float = 0.5 # K/s to ambient
var ambient_temperature: float = 20.0
var max_operating_temperature: float = 300.0

# Internal state
var _previous_engagement: float = 1.0
var _anti_stall_active: bool = false
var _launch_assist_active: bool = false
var _auto_clutch_waiting_for_rpm: bool = false
var _debug_auto_clutch_state: String = ""

func update_clutch(
	transmission_controller: TransmissionController,
	engine_rpm: float,
	engine_angular_velocity: float,
	gearbox_angular_velocity: float,
	max_engine_torque: float,
	clutch_pedal: float, # 0.0 = released, 1.0 = fully pressed
	throttle_pedal: float,
	current_gear: int,
	vehicle_speed: float, # m/s
	is_grounded: bool,
	delta: float
) -> float:
	# Update clutch engagement based on input and assists
	_update_clutch_engagement(
		transmission_controller, engine_rpm, clutch_pedal, throttle_pedal, current_gear, vehicle_speed, is_grounded,
		delta
	)

	# Calculate clutch slip
	var slip_velocity = engine_angular_velocity - gearbox_angular_velocity
	var slip_rpm = abs(slip_velocity * 60.0 / (2.0 * PI))

	# Calculate maximum transferable torque based on engagement
	var max_clutch_torque = max_engine_torque * clutch_capacity_multiplier * clutch_engagement

	# Apply wear and temperature effects
	max_clutch_torque *= (1.0 - clutch_wear * 0.3) # Wear reduces capacity
	if clutch_temperature > max_operating_temperature:
		max_clutch_torque *= 0.7 # Overheating reduces capacity

	# Calculate target clutch torque
	var clutch_torque_target = clampf(
		slip_velocity * clutch_stiffness * clutch_engagement,
		- max_clutch_torque,
		max_clutch_torque
	)

	# Smooth the clutch torque to prevent oscillation
	clutch_torque = lerp(clutch_torque, clutch_torque_target, clutch_smoothing_factor)

	# Update thermal simulation
	_update_clutch_temperature(slip_rpm, abs(clutch_torque), delta)

	# Update wear simulation
	_update_clutch_wear(slip_rpm, abs(clutch_torque), delta)

	last_slip_velocity = slip_velocity
	return clutch_torque

func _update_clutch_engagement(
	transmission_controller: TransmissionController,
	engine_rpm: float,
	clutch_pedal: float,
	throttle_pedal: float,
	current_gear: int,
	vehicle_speed: float,
	is_grounded: bool,
	delta: float
) -> void:
	var target_engagement: float = 1.0 - clutch_pedal # Inverted - pedal pressed = disengaged
	_anti_stall_active = false
	_launch_assist_active = false
	_auto_clutch_waiting_for_rpm = false

	# Automatic disengagement in neutral or reverse at low speed
	if current_gear <= 1: # Assuming gear 0 = reverse, 1 = neutral
		target_engagement = 0.0

	# Auto-clutch mode overrides everything
	elif auto_clutch_enabled and current_gear > 1:
		# Determine if we should stay engaged based on speed and RPM
		var should_stay_engaged = (vehicle_speed > auto_clutch_min_engagement_speed or engine_rpm > auto_clutch_engagement_threshold) and not transmission_controller.is_shifting

		# If we're moving and have reasonable RPM, stay engaged for engine braking
		if should_stay_engaged and clutch_engagement > 0.1:
			# Once we're moving at reasonable speed, go to full engagement
			if vehicle_speed > auto_clutch_min_engagement_speed:
				target_engagement = 1.0 # Full lock when moving
			else:
				target_engagement = clutch_engagement # Maintain current engagement

			# Only disengage if we're really slow and RPM is very low
			if vehicle_speed < 1.0 and engine_rpm < (auto_clutch_engagement_threshold - 200):
				target_engagement = 0.0
				_auto_clutch_waiting_for_rpm = true
		else:
			# Normal engagement logic for initial launch
			target_engagement = 0.0

			# Only engage if we have throttle input and sufficient RPM
			if throttle_pedal > 0.1 and is_grounded:
				if engine_rpm < auto_clutch_engagement_threshold or transmission_controller.is_shifting:
					# Keep disengaged until RPM builds up
					target_engagement = 0.0
					_auto_clutch_waiting_for_rpm = true
				else:
					# Progressive engagement based on RPM above threshold
					var rpm_above_threshold = engine_rpm - auto_clutch_engagement_threshold
					var engagement_factor = rpm_above_threshold / 1000.0 # Full engagement at +1000 RPM instead of 1500
					target_engagement = clampf(engagement_factor, 0.0, 1.0)

		# Launch assist within auto-clutch mode (only during initial acceleration and low engagement)
		if launch_assist_enabled and vehicle_speed < 5.0 and engine_rpm > launch_assist_rpm_threshold and throttle_pedal > 0.1 and target_engagement < 0.8:
			var current_slip_rpm = abs(last_slip_velocity * 60.0 / (2.0 * PI))
			_launch_assist_active = true
			if current_slip_rpm < launch_assist_slip_target * 0.5:
				# Too little slip, reduce engagement
				target_engagement *= 0.7
			elif current_slip_rpm > launch_assist_slip_target * 2.0:
				# Too much slip, increase engagement slightly
				target_engagement = min(1.0, target_engagement * 1.2)

		# Anti-stall protection in auto mode (only disengage if really necessary)
		if anti_stall_enabled and engine_rpm < stall_rpm_threshold and vehicle_speed < 1.0:
			target_engagement = 0.0
			_anti_stall_active = true

	# Manual clutch with assists
	elif current_gear > 1:
		# Anti-stall assist in manual mode
		if anti_stall_enabled and engine_rpm < stall_rpm_threshold and vehicle_speed < 2.0 and is_grounded:
			var stall_protection = 1.0 - ((stall_rpm_threshold - engine_rpm) / stall_rpm_threshold)
			stall_protection = clampf(stall_protection, 0.0, 1.0)
			target_engagement = min(target_engagement, stall_protection)
			_anti_stall_active = true

		# Launch assist in manual mode
		if launch_assist_enabled and vehicle_speed < 5.0 and engine_rpm > launch_assist_rpm_threshold and throttle_pedal > 0.3:
			var current_slip_rpm = abs(last_slip_velocity * 60.0 / (2.0 * PI))
			_launch_assist_active = true
			if current_slip_rpm < launch_assist_slip_target:
				# Too little slip, reduce engagement slightly
				target_engagement *= 0.85
			elif current_slip_rpm > launch_assist_slip_target * 2.0:
				# Too much slip, increase engagement
				target_engagement = min(1.0, target_engagement * 1.1)

	# Smooth engagement changes with different speeds for different modes
	var engagement_speed: float
	if auto_clutch_enabled:
		if current_gear < 2:
			# Make clutch engage slower in first gear
			if target_engagement > clutch_engagement:
				engagement_speed = auto_clutch_first_gear_engagement_speed
			else:
				engagement_speed = auto_clutch_first_gear_disengage_speed
		else:
			# Make clutch engage faster in higher gears
			if target_engagement > clutch_engagement:
				engagement_speed = auto_clutch_speed
			else:
				engagement_speed = auto_clutch_disengage_speed
	elif _anti_stall_active or _launch_assist_active:
		engagement_speed = anti_stall_engagement_speed
	else:
		engagement_speed = 10.0 # Fast response for manual control

	if target_engagement > clutch_engagement:
		clutch_engagement = min(target_engagement, clutch_engagement + engagement_speed * delta)
	else:
		clutch_engagement = max(target_engagement, clutch_engagement - engagement_speed * delta)

	clutch_engagement = clampf(clutch_engagement, 0.0, 1.0)
	_previous_engagement = clutch_engagement

func _update_clutch_temperature(slip_rpm: float, torque: float, delta: float) -> void:
	# Heat generation from slip
	var power_dissipated = (slip_rpm * PI / 30.0) * torque # Watts
	var heat_generated = power_dissipated * delta # Joules

	# Temperature increase
	var temperature_increase = heat_generated / clutch_heat_capacity
	clutch_temperature += temperature_increase

	# Cooling to ambient
	var temperature_difference = clutch_temperature - ambient_temperature
	var cooling = temperature_difference * clutch_cooling_rate * delta
	clutch_temperature -= cooling

	clutch_temperature = max(clutch_temperature, ambient_temperature)

func _update_clutch_wear(slip_rpm: float, torque: float, delta: float) -> void:
	# Wear increases with slip and torque
	var wear_rate = (slip_rpm / 1000.0) * (torque / 1000.0) * delta * 0.001

	# Temperature accelerates wear
	if clutch_temperature > 200.0:
		var temp_factor = 1.0 + ((clutch_temperature - 200.0) / 100.0)
		wear_rate *= temp_factor

	clutch_wear = min(1.0, clutch_wear + wear_rate)

func get_clutch_info() -> Dictionary:
	var slip_rpm = abs(last_slip_velocity * 60.0 / (2.0 * PI))
	return {
		"engagement": clutch_engagement,
		"torque": clutch_torque,
		"temperature": clutch_temperature,
		"wear": clutch_wear,
		"slip_velocity": last_slip_velocity,
		"slip_rpm": slip_rpm,
		"anti_stall_active": _anti_stall_active,
		"launch_assist_active": _launch_assist_active,
		"auto_clutch_mode": auto_clutch_enabled,
		"waiting_for_rpm": _auto_clutch_waiting_for_rpm,
		"debug_state": _debug_auto_clutch_state
	}

# Configuration methods
func set_anti_stall_enabled(enabled: bool) -> void:
	anti_stall_enabled = enabled

func set_launch_assist_enabled(enabled: bool) -> void:
	launch_assist_enabled = enabled

func set_auto_clutch_enabled(enabled: bool) -> void:
	auto_clutch_enabled = enabled

func set_clutch_parameters(stiffness: float, capacity: float, smoothing: float) -> void:
	clutch_stiffness = stiffness
	clutch_capacity_multiplier = capacity
	clutch_smoothing_factor = smoothing

func reset_clutch_condition() -> void:
	clutch_wear = 0.0
	clutch_temperature = ambient_temperature