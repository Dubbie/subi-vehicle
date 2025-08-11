class_name VehicleController
extends RigidBody3D

## Simplified vehicle controller that delegates drivetrain management to DrivetrainController
## This creates a cleaner separation of concerns

#region Export Variables
@export_group("Components")
@export var pedal_controller: PedalController
@export var steering_controller: SteeringController
@export var drivetrain_controller: DrivetrainController

@export_group("Vehicle Configuration")
@export var weight: float = 1250.0
@export var axles: Array[AxleController] = []
@export var max_steer_angle: float = 38.0

@export_group("Brakes")
@export var max_brake_torque: float = 4000.0
@export var max_handbrake_torque: float = 2000.0

@export_group("Debug")
@export var physics_sub_steps: int = 8
@export var debug_mode: bool = true:
	set(value):
		debug_mode = value
		for axle in axles:
			axle.left_wheel.debug_mode = value
			axle.right_wheel.debug_mode = value
#endregion

#region Internal Variables
var _driven_axles: Array[AxleController] = []
var wheelbase: float = 0.0
var vehicle_speed: float = 0.0
var grounded: bool = false
var restrict_gas: bool = false

# Cached drivetrain state
var current_drivetrain_torque: float = 0.0
var current_gear: int = 1
var current_engine_rpm: float = 0.0
#endregion

@onready var engine_label: Label = %EngineLabel

func _ready():
	mass = weight

	# Validate required components
	if not drivetrain_controller:
		push_error("DrivetrainController is required but not assigned.")
		return

	if not pedal_controller or not steering_controller:
		push_error("PedalController and SteeringController are required.")
		return

	# Setup vehicle systems
	_setup_axles()
	_initialize_drivetrain()

	steering_controller.initialize(self)

	# Connect drivetrain signals
	drivetrain_controller.gear_changed.connect(_on_gear_changed)
	drivetrain_controller.engine_stalled.connect(_on_engine_stalled)
	drivetrain_controller.clutch_overheated.connect(_on_clutch_overheated)

func _process(_delta: float):
	if debug_mode:
		_update_debug_display()

func _physics_process(delta: float):
	if not _validate_components():
		return

	# Process user inputs
	_process_user_controls(delta)

	# Calculate vehicle speed
	var horizontal_velocity = Vector3(linear_velocity.x, 0, linear_velocity.z)
	vehicle_speed = horizontal_velocity.length()

	# Update axle states and check ground contact
	_update_axles(delta)

	# Get wheel angular velocities for drivetrain
	var wheel_angular_velocities = _get_driven_wheel_speeds()

	# Update drivetrain system
	var sub_step_delta = delta / physics_sub_steps
	for i in range(physics_sub_steps):
		var drivetrain_result = drivetrain_controller.update_drivetrain(
			pedal_controller.get_throttle() if not restrict_gas else 0.0,
			pedal_controller.get_clutch(),
			vehicle_speed,
			wheel_angular_velocities,
			grounded,
			sub_step_delta
		)

		# Cache drivetrain results
		current_drivetrain_torque = drivetrain_result.torque
		current_gear = drivetrain_result.gear
		current_engine_rpm = drivetrain_result.rpm

		# Apply torque to wheels
		_apply_wheel_torques(sub_step_delta)

	# Apply final forces to rigidbody
	_apply_wheel_forces()

#region Private Methods
func _validate_components() -> bool:
	if not pedal_controller or not steering_controller or not drivetrain_controller:
		push_error("Required controllers not assigned")
		set_physics_process(false)
		return false
	return true

func _process_user_controls(delta: float):
	"""Process all user inputs"""
	var gas_input = Input.is_action_pressed("gas") and not restrict_gas
	var brake_input = Input.is_action_pressed("brake")
	var handbrake_input = Input.is_action_pressed("handbrake")
	var clutch_input = Input.is_action_pressed("clutch")
	var steer_input = Input.get_axis("steer_right", "steer_left")

	# Manual transmission controls
	if Input.is_action_just_pressed("shift_up"):
		drivetrain_controller.manual_shift_up()
	if Input.is_action_just_pressed("shift_down"):
		drivetrain_controller.manual_shift_down()

	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, delta)
	steering_controller.process_inputs(steer_input, delta)

func _update_axles(delta: float):
	"""Update all axles and check ground contact"""
	grounded = false

	for i in range(axles.size()):
		var axle = axles[i]

		# Apply steering to front axle
		if i == 0:
			axle.set_steer_value(steering_controller.get_steer_value())

		axle.update_wheel_states(delta)

		# Check ground contact
		if axle.left_wheel.has_contact or axle.right_wheel.has_contact:
			grounded = true

func _get_driven_wheel_speeds() -> Array[float]:
	"""Get angular velocities of driven wheels for drivetrain calculation"""
	var speeds: Array[float] = []

	for axle in _driven_axles:
		speeds.append(axle.left_wheel.current_angular_velocity)
		speeds.append(axle.right_wheel.current_angular_velocity)

	return speeds

func _apply_wheel_forces():
	"""Apply all wheel forces to the vehicle rigidbody"""
	for axle in axles:
		axle.left_wheel.apply_forces_to_rigidbody()
		axle.right_wheel.apply_forces_to_rigidbody()

func _apply_wheel_torques(delta: float):
	"""Apply drivetrain and brake torques to wheels"""
	var brake_torque = pedal_controller.get_brake() * max_brake_torque
	var handbrake_torque = pedal_controller.get_handbrake() * max_handbrake_torque

	for axle in axles:
		# Calculate drive torque for this axle
		var axle_drive_torque = 0.0
		if axle.drive_ratio > 0.0:
			axle_drive_torque = current_drivetrain_torque * axle.diff_ratio

		# Distribute drive torque per wheel
		var drive_torque_per_wheel = axle_drive_torque / 2.0

		# Calculate brake torque per wheel
		var brake_torque_per_wheel = brake_torque * axle.brake_ratio
		var handbrake_per_wheel = handbrake_torque * axle.handbrake_ratio

		# Apply to wheels
		axle.left_wheel.calculate_wheel_physics(
			drive_torque_per_wheel,
			brake_torque_per_wheel + handbrake_per_wheel,
			1.0, # Load transfer (could be calculated)
			delta
		)

		axle.right_wheel.calculate_wheel_physics(
			drive_torque_per_wheel,
			brake_torque_per_wheel + handbrake_per_wheel,
			1.0, # Load transfer (could be calculated)
			delta
		)

func _setup_axles():
	"""Initialize axle configuration and calculate vehicle geometry"""
	if axles.size() < 2:
		push_error("Vehicle requires at least 2 axles")
		return

	# Calculate wheelbase
	var front_axle = axles[0]
	var rear_axle = axles[axles.size() - 1]
	wheelbase = abs(front_axle.z_offset - rear_axle.z_offset)

	if wheelbase < 0.01:
		push_error("Wheelbase is too small")
		return

	print("Vehicle wheelbase: %.2f m" % wheelbase)

	# Initialize axles and identify driven ones
	for axle in axles:
		if axle.drive_ratio > 0.0:
			_driven_axles.append(axle)
			axle.left_wheel.driven_wheel = true
			axle.right_wheel.driven_wheel = true

		var max_steer = max_steer_angle if axle == front_axle else 0.0
		axle.initialize(wheelbase, max_steer)

	print("Found %d driven axles" % _driven_axles.size())

func _initialize_drivetrain():
	"""Initialize the drivetrain with vehicle-specific parameters"""
	if _driven_axles.is_empty():
		push_error("No driven axles found")
		return

	# Use first driven axle for drivetrain calculations
	var primary_axle = _driven_axles[0]
	drivetrain_controller.initialize_drivetrain(
		primary_axle.left_wheel.wheel_radius,
		primary_axle.diff_ratio
	)

func _update_debug_display():
	"""Update debug information display"""
	if not engine_label:
		return

	engine_label.visible = debug_mode
	if not debug_mode:
		return

	var diagnostics = drivetrain_controller._get_diagnostics()
	var engine_info = diagnostics.engine
	var clutch_info = diagnostics.clutch
	var trans_info = diagnostics.transmission

	var text_parts: Array[String] = []

	# Engine information
	text_parts.append("RPM: %.0f" % engine_info.rpm)
	text_parts.append("Torque: %.1f Nm (%.1f%%)" % [engine_info.torque, engine_info.load_percent])

	# Engine status
	if engine_info.is_idle:
		text_parts.append("[IDLE]")
	if engine_info.is_redline:
		text_parts.append("[REDLINE]")

	# Clutch information
	text_parts.append("Clutch: %.1f%% (%.1f Nm)" % [clutch_info.engagement_percent, clutch_info.torque])
	text_parts.append("Slip: %.0f RPM" % clutch_info.slip_rpm)
	text_parts.append("Temp: %.0f°C" % clutch_info.temperature)

	# Transmission
	text_parts.append("Gear: %s" % trans_info.gear_name)
	if trans_info.is_shifting:
		text_parts.append("[SHIFTING]")

	# Vehicle state
	text_parts.append("Speed: %.1f m/s (%.1f km/h)" % [vehicle_speed, vehicle_speed * 3.6])
	text_parts.append("Grounded: %s" % ("YES" if grounded else "NO"))

	# Smart assists
	if clutch_info.anti_stall_active:
		text_parts.append("[ANTI-STALL]")
	if clutch_info.launch_assist_active:
		text_parts.append("[LAUNCH ASSIST]")
	if drivetrain_controller.auto_clutch_enabled:
		text_parts.append("[AUTO-CLUTCH]")

	engine_label.text = "\n".join(text_parts)
#endregion

#region Signal Handlers
func _on_gear_changed(old_gear: int, new_gear: int):
	"""Handle gear change completion"""
	restrict_gas = false
	print("Gear changed from %d to %d" % [old_gear, new_gear])

func _on_engine_stalled():
	"""Handle engine stall event"""
	print("Engine stalled!")
	# Could trigger stall recovery logic or audio cues

func _on_clutch_overheated():
	"""Handle clutch overheating"""
	print("Clutch overheated!")
	# Could trigger warning indicators or performance reduction
#endregion

#region Public API
func get_engine_rpm() -> float:
	return drivetrain_controller.get_engine_rpm()

func get_engine_load() -> float:
	return drivetrain_controller.get_engine_load()

func get_current_gear() -> int:
	return current_gear

func get_vehicle_speed() -> float:
	return vehicle_speed

func get_drivetrain_diagnostics() -> Dictionary:
	return drivetrain_controller._get_diagnostics()

func force_gear(gear: int):
	"""Force specific gear (for debugging)"""
	drivetrain_controller.force_gear(gear)

func stall_engine():
	"""Force engine stall"""
	drivetrain_controller.stall_engine()

func reset_drivetrain():
	"""Reset drivetrain to initial state"""
	drivetrain_controller.reset_clutch_condition()
	drivetrain_controller.force_gear(1) # Neutral
#endregion
