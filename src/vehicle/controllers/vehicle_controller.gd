class_name VehicleController
extends RigidBody3D

#region Export Variables
@export var is_controlled: bool = false

@export_group("Components")
@export var pedal_controller: PedalController
@export var steering_controller: SteeringController
@export var drivetrain_controller: DrivetrainController
@export var aero_controller: AeroController
@export var ecu_controller: ECUController

@export_group("Vehicle Configuration")
## The axles of the vehicle, the first one is the steered one always.
@export var axles: Array[AxleController] = []
## The maximum steering angle in degrees.
@export var max_steer_angle: float = 38.0
## Damping force applied to driven axles at very low speed to prevent oscillation.
@export var low_speed_axle_damping: float = 25.0

@export_group("Brakes")
## Maximum brake torque in Newton-meters.
@export var max_brake_torque: float = 4000.0
## Maximum handbrake torque in Newton-meters.
@export var max_handbrake_torque: float = 2000.0

@export_group("Debug")
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
	# Validate required components
	if not drivetrain_controller:
		push_error("DrivetrainController is required but not assigned.")
		return

	if not ecu_controller:
		push_error("ECUController is required but not assigned.")
		return

	if not pedal_controller or not steering_controller:
		push_error("PedalController and SteeringController are required.")
		return

	if not aero_controller:
		push_error("AeroController is required but not assigned.")
		return

	# Setup vehicle systems
	_setup_axles()
	_initialize_drivetrain()

	steering_controller.initialize(self)
	aero_controller.initialize(self)

	# Connect drivetrain signals
	drivetrain_controller.gear_changed.connect(_on_gear_changed)
	drivetrain_controller.engine_stalled.connect(_on_engine_stalled)
	drivetrain_controller.clutch_overheated.connect(_on_clutch_overheated)

func _process(_delta: float):
	pass
	if debug_mode and is_controlled:
		_update_debug_display()
		var com_world: Vector3 = to_global(center_of_mass)
		DebugDraw3D.draw_sphere(com_world, 0.1, Color.WHITE)

func _physics_process(delta: float):
	if not _validate_components():
		return

	# Process user inputs
	_process_user_controls(delta)

	# Calculate vehicle speed
	var horizontal_velocity = Vector3(linear_velocity.x, 0, linear_velocity.z)
	vehicle_speed = horizontal_velocity.length()

	# Update axles
	grounded = false
	for i in range(axles.size()):
		var axle = axles[i]
		# Apply steering to front axle
		if i == 0:
			axle.set_steer_value(steering_controller.get_steer_value())

		# Update suspension
		axle.update_axle_and_wheel_states()

		# Check ground contact
		if axle.left_wheel.has_contact or axle.right_wheel.has_contact:
			grounded = true

	# Stiction logic
	var all_wheels_on_ground = true
	for axle in axles:
		if not axle.left_wheel.has_contact or not axle.right_wheel.has_contact:
			all_wheels_on_ground = false
			break # No need to check further

	# Determine if brakes are active (using a small threshold).
	var brakes_are_active = pedal_controller.get_brake() > 0.1 or pedal_controller.get_handbrake() > 0.1

	# Combine conditions to get the final stiction state.
	var should_allow_stiction = all_wheels_on_ground and brakes_are_active

	# Pass this state to all wheels.
	for axle in axles:
		axle.left_wheel.allow_stiction = should_allow_stiction
		axle.right_wheel.allow_stiction = should_allow_stiction

	# Get wheel angular velocities for drivetrain
	var wheel_angular_velocities = _get_driven_wheel_speeds()

	# Update drivetrain
	var drivetrain_result = drivetrain_controller.update_drivetrain(
		pedal_controller.get_throttle() if not restrict_gas else 0.0,
		pedal_controller.get_clutch(),
		pedal_controller.get_brake(),
		pedal_controller.get_handbrake(),
		vehicle_speed,
		wheel_angular_velocities,
		grounded,
		delta
	)

	# Cache drivetrain results
	current_drivetrain_torque = drivetrain_result.torque
	current_gear = drivetrain_result.gear
	current_engine_rpm = drivetrain_result.rpm

	# Apply torque to wheels. This function calculates wheel spin and tire forces.
	_apply_wheel_torques(delta)

	# Apply the final calculated forces from the last sub-step to the rigidbody
	_apply_wheel_forces()

#region Private Methods
func _validate_components() -> bool:
	if not pedal_controller or not steering_controller or not drivetrain_controller:
		push_error("Required controllers not assigned")
		set_physics_process(false)
		return false
	return true

## Process all user inputs
func _process_user_controls(delta: float):
	if not is_controlled: return

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

	# Debug
	if Input.is_action_just_pressed("debug_load"):
		debug_load_distribution()
	if Input.is_action_just_pressed("toggle_debug"):
		debug_mode = not debug_mode

	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, delta)
	steering_controller.process_inputs(steer_input, delta)

## Get angular velocities of driven wheels for drivetrain calculation
func _get_driven_wheel_speeds() -> Array[float]:
	var speeds: Array[float] = []

	for axle in _driven_axles:
		speeds.append(axle.left_wheel.current_angular_velocity)
		speeds.append(axle.right_wheel.current_angular_velocity)

	return speeds

## Apply all wheel forces to the vehicle rigidbody
func _apply_wheel_forces() -> void:
	for axle in axles:
		axle.left_wheel.apply_forces_to_rigidbody()
		axle.right_wheel.apply_forces_to_rigidbody()

## Apply drivetrain and brake torques to wheels
func _apply_wheel_torques(delta: float) -> void:
	var brake_torque = pedal_controller.get_brake() * max_brake_torque
	var handbrake_torque = pedal_controller.get_handbrake() * max_handbrake_torque

	for axle in axles:
		# Calculate total drive torque for this axle from the drivetrain
		var axle_drive_torque = 0.0
		if axle.drive_ratio > 0.0:
			axle_drive_torque = current_drivetrain_torque * axle.diff_ratio

		# --- NEW AXLE DAMPING LOGIC (THE FIX) ---
		# At very low vehicle speeds, apply a damping force to the entire axle's
		# drive torque *before* it gets to the differential.
		var stiction_speed_threshold: float = 1.0 # m/s
		if vehicle_speed < stiction_speed_threshold and axle.drive_ratio > 0.0:
			# Get the average rotational speed of the axle
			var avg_axle_speed = (axle.left_wheel.current_angular_velocity + axle.right_wheel.current_angular_velocity) * 0.5

			# Calculate a damping torque that opposes the average rotation
			var axle_damping_torque = avg_axle_speed * low_speed_axle_damping

			# Apply the damping directly to the axle's input torque
			axle_drive_torque -= axle_damping_torque
		# --- END OF NEW LOGIC ---

		# Get the distributed drive torques from the axle's differential simulation
		# This now receives the potentially damped torque, solving the feedback loop.
		var distributed_drive_torques: Vector2 = axle.get_distributed_torques(axle_drive_torque)
		var drive_torque_left = distributed_drive_torques.x
		var drive_torque_right = distributed_drive_torques.y

		# Calculate brake torque per wheel
		var brake_torque_per_wheel = brake_torque * axle.brake_ratio
		var handbrake_per_wheel = handbrake_torque * axle.handbrake_ratio

		# Apply the final calculated torques to each wheel
		axle.left_wheel.calculate_wheel_physics(
			drive_torque_left,
			brake_torque_per_wheel + handbrake_per_wheel,
			1.0,
			delta
		)

		axle.right_wheel.calculate_wheel_physics(
			drive_torque_right,
			brake_torque_per_wheel + handbrake_per_wheel,
			1.0,
			delta
		)

## Initialize axle configuration and calculate vehicle geometry
func _setup_axles() -> void:
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

## Initialize the drivetrain with vehicle-specific parameters
func _initialize_drivetrain():
	if _driven_axles.is_empty():
		push_error("No driven axles found")
		return

	# Use first driven axle for drivetrain calculations
	var primary_axle = _driven_axles[0]
	drivetrain_controller.initialize_drivetrain(
		primary_axle.left_wheel.wheel_radius,
		primary_axle.diff_ratio,
		ecu_controller,
	)

## Update debug information display
func _update_debug_display():
	if not engine_label:
		return

	engine_label.visible = debug_mode and is_controlled
	if not engine_label.visible:
		return

	var diagnostics = drivetrain_controller._get_diagnostics()
	var engine_info = diagnostics.engine
	var clutch_info = diagnostics.clutch
	var trans_info = diagnostics.transmission

	var text_parts: Array[String] = []

	# Engine information
	text_parts.append("RPM: %.0f" % engine_info.rpm)
	text_parts.append("Torque: %.1f Nm (%.1f%%)" % [engine_info.torque, engine_info.load_percent])

	# Clutch information
	text_parts.append("Clutch: %.1f%% (%.1f Nm)" % [clutch_info.engagement_percent, clutch_info.torque])
	text_parts.append("Locked: %s" % ("YES" if clutch_info.locked else "NO"))
	text_parts.append("Corrective Torque: %.1f Nm" % clutch_info.corrective_torque)
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

	# Steering
	text_parts.append("\n--- Steering ---")
	var steer_info = steering_controller.get_debug_info()
	text_parts.append("Slip Angle: %.1f°" % steer_info.front_slip_angle)
	text_parts.append("Grip Util: %.2f%%" % (steer_info.grip_utilization * 100.0))
	text_parts.append("Steer Limit: %.2f" % steer_info.steering_limit)

	if steer_info.is_counter_steering:
		text_parts.append("[COUNTER-STEERING]")
	if steer_info.grip_limiting_active:
		text_parts.append("[GRIP LIMITING]")

	engine_label.text = "\n".join(text_parts)
#endregion

#region Signal Handlers
## Handle gear change completion
func _on_gear_changed(old_gear: int, new_gear: int) -> void:
	restrict_gas = false
	print("Gear changed from %d to %d" % [old_gear, new_gear])

## Handle engine stall event
func _on_engine_stalled() -> void:
	print("Engine stalled!")
	# Could trigger stall recovery logic or audio cues

## Handle clutch overheating
func _on_clutch_overheated() -> void:
	print("Clutch overheated!")
	# Could trigger warning indicators or performance reduction
#endregion

#region Public API
func get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)

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

## Force specific gear (for debugging)
func force_gear(gear: int) -> void:
	drivetrain_controller.force_gear(gear)

## Force engine stall
func stall_engine() -> void:
	drivetrain_controller.stall_engine()

## Reset drivetrain to initial state
func reset_drivetrain() -> void:
	drivetrain_controller.reset_clutch_condition()
	drivetrain_controller.force_gear(1) # Neutral

## Prints debug information about the load distribution
func debug_load_distribution():
	print("=== LOAD DISTRIBUTION DEBUG ===")
	var total_load = 0.0
	var wheel_loads = []

	# Assuming you have references to your axles
	var wheels = [axles[0].left_wheel, axles[0].right_wheel,
				  axles[axles.size() - 1].left_wheel, axles[axles.size() - 1].right_wheel]
	var names = ["FL", "FR", "RL", "RR"]

	for i in range(wheels.size()):
		var wheel = wheels[i]
		var p_load = wheel.local_force.y
		wheel_loads.append(p_load)
		total_load += p_load

		var expected_static = (mass * 9.81) / 4.0
		var load_percent = (p_load / expected_static) * 100.0

		print("%s: %.0f N (%.1f%% of static quarter-weight)" % [names[i], p_load, load_percent])

	print("Total: %.0f N vs Expected: %.0f N" % [total_load, mass * 9.81])
	print("Front/Rear: %.1f%% / %.1f%%" % [
		((wheel_loads[0] + wheel_loads[1]) / total_load) * 100.0,
		((wheel_loads[2] + wheel_loads[3]) / total_load) * 100.0
	])
	print("Left/Right: %.1f%% / %.1f%%" % [
		((wheel_loads[0] + wheel_loads[2]) / total_load) * 100.0,
		((wheel_loads[1] + wheel_loads[3]) / total_load) * 100.0
	])
	print("Centrifugal Force: %.2f m/s²" % [linear_velocity.cross(angular_velocity).length()])
	print("==============================")
