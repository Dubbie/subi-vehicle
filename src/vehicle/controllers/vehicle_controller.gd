class_name VehicleController
extends RigidBody3D

const RADS_TO_RPM = 60.0 / (2.0 * PI)
const RPM_TO_RADS = (2.0 * PI) / 60.0

#region Export Variables
# --- Components ---
@export var pedal_controller: PedalController
@export var clutch_controller: ClutchController
@export var steering_controller: SteeringController
@export var transmission_controller: TransmissionController

@export_group("Debug")
## Number of iterations to solve the drivetrain per frame.
@export var physics_sub_steps: int = 8
@export var debug_mode: bool = true:
	set(value):
		debug_mode = value
		for axle in axles:
			axle.left_wheel.debug_mode = value
			axle.right_wheel.debug_mode = value

# --- Vehicle Configuration ---
@export_group("Chassis")
## The mass of the car.
@export var weight: float = 1250.0 # kg
## The axles of the car. First axle is always the steered one currently.
@export var axles: Array[AxleController] = []
## The turn diameter of the car.
@export var turn_diameter: float = 10.4 # m
## The maximum steer angle in degrees. This is how much the inner tire will rotate at full steer.
@export var max_steer_angle: float = 38.0

@export_group("Engine")
## The torque curve. X = RPM, Y = Torque
@export var torque_curve: Curve
## Flywheel inertia basically.
@export var engine_inertia: float = 0.3
## Internal friction of the engine, causing drag.
@export var engine_friction_torque: float = 15.0
## Idle RPM of the engine.
@export var engine_min_rpm: float = 800.0
## The maximum engine RPM.
@export var engine_max_rpm: float = 7000.0

@export_group("Transmission")
## Gear ratios are defined in the order they appear in the array.
## R, N, 1, 2, 3, 4, 5...
@export var gear_ratios: Array[float] = [-2.9, 0.0, 2.66, 1.78, 1.3, 0.9]

@export_group("Clutch")
@export var clutch_stiffness: float = 250.0
@export var clutch_capacity: float = 1.2
@export var clutch_smoothing_factor: float = 0.05

@export_subgroup("Smart Assists")
@export var anti_stall_enabled: bool = true
@export var launch_assist_enabled: bool = true
@export var auto_clutch_enabled: bool = true

@export_subgroup("Anti-Stall")
@export var stall_rpm_threshold: float = 1200.0
@export var anti_stall_engagement_speed: float = 4.0

@export_subgroup("Launch Assist")
@export var launch_assist_rpm_threshold: float = 2000.0
@export var launch_assist_slip_target: float = 200.0

@export_group("Brakes")
@export var max_brake_torque: float = 4000.0
@export var max_handbrake_torque: float = 2000.0
#endregion

#region Internal
var _driven_axles: Array[AxleController] = []
var engine_angular_velocity: float = 0.0 # rad/s
var engine_torque: float = 0.0
var engine_rpm: float = 0.0
var clutch_torque: float = 0.0 # The final torque passed to the wheels
var current_gear: int = 0 # For UI/sound display
## Needed for clutch logic in air
var grounded: bool = false
## Calculated wheelbase from the axles
var wheelbase: float = 0.0 # m
## Min turn radius calculated from axle setup
var min_turn_radius: float = 0.0
var last_clutch_slip_velocity: float = 0.0
var vehicle_speed: float = 0.0
var restrict_gas: bool = false
#endregion

@onready var engine_label: Label = %EngineLabel

func _ready():
	mass = weight
	engine_rpm = engine_min_rpm

	# Initialize clutch controller
	clutch_controller.set_clutch_parameters(clutch_stiffness, clutch_capacity, clutch_smoothing_factor)
	clutch_controller.set_anti_stall_enabled(anti_stall_enabled)
	clutch_controller.set_launch_assist_enabled(launch_assist_enabled)
	clutch_controller.set_auto_clutch_enabled(auto_clutch_enabled)

	_setup_axles()

	transmission_controller.gear_ratios = gear_ratios
	if _driven_axles.size() > 0:
		transmission_controller.initialize(_driven_axles[0].left_wheel.wheel_radius, _driven_axles[0].diff_ratio)

	transmission_controller.shift_requested.connect(_on_shift_requested.unbind(1))
	transmission_controller.gear_changed.connect(_on_gear_changed.unbind(2))

func _process(_delta: float) -> void:
	if debug_mode and not engine_label.visible:
		engine_label.visible = true
	elif not debug_mode and engine_label.visible:
		engine_label.visible = false

	if not debug_mode: return

	# Get clutch info for debugging
	var clutch_info = clutch_controller.get_clutch_info()
	var rpm_string: String = "RPM: %.0f" % engine_rpm
	var clutch_string: String = "\nClutch: %.1f%% (%.1f Nm)" % [clutch_info.engagement * 100, clutch_info.torque]
	var slip_string: String = "\nSlip: %.0f RPM" % clutch_info.slip_rpm
	var temp_string: String = "\nTemp: %.0f°C" % clutch_info.temperature

	var mode_info: String = ""
	if clutch_info.auto_clutch_mode:
		mode_info += "\n[AUTO-CLUTCH]"
		if clutch_info.waiting_for_rpm:
			if not grounded:
				mode_info += " AIRBORNE"
			else:
				mode_info += " WAITING RPM"

	var assist_info: String = ""
	if clutch_info.anti_stall_active:
		assist_info += "\n[ANTI-STALL]"
	if clutch_info.launch_assist_active:
		assist_info += "\n[LAUNCH ASSIST]"

	var gear_string: String = "\nGear: %d" % transmission_controller.get_current_gear()
	var speed_string: String = "\nSpeed: %.1f m/s (%.1f km/h)" % [vehicle_speed, vehicle_speed * 3.6]
	var ground_string: String = "\nGrounded: %s" % ("YES" if grounded else "NO")
	var debug_string: String = "\nDebug: %s" % clutch_info.debug_state

	engine_label.text = rpm_string + clutch_string + slip_string + temp_string + gear_string + speed_string + ground_string + mode_info + assist_info + debug_string

#region Physics
func _physics_process(delta: float):
	if not pedal_controller or not steering_controller:
		push_error("Controllers not set.")
		set_physics_process(false)
		return

	# --- 1. Process User Controls ---
	_controls(delta)

	# --- 2. Calculate vehicle speed (horizontal only) ---
	var horizontal_velocity = Vector3(linear_velocity.x, 0, linear_velocity.z)
	vehicle_speed = horizontal_velocity.length()

	# --- 3. Per-Axle Logic (Steering and Grounded Check) ---
	grounded = false
	for i in range(axles.size()):
		var axle: AxleController = axles[i]
		if i == 0:
			axle.set_steer_value(steering_controller.get_steer_value())
		axle.update_wheel_states(delta)
		if axle.left_wheel.has_contact or axle.right_wheel.has_contact:
			grounded = true

	# --- 4. Update Transmission ---
	var new_gear_index = transmission_controller.update_transmission(
		vehicle_speed,
		pedal_controller.throttle_pedal,
		pedal_controller.brake_pedal,
		grounded,
		delta
	)

	# --- 5. Iterative Drivetrain Solver ---
	var sub_step_delta = delta / physics_sub_steps
	for i in range(physics_sub_steps):
		# --- A. Calculate Gearbox RPM from current wheel state ---
		var gearbox_rpm: float = 0.0
		var gearbox_angular_velocity: float = 0.0
		var current_gear_ratio = transmission_controller.get_current_gear_ratio()

		if _driven_axles.size() > 0:
			var total_gearbox_rpm: float = 0.0
			for axle in _driven_axles:
				var avg_wheel_rpm = (axle.left_wheel.current_angular_velocity + axle.right_wheel.current_angular_velocity) / 2.0 * RADS_TO_RPM
				total_gearbox_rpm += avg_wheel_rpm * current_gear_ratio * axle.diff_ratio
			gearbox_rpm = total_gearbox_rpm / _driven_axles.size()
			gearbox_angular_velocity = gearbox_rpm * RPM_TO_RADS

		# --- B. Update Engine ---
		var initial_torque = torque_curve.sample_baked(engine_rpm) * pedal_controller.throttle_pedal
		var max_engine_torque: float = torque_curve.max_value

		# --- C. Update Clutch with realistic simulation ---
		clutch_torque = clutch_controller.update_clutch(
			transmission_controller,
			engine_rpm,
			engine_angular_velocity,
			gearbox_angular_velocity,
			max_engine_torque,
			pedal_controller.clutch_pedal,
			pedal_controller.throttle_pedal,
			new_gear_index,
			vehicle_speed,
			grounded,
			sub_step_delta
		)

		# Update engine with clutch torque
		var engine_effective_torque = initial_torque - engine_friction_torque - clutch_torque
		engine_torque = engine_effective_torque
		var engine_acceleration = (engine_effective_torque / engine_inertia) * sub_step_delta
		engine_angular_velocity += engine_acceleration

		var engine_min_angular_velocity = engine_min_rpm * RPM_TO_RADS
		var engine_max_angular_velocity = engine_max_rpm * RPM_TO_RADS
		engine_angular_velocity = clampf(engine_angular_velocity, engine_min_angular_velocity, engine_max_angular_velocity)
		engine_rpm = engine_angular_velocity * RADS_TO_RPM

		# --- D. Update Wheels with new torque for this sub-step ---
		var final_brake_torque: float = pedal_controller.brake_pedal * max_brake_torque
		for axle in axles:
			var axle_torque: float = clutch_torque * current_gear_ratio * axle.diff_ratio
			var drive_torque_per_wheel: float = (axle_torque / 2.0) if axle.drive_ratio > 0.0 else 0.0
			var brake_torque_per_wheel: float = final_brake_torque * axle.brake_ratio

			axle.left_wheel.calculate_wheel_physics(drive_torque_per_wheel, brake_torque_per_wheel, 1.0, sub_step_delta)
			axle.right_wheel.calculate_wheel_physics(drive_torque_per_wheel, brake_torque_per_wheel, 1.0, sub_step_delta)

	# --- 6. Apply Final Forces to Rigidbody ---
	for axle in axles:
		axle.left_wheel.apply_forces_to_rigidbody()
		axle.right_wheel.apply_forces_to_rigidbody()
#endregion

#region Private methods
func _controls(d: float):
	var gas_input = Input.is_action_pressed("gas") and not restrict_gas
	var brake_input = Input.is_action_pressed("brake")
	var handbrake_input = Input.is_action_pressed("handbrake")
	var clutch_input = Input.is_action_pressed("clutch")
	var steer_input = Input.get_axis("steer_right", "steer_left")

	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, d)
	steering_controller.process_inputs(steer_input, d)

func _setup_axles() -> void:
	if axles.size() < 2:
		push_error("Vehicle requires at least 2 axles.")
		return

	# Assuming front axle is at index 0
	var front_axle = axles[0]
	var rear_axle = axles[axles.size() - 1]

	wheelbase = abs(front_axle.z_offset - rear_axle.z_offset)
	if wheelbase < 0.01:
		push_error("Wheelbase is nearly zero.")
		return

	# 1. Convert the physically-defined MAX INNER wheel angle to radians.
	var max_inner_wheel_angle_rad = deg_to_rad(max_steer_angle)

	if abs(tan(max_inner_wheel_angle_rad)) < 0.0001:
		min_turn_radius = 10000 # Avoid division by zero, effectively straight
	else:
		# 2. Calculate the turning radius of the INNER WHEEL at full lock.
		var min_radius_at_inner_wheel = wheelbase / tan(max_inner_wheel_angle_rad)

		# 3. The vehicle's minimum turning radius is measured from the center,
		#    so we add half the track width of the front axle.
		min_turn_radius = min_radius_at_inner_wheel + (front_axle.track_width / 2.0)

	# Update the turn_diameter for debugging/UI purposes
	turn_diameter = min_turn_radius * 2.0
	print("--- Vehicle Setup ---")
	print("Calculated Wheelbase: ", wheelbase, " m")
	print("Defined Max Steer Angle: ", max_steer_angle, " deg")
	print("--------------------")

	# Initialize the axles
	for axle in axles:
		# Check if we have to add the wheels to driven wheels
		if axle.drive_ratio > 0.0:
			_driven_axles.append(axle)
			axle.left_wheel.driven_wheel = true
			axle.right_wheel.driven_wheel = true

		var max_steer: float = max_steer_angle if axle == front_axle else 0.0
		axle.initialize(wheelbase, max_steer)

func _on_shift_requested() -> void:
	restrict_gas = true

func _on_gear_changed() -> void:
	restrict_gas = false
