class_name VehicleController
extends RigidBody3D

const RADS_TO_RPM = 60.0 / (2.0 * PI)
const RPM_TO_RADS = (2.0 * PI) / 60.0

#region Export Variables
# --- Components ---
@export var pedal_controller: PedalController
@export var steering_controller: SteeringController

@export_group("Debug")
@export var physics_sub_steps: int = 8 # Number of iterations to solve the drivetrain per frame.
@export var debug_mode: bool = true:
	set(value):
		debug_mode = value
		for axle in axles:
			axle.left_wheel.debug_mode = value
			axle.right_wheel.debug_mode = value

# --- Vehicle Configuration ---
@export_group("Chassis")
@export var weight: float = 1250.0 # Mass in kilograms (kg)
@export var axles: Array[AxleController] = []
@export var turn_diameter: float = 10.4 # m
@export var max_steer_angle: float = 38.0 # degrees

@export_group("Clutch Configuration")
@export var clutch_input_curve: Curve
@export var launch_assist_max_speed: float = 5.0
@export var launch_assist_factor: float = 1.3

@export_group("Engine")
@export var torque_curve: Curve # Assign a Curve resource in the Inspector
@export var engine_inertia: float = 0.3
@export var engine_friction_torque: float = 15.0
@export var engine_min_rpm: float = 800.0
@export var engine_max_rpm: float = 7000.0

@export_group("Transmission")
@export var gear_ratios: Array[float] = [-2.9, 0.0, 2.66, 1.78, 1.3, 0.9]

@export_group("Clutch")
@export var clutch_stiffness: float = 250.0 # How sharply the clutch engages
@export var clutch_capacity: float = 1.2 # Multiplier for max torque transfer.
## This factor, derived from the original code's 0.95f, controls the smoothing.
## A value of 0.05 means the torque moves 5% towards the new target each frame.
## This is the primary tuning value for preventing oscillation.
@export var clutch_smoothing_factor: float = 0.05

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
var gear_index: int = 2 # Start in Neutral
var current_gear: int = 0 # For UI/sound display
## Needed for clutch logic in air
var grounded: bool = false
## Calculated wheelbase from the axles
var wheelbase: float = 0.0 # m
## Min turn radius calculated from axle setup
var min_turn_radius: float = 0.0
var last_clutch_slip_velocity: float = 0.0
#endregion

@onready var engine_label: Label = %EngineLabel

func _ready():
	mass = weight
	engine_rpm = engine_min_rpm

	_setup_axles()

func _process(_delta: float) -> void:
	if debug_mode and not engine_label.visible:
		engine_label.visible = true
	elif not debug_mode and engine_label.visible:
		engine_label.visible = false

	if not debug_mode: return

	var rpm_string: String = "RPM: %.0f" % engine_rpm
	engine_label.text = rpm_string

#region Physics
func _physics_process(delta: float):
	if not pedal_controller or not steering_controller:
		push_error("Controllers not set.")
		set_physics_process(false)
		return

	# --- 1. Process User Controls ---
	_controls(delta)

	# --- 2. Per-Axle Logic (Steering and Grounded Check) ---
	grounded = false
	for i in range(axles.size()):
		var axle: AxleController = axles[i]
		# First axle gets steered
		if i == 0:
			axle.set_steer_value(steering_controller.get_steer_value())
		axle.update_wheel_states(delta) # Handles raycasts, suspension, etc.
		if axle.left_wheel.has_contact or axle.right_wheel.has_contact:
			grounded = true

	# --- 3. Iterative Drivetrain Solver ---
	# We run the simulation in sub-steps to allow the engine and wheels to converge
	# on a stable state within a single physics frame, eliminating the 1-frame lag.
	var sub_step_delta = delta / physics_sub_steps
	for i in range(physics_sub_steps):
		# --- A. Calculate Gearbox RPM from current wheel state ---
		var gearbox_rpm: float = 0.0
		var current_gear_ratio = gear_ratios[gear_index]
		if _driven_axles.size() > 0:
			var total_gearbox_rpm: float = 0.0
			for axle in _driven_axles:
				var avg_wheel_rpm = (axle.left_wheel.current_angular_velocity + axle.right_wheel.current_angular_velocity) / 2.0 * RADS_TO_RPM
				total_gearbox_rpm += avg_wheel_rpm * current_gear_ratio * axle.diff_ratio
			gearbox_rpm = total_gearbox_rpm / _driven_axles.size()

		# --- B. Update Engine ---
		var initial_torque = torque_curve.sample_baked(engine_rpm) * pedal_controller.throttle_pedal
		var max_engine_torque: float = torque_curve.max_value
		var engine_effective_torque = initial_torque - engine_friction_torque - clutch_torque
		engine_torque = engine_effective_torque
		var engine_acceleration = (engine_effective_torque / engine_inertia) * sub_step_delta
		engine_angular_velocity += engine_acceleration

		var engine_min_angular_velocity = engine_min_rpm * RPM_TO_RADS
		var engine_max_angular_velocity = engine_max_rpm * RPM_TO_RADS
		engine_angular_velocity = clampf(engine_angular_velocity, engine_min_angular_velocity, engine_max_angular_velocity)
		engine_rpm = engine_angular_velocity * RADS_TO_RPM

		# --- C. Update Clutch ---
		var clutch_lock = 1.0
		if (gear_index <= 2) and engine_rpm < (engine_min_rpm + 1500.0): # Assuming gear 1 is Reverse
			clutch_lock = 0.0
		if pedal_controller.brake_pedal > 0.5:
			clutch_lock = 0.0

		var clutch_slip_velocity = engine_angular_velocity - (gearbox_rpm * RPM_TO_RADS)
		var clutch_torque_max = max_engine_torque * clutch_capacity * clutch_lock
		var clutch_torque_target = clampf(clutch_slip_velocity * clutch_stiffness, -clutch_torque_max, clutch_torque_max)
		clutch_torque = lerp(clutch_torque, clutch_torque_target, clutch_smoothing_factor)

		# --- D. Update Wheels with new torque for this sub-step ---
		var final_brake_torque: float = pedal_controller.brake_pedal * max_brake_torque
		for axle in axles:
			var axle_torque: float = clutch_torque * current_gear_ratio * axle.diff_ratio
			var drive_torque_per_wheel: float = (axle_torque / 2.0) if axle.drive_ratio > 0.0 else 0.0
			var brake_torque_per_wheel: float = final_brake_torque * axle.brake_ratio

			axle.left_wheel.calculate_wheel_physics(drive_torque_per_wheel, brake_torque_per_wheel, 1.0, sub_step_delta)
			axle.right_wheel.calculate_wheel_physics(drive_torque_per_wheel, brake_torque_per_wheel, 1.0, sub_step_delta)

	# --- 4. Apply Final Forces to Rigidbody ---
	# The forces have been accumulating in the wheel scripts during the sub-steps.
	# Now we apply the single, final, combined force to the rigidbody.
	for axle in axles:
		axle.left_wheel.apply_forces_to_rigidbody()
		axle.right_wheel.apply_forces_to_rigidbody()

func _controls(d: float):
	var gas_input = Input.is_action_pressed("gas")
	var brake_input = Input.is_action_pressed("brake")
	var handbrake_input = Input.is_action_pressed("handbrake")
	var clutch_input = Input.is_action_pressed("clutch")
	var steer_input = Input.get_axis("steer_right", "steer_left")

	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, d)
	steering_controller.process_inputs(steer_input, d)
#endregion

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
