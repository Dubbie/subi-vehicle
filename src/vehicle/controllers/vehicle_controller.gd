class_name VehicleController
extends RigidBody3D

const RPM_TO_RADS = PI / 30.0
const RADS_TO_RPM = 30.0 / PI

#region Export Variables
# --- Components ---
@export var pedal_controller: PedalController
@export var steering_controller: SteeringController

@export_group("Debug")
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
@export var engine_min_rpm: float = 800.0
@export var engine_max_rpm: float = 7000.0
@export var engine_friction_constant: float = 0.01 # Scales friction with RPM

@export_group("Transmission")
@export var gear_ratios: Array[float] = [-2.9, 0.0, 2.66, 1.78, 1.3, 0.9]

@export_group("Clutch")
@export var clutch_stiffness: float = 10.0 # How sharply the clutch "bites"
@export var clutch_capacity: float = 1.2 # Multiplier for max torque transfer

@export_group("Brakes")
@export var max_brake_torque: float = 4000.0
@export var max_handbrake_torque: float = 2000.0
#endregion

#region Internal
var _driven_axles: Array[AxleController] = []
var engine_angular_velocity: float = 0.0 # rad/s
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
	if not pedal_controller:
		push_error("Pedal controller is not set.")
		set_physics_process(false)
		return

	if not steering_controller:
		push_error("Steering controller is not set.")
		set_physics_process(false)
		return

	_controls(delta)

	var gearbox_rpm: float = 0.0
	var current_gear_ratio = gear_ratios[gear_index]
	for axle in _driven_axles:
		var left_wheel_rpm: float = axle.left_wheel.current_angular_velocity * RADS_TO_RPM
		var right_wheel_rpm: float = axle.right_wheel.current_angular_velocity * RADS_TO_RPM
		var average_rpm: float = (left_wheel_rpm + right_wheel_rpm) / 2.0
		var average_rpm_after_ratios: float = average_rpm * current_gear_ratio * axle.diff_ratio
		gearbox_rpm += average_rpm_after_ratios

	# Steering logic
	grounded = false
	for i in range(axles.size()):
		var axle: AxleController = axles[i]

		# First axle gets steered for now
		if i == 0:
			axle.set_steer_value(steering_controller.get_steer_value())

		# Update the states, which handles the steering as well.
		axle.update_wheel_states(delta)

		# Update the grounded state
		if axle.left_wheel.has_contact or axle.right_wheel.has_contact:
			grounded = true

	# Update the drivetrain simulation
	_update_drivetrain(gearbox_rpm, false, delta)

	var final_brake_torque: float = pedal_controller.brake_pedal * max_brake_torque

	for axle in axles:
		var axle_torque: float = clutch_torque * current_gear_ratio * axle.diff_ratio
		var drive_torque_per_wheel: float = axle_torque / 2.0
		var brake_torque_per_wheel: float = final_brake_torque * axle.brake_ratio

		axle.left_wheel.calculate_wheel_physics(drive_torque_per_wheel, brake_torque_per_wheel, 1.0, delta)
		axle.right_wheel.calculate_wheel_physics(drive_torque_per_wheel, brake_torque_per_wheel, 1.0, delta)

		axle.left_wheel.apply_forces_to_rigidbody()
		axle.right_wheel.apply_forces_to_rigidbody()

func _update_drivetrain(p_gearbox_rpm: float, is_reverse: bool, delta: float):
	# --- GET INPUTS FROM PEDAL CONTROLLER ---
	var throttle_input = pedal_controller.get_throttle()

	# --- 1. ENGINE PHYSICS (Unchanged) ---
	engine_rpm = engine_angular_velocity * RADS_TO_RPM

	var initial_torque = torque_curve.sample(engine_rpm) * throttle_input
	var friction_torque = engine_friction_constant * engine_angular_velocity
	var engine_effective_torque = initial_torque - friction_torque - clutch_torque

	var engine_acceleration = engine_effective_torque / engine_inertia
	engine_angular_velocity += engine_acceleration * delta

	var engine_min_ang_vel = engine_min_rpm * RPM_TO_RADS
	var engine_max_ang_vel = engine_max_rpm * RPM_TO_RADS
	engine_angular_velocity = clamp(engine_angular_velocity, engine_min_ang_vel, engine_max_ang_vel)

	# --- 2. CLUTCH LOGIC ---
	# This new function calculates the final clutch engagement value based on all factors.
	var final_clutch_engagement = _get_final_clutch_engagement(throttle_input)

	# The engagement value is passed through the curve to get the actual lock factor.
	var clutch_lock = clutch_input_curve.sample_baked(final_clutch_engagement)

	# --- 3. CLUTCH PHYSICS ---
	var clutch_slip_velocity = engine_angular_velocity - (p_gearbox_rpm * RPM_TO_RADS)
	var clutch_torque_max = torque_curve.get_max_value() * clutch_capacity * clutch_lock
	var clutch_torque_next = clamp(clutch_slip_velocity * clutch_stiffness, -clutch_torque_max, clutch_torque_max)

	clutch_torque = lerp(clutch_torque, clutch_torque_next, 0.5)

	# --- 4. GEAR DISPLAY LOGIC ---
	current_gear = gear_index - 1
	if is_reverse:
		current_gear = -1
	if clutch_lock < 0.1:
		current_gear = 0

func _controls(d: float):
	var gas_input = Input.is_action_pressed("gas")
	var brake_input = Input.is_action_pressed("brake")
	var handbrake_input = Input.is_action_pressed("handbrake")
	var clutch_input = Input.is_action_pressed("clutch")
	var steer_input = Input.get_axis("steer_right", "steer_left")

	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, d)
	steering_controller.process_inputs(steer_input, d)

func _get_final_clutch_engagement(throttle: float) -> float:
	# Get manual input from the player's foot
	var manual_clutch_pedal = pedal_controller.get_clutch()

	# Priority 1: Manual Override. If the player is pressing the clutch, they are in control.
	if manual_clutch_pedal > 0.01:
		# get_clutch() is 1.0 when pressed, so we invert it for engagement.
		return 1.0 - manual_clutch_pedal

	# If no manual input, the "Driver AI" takes over.
	var car_speed = abs(linear_velocity.dot(basis.z))

	# Priority 2: Airborne check
	if not grounded:
		return 0.0 # Disengage

	# Priority 3: Launch Assist
	# Check for low speed and throttle application.
	if car_speed < launch_assist_max_speed and throttle > 0.01:
		# Slip the clutch based on throttle input for a smooth launch.
		return clamp(throttle * launch_assist_factor, 0.0, 1.0)

	# Priority 4: Anti-Stall
	# Check for coasting to a stop.
	if car_speed < 5.0 and throttle < 0.01:
		return 0.0 # Disengage

	# Default State: If none of the above conditions are met, the clutch should be fully engaged.
	return 1.0
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
