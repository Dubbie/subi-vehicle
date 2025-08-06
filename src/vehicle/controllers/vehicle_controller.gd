class_name VehicleController
extends RigidBody3D

const RPM_TO_RADS = PI / 30.0
const RADS_TO_RPM = 30.0 / PI

#region Ex.Variables
# --- Components ---
@export var pedal_controller: PedalController

@export_group("Debug")
@export var debug_mode: bool = true:
	set(value):
		debug_mode = value
		for wheel in wheels:
			wheel.debug_mode = value

# --- Vehicle Configuration ---
@export_group("Chassis")
@export var weight: float = 1250.0 # Mass in kilograms (kg)
@export var wheels: Array[WheelController] = []
@export var driven_wheels: Array[WheelController] = []
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

@export_group("Drivetrain")
@export_group("Transmission")
@export var gear_ratios: Array[float] = [-2.9, 0.0, 2.66, 1.78, 1.3, 0.9]
@export var final_drive_ratio: float = 3.42 # The differential

@export_group("Clutch")
@export var clutch_stiffness: float = 10.0 # How sharply the clutch "bites"
@export var clutch_capacity: float = 1.2 # Multiplier for max torque transfer

@export_group("Brakes")
@export var max_brake_torque: float = 4000.0
@export var max_handbrake_torque: float = 2000.0
#endregion

#region Internal
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


# The main physics loop. All time-dependent calculations now use the 'delta' parameter.
func _physics_process(delta: float):
	controls(delta)

	var average_drive_wheel_rpm: float = 0.0
	for wheel in driven_wheels:
		average_drive_wheel_rpm += wheel.current_angular_velocity * RADS_TO_RPM
		wheel.driven_wheel = true
	average_drive_wheel_rpm /= driven_wheels.size()

	grounded = false
	for wheel in wheels:
		wheel.update_state(0.0, delta)

		if wheel.has_contact and not grounded:
			grounded = true

	# Calculate gearbox RPM based on wheel speed and gear ratios
	var current_gear_ratio = gear_ratios[gear_index]
	var gearbox_rpm = average_drive_wheel_rpm * current_gear_ratio * final_drive_ratio

	# Update the drivetrain simulation
	update_drivetrain(gearbox_rpm, false, delta)

	var final_brake_torque: float = pedal_controller.brake_pedal * max_brake_torque

	var axle_torque: float = clutch_torque * current_gear_ratio * final_drive_ratio
	var drive_torque_per_wheel: float = axle_torque / driven_wheels.size()

	for wheel in wheels:
		var drive_torque: float = 0.0
		if wheel in driven_wheels:
			drive_torque = drive_torque_per_wheel

		wheel.calculate_wheel_physics(drive_torque, final_brake_torque, 1.0, delta)
		wheel.apply_forces_to_rigidbody()

func controls(d: float):
	var gas_input = Input.is_action_pressed("gas")
	var brake_input = Input.is_action_pressed("brake")
	var handbrake_input = Input.is_action_pressed("handbrake")
	var clutch_input = Input.is_action_pressed("clutch")

	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, d)

func update_drivetrain(p_gearbox_rpm: float, is_reverse: bool, delta: float):
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

	if randf() < 0.1:
		print("Clutch lock: ", clutch_lock)

	# --- 3. CLUTCH PHYSICS (Unchanged) ---
	var clutch_slip_velocity = engine_angular_velocity - (p_gearbox_rpm * RPM_TO_RADS)
	var clutch_torque_max = torque_curve.get_max_value() * clutch_capacity * clutch_lock
	var clutch_torque_next = clamp(clutch_slip_velocity * clutch_stiffness, -clutch_torque_max, clutch_torque_max)

	clutch_torque = lerp(clutch_torque, clutch_torque_next, 0.5)

	# --- 4. GEAR DISPLAY LOGIC (Unchanged) ---
	current_gear = gear_index - 1
	if is_reverse:
		current_gear = -1
	if clutch_lock < 0.1:
		current_gear = 0

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

func _setup_axles() -> void:
	# --- 1. VALIDATE AXLE SETUP ---
	# We need at least a front and a rear axle to define a wheelbase.
	if axles.size() < 2:
		push_error("Vehicle requires at least 2 axles to calculate wheelbase.")
		return

	# The front axle is always the first.
	var front_axle_z = axles[0].z_offset
	var rear_axle_z = axles[axles.size() - 1].z_offset

	# The distance between the front and rear axles.
	wheelbase = abs(front_axle_z - rear_axle_z)

	if wheelbase < 0.01:
		push_error("Wheelbase is nearly zero. Check axle z_offsets.")
		return

	# Calculate the tightest turning radius based on physics
	var max_steer_rad = deg_to_rad(max_steer_angle)
	if abs(tan(max_steer_rad)) < 0.0001:
		min_turn_radius = 10000 # Effectively straight
	else:
		min_turn_radius = wheelbase / tan(max_steer_rad)

	# Update the turn_diameter for debugging/UI purposes
	turn_diameter = min_turn_radius * 2.0
	print("Calculated Wheelbase: ", wheelbase, " m")
	print("Calculated Min Turn Radius: ", min_turn_radius, " m")
	print("Calculated Turn Diameter: ", turn_diameter, " m")

	# Initialize the axles
	for axle in axles:
		axle.initialize(wheelbase, min_turn_radius)
