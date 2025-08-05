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

@export_group("Limits")
@export var throttle_limit: float = 0.0

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
@export var max_brake_torque: float = 2000.0
#endregion

#region Internal
var engine_angular_velocity: float = 0.0 # rad/s
var engine_rpm: float = 0.0
var clutch_torque: float = 0.0 # The final torque passed to the wheels
var gear_index: int = 2 # Start in Neutral
var current_gear: int = 0 # For UI/sound display
## Needed for clutch logic in air
var grounded: bool = false
#endregion

@onready var engine_label: Label = %EngineLabel

func _ready():
	mass = weight
	engine_rpm = engine_min_rpm

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
	update_drivetrain(pedal_controller.throttle_pedal, pedal_controller.brake_pedal, gearbox_rpm, false, delta)

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

func update_drivetrain(throttle_input: float, brake_input: float, p_gearbox_rpm: float, is_reverse: bool, delta: float):
	# Convert engine's angular velocity to the more readable RPM format
	engine_rpm = engine_angular_velocity * RADS_TO_RPM

	# Sample the curve to get the max potential torque at the current RPM
	var initial_torque = torque_curve.sample(engine_rpm)
	# Apply the player's throttle input
	initial_torque *= throttle_input

	# Calculate engine friction
	var friction_torque = engine_friction_constant * engine_angular_velocity

	# This line was provided. It's the net torque available to accelerate the engine.
	var engine_effective_torque = initial_torque - friction_torque - clutch_torque

	# Calculate engine acceleration using its inertia
	var engine_acceleration = engine_effective_torque / engine_inertia
	engine_angular_velocity += engine_acceleration * delta

	# Clamp the engine speed between idle and redline
	var engine_min_ang_vel = engine_min_rpm * RPM_TO_RADS
	var engine_max_ang_vel = engine_max_rpm * RPM_TO_RADS
	engine_angular_velocity = clamp(engine_angular_velocity, engine_min_ang_vel, engine_max_ang_vel)

	# --- CLUTCH LOGIC ---
	var clutch_lock = 1.0 # Default to locked for higher gears/speeds

	# Define the RPM range for clutch engagement in low gears
	var engagement_start_rpm = engine_min_rpm # e.g., 800 RPM
	var engagement_end_rpm = engine_min_rpm + 1500.0 # e.g., 2300 RPM

	# In low gears, calculate a smooth engagement factor
	if is_reverse or gear_index <= 2:
		# inverse_lerp calculates how far we are into the range (0.0 to 1.0)
		var engagement_factor = inverse_lerp(engagement_start_rpm, engagement_end_rpm, engine_rpm)
		clutch_lock = clamp(engagement_factor, 0.0, 1.0)

	# Disengage clutch if player is braking (arcade helper)
	if brake_input > 0.5:
		clutch_lock = 0.0

	# Disengage clutch if vehicle is airborn
	if not grounded:
		clutch_lock = 0.0

	# Calculate the speed difference between the engine and the drivetrain
	var clutch_slip_velocity = engine_angular_velocity - (p_gearbox_rpm * RPM_TO_RADS)

	# Calculate the maximum torque the clutch can handle
	# The curve's max value is a good proxy for engineTorqueMax
	var clutch_torque_max = torque_curve.get_max_value() * clutch_capacity * clutch_lock

	# Calculate the torque transferred by the clutch based on slip and stiffness
	var clutch_torque_next = clamp(clutch_slip_velocity * clutch_stiffness, -clutch_torque_max, clutch_torque_max)

	clutch_torque_next *= clutch_lock

	# Smooth the clutch torque to avoid jerky behavior (a simple low-pass filter)
	clutch_torque_next = lerp(clutch_torque, clutch_torque_next, 0.5)
	clutch_torque = clutch_torque_next

	# --- GEAR DISPLAY LOGIC ---
	current_gear = gear_index - 1 # Convert array index to gear number (N=0, 1st=1)
	if is_reverse:
		current_gear = -1
	if clutch_lock == 0.0:
		current_gear = 0 # Show "Neutral" if clutch is disengaged
