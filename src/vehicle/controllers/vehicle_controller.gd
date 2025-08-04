# VehicleBody.gd
extends RigidBody3D
class_name VehicleBody

# --- Components ---
@onready var pedal_controller: PedalController = $PedalController

# --- Vehicle Configuration ---
@export_group("Chassis")
@export var Weight: float = 900.0 # Mass in kilograms (kg)

@export_group("Engine")
@export var torque_curve: Curve # Assign a Curve resource in the Inspector
@export var IdleRPM: float = 800.0
@export var RPMLimit: float = 7000.0
@export var DeadRPM: float = 100.0 # RPM below which the engine is considered stalled
# Note: LimiterDelay is now in seconds, not frames (e.g., 0.07)
@export var LimiterDelay: float = 0.07
@export var ThrottleIdle: float = 0.25
# NOTE: These values will need retuning as they are now used with 'delta'.
@export var RevSpeed: float = 20000.0 # Acts as torque->acceleration multiplier (inverse inertia)
@export var EngineFriction: float = 5e-05 # Coefficient for friction that scales with RPM^2
@export var EngineDrag: float = 1.0 # Coefficient for linear RPM drag
@export var ThrottleResponse: float = 15.0 # Rate of throttle change per second

@export_group("Braking")
@export var ABS_Enabled: bool = true
# Note: ABS_Pump_Time is now in seconds, not frames (e.g., 0.02)
@export var ABS_Pump_Time: float = 0.02
@export var ABS_Pump_Force: float = 25.0 # Rate of brake pressure change per second

# --- Internal Physics State (Class-Level Variables) ---
var throttle: float = 0.0
var rpm: float = 0.0
var rpmforce: float = 0.0 # The net rotational acceleration on the engine (in RPM/sec)
var gear: int = 0
var limdel_timer: float = 0.0 # Timer for the rev limiter

var past_velocity := Vector3.ZERO
var g_force := Vector3.ZERO

var brakeline: float = 0.0
var brake_allowed: float = 1.0
var abs_pump_timer: float = 0.0

const GRAVITY_ACCELERATION = 9.80665


func _ready():
	mass = Weight
	rpm = IdleRPM


# The main physics loop. All time-dependent calculations now use the 'delta' parameter.
func _physics_process(delta: float):
	# 1. UPDATE VEHICLE STATE
	# ----------------------------------------------------------------------
	var local_velocity = global_transform.basis.transposed() * linear_velocity
	var local_angular_velocity = global_transform.basis.transposed() * angular_velocity

	var acceleration = (linear_velocity - past_velocity) / delta
	g_force = acceleration / GRAVITY_ACCELERATION
	past_velocity = linear_velocity

	# 2. CALL SIMULATION SUB-SYSTEMS
	# ----------------------------------------------------------------------
	aero(local_velocity, delta)
	controls(local_velocity, local_angular_velocity, delta)
	transmission(local_velocity, delta)
	limits()

	# 3. PROCESS BRAKING AND THROTTLE LOGIC
	# ----------------------------------------------------------------------
	# ABS pump timer, now using delta.
	abs_pump_timer -= delta
	if abs_pump_timer <= 0:
		brake_allowed += ABS_Pump_Force * delta
	else:
		brake_allowed -= ABS_Pump_Force * delta
	brake_allowed = clamp(brake_allowed, 0.0, 1.0)

	brakeline = pedal_controller.get_brake() * brake_allowed

	# Rev limiter timer, now using delta.
	limdel_timer -= delta

	# Calculate the final throttle value using frame-rate independent interpolation.
	var current_gas_pedal = pedal_controller.get_throttle()
	var target_throttle = current_gas_pedal # Placeholder for TCS factor

	if limdel_timer > 0:
		target_throttle = 0.0 # If limiter is active, force throttle to close.

	throttle = lerp(throttle, target_throttle, ThrottleResponse * delta)

	# Apply engine idle control
	if rpm < IdleRPM:
		if throttle < ThrottleIdle:
			throttle = lerp(throttle, ThrottleIdle, ThrottleResponse * delta)

	# Check if the rev limiter should be activated
	if rpm > RPMLimit and limdel_timer <= 0:
		limdel_timer = LimiterDelay

	# 4. CALCULATE ENGINE TORQUE
	# ----------------------------------------------------------------------
	var engine_torque: float = 0.0
	if torque_curve and rpm > DeadRPM:
		engine_torque = torque_curve.sample(rpm) * throttle

	# 5. SIMULATE ENGINE INERTIA AND FRICTION
	# ----------------------------------------------------------------------
	# This section calculates the net rotational acceleration on the engine ('rpmforce')
	# in RPM per second, making it fully frame-rate independent.

	# Frictional/drag forces that resist engine rotation (in RPM/sec)
	var friction_force = rpm * abs(rpm) * EngineFriction
	var drag_force = rpm * EngineDrag

	# Force from combustion that accelerates the engine (in RPM/sec)
	var torque_force = engine_torque * RevSpeed # RevSpeed is inverse inertia

	# 'rpmforce' is the final rate of change for the RPM.
	rpmforce = torque_force - friction_force - drag_force

	# Apply the change to the RPM over the last frame.
	rpm += rpmforce * delta

	if rpm < 0:
		rpm = 0

	# 6. APPLY FORCES TO DRIVETRAIN
	# ----------------------------------------------------------------------
	# The drivetrain method can now use 'rpmforce' (rate of RPM change)
	# and 'engine_torque' to calculate wheel forces.
	drivetrain(engine_torque, delta)


# --- Placeholder Methods ---

func aero(_local_vel: Vector3, _d: float):
	pass

func controls(_local_vel: Vector3, _local_ang_vel: Vector3, d: float):
	var gas_input = Input.is_action_pressed("gas")
	var brake_input = Input.is_action_pressed("brake")
	var handbrake_input = Input.is_action_pressed("handbrake")
	var clutch_input = Input.is_action_pressed("clutch")

	# The PedalController must also use delta directly.
	# The previously provided PedalController code already does this.
	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, d)
	pass

func transmission(_local_vel: Vector3, _d: float):
	pass

func limits():
	throttle = clamp(throttle, 0.0, 1.0)
	pass

func drivetrain(_torque_from_engine: float, _d: float):
	# This method will use 'torque_from_engine' and can also access 'self.rpm'
	# and 'self.rpmforce' to simulate the load from the wheels on the engine.
	pass