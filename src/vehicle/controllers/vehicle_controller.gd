# VehicleBody.gd
extends RigidBody3D
class_name VehicleBody

# --- Components ---
@export var pedal_controller: PedalController

# --- Vehicle Configuration ---
@export_group("Chassis")
@export var Weight: float = 900.0 # Mass in kilograms (kg)

@export_group("Limits")
@export var throttle_limit: float = 0.0

@export_group("Engine")
@export var torque_curve: Curve # Assign a Curve resource in the Inspector
@export var idle_rpm: float = 800.0
@export var rpm_limit: float = 7000.0
@export var dead_rpm: float = 100.0 # RPM below which the engine is considered stalled
# Note: limiter_delay is now in seconds, not frames (e.g., 0.07)
@export var limiter_delay: int = 12
@export var throttle_idle: float = 0.25
# NOTE: These values will need retuning as they are now used with 'delta'.
@export var rev_speed: float = 20000.0 # Acts as torque->acceleration multiplier (inverse inertia)
@export var engine_friction: float = 18000.0 # Coefficient for friction that scales with RPM^2
@export var engine_drag: float = 1.0 # Coefficient for linear RPM drag
@export var throttle_response: float = 15.0 # Rate of throttle change per second

@export_group("Braking")
@export var ABS_Enabled: bool = true
# Note: ABS_Pump_Time is now in seconds, not frames (e.g., 0.02)
@export var ABS_Pump_Time: float = 0.02
@export var ABS_Pump_Force: float = 25.0 # Rate of brake pressure change per second

@export_group("Drivetrain")
@export var driveshaft_inertia_factor: float = 150.0 # Corresponds to DSWeight
@export var differential_lock_factor: float = 0.1
@export var powered_wheels: Array[WheelController]

@export_group("Clutch")
@export var clutch_grip_torque: float = 400.0
@export var clutch_stability_factor: float = 0.5

# --- Internal Physics State (Class-Level Variables) ---
var throttle: float = 0.0
var rpm: float = 0.0
var rpmforce: float = 0.0 # The net rotational acceleration on the engine (in RPM/sec)
var gear: int = 0
var drivetrain_resistance: float = 0.0 # Total resistance from wheels fed back to the engine.
var clutch_engagement: float = 0.0
var ratio: float = 0.0
var stalled: float = 0.0

var local_velocity: Vector3 = Vector3.ZERO
var local_angular_velocity: Vector3 = Vector3.ZERO

var limdel: int = 0 # Timer for the rev limiter
var sassistdel: int = 0 # Timer for the steering assist

var past_velocity: Vector3 = Vector3.ZERO
var g_force: Vector3 = Vector3.ZERO

var brakeline: float = 0.0
var brake_allowed: float = 1.0
var abs_pump_timer: float = 0.0

const GRAVITY_ACCELERATION: float = 9.80665


func _ready():
	mass = Weight
	rpm = idle_rpm

# The main physics loop. All time-dependent calculations now use the 'delta' parameter.
func _physics_process(delta: float):
	# 1. UPDATE VEHICLE STATE
	# ----------------------------------------------------------------------
	local_velocity = global_transform.basis.transposed() * linear_velocity
	local_angular_velocity = global_transform.basis.transposed() * angular_velocity

	# 2. CALL SIMULATION SUB-SYSTEMS
	# ----------------------------------------------------------------------
	aero()

	var acceleration = (linear_velocity - past_velocity) / delta
	g_force = acceleration / GRAVITY_ACCELERATION
	past_velocity = linear_velocity

	controls(delta)

	ratio = 10.0
	sassistdel -= 1

	transmission()
	limits()

	# Steering stuff

	# ABS pump timer, now using delta.
	abs_pump_timer -= delta
	if abs_pump_timer <= 0:
		brake_allowed += ABS_Pump_Force * delta
	else:
		brake_allowed -= ABS_Pump_Force * delta
	brake_allowed = clamp(brake_allowed, 0.0, 1.0)

	brakeline = pedal_controller.get_brake() * brake_allowed

	limdel -= 1

	# Calculate the final throttle value using frame-rate independent interpolation.
	var current_gas_pedal = pedal_controller.get_throttle()
	var target_throttle = current_gas_pedal # Placeholder for TCS factor

	if limdel < 0:
		throttle -= (throttle - (current_gas_pedal / (target_throttle * clutch_engagement + 1.0))) * throttle_response
	else:
		throttle -= throttle * throttle_response

	if rpm > rpm_limit:
		if throttle > throttle_limit:
			throttle = throttle_limit
			limdel = limiter_delay
	elif rpm < idle_rpm:
		if throttle < throttle_idle:
			throttle = throttle_idle

	# 4. CALCULATE ENGINE TORQUE
	# ----------------------------------------------------------------------
	var torque: float = 0.0
	if torque_curve and rpm > dead_rpm:
		torque = torque_curve.sample(rpm) * throttle

	rpmforce = (rpm / (abs(rpm * abs(rpm)) / (engine_friction) + 1.0))
	if rpm < dead_rpm:
		torque = 0.0
		rpmforce /= 5.0
		stalled = 1.0 - rpm / dead_rpm
	else:
		stalled = 0.0

	rpmforce += (rpm * (engine_drag))
	rpmforce -= torque
	rpm -= rpmforce * rev_speed

	drivetrain()

	print("RPM: %d, Throttle: %.2f" % [rpm, throttle])

# --- Placeholder Methods ---
func aero():
	pass

func controls(d: float):
	var gas_input = Input.is_action_pressed("gas")
	var brake_input = Input.is_action_pressed("brake")
	var handbrake_input = Input.is_action_pressed("handbrake")
	var clutch_input = Input.is_action_pressed("clutch")

	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, d)

func transmission():
	pass

func limits():
	pass

func drivetrain():
	pass
