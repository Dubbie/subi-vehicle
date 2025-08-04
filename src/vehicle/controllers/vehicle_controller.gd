# VehicleBody.gd
extends RigidBody3D
class_name VehicleBody

# --- Components ---
@export var pedal_controller: PedalController

# --- Vehicle Configuration ---
@export_group("Chassis")
@export var Weight: float = 900.0 # Mass in kilograms (kg)

@export_group("Engine")
@export var torque_curve: Curve # Assign a Curve resource in the Inspector
@export var idle_rpm: float = 800.0
@export var rpm_limit: float = 7000.0
@export var dead_rpm: float = 100.0 # RPM below which the engine is considered stalled
# Note: limiter_delay is now in seconds, not frames (e.g., 0.07)
@export var limiter_delay: float = 0.07
@export var throttle_idle: float = 0.25
# NOTE: These values will need retuning as they are now used with 'delta'.
@export var rev_speed: float = 20000.0 # Acts as torque->acceleration multiplier (inverse inertia)
@export var engine_friction: float = 5e-05 # Coefficient for friction that scales with RPM^2
@export var engine_drag: float = 1.0 # Coefficient for linear RPM drag
@export var throttle_response: float = 15.0 # Rate of throttle change per second

@export_group("Braking")
@export var ABS_Enabled: bool = true
# Note: ABS_Pump_Time is now in seconds, not frames (e.g., 0.02)
@export var ABS_Pump_Time: float = 0.02
@export var ABS_Pump_Force: float = 25.0 # Rate of brake pressure change per second

@export_group("Drivetrain")
@export var clutch_grip_torque: float = 400.0
@export var clutch_stability_factor: float = 0.5 # Corresponds to ClutchStable
@export var driveshaft_inertia_factor: float = 150.0 # Corresponds to DSWeight
@export var differential_lock_factor: float = 0.1
@export var powered_wheels: Array[WheelController]

# --- Internal Physics State (Class-Level Variables) ---
var throttle: float = 0.0
var rpm: float = 0.0
var rpmforce: float = 0.0 # The net rotational acceleration on the engine (in RPM/sec)
var gear: int = 0
var limdel_timer: float = 0.0 # Timer for the rev limiter
var drivetrain_resistance: float = 0.0 # Total resistance from wheels fed back to the engine.
var clutch_engagement: float = 0.0

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

	throttle = lerp(throttle, target_throttle, throttle_response * delta)

	# Apply engine idle control
	if rpm < idle_rpm:
		if throttle < throttle_idle:
			throttle = lerp(throttle, throttle_idle, throttle_response * delta)

	# Check if the rev limiter should be activated
	if rpm > rpm_limit and limdel_timer <= 0:
		limdel_timer = limiter_delay

	# 4. CALCULATE ENGINE TORQUE
	# ----------------------------------------------------------------------
	var engine_torque: float = 0.0
	if torque_curve and rpm > dead_rpm:
		engine_torque = torque_curve.sample(rpm) * throttle

	# 5. SIMULATE ENGINE INERTIA AND FRICTION
	# ----------------------------------------------------------------------
	# This section calculates the net rotational acceleration on the engine ('rpmforce')
	# in RPM per second, making it fully frame-rate independent.

	# Frictional/drag forces that resist engine rotation (in RPM/sec)
	var friction_force = rpm * abs(rpm) * engine_friction
	var drag_force = rpm * engine_drag

	# Force from combustion that accelerates the engine (in RPM/sec)
	var torque_force = engine_torque * rev_speed # rev_speed is inverse inertia

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

	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, d)

func transmission(_local_vel: Vector3, _d: float):
	pass

func limits():
	throttle = clamp(throttle, 0.0, 1.0)
	pass

func drivetrain(_torque_from_engine: float, _delta: float):
	pass

func get_final_gear_ratio() -> float:
	# This needs to be connected to your future transmission logic.
	# For now, we'll return a placeholder. Let's assume a final drive of 4.1.
	# A value of 0 would cause a division by zero error.
	var gear_ratio = 3.25 # 1st gear
	var final_drive = 4.1
	var ratio_mult = 9.5 # From original script
	var total_ratio = gear_ratio * final_drive * ratio_mult
	return total_ratio if total_ratio != 0 else 1.0
