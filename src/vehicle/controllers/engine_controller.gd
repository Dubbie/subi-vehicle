class_name EngineController
extends Node

# Constants
const RADS_TO_RPM = 60.0 / (2.0 * PI)
const RPM_TO_RADS = (2.0 * PI) / 60.0

# Engine Configuration
@export_group("Engine Configuration")
## The torque curve. X = RPM, Y = Torque
@export var torque_curve: Curve
## Flywheel inertia (rotational inertia of the engine)
@export var engine_inertia: float = 0.3 # kg⋅m²
## Engine brake torque (compression braking when throttle is closed)
@export var engine_brake_torque: float = 25.0 # Nm
## Engine friction coefficient (scales with RPM)
@export var engine_friction_coefficient: float = 0.006 # Nm⋅s/rad
## Idle RPM of the engine
@export var engine_min_rpm: float = 800.0
## The maximum engine RPM
@export var engine_max_rpm: float = 7000.0

# Engine State
var angular_velocity: float = 0.0 # rad/s
var rpm: float = 0.0
var current_torque: float = 0.0
var throttle_torque: float = 0.0
var friction_torque: float = 0.0
var brake_torque: float = 0.0
var net_torque: float = 0.0

func _ready():
	rpm = engine_min_rpm
	angular_velocity = rpm * RPM_TO_RADS

## Updates the engine physics for one time step
## Returns the torque available at the flywheel (after internal losses)
func update_engine(throttle_input: float, clutch_torque_reaction: float, delta_time: float) -> float:
	# 1. Calculate throttle-based torque
	throttle_torque = torque_curve.sample_baked(rpm) * clampf(throttle_input, 0.0, 1.0)

	# 2. Calculate RPM-scaled friction torque (pumping losses, bearing friction, etc.)
	friction_torque = engine_friction_coefficient * angular_velocity

	# 3. Calculate engine brake torque (only when throttle is closed)
	var throttle_factor = 1.0 - clampf(throttle_input, 0.0, 1.0)
	brake_torque = engine_brake_torque * throttle_factor

	# 4. Calculate net torque (throttle input minus all losses and clutch reaction)
	net_torque = throttle_torque - friction_torque - brake_torque - clutch_torque_reaction

	# 5. Update angular velocity using rotational dynamics
	var angular_acceleration = net_torque / engine_inertia
	angular_velocity += angular_acceleration * delta_time

	# 6. Clamp to engine RPM limits
	var min_angular_velocity = engine_min_rpm * RPM_TO_RADS
	var max_angular_velocity = engine_max_rpm * RPM_TO_RADS
	angular_velocity = clampf(angular_velocity, min_angular_velocity, max_angular_velocity)

	# 7. Update RPM for display/logic
	rpm = angular_velocity * RADS_TO_RPM

	# 8. Store current torque for debugging
	current_torque = net_torque

	# Return the torque that was actually absorbed by the clutch
	# This is the throttle torque minus internal engine losses
	return throttle_torque - friction_torque - brake_torque

## Get the maximum torque the engine can produce at current RPM
func get_max_torque_at_current_rpm() -> float:
	return torque_curve.sample_baked(rpm)

## Get the maximum torque the engine can produce across all RPMs
func get_max_torque() -> float:
	return torque_curve.max_value

## Get current engine RPM
func get_rpm() -> float:
	return rpm

## Get current engine angular velocity in rad/s
func get_angular_velocity() -> float:
	return angular_velocity

## Set engine RPM directly (useful for initialization or emergency situations)
func set_rpm(new_rpm: float) -> void:
	rpm = clampf(new_rpm, engine_min_rpm, engine_max_rpm)
	angular_velocity = rpm * RPM_TO_RADS

## Check if engine is at idle
func is_at_idle() -> bool:
	return rpm <= engine_min_rpm + 50.0 # Small buffer for idle detection

## Check if engine is at redline
func is_at_redline() -> bool:
	return rpm >= engine_max_rpm - 50.0 # Small buffer for redline detection

## Get detailed engine information for debugging
func get_engine_info() -> Dictionary:
	return {
		"rpm": rpm,
		"angular_velocity": angular_velocity,
		"throttle_torque": throttle_torque,
		"friction_torque": friction_torque,
		"brake_torque": brake_torque,
		"net_torque": net_torque,
		"current_torque": current_torque,
		"max_torque_at_rpm": get_max_torque_at_current_rpm(),
		"is_idle": is_at_idle(),
		"is_redline": is_at_redline()
	}

## Force engine to stall (set to minimum RPM)
func stall_engine() -> void:
	set_rpm(engine_min_rpm)

## Get engine load percentage (0-100%)
func get_engine_load() -> float:
	var max_torque = get_max_torque_at_current_rpm()
	if max_torque <= 0.0:
		return 0.0
	return (throttle_torque / max_torque) * 100.0