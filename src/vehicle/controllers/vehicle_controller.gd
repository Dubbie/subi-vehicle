# VehicleBody.gd
extends RigidBody3D

# --- Components ---
# We get a reference to the controller node to read pedal values.
@export var pedal_controller: PedalController

# --- Vehicle Configuration ---
@export_group("Chassis")
@export var weight: float = 900.0 # Mass in kilograms (kg)

@export_group("Engine")
@export var torque_curve: Curve # Assign a Curve resource in the Inspector
@export var idle_rpm: float = 800.0
@export var rpm_limit: float = 7000.0
@export var rev_speed: float = 2.0 # Represents flywheel inertia/lightness
@export var engine_friction: float = 18000.0 # Internal friction of the engine
@export var engine_drag: float = 0.006 # A drag coefficient applied to the RPM
@export var throttle_response: float = 0.5

# --- Internal Physics State ---
var throttle: float = 0.0
var rpm: float = 0.0
var gear: int = 0 # Will be managed by the transmission() function

# G-Force and Velocity tracking
var past_velocity := Vector3.ZERO
var g_force := Vector3.ZERO

# --- System-Specific Variables ---
# These are kept for maintaining the original script's logic flow.
# Their values will be set by their respective (currently empty) methods.
var brakeline: float = 0.0
var brake_allowed: float = 1.0
var abspump: int = 0
var limdel: int = 0

# A constant for Earth's gravity, used for G-force calculation.
const GRAVITY_ACCELERATION = 9.80665

@onready var pedal_debug_label: Label = %PedalDebugLabel

func _ready():
	# Initialize RPM at idle.
	rpm = idle_rpm


# The main physics loop. The order of function calls here is critical
# and matches the logic of the original script.
func _physics_process(delta: float):
	# 1. UPDATE VEHICLE STATE
	# ----------------------------------------------------------------------
	# Convert global linear and angular velocities to the car's local space.
	var local_velocity = global_transform.basis.transposed() * linear_velocity
	var local_angular_velocity = global_transform.basis.transposed() * angular_velocity

	# Set the mass correctly.
	if not mass == weight:
		mass = weight

	# Calculate G-force correctly for a metric system.
	# Acceleration = (change in velocity) / time
	# G-Force = Acceleration / Earth's Gravity
	var acceleration = (linear_velocity - past_velocity) / delta
	g_force = acceleration / GRAVITY_ACCELERATION
	past_velocity = linear_velocity

	# 2. CALL SIMULATION SUB-SYSTEMS
	# ----------------------------------------------------------------------
	# These methods are called in a specific order to ensure
	# dependencies are met (e.g., controls are read before the engine
	# calculates torque).

	aero(local_velocity, delta)

	controls(local_velocity, local_angular_velocity, delta)

	transmission(local_velocity, delta)

	limits()

	# 3. PROCESS BRAKING AND THROTTLE LOGIC
	# ----------------------------------------------------------------------
	# This logic remains here as it's a direct consequence of control inputs
	# and system states like ABS.

	# ABS pump timer
	abspump -= 1
	if abspump < 0:
		brake_allowed = 1.0 # Or some other logic to restore brake pressure
	else:
		brake_allowed = 0.0 # No braking allowed during an ABS pump

	# Final brake pressure applied to the wheels
	brakeline = pedal_controller.get_brake() * brake_allowed

	# Rev limiter timer
	limdel -= 1

	# Calculate the final throttle value based on pedal input and engine state.
	var current_gas_pedal = pedal_controller.get_throttle()
	if limdel < 0:
		# The clutch engagement here is a simplification from the original logic.
		# A disengaged clutch (value of 0) means the denominator is 1, so no TCS effect.
		var tcs_reduction_factor = 1.0 # Placeholder for TCS logic
		var clutch_engagement = pedal_controller.get_clutch_engagement()
		throttle -= (throttle - (current_gas_pedal / (tcs_reduction_factor * clutch_engagement + 1.0))) * (throttle_response / (1.0 / 60.0))
	else:
		# If the rev limiter is active, force the throttle to close.
		throttle -= throttle * (throttle_response / (1.0 / 60.0))

	# Apply engine limits (rev limiter and idle control)
	if rpm > rpm_limit:
		if throttle > 0.0: # Simplified from ThrottleLimit
			throttle = 0.0
			limdel = 4 # LimiterDelay
	elif rpm < idle_rpm:
		if throttle < 0.25: # Simplified from ThrottleIdle
			throttle = 0.25

	# 4. CALCULATE ENGINE TORQUE
	# ----------------------------------------------------------------------
	# This section is now much simpler. We sample the torque curve and apply throttle.
	# All the complex VVT, turbo, and boost logic is replaced.

	var engine_torque: float = 0.0
	if torque_curve:
		engine_torque = torque_curve.sample(rpm) * throttle

	# 5. SIMULATE ENGINE INERTIA AND FRICTION
	# ----------------------------------------------------------------------
	# This calculates the forces acting *on* the engine's RPM.
	# `rpmforce` is the net result of internal friction and the torque being produced.
	# A high torque output will fight against friction and accelerate the RPM.
	var rpmforce: float = 0.0

	# Engine friction is modeled as a force that increases with the square of the RPM.
	var internal_friction = (rpm * abs(rpm)) / (engine_friction / (1.0 / 60.0))
	var rotational_drag = rpm * (engine_drag / (1.0 / 60.0))

	# Calculate the net force on the RPM
	rpmforce = internal_friction + rotational_drag
	rpmforce -= (engine_torque / (1.0 / 60.0)) # Torque from combustion works against friction

	# Apply the net force to the RPM, scaled by the flywheel's inertia (rev_speed)
	rpm -= rpmforce * rev_speed

	# Ensure RPM doesn't drop to an unrealistic negative value
	if rpm < 0:
		rpm = 0

	# 6. APPLY FORCES TO DRIVETRAIN
	# ----------------------------------------------------------------------
	drivetrain(engine_torque, delta)

# --- Placeholder Methods ---
# These will be filled in as we continue the refactoring process.

func aero(_local_vel: Vector3, _delta: float):
	# Aerodynamic calculations will go here.
	pass

func controls(_local_vel: Vector3, _local_ang_vel: Vector3, delta: float):
	# This method will handle steering and call the PedalController.
	# For now, we just call the pedal controller.
	var gas_input = Input.is_action_pressed("gas")
	var brake_input = Input.is_action_pressed("brake")
	var handbrake_input = Input.is_action_pressed("handbrake")
	var clutch_input = Input.is_action_pressed("clutch")

	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, delta)

func transmission(_local_vel: Vector3, _delta: float):
	# Gear shifting logic (manual, auto, etc.) will go here.
	pass

func limits():
	# This can be used to clamp any final values before they are used.
	# The pedal limits are already handled in the PedalController.
	pass

func drivetrain(_torque_from_engine: float, _delta: float):
	# This crucial method will take the engine's torque, pass it through
	# the clutch and gearbox, and apply the final forces to the wheels.
	pass

func _process(_delta: float) -> void:
	var text = "Throttle: %.1f" % pedal_controller.get_throttle()
	text += "\nBrake: %.1f" % pedal_controller.get_brake()
	text += "\nHandbrake: %.1f" % pedal_controller.get_handbrake()
	text += "\nClutch: %.1f" % pedal_controller.get_clutch_engagement()
	pedal_debug_label.text = text
