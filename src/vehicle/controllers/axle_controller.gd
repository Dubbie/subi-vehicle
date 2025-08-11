class_name AxleController
extends Node3D

#region Export variables
## Label for debugging purposes.
@export var label_prefix: String = "Axle"
## Wheelbase half to position the wheels.
## A front axle might be position at -1.262m
@export var z_offset: float = -1.35 # Meters
## Distance between the knuckles.
@export var track_width: float = 1.75 # Meters

@export_group("Drivetrain Ratios")
## Drive ratio indicates how much drive torque is applied to this axle from the clutch.
@export_range(0.0, 1.0) var drive_ratio: float = 0.0
## Brake ratio indicates how much brake torque is applied to this axle.
@export_range(0.0, 1.0) var brake_ratio: float = 0.5
## Handbrake ratio indicates how much handbrake torque is applied to this axle.
## Usually only the rear axle is held by the handbrake.
@export_range(0.0, 1.0) var handbrake_ratio: float = 0.0
## This is the final drive ratio usually. Keep this realistic.
@export var diff_ratio: float = 4.5

@export_group("Suspension & Damping")
## Maximum travel distance of the spring.
@export var max_spring_length: float = 0.21 # m
## Stiffness of the suspension spring.
@export var spring_stiffness: float = 25000.0 # N/m
## Damping of the suspension spring when compressing.
@export var spring_damping: float = 1350.0 # Ns/m

@export_group("Suspension Geometry")
## Camber angle in degrees. Negative camber tilts the top of the wheel inwards.
@export var camber_angle_deg: float = -1.0 # Degrees
## Caster angle in degrees. Positive caster tilts the steering axis backwards.
@export var caster_angle_deg: float = 5.0 # Degrees
## Toe angle in degrees. Positive for toe-in (front of wheels point inwards).
@export var toe_angle_deg: float = 0.1 # Degrees


@export_group("Wheels")
## Visually positioning the wheels. Not used at the moment.
@export var suspension_y_offset: float = 0.0 # m
## Radius of the wheels, used in inertia calculation.
@export var wheel_radius: float = 0.316 # m
## Mass of the wheels, used in inertia calculation.
@export var wheel_mass: float = 18.0 # kg
## Width of the wheels. Not used at the moment.
@export var wheel_width: float = 0.3 # m
## The tire model to use for the wheels
@export var tire_model: BaseTireModel
## The left wheel.
@export var left_wheel: WheelController
## The right wheel.
@export var right_wheel: WheelController
#endregion

#region Internals
var steer_angle_left: float = 0.0
var steer_angle_right: float = 0.0
var _wheelbase: float = 0.0
var _max_angle_inner_rad: float = 0.0
var _max_angle_outer_rad: float = 0.0
#endregion

func initialize(p_wheelbase: float, p_max_steer_angle_deg: float) -> void:
	_wheelbase = p_wheelbase

	# Set up the wheels to use the axle's configuration.
	var wheels: Array[WheelController] = [left_wheel, right_wheel]
	for i in range(wheels.size()):
		var wheel = wheels[i]
		# Tire model
		wheel.tire_model = tire_model.duplicate()

		# Suspension
		wheel.suspension_stiffness = spring_stiffness
		wheel.suspension_damping = spring_damping
		wheel.suspension_max_length = max_spring_length

		# Wheel properties
		wheel.wheel_mass = wheel_mass
		wheel.wheel_radius = wheel_radius
		wheel.wheel_width = wheel_width

		# Suspension Geometry
		wheel.camber_angle_deg = camber_angle_deg
		wheel.caster_angle_deg = caster_angle_deg
		# Apply toe symmetrically. Positive toe makes wheels point inward.
		# The right wheel gets a positive angle (yaws left), left wheel gets negative (yaws right).
		if i == 0: # Left Wheel
			wheel.toe_angle_deg = - toe_angle_deg
		else: # Right Wheel
			wheel.toe_angle_deg = toe_angle_deg

	# If this axle can't steer, we're done.
	if p_max_steer_angle_deg <= 0.0:
		return

	# Ensure we convert the incoming DEGREES to RADIANS for all calculations.
	_max_angle_inner_rad = deg_to_rad(p_max_steer_angle_deg)

	# Calculate the turning radius of the inner wheel at full lock.
	var radius_at_inner_wheel = _wheelbase / tan(_max_angle_inner_rad)

	# Calculate the turning radius of the outer wheel at full lock.
	var radius_at_outer_wheel = radius_at_inner_wheel + track_width

	# Calculate the corresponding max angle for the outer wheel (will also be in radians).
	_max_angle_outer_rad = atan(_wheelbase / radius_at_outer_wheel)

func set_steer_value(steer_value: float) -> void:
	# Exit if this axle cannot steer (max angle is 0).
	if _max_angle_inner_rad == 0.0:
		return

	# 1. Get the absolute steering input (0.0 to 1.0).
	var steer_abs = abs(steer_value)

	# 2. Linearly interpolate to find the CURRENT angle for each wheel.
	var current_angle_inner_rad = lerp(0.0, _max_angle_inner_rad, steer_abs)
	var current_angle_outer_rad = lerp(0.0, _max_angle_outer_rad, steer_abs)

	var current_angle_inner_deg = rad_to_deg(current_angle_inner_rad)
	var current_angle_outer_deg = rad_to_deg(current_angle_outer_rad)

	# 3. Assign the angles to the correct wheels with the correct sign.
	if steer_value > 0: # Steering RIGHT
		steer_angle_right = current_angle_outer_deg
		steer_angle_left = current_angle_inner_deg
	else: # Steering LEFT
		steer_angle_left = - current_angle_outer_deg
		steer_angle_right = - current_angle_inner_deg

func update_wheel_states(delta: float) -> void:
	left_wheel.update_state(steer_angle_left, delta)
	right_wheel.update_state(steer_angle_right, delta)