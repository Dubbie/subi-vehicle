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
var _min_turn_radius: float = 0.0
#endregion

func initialize(p_wheelbase: float, p_min_turn_radius: float) -> void:
    _wheelbase = p_wheelbase
    _min_turn_radius = p_min_turn_radius

    # Set up the wheels to use the axle's configuration.
    var wheels: Array[WheelController] = [left_wheel, right_wheel]
    for wheel in wheels:
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

func set_steer_value(steer_value: float) -> void:
    if _wheelbase == 0:
        push_warning("Wheelbase is 0, can't calculate steering")
        return

    # If steer value is negligible, straighten the wheels.
    if abs(steer_value) < 0.000001:
        steer_angle_left = 0.0
        steer_angle_right = 0.0
        return

    # Current turning radius of center point
    var r: float = _min_turn_radius / abs(steer_value)

    # Radii for inner and outer wheel
    var r_inner: float = r - track_width / 2.0
    var r_outer: float = r + track_width / 2.0

    # Calculate angles in degrees
    var inner_deg: float = rad_to_deg(atan(_wheelbase / r_inner))
    var outer_deg: float = rad_to_deg(atan(_wheelbase / r_outer))

    # Sign and assignment of angles to the correct wheels
    if steer_value < 0: # Steering Left
        # The left wheel is the inner wheel and needs a greater angle.
        # Angles are negative for a left turn.
        steer_angle_left = - inner_deg
        steer_angle_right = - outer_deg
    else: # Steering Right
        # The right wheel is the inner wheel and needs a greater angle.
        # Angles are positive for a right turn.
        steer_angle_left = outer_deg
        steer_angle_right = inner_deg