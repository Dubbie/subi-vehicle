@tool
class_name AxleController
extends Node3D

enum DifferentialType {
	OPEN,
	LOCKED,
	LIMITED_SLIP
}

#region Export variables
## Label for debugging purposes.
@export var label_prefix: String = "Axle"
## Wheelbase half to position the wheels.
## A front axle might be position at -1.262m
@export var z_offset: float = -1.35: set = _set_z_offset
## Distance between the knuckles in metres.
@export var track_width: float = 1.75: set = _set_track_width

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
@export var spring_bump_damping: float = 1350.0 # Ns/m
## Damping of the suspension spring when extending.
@export var spring_rebound_damping: float = 3375.0 # Ns/m
## The stiffness of the anti-roll bar. Only used for independent suspension.
## A higher value reduces body roll. Set to 0 to disable.
@export var anti_roll_stiffness: float = 8000.0 # N/m

@export_group("Suspension Geometry")
## Camber angle in degrees. Negative camber tilts the top of the wheel inwards.
@export var camber_angle_deg: float = -1.0 # Degrees
## Caster angle in degrees. Positive caster tilts the steering axis backwards.
@export var caster_angle_deg: float = 5.0 # Degrees
## Toe angle in degrees. Positive for toe-in (front of wheels point inwards).
@export var toe_angle_deg: float = 0.1 # Degrees

@export_group("Axle Type")
## If true, the axle is simulated as a solid beam. If false, it's an independent suspension.
@export var is_solid_axle: bool = false

@export_group("Differential")
## The type of differential to simulate for this axle.
@export var differential_type: DifferentialType = DifferentialType.OPEN
## For Limited-Slip Differentials, this determines the percentage of power to lock.
## A value of 0.25 means it can transfer up to 25% of the input torque to the gripping wheel.
## Good values are between 0.2 (light LSD) and 0.75 (very aggressive).
@export_range(0.0, 1.0) var lsd_power_lock_factor: float = 0.3
## The speed difference (in rad/s) at which the LSD achieves maximum lock.
## A lower value makes the LSD react faster and more aggressively. Good values: 5.0 to 20.0
@export var lsd_engagement_speed: float = 10.0
## For a Locked Differential, this determines how rigidly it forces the wheels
## to the same speed. This needs to be somewhat low so we don't break the physics.
@export var locking_stiffness: float = 150.0
## It's the maximum torque (in Nm) the diff can transfer to resist slip.
## 200-800 is a good range for an LSD. 5000+ simulates a locked diff.
@export var differential_lock_capacity: float = 400.0

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

func _ready() -> void:
	# Synchronize visuals when the scene is first loaded in the editor
	if Engine.is_editor_hint():
		_set_z_offset(z_offset)
		_set_track_width(track_width)

#region Physics
func _physics_process(delta: float) -> void:
	# Guard against running physics in the editor, which causes errors.
	if Engine.is_editor_hint():
		return

	# --- Geometric Setup ---
	if is_solid_axle:
		# For a solid axle, we enforce the rigid beam constraint first.
		_update_solid_axle_geometry()
	else:
		# For an independent suspension, we calculate and apply anti-roll bar forces.
		_update_independent_suspension_forces()

	# --- Wheel State Update ---
	# Now that the geometry and extra forces are set, we tell each wheel to
	# update its own state based on its (potentially new) position.
	left_wheel.update_state(steer_angle_left, delta)
	right_wheel.update_state(steer_angle_right, delta)
#endregion

#region Public API
func initialize(p_wheelbase: float, p_max_steer_angle_deg: float) -> void:
	_wheelbase = p_wheelbase

	# Set up the wheels to use the axle's configuration.
	var wheels: Array[WheelController] = [left_wheel, right_wheel]
	for i in range(wheels.size()):
		var wheel = wheels[i]
		# Tire model
		wheel.tire_model = tire_model.duplicate(true)

		# Suspension
		wheel.suspension_stiffness = spring_stiffness
		wheel.suspension_bump_damping = spring_bump_damping
		wheel.suspension_rebound_damping = spring_rebound_damping
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

func get_distributed_torques(total_axle_torque: float) -> Vector2:
	# 1. Start with a basic 50/50 open differential split.
	var torque_left = total_axle_torque * 0.5
	var torque_right = total_axle_torque * 0.5

	# 2. Calculate the speed difference between the wheels.
	var speed_difference = left_wheel.current_angular_velocity - right_wheel.current_angular_velocity

	# 3. Determine the locking torque the differential wants to apply to resist this difference.
	# It's the speed difference multiplied by a high stiffness, but CAPPED by the diff's physical capacity.
	# This CLAMP is the crucial step that prevents instability.
	var stiffness = 1000.0 # A high value to make it react quickly
	var locking_torque = clamp(abs(speed_difference) * stiffness, 0.0, differential_lock_capacity)

	# 4. Transfer the locking torque from the faster wheel to the slower wheel.
	# The sign of the speed_difference tells us which way to transfer.
	if speed_difference > 0: # Left wheel is faster
		torque_left -= locking_torque
		torque_right += locking_torque
	else: # Right wheel is faster
		torque_left += locking_torque
		torque_right -= locking_torque

	return Vector2(torque_left, torque_right)

#region Private API
func _set_z_offset(new_value: float) -> void:
	z_offset = new_value
	if Engine.is_editor_hint():
		position.z = z_offset

func _set_track_width(new_value: float) -> void:
	track_width = new_value
	if Engine.is_editor_hint():
		if is_instance_valid(left_wheel) and is_instance_valid(right_wheel):
			var half_track = track_width / 2.0
			left_wheel.position.x = - half_track
			right_wheel.position.x = half_track

func _update_solid_axle_geometry():
	# Instead of directly positioning wheels, we enforce the solid axle constraint
	# by averaging the suspension forces and heights, then applying corrective forces
	if not (left_wheel.has_contact or right_wheel.has_contact):
		return

	# Calculate the average suspension compression to determine beam height
	var left_compression = left_wheel.suspension_max_length - left_wheel.current_spring_length
	var right_compression = right_wheel.suspension_max_length - right_wheel.current_spring_length
	var avg_compression = (left_compression + right_compression) * 0.5

	# Calculate the beam tilt angle based on compression difference
	var compression_diff = right_compression - left_compression
	var beam_tilt_angle = atan2(compression_diff, track_width)

	# Limit the tilt angle to prevent extreme angles
	beam_tilt_angle = clamp(beam_tilt_angle, deg_to_rad(-15.0), deg_to_rad(15.0))

	# Calculate target spring lengths based on beam tilt
	var half_track = track_width * 0.5
	var left_target_compression = avg_compression - half_track * tan(beam_tilt_angle)
	var right_target_compression = avg_compression + half_track * tan(beam_tilt_angle)

	# Convert back to spring lengths
	var left_target_length = left_wheel.suspension_max_length - left_target_compression
	var right_target_length = right_wheel.suspension_max_length - right_target_compression

	# Apply corrective forces to enforce the beam constraint
	var beam_stiffness = spring_stiffness * 2.0 # Stiffer than individual springs

	# For very stiff beams, you might want to add an export variable:
	# @export var beam_stiffness_multiplier: float = 2.0
	# var beam_stiffness = spring_stiffness * beam_stiffness_multiplier

	# Calculate corrective forces
	var left_correction = (left_target_length - left_wheel.current_spring_length) * beam_stiffness
	var right_correction = (right_target_length - right_wheel.current_spring_length) * beam_stiffness

	# Apply the corrections as additional forces
	left_wheel.anti_roll_force += left_correction
	right_wheel.anti_roll_force += right_correction

	# Optional: Add some damping to prevent oscillations
	var beam_damping = spring_bump_damping * 0.5
	var left_velocity = (left_wheel.current_spring_length - left_wheel.last_spring_length) / get_physics_process_delta_time()
	var right_velocity = (right_wheel.current_spring_length - right_wheel.last_spring_length) / get_physics_process_delta_time()

	var velocity_diff = right_velocity - left_velocity
	var damping_force = velocity_diff * beam_damping

	left_wheel.anti_roll_force += damping_force * 0.5
	right_wheel.anti_roll_force -= damping_force * 0.5

func _update_independent_suspension_forces():
	# This logic is for the anti-roll bar on independent suspensions. It remains the same.
	var arb_force: float = 0.0
	if anti_roll_stiffness > 0.0:
		var compression_diff = left_wheel.current_spring_length - right_wheel.current_spring_length
		arb_force = compression_diff * anti_roll_stiffness

	left_wheel.anti_roll_force = arb_force
	right_wheel.anti_roll_force = - arb_force
