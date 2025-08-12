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

#region Physics
func _physics_process(delta: float) -> void:
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
		wheel.tire_model = tire_model.duplicate()

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
	var torque_left: float = 0.0
	var torque_right: float = 0.0

	match differential_type:
		DifferentialType.OPEN:
			# For a simple simulation, both Open and Locked differentials start with a 50/50 torque split.
			# - An Open diff is limited by the wheel with the least grip. Our tire model handles this naturally
			#   by allowing the low-grip wheel to spin up (slip).
			# - A Locked diff forces wheels to the same speed. The tire model creates counteracting forces
			#   when they are forced to slip against the road surface, simulating the lock.
			torque_left = total_axle_torque * 0.5
			torque_right = total_axle_torque * 0.5

		DifferentialType.LOCKED:
			var wheel_speed_left_rads = left_wheel.current_angular_velocity
			var wheel_speed_right_rads = right_wheel.current_angular_velocity
			var speed_difference = wheel_speed_left_rads - wheel_speed_right_rads

			# 1. Calculate a massive corrective torque based on the speed difference and a high stiffness value.
			# This simulates a near-rigid connection between the wheels.
			var correction_torque = speed_difference * locking_stiffness

			# 2. Apply this torque to counteract the speed difference.
			# It's subtracted from the faster wheel and added to the slower wheel.
			torque_left = (total_axle_torque * 0.5) - correction_torque
			torque_right = (total_axle_torque * 0.5) + correction_torque

		DifferentialType.LIMITED_SLIP:
			var wheel_speed_left_rads = left_wheel.current_angular_velocity
			var wheel_speed_right_rads = right_wheel.current_angular_velocity
			var speed_difference = wheel_speed_left_rads - wheel_speed_right_rads

			# 1. Determine the maximum possible locking torque based on input power.
			var max_locking_torque = total_axle_torque * lsd_power_lock_factor

			# 2. Determine how much of that lock to apply based on the slip speed.
			# This creates a smooth engagement instead of an instant, jerky lock.
			var engagement_factor = clamp(abs(speed_difference) / lsd_engagement_speed, 0.0, 1.0)

			# 3. Calculate the final locking torque.
			var locking_torque = max_locking_torque * engagement_factor

			# The locking torque is applied from the faster wheel to the slower wheel.
			# The sign of speed_difference tells us which way to apply it.
			locking_torque *= -sign(speed_difference)

			# Start with a 50/50 split and then apply the locking torque.
			torque_left = (total_axle_torque * 0.5) + locking_torque
			torque_right = (total_axle_torque * 0.5) - locking_torque

			# Debug
			# print("Axle: ", name, " SpeedDiff: ", speed_difference, " LockingTorque: ", locking_torque, " TotalAxleTorque: ", total_axle_torque)

	return Vector2(torque_left, torque_right)

#region Private API
func _update_solid_axle_geometry():
	# This method enforces the rigid link between the wheels.
	# It calculates the axle's tilt and repositions the wheel nodes.
	# 1. Define the axle's orientation (Basis).
	# We start with the AxleController's own orientation.
	var axle_basis = self.global_transform.basis

	# 2. Calculate the tilt based on wheel contact.
	# Using the contact points from the last physics frame gives us a stable result.
	if left_wheel.has_contact and right_wheel.has_contact:
		var left_contact = left_wheel.get_collision_point()
		var right_contact = right_wheel.get_collision_point()

		# Create a vector representing the tilted axle beam.
		var axle_right_vector = (right_contact - left_contact).normalized()
		# Create an up vector that is orthogonal to the axle and the car's forward direction.
		var axle_up_vector = axle_right_vector.cross(axle_basis.z).normalized()
		# Create the final forward vector.
		var axle_forward_vector = axle_up_vector.cross(axle_right_vector).normalized()

		axle_basis = Basis(axle_right_vector, axle_up_vector, axle_forward_vector)

	# 3. Position the wheels at the ends of the tilted axle beam.
	var half_track = track_width / 2.0
	left_wheel.global_position = self.global_position - axle_basis.x * half_track
	right_wheel.global_position = self.global_position + axle_basis.x * half_track

func _update_independent_suspension_forces():
	# This logic is for the anti-roll bar on independent suspensions. It remains the same.
	var arb_force: float = 0.0
	if anti_roll_stiffness > 0.0:
		var compression_diff = left_wheel.current_spring_length - right_wheel.current_spring_length
		arb_force = compression_diff * anti_roll_stiffness

	left_wheel.anti_roll_force = arb_force
	right_wheel.anti_roll_force = - arb_force
