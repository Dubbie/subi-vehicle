class_name WheelController
extends RayCast3D

#region Axle Config
## How large the wheels are, required for inertia calculation
var wheel_radius: float = 0.316
## How heavy the wheels are, required for inertia calculation
var wheel_mass: float = 18.0
## The wheels width in meters
var wheel_width: float = 0.18

## Spring force in Newtons per meter (N/m)
var spring_stiffness: float = 25000.0
## Damping force in Newtons per meter (N/m)
var spring_bump_damping: float = 1350.0
## Damping force in Newtons per meter (N/m)
var spring_rebound_damping: float = 3350.0
## Max travel distance in meters
var spring_max_travel: float = 0.18
## Spring preload simulation
var ride_height_adjust: float = 0.0
## > 0 makes wheels stick to the ground better but is unrealistic
var droop_extension: float = 0.0

var tire_model: BaseTireModel

# Suspension Geometry from Axle
var camber_angle_deg: float = 0.0
var caster_angle_deg: float = 0.0
var toe_angle_deg: float = 0.0
#endregion

#region Internal
## Shows debug information
var debug_mode: bool = true
## The spring's length the previous frame
var last_spring_length: float = 0.0
## Positions the wheel visually
var current_wheel_height: float = 0.0
## Spring compression ratio
var compression_ratio: float = 0.0
## Is the wheel on the ground
var has_contact: bool = false
## Inverse inertia calculation for wheel
var inertia_inverse: float = 0.1
## How much the wheel rotates
var delta_rotation: float = 0.0
## Is the wheel being driven, used in debug
var driven_wheel: bool = false
## The current steering angle
var current_steer_angle: float = 0.0

# Wheel basis vectors
var knuckle_forward: Vector3 = Vector3.ZERO
var knuckle_right: Vector3 = Vector3.ZERO
var knuckle_up: Vector3 = Vector3.ZERO

var contact_point: Vector3 = Vector3.ZERO
var contact_normal: Vector3 = Vector3.UP
var contact_right: Vector3 = Vector3.ZERO
var contact_forward: Vector3 = Vector3.ZERO
var contact_velocity: Vector3 = Vector3.ZERO
var contact_lat_velocity: float = 0.0
var contact_lon_velocity: float = 0.0

## Current angular velocity in radians per second
var current_angular_velocity: float = 0.0
## Current drive torque on the wheel, in N-m
var drive_torque: float = 0.0

# Slip for tire model
var lon_slip: float = 0.0
var lat_slip: float = 0.0
#endregion

#region Forces
var anti_roll_force: float = 0.0
var local_force: Vector3 = Vector3.ZERO
var load_force_vector: Vector3 = Vector3.ZERO
var lon_force_vector: Vector3 = Vector3.ZERO
var lat_force_vector: Vector3 = Vector3.ZERO
#endregion

# Node references
@onready var car: VehicleController = get_owner()
# Visual hierarchy nodes
@onready var animation_node: Marker3D = $Animation
@onready var camber_node: Marker3D = $Animation/Camber
@onready var wheel_node: Marker3D = $Animation/Camber/Wheel

func _ready() -> void:
	target_position.y = - (spring_max_travel + wheel_radius + droop_extension)
	last_spring_length = spring_max_travel

	var inertia = 0.5 * wheel_mass * wheel_radius * wheel_radius
	if inertia > 0.0:
		inertia_inverse = 1.0 / inertia
	else:
		inertia_inverse = 0.1
		push_warning("Wheel inertia is zero or negative. Check wheel_mass and wheel_radius.")

	add_exception(car)

func _process(_delta: float) -> void:
	# Update wheel visuals using the node hierarchy (Animation -> Camber -> Wheel)
	# Suspension travel - animation node moves up/down
	wheel_node.position.y = current_wheel_height

	# Caster and steering - caster tilts the steering axis
	# This creates realistic camber change when steering
	var caster_rad = deg_to_rad(caster_angle_deg)
	var steer_rad = deg_to_rad(current_steer_angle)
	var steering_axis = Vector3.UP.rotated(Vector3.RIGHT, caster_rad)
	animation_node.basis = Basis().rotated(steering_axis, steer_rad)

	# Camber - static rotation around forward axis after steering
	var camber_rad = deg_to_rad(camber_angle_deg)
	camber_node.basis = Basis().rotated(Vector3.FORWARD, camber_rad)

	# Toe and wheel spin - final node applies toe (static) then spin (dynamic)
	var toe_rad = deg_to_rad(toe_angle_deg)
	var toe_rotation = Basis().rotated(Vector3.UP, toe_rad)
	var spin_rotation = Basis().rotated(Vector3.RIGHT, -delta_rotation)
	wheel_node.basis = toe_rotation * spin_rotation

	# Debug visualization
	if not (debug_mode and car.is_controlled) or not has_contact: return

	var force_scale: float = car.mass
	DebugDraw3D.draw_arrow(global_position, global_position + (load_force_vector / force_scale), Color.GREEN, 0.1)
	DebugDraw3D.draw_arrow(contact_point, contact_point + (lat_force_vector / force_scale), Color.RED, 0.1)
	DebugDraw3D.draw_arrow(contact_point, contact_point + (lon_force_vector / force_scale), Color.BLUE, 0.1)
	DebugDraw3D.draw_arrow(global_position, global_position + (target_position + Vector3(0.0, wheel_radius, 0.0)), Color.BLACK, 0.1)

	if has_contact:
		var text_position = global_position + Vector3.UP * 3.0
		var slip_text = "Lon Slip: %.2f\nLat Slip: %.2f\n" % [lon_slip, lat_slip]
		var load_text = "%d N | %.2f%%" % [local_force.y, compression_ratio * 100.0]
		DebugDraw3D.draw_text(text_position, slip_text + load_text, 32)

	if Input.is_action_just_pressed("debug_load"):
		debug_suspension_forces()

func debug_suspension_forces():
	print("=== SUSPENSION DEBUG for %s ===" % name)
	print("Compression Ratio: %.2f" % compression_ratio)
	print("Vertical load: %.2f N" % [local_force.y])
	print("Current Spring Length: %.3f N" % (spring_max_travel - (compression_ratio * spring_max_travel)))
	print("Has contact: %s" % has_contact)
	print("Anti-roll force: %.0f N" % anti_roll_force)
	print("=============================")

	debug_tire_forces(drive_torque)

func debug_tire_forces(current_drive_torque: float):
	if not debug_mode or not has_contact:
		return

	print("=== TIRE FORCES DEBUG for %s ===" % name)
	print("Load: %.0f N" % local_force.y)
	print("Lon slip ratio: %.3f" % lon_slip)
	print("Lat slip angle: %.1f°" % rad_to_deg(lat_slip))
	print("Contact velocities - Lat: %.2f, Lon: %.2f" % [contact_lat_velocity, contact_lon_velocity])
	print("Drive torque: %.0f Nm" % current_drive_torque)
	print("Angular velocity: %.1f rad/s" % current_angular_velocity)

	# Check what the tire model is actually producing
	var model_params = TireParams.new()
	model_params.vertical_load = local_force.y
	model_params.lon_slip_ratio = lon_slip
	model_params.lat_slip_angle_rad = lat_slip
	model_params.surface_friction = 1.0 # Assuming standard surface
	model_params.longitudinal_velocity = contact_lon_velocity
	model_params.wheel_angular_velocity = current_angular_velocity
	model_params.wheel_radius = wheel_radius
	model_params.delta = get_physics_process_delta_time()

	var tire_forces = tire_model.calculate_forces(model_params)
	print("Raw tire forces - Lat: %.0f N, Lon: %.0f N" % [tire_forces.x, tire_forces.y])
	print("Final local forces - Lat: %.0f N, Lon: %.0f N, Vert: %.0f N" % [local_force.x, local_force.z, local_force.y])

	# Test what forces SHOULD be at this load
	var expected_forces = tire_model.get_peak_forces_at_load(local_force.y, 1.0)
	print("Expected peak forces - Lat: %.0f N, Lon: %.0f N" % [expected_forces.peak_lateral_force, expected_forces.peak_longitudinal_force])
	print("Load factor: %.2f" % expected_forces.load_factor)

	print("=====================================")

#region Public methods
func update_state(p_steer_angle: float) -> void:
	_update_spring_and_contact()
	_update_knuckle(p_steer_angle)
	_update_contact_velocities()

func _update_spring_and_contact() -> void:
	force_raycast_update()
	has_contact = is_colliding()
	target_position.y = - (spring_max_travel + wheel_radius + droop_extension)

	if has_contact:
		contact_point = get_collision_point()
		contact_normal = get_collision_normal()

		var ground_contact_point: Vector3 = get_collision_point()
		var current_spring_length: float = global_position.distance_to(ground_contact_point) - wheel_radius
		var spring_compression: float = (spring_max_travel - current_spring_length) + ride_height_adjust
		spring_compression = max(0.0, spring_compression)

		var physical_compression = spring_max_travel - current_spring_length
		compression_ratio = clamp(physical_compression / spring_max_travel, 0.0, 1.0)
		current_wheel_height = - current_spring_length

		var spring_force: float = spring_stiffness * spring_compression
		var tire_contact_velocity: Vector3 = car.get_point_velocity(contact_point)
		var suspension_relative_velocity: float = global_basis.y.dot(tire_contact_velocity)
		var damping_force: float = spring_bump_damping * suspension_relative_velocity if suspension_relative_velocity > 0 else spring_rebound_damping * suspension_relative_velocity

		var total_vertical_force: float = spring_force - damping_force + anti_roll_force

		load_force_vector = total_vertical_force * contact_normal
		local_force.y = total_vertical_force

		last_spring_length = current_spring_length
	else:
		contact_point = Vector3.ZERO
		contact_normal = Vector3.UP

		current_wheel_height = - spring_max_travel
		compression_ratio = 0.0
		load_force_vector = Vector3.ZERO

		last_spring_length = spring_max_travel

func calculate_wheel_physics(current_drive_torque: float, current_brake_torque: float, dyn_muk: float, delta: float):
	# Store drive torque from axle (can be positive or negative)
	drive_torque = current_drive_torque

	# Calculate tire forces based on last frame's velocity
	# Need to do this before applying new torques
	var tire_forces = Vector2.ZERO
	if has_contact:
		 # Calculate longitudinal slip - improved low speed handling
		_calculate_longitudinal_slip()

		# Calculate lateral slip using actual wheel contact velocity
		_calculate_lateral_slip()

		# Get forces from tire model
		var model_params = TireParams.new()
		model_params.vertical_load = local_force.y
		model_params.lon_slip_ratio = lon_slip
		model_params.lat_slip_angle_rad = lat_slip
		model_params.surface_friction = dyn_muk
		model_params.longitudinal_velocity = contact_lon_velocity
		model_params.wheel_angular_velocity = current_angular_velocity
		model_params.wheel_radius = wheel_radius
		model_params.delta = delta
		tire_forces = tire_model.calculate_forces(model_params)

	# Store forces for later application to car body
	local_force.x = tire_forces.x # Lateral Force (Fx)
	local_force.z = tire_forces.y # Longitudinal Force (Fz)

	# Sum all torques acting on the wheel
	# Traction torque from ground resistance
	var traction_torque = local_force.z * wheel_radius

	# Rolling resistance always opposes rotation
	var crr = 0.015
	var rolling_resistance_force = crr * local_force.y
	var rolling_resistance_torque = rolling_resistance_force * wheel_radius

	# Net torque calculation
	# Drive torque from engine, minus traction from ground, minus brakes and rolling resistance
	var net_torque = drive_torque - traction_torque - (current_brake_torque * sign(current_angular_velocity)) - (rolling_resistance_torque * sign(current_angular_velocity))

	# Apply net torque to update angular velocity
	var angular_acceleration = net_torque * inertia_inverse
	current_angular_velocity += angular_acceleration * delta

	# Prevent infinite spinning in air with simple damping
	if not has_contact:
		current_angular_velocity *= 0.998

	# Update wheel visuals
	delta_rotation += current_angular_velocity * delta
	delta_rotation = fposmod(delta_rotation, 2 * PI)

	# Update world-space force vectors for debug drawing
	lon_force_vector = contact_forward * local_force.z
	lat_force_vector = contact_right * -local_force.x

func apply_forces_to_rigidbody():
	if not has_contact:
		return

	var total_friction_force = lat_force_vector + lon_force_vector
	car.apply_force(load_force_vector, global_position - car.global_position)
	car.apply_force(total_friction_force, contact_point - car.global_position)

#region Private methods
func _update_knuckle(steer_angle: float) -> void:
	current_steer_angle = steer_angle
	var right_vec = global_transform.basis.x
	var up_vec = global_transform.basis.y
	var fwd_vec = - global_transform.basis.z
	var steer_rad = deg_to_rad(steer_angle)
	var camber_rad = deg_to_rad(camber_angle_deg)
	var caster_rad = deg_to_rad(caster_angle_deg)
	var toe_rad = deg_to_rad(toe_angle_deg)
	var fwd_with_toe = fwd_vec.rotated(up_vec, toe_rad)
	var right_with_toe = right_vec.rotated(up_vec, toe_rad)
	var pivot_axis = up_vec.rotated(right_with_toe, caster_rad)
	var fwd_steered = fwd_with_toe.rotated(pivot_axis, steer_rad)
	var right_steered = right_with_toe.rotated(pivot_axis, steer_rad)
	var up_steered = up_vec.rotated(pivot_axis, steer_rad)
	knuckle_forward = fwd_steered
	knuckle_right = right_steered.rotated(knuckle_forward, -camber_rad)
	knuckle_up = up_steered.rotated(knuckle_forward, -camber_rad)

func _update_contact_velocities() -> void:
	if has_contact:
		# Calculate contact basis vectors
		contact_right = knuckle_right.slide(contact_normal).normalized()
		contact_forward = - contact_right.cross(contact_normal).normalized()

		# Get velocity of the contact point on the colliding object
		var collider = get_collider()
		var contact_object_velocity: Vector3 = Vector3.ZERO
		if collider is RigidBody3D:
			contact_object_velocity = collider.get_velocity_at_local_point(contact_point - collider.global_position)
		elif collider is CharacterBody3D:
			contact_object_velocity = collider.velocity

		# Calculate velocity of this wheel's contact point
		var body_point_velocity = car.linear_velocity + car.angular_velocity.cross(contact_point - car.global_position)

		# Relative velocity between wheel and ground
		var relative_velocity = body_point_velocity - contact_object_velocity

		# Project onto contact plane (remove normal component)
		contact_velocity = relative_velocity - contact_normal * relative_velocity.dot(contact_normal)

		# Decompose into lateral and longitudinal components
		contact_lat_velocity = contact_velocity.dot(contact_right)
		contact_lon_velocity = contact_velocity.dot(contact_forward)
	else:
		contact_right = knuckle_right
		contact_forward = knuckle_forward
		contact_velocity = Vector3.ZERO
		contact_lat_velocity = 0.0
		contact_lon_velocity = 0.0

func _calculate_longitudinal_slip() -> void:
	var surface_speed = current_angular_velocity * wheel_radius

	# Below this speed, the standard slip formula is too unstable.
	var low_speed_threshold = 1.0

	if abs(contact_lon_velocity) > low_speed_threshold:
		# STANDARD FORMULA: Used at normal driving speeds
		lon_slip = (surface_speed - contact_lon_velocity) / abs(contact_lon_velocity)
	else:
		# LOW-SPEED APPROXIMATION: Used when stationary or crawling
		# This prevents division by zero and provides stable grip for launching.
		# We use a slightly larger reference speed to make it less sensitive.
		var reference_speed = max(low_speed_threshold, 0.5)
		lon_slip = (surface_speed - contact_lon_velocity) / reference_speed

	# Clamp to reasonable limits to prevent numerical issues
	lon_slip = clamp(lon_slip, -2.0, 2.0)

func _calculate_lateral_slip() -> void:
	if contact_velocity.length() < 0.5:
		lat_slip = 0.0
		return

	# Since knuckle_forward now points forward and knuckle_right points right,
	# we can directly use the velocity components we already calculated

	# contact_lat_velocity = how fast we're moving sideways (+ = moving right)
	# contact_lon_velocity = how fast we're moving forward (+ = moving forward)

	if abs(contact_lon_velocity) > 0.1:
		# Slip angle is simply the arctangent of lateral velocity / longitudinal velocity
		# This gives us the angle between where we're going vs where wheel points
		lat_slip = atan2(contact_lat_velocity, abs(contact_lon_velocity))
	else:
		# At very low speeds, use a different approach
		# Use lateral velocity directly with some scaling
		lat_slip = contact_lat_velocity / 2.0 # Arbitrary scaling for low speeds

	# Clamp to reasonable limits (±30 degrees is more realistic than ±45)
	lat_slip = clamp(lat_slip, deg_to_rad(-30), deg_to_rad(30))
