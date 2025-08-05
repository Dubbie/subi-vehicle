class_name WheelController
extends RayCast3D

#region Export
@export_group("Wheel Setup")
@export var wheel_radius: float = 0.316 # meters
@export var wheel_mass: float = 18.0 # kg
@export var wheel_width: float = 0.18 # meters

@export_group("Suspension")
@export var suspension_stiffness: float = 25000.0 # Spring force in Newtons per meter (N/m)
@export var suspension_damping: float = 1350.0 # Damping force in Ns/m
@export var suspension_rebound_damping: float = 1350.0 # Damping during extension
@export var suspension_max_length: float = 0.21 # Max travel distance in meters

@export_group("Grip")
@export var muk: float = 1.2
#endregion

#region Internal
var last_spring_length: float = 0.0
var current_spring_length: float = 0.0
var has_contact: bool = false
var inertia_inverse: float = 0.1
var delta_rotation: float = 0.0

var knuckle_forward: Vector3 = Vector3.ZERO
var knuckle_right: Vector3 = Vector3.ZERO
var knuckle_up: Vector3 = Vector3.ZERO

var contact_right: Vector3 = Vector3.ZERO
var contact_forward: Vector3 = Vector3.ZERO
var contact_velocity: Vector3 = Vector3.ZERO
var contact_lat_velocity: float = 0.0
var contact_lon_velocity: float = 0.0

var current_angular_velocity: float = 0.0
var drive_torque: float = 0.0
var lon_slip: float = 0.0
var lat_slip: float = 0.0
#endregion

#region Forces
var local_force: Vector3 = Vector3.ZERO
var load_force_vector: Vector3 = Vector3.ZERO
var lon_force_vector: Vector3 = Vector3.ZERO
var lat_force_vector: Vector3 = Vector3.ZERO
#endregion

# --- Node References ---
@onready var car: VehicleBody = get_owner()

func _ready() -> void:
	# Calculate wheel radius based on params
	target_position = Vector3.DOWN * (suspension_max_length + wheel_radius)

	# Calculate the wheel's moment of inertia based on its mass and radius
	var inertia = 0.5 * wheel_mass * wheel_radius * wheel_radius

	# Calculate the inverse, with a safety check to prevent division by zero
	if inertia > 0.0:
		inertia_inverse = 1.0 / inertia
	else:
		# Fallback value if mass or radius is zero
		inertia_inverse = 0.1
		push_warning("Wheel inertia is zero or negative. Check wheel_mass and wheel_radius.")

	add_exception(car)

#region Physics
func _physics_process(delta: float) -> void:
	calculate_spring_physics(0.0, delta)
	calculate_wheel_physics(100.0, 0.0, 1.0, delta)

	# Apply the force now
	car.apply_force(load_force_vector, global_position - car.global_position)
	car.apply_force(lon_force_vector, global_position - car.global_position)
	car.apply_force(lat_force_vector, global_position - car.global_position)

#endregion
func calculate_spring_physics(steer_angle: float, delta: float) -> void:
	has_contact = is_colliding()
	_update_spring(delta)
	_update_knuckle(steer_angle)
	_update_contact()

func _update_spring(delta: float) -> void:
	force_raycast_update()

	if has_contact:
		# Get the distance to the collision point
		var ray_length = global_position.distance_to(get_collision_point())

		# Calculate the current spring length
		current_spring_length = ray_length - wheel_radius

		# --- Suspension Force Calculation (Hooke's Law) ---
		var spring_depth = suspension_max_length - current_spring_length
		var spring_force = suspension_stiffness * spring_depth

		# --- Damper Force Calculation ---
		var spring_speed = (last_spring_length - current_spring_length) / delta
		var damper_force = suspension_damping * spring_speed

		# --- Suspension Force ---
		var suspension_force = max(0.0, spring_force + damper_force)

		local_force.y = suspension_force
		load_force_vector = (suspension_force * knuckle_up.y) * get_collision_normal()
	else:
		local_force.y = 0.0
		load_force_vector = Vector3.ZERO

	# Update the last spring length for the next frame's calculation
	last_spring_length = current_spring_length

func _update_knuckle(steer_angle: float) -> void:
 	# Rotate the wheel's basis around its local UP axis by the steer angle.
	# basis.x is the local "right" vector.
	# basis.z is the local "forward" vector.
	var rotated_basis = basis.rotated(Vector3.UP, deg_to_rad(steer_angle))

	# Store the world-space directions of the wheel.
	knuckle_right = global_transform.basis * rotated_basis.x
	knuckle_forward = global_transform.basis * -rotated_basis.z
	knuckle_up = global_transform.basis.y

func _update_contact() -> void:
	var contact_normal = get_collision_normal()

	 # 1. Calculate 'contact_right' direction
	if has_contact:
		contact_right = knuckle_right.slide(contact_normal).normalized()
	else:
		contact_right = Vector3.ZERO

	# 2. Calculate 'contact_forward' direction
	if has_contact:
		contact_forward = knuckle_forward.slide(contact_normal).normalized()
	else:
		contact_forward = Vector3.ZERO

	# 3. Calculate 'contact_object_velocity'
	var contact_object_velocity := Vector3.ZERO
	if has_contact:
		var collider = get_collider()
		# Check if the collider is a RigidBody3D. These are dynamic objects that can move and rotate.
		if collider is RigidBody3D:
			var contact_point = get_collision_point()
			var vector_from_collider_center = contact_point - collider.global_position
			contact_object_velocity = collider.linear_velocity + collider.angular_velocity.cross(vector_from_collider_center)
		# Check for CharacterBody3D, which has linear velocity but no angular velocity.
		elif collider is CharacterBody3D:
			contact_object_velocity = collider.velocity

	# 4. Calculate 'contact_velocity' (using your "car" variable)
	if has_contact:
		var contact_point = get_collision_point()
		# Note: Ensure your script has a reference to the car body, here named "car"
		var body_point_velocity = car.linear_velocity + \
								  car.angular_velocity.cross(contact_point - car.global_position)

		var relative_velocity = body_point_velocity - contact_object_velocity
		contact_velocity = relative_velocity.slide(contact_normal)
	else:
		contact_velocity = Vector3.ZERO

	# 5. Calculate 'contact_lat_velocity'
	if has_contact:
		contact_lat_velocity = contact_velocity.dot(contact_right)
	else:
		contact_lat_velocity = 0.0

	# 6. Calculate 'contact_lon_velocity'
	if has_contact:
		# I've corrected this to 'contact_lon_velocity' to match your original variable name
		contact_lon_velocity = contact_velocity.dot(contact_forward)
	else:
		contact_lon_velocity = 0.0

func calculate_wheel_physics(current_drive_torque: float, current_brake_torque: float, dyn_muk: float, delta: float):
	drive_torque = current_drive_torque

	# Apply motor torque to spin the wheel up
	current_angular_velocity += current_drive_torque * inertia_inverse * delta

	# Rolling Resistance and Braking
	var w_brake_overshoot: float = 0.0
	if abs(current_angular_velocity) > 0.0:
		var rolling_resistance_torque = 5.0 * -sign(current_angular_velocity)
		var total_resist_torque = current_brake_torque + rolling_resistance_torque
		var w_brake = total_resist_torque * inertia_inverse * delta

		# Check if the brake force is strong enough to lock the wheel
		if w_brake > abs(current_angular_velocity):
			w_brake_overshoot = w_brake - abs(current_angular_velocity)
			current_angular_velocity = 0.0 # Lock the wheel
		else:
			current_angular_velocity -= sign(current_angular_velocity) * w_brake

	# Longitudinal Slip (lngSlip) Calculation
	lon_slip = _calc_lon_slip(current_angular_velocity, contact_lon_velocity)

	# Lateral Slip (latSlip) Calculation - This part was provided
	var lat_slip_target = _calc_lat_slip(contact_lon_velocity, contact_lat_velocity) * -sign(contact_lat_velocity)
	var lat_interpolator = clamp(abs(contact_lat_velocity / 2.0 * delta), 0.0, 1.0)
	lat_slip += (lat_slip_target - lat_slip) * lat_interpolator

	# Calculate tire forces based on slip
	var max_friction_force = local_force.y * (muk * dyn_muk)
	local_force.z = lon_slip * max_friction_force
	local_force.x = lat_slip * max_friction_force

	# Combine forces and limit to friction circle
	var combined_force = sqrt(local_force.z * local_force.z + local_force.x * local_force.x)
	if combined_force > max_friction_force and combined_force > 0:
		var force_sale = max_friction_force / combined_force
		local_force.z *= force_sale
		local_force.x *= force_sale

	# Apply traction force from the road back to the wheel
	var t_traction = local_force.z * wheel_radius * -1.0
	var w_traction = t_traction * inertia_inverse * delta
	current_angular_velocity += w_traction

	if w_brake_overshoot > 0.0:
		current_angular_velocity = 0.0

	# Update visual rotation
	delta_rotation += current_angular_velocity * delta
	delta_rotation = fposmod(delta_rotation, 2 * PI)

	# Save final world-space force vectors
	lon_force_vector = contact_forward * local_force.z
	lat_force_vector = contact_right * local_force.x

func _calc_lat_slip(v_long: float, v_lat: float) -> float:
	# No sideways velocity => no slip
	if abs(v_lat) < 0.01:
		return 0.0

	# Slip angle is the angle of the velocity vector relative to the wheel's forward direction
	var slip_angle_rad = atan2(v_lat, v_long)

	# Linear mapping from angle to a 0..1 value
	# A 90-degree slip (PI/2 radians) is considered maximum slip (1.0)
	# We take the absolute value as the direction is handled separately.
	return clamp(abs(slip_angle_rad / (PI / 2.0)), 0.0, 1.0)

func _calc_lon_slip(p_angular_velocity: float, p_contact_lon_velocity: float) -> float:
	var surface_speed = p_angular_velocity * wheel_radius

	# The denominator is the car's ground speed. We prevent division by zero at a standstill.
	var denominator = abs(p_contact_lon_velocity)
	if denominator < 0.1:
		denominator = 0.1

	var slip_ratio = (surface_speed - p_contact_lon_velocity) / denominator

	# Clamp the value to a reasonable range. -1.2 to 1.2 is a good starting point.
	return clamp(slip_ratio, -1.2, 1.2)
