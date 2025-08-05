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
@export var suspension_max_length: float = 0.21 # Max travel distance in meters

@export_group("Grip and Stiffness")
@export var muk: float = 0.9 # Coefficient of friction
@export var longitudinal_stiffness: float = 40000.0 # Stiffness in the direction of rolling
@export var lateral_stiffness: float = 30000.0 # Cornering stiffness
@export var relaxation_length: float = 0.2 # Distance in meters for forces to build up

#endregion

#region Internal
var debug_mode: bool = true
var last_spring_length: float = 0.0
var current_spring_length: float = 0.0
var has_contact: bool = false
var inertia_inverse: float = 0.1
var delta_rotation: float = 0.0
var driven_wheel: bool = false

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

var current_angular_velocity: float = 0.0
var drive_torque: float = 0.0

# Slip and bristle deflection for the brush model
var lon_slip: float = 0.0
var lat_slip: float = 0.0
var longitudinal_deflection: float = 0.0
var lateral_deflection: float = 0.0

var smoothed_lon_slip: float = 0.0
#endregion

#region Forces
var local_force: Vector3 = Vector3.ZERO
var load_force_vector: Vector3 = Vector3.ZERO
var lon_force_vector: Vector3 = Vector3.ZERO
var lat_force_vector: Vector3 = Vector3.ZERO
#endregion

# --- Node References ---
@onready var car: VehicleController = get_owner()

func _ready() -> void:
	target_position = Vector3.DOWN * (suspension_max_length + wheel_radius)
	current_spring_length = suspension_max_length
	last_spring_length = suspension_max_length

	var inertia = 0.5 * wheel_mass * wheel_radius * wheel_radius
	if inertia > 0.0:
		inertia_inverse = 1.0 / inertia
	else:
		inertia_inverse = 0.1
		push_warning("Wheel inertia is zero or negative. Check wheel_mass and wheel_radius.")

	add_exception(car)

func _process(_delta: float) -> void:
	if not debug_mode or not has_contact: return

	var force_scale: float = car.mass
	DebugDraw3D.draw_arrow(contact_point, contact_point + (load_force_vector / force_scale), Color.GREEN, 0.1)
	DebugDraw3D.draw_arrow(contact_point, contact_point + (lat_force_vector / force_scale), Color.RED, 0.1)
	DebugDraw3D.draw_arrow(contact_point, contact_point + (lon_force_vector / force_scale), Color.BLUE, 0.1)

	# Cylinder should mimic what the wheel does.
	var wheel_position: Vector3 = global_position
	wheel_position.y -= last_spring_length

	# Create basis with spin
	var wheel_basis := Basis(Vector3.UP, PI * 0.5) \
		.rotated(Vector3.FORWARD, deg_to_rad(90)) \
		.rotated(Vector3.RIGHT, -delta_rotation) # Spin from angular velocity

	# Apply scale
	wheel_basis = wheel_basis.scaled(Vector3(wheel_width, wheel_radius, wheel_radius))

	# Build transform
	var wheel_cylinder: Transform3D = Transform3D(wheel_basis, wheel_position)

	# Slip fade from yellow → red
	var slip_strength: float = clampf(lon_slip / 1.0, 0.0, 1.0)
	var cyl_color: Color = Color.YELLOW.lerp(Color.RED, slip_strength)

	# Draw
	DebugDraw3D.draw_cylinder(wheel_cylinder, cyl_color, 0.0)

	# Debug text for slip values
	var text_position = global_position + Vector3.UP * 0.5
	var slip_text = "Lon Slip: %.2f\nLat Slip: %.2f" % [smoothed_lon_slip, lat_slip]
	DebugDraw3D.draw_text(text_position, slip_text)


func update_state(p_steer_angle: float, delta: float) -> void:
	_update_spring_and_contact(delta)
	_update_knuckle(p_steer_angle)
	_update_contact_velocities()

func _update_spring_and_contact(delta: float) -> void:
	force_raycast_update()
	has_contact = is_colliding()

	if has_contact:
		# Get the distance to the collision point
		contact_point = get_collision_point()
		contact_normal = get_collision_normal()

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
		contact_point = Vector3.ZERO
		contact_normal = Vector3.ZERO
		local_force.y = 0.0
		load_force_vector = Vector3.ZERO

	# Update the last spring length for the next frame's calculation
	last_spring_length = current_spring_length

func _update_knuckle(steer_angle: float) -> void:
	var rotated_basis = basis.rotated(Vector3.UP, deg_to_rad(steer_angle))

	# Store the world-space directions of the wheel.
	knuckle_right = global_transform.basis * rotated_basis.x
	knuckle_forward = global_transform.basis * -rotated_basis.z
	knuckle_up = global_transform.basis.y

func _update_contact_velocities() -> void:
	if has_contact:
		contact_right = knuckle_right.slide(contact_normal).normalized()
		contact_forward = - contact_right.cross(contact_normal).normalized()

		var collider = get_collider()
		var contact_object_velocity: Vector3 = Vector3.ZERO
		if collider is RigidBody3D:
			contact_object_velocity = collider.get_velocity_at_local_point(contact_point - collider.global_position)
		elif collider is CharacterBody3D:
			contact_object_velocity = collider.velocity

		var body_point_velocity = car.linear_velocity + car.angular_velocity.cross(contact_point - car.global_position)
		var relative_velocity = body_point_velocity - contact_object_velocity

		contact_velocity = relative_velocity - contact_normal * relative_velocity.dot(contact_normal)
		contact_lat_velocity = contact_velocity.dot(contact_right)
		contact_lon_velocity = contact_velocity.dot(contact_forward)
	else:
		contact_right = knuckle_right
		contact_forward = knuckle_forward
		contact_velocity = Vector3.ZERO
		contact_lat_velocity = 0.0
		contact_lon_velocity = 0.0

func calculate_wheel_physics(current_drive_torque: float, current_brake_torque: float, dyn_muk: float, delta: float):
	drive_torque = current_drive_torque

	# Apply motor torque to spin the wheel up
	current_angular_velocity += drive_torque * inertia_inverse * delta

	# Rolling Resistance and Braking
	var rolling_resistance_torque = 5.0 * -sign(current_angular_velocity)
	var total_resist_torque = current_brake_torque + rolling_resistance_torque
	var w_brake = total_resist_torque * inertia_inverse * delta

	if w_brake > abs(current_angular_velocity):
		current_angular_velocity = 0.0
	else:
		current_angular_velocity -= sign(current_angular_velocity) * w_brake

	# Brush Model Implementation
	if has_contact:
		var surface_speed = current_angular_velocity * wheel_radius
		var slip_velocity_lon = surface_speed - contact_lon_velocity
		var slip_velocity_lat = - contact_lat_velocity

		# Update bristle deflection based on slip and relaxation length
		var relaxation_factor_lon = (abs(contact_lon_velocity) / relaxation_length) if relaxation_length > 0 else 0
		longitudinal_deflection += (slip_velocity_lon - relaxation_factor_lon * longitudinal_deflection) * delta

		var relaxation_factor_lat = (abs(contact_lon_velocity) / relaxation_length) if relaxation_length > 0 else 0
		lateral_deflection += (slip_velocity_lat - relaxation_factor_lat * lateral_deflection) * delta

		# Calculate forces from bristle deflection and stiffness
		local_force.z = longitudinal_deflection * longitudinal_stiffness
		local_force.x = lateral_deflection * lateral_stiffness

		# Update slip ratios for debugging/external use
		lon_slip = (surface_speed - contact_lon_velocity) / max(abs(contact_lon_velocity), 0.1)
		lat_slip = atan2(contact_lat_velocity, contact_lon_velocity)

		smoothed_lon_slip = lerp(smoothed_lon_slip, lon_slip, 0.1)
	else:
		local_force.z = 0.0
		local_force.x = 0.0
		longitudinal_deflection = 0.0
		lateral_deflection = 0.0
		lon_slip = 0.0
		lat_slip = 0.0

		smoothed_lon_slip = 0.0

	# Combine forces and limit to friction circle
	var max_friction_force = local_force.y * (muk * dyn_muk)
	var combined_force_sq = local_force.z * local_force.z + local_force.x * local_force.x

	if combined_force_sq > max_friction_force * max_friction_force and combined_force_sq > 0:
		var force_scale = max_friction_force / sqrt(combined_force_sq)
		local_force.z *= force_scale
		local_force.x *= force_scale

		# Adjust deflection to match the scaled force
		longitudinal_deflection = local_force.z / longitudinal_stiffness
		lateral_deflection = local_force.x / lateral_stiffness


	# Apply traction force from the road back to the wheel
	var t_traction = local_force.z * wheel_radius
	var w_traction = t_traction * inertia_inverse * delta
	current_angular_velocity -= w_traction

	# Update visual rotation
	delta_rotation += current_angular_velocity * delta
	delta_rotation = fposmod(delta_rotation, 2 * PI)

	# Save final world-space force vectors
	lon_force_vector = contact_forward * local_force.z
	lat_force_vector = contact_right * local_force.x

func apply_forces_to_rigidbody():
	if not has_contact:
		return

	var total_force = lat_force_vector + lon_force_vector + load_force_vector
	car.apply_force(total_force, global_position - car.global_position)
