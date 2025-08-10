class_name WheelController
extends RayCast3D

#region Axle Config
var wheel_radius: float = 0.316 # meters
var wheel_mass: float = 18.0 # kg
var wheel_width: float = 0.18 # meters

var suspension_stiffness: float = 25000.0 # Spring force in Newtons per meter (N/m)
var suspension_damping: float = 1350.0 # Damping force in Ns/m
var suspension_max_length: float = 0.21 # Max travel distance in meters

var tire_model: BaseTireModel
#endregion

#region Internal
var debug_mode: bool = true
var last_spring_length: float = 0.0
var current_spring_length: float = 0.0
var has_contact: bool = false
var inertia_inverse: float = 0.1
var delta_rotation: float = 0.0
var driven_wheel: bool = false
var current_steer_angle: float = 0.0

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
@onready var wheel_marker: Marker3D = $Animation/Camber/Wheel

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
	# We use current_spring_length for responsiveness.
	wheel_marker.position = Vector3.DOWN * current_spring_length

	# Create a pure steering rotation (Yaw around the Y-axis)
	var steer_rotation = Basis().rotated(Vector3.UP, deg_to_rad(current_steer_angle))

	# Create a pure spin rotation (Roll around the X-axis)
	var spin_rotation = Basis().rotated(Vector3.RIGHT, -delta_rotation)

	# Combine the rotations.
	wheel_marker.basis = steer_rotation * spin_rotation

	if not debug_mode or not has_contact: return

	# Debug drawing
	var force_scale: float = car.mass
	DebugDraw3D.draw_arrow(global_position, global_position + (load_force_vector / force_scale), Color.GREEN, 0.1)
	DebugDraw3D.draw_arrow(contact_point, contact_point + (lat_force_vector / force_scale), Color.RED, 0.1)
	DebugDraw3D.draw_arrow(contact_point, contact_point + (lon_force_vector / force_scale), Color.BLUE, 0.1)

	# Debug text for slip values
	if has_contact:
		var text_position = global_position + Vector3.UP * 3.0
		var slip_text = "Lon Slip: %.2f\nLat Slip: %.2f" % [smoothed_lon_slip, lat_slip]
		DebugDraw3D.draw_text(text_position, slip_text)

#region Public methods
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

func calculate_wheel_physics(current_drive_torque: float, current_brake_torque: float, dyn_muk: float, delta: float):
	drive_torque = current_drive_torque

	# Apply motor torque to spin the wheel up
	current_angular_velocity += drive_torque * inertia_inverse * delta

	# Rolling Resistance and Braking
	var crr = 0.015 # Coefficient of Rolling Resistance - Tweak this value!
	var rolling_resistance_force = crr * local_force.y # Based on normal force
	var rolling_resistance_torque = rolling_resistance_force * wheel_radius

	# Apply it in the correct direction
	rolling_resistance_torque *= -sign(current_angular_velocity)
	var total_resist_torque = current_brake_torque + rolling_resistance_torque
	var w_brake = total_resist_torque * inertia_inverse * delta

	if w_brake > abs(current_angular_velocity):
		current_angular_velocity = 0.0
	else:
		current_angular_velocity -= sign(current_angular_velocity) * w_brake

	# --- Tire Force Calculation ---
	var tire_forces = Vector2.ZERO
	if has_contact:
		var surface_speed = current_angular_velocity * wheel_radius

		# Calculate slip values
		# Longitudinal Slip Ratio (a dimensionless number)
		lon_slip = (surface_speed - contact_lon_velocity) / max(abs(contact_lon_velocity), 0.1)
		smoothed_lon_slip = lerp(smoothed_lon_slip, lon_slip, 0.1)

		# Lateral Slip Angle (in radians)
		lat_slip = atan2(-contact_lat_velocity, abs(contact_lon_velocity))

		# --- DELEGATE TO THE TIRE MODEL ---
		# Bundle up all the data the tire model needs
		var model_params = TireParams.new()
		model_params.vertical_load = local_force.y
		model_params.lon_slip_ratio = lon_slip
		model_params.lat_slip_angle_rad = lat_slip
		model_params.surface_friction = dyn_muk # Pass dynamic friction
		model_params.longitudinal_velocity = contact_lon_velocity
		model_params.wheel_angular_velocity = current_angular_velocity
		model_params.wheel_radius = wheel_radius
		model_params.delta = delta

		# Call the model to get the forces
		tire_forces = tire_model.calculate_forces(model_params)

	# Assign the calculated forces
	local_force.x = tire_forces.x # Lateral force
	local_force.z = tire_forces.y # Longitudinal force

	# Apply traction back to wheel
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

	var total_friction_force = lat_force_vector + lon_force_vector
	car.apply_force(load_force_vector, global_position - car.global_position)
	car.apply_force(total_friction_force, contact_point - car.global_position)

#region Private methods
func _update_knuckle(steer_angle: float) -> void:
	# Store the current angle so the _process function can use it for visuals
	current_steer_angle = steer_angle

	# 1. Get the wheel's base orientation vectors in world space from the parent node.
	var right_vec = global_transform.basis.x
	var up_vec = global_transform.basis.y
	var fwd_vec = global_transform.basis.z

	# 2. Define the pivot axis for steering (the "up" direction of the suspension).
	var pivot_axis = up_vec

	# 3. Calculate the rotation in radians, inverting the sign to match Godot's coordinate system.
	var rotation_rads = deg_to_rad(steer_angle)

	# 4. Rotate the base vectors to get the final "knuckle" orientation.
	knuckle_forward = fwd_vec.rotated(pivot_axis, rotation_rads)
	knuckle_right = right_vec.rotated(pivot_axis, rotation_rads)
	knuckle_up = up_vec # The suspension axis itself does not rotate.

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