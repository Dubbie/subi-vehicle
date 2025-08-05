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
#endregion

#region Internal
var last_spring_length: float = 0.0
var current_spring_length: float = 0.0
var has_contact: bool = false

var knuckle_forward: Vector3 = Vector3.ZERO
var knuckle_right: Vector3 = Vector3.ZERO
var knuckle_up: Vector3 = Vector3.ZERO

var contact_right: Vector3 = Vector3.ZERO
var contact_forward: Vector3 = Vector3.ZERO
var contact_velocity: Vector3 = Vector3.ZERO
var contact_lat_velocity: float = 0.0
var contact_lon_velocity: float = 0.0
#endregion

#region Forces
var local_force: Vector3 = Vector3.ZERO
var load_force_vector: Vector3 = Vector3.ZERO
#endregion

# --- Node References ---
@onready var car: VehicleBody = get_owner()

func _ready() -> void:
	# Calculate wheel radius based on params
	target_position = Vector3.DOWN * (suspension_max_length + wheel_radius)

	add_exception(car)

#region Physics
func _physics_process(delta: float) -> void:
	calculate_spring_physics(0.0, delta)

	# Apply the force now
	car.apply_force(load_force_vector, global_position - car.global_position)

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

		# --- Total Suspension Force ---
		# The force is applied in the direction of the raycast (local up)
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
	# basis.z is the local "forward" vector (negative because of Godot's convention).
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

	# 6. Calculate 'contact_lng_velocity'
	if has_contact:
		# I've corrected this to 'contact_lng_velocity' to match your original variable name
		contact_lon_velocity = contact_velocity.dot(contact_forward)
	else:
		contact_lon_velocity = 0.0
