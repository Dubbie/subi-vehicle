class_name WheelController
extends RayCast3D

#region Export Variables
@export_group("Wheel Setup")
@export var is_steering_wheel: bool = true
@export var power_bias: float = 1.0 # How much of the engine torque this wheel receives
@export var brake_bias: float = 1.0 # Main brake force distribution
@export var handbrake_bias: float = 0.0 # Handbrake force distribution
@export var wheel_mass_kg: float = 15.0

@export_group("Tyre Properties")
@export var tyre_width_mm: float = 185.0
@export var tyre_aspect_ratio: float = 60.0
@export var rim_diameter_inches: float = 14.0
@export var tyre_pressure_psi: float = 30.0

@export_group("Tyre Compound")
@export var compound_grip_factor: float = 1.0 # Base grip multiplier
@export var compound_stiffness: float = 1.0 # How much the compound resists deformation

@export_group("Suspension")
@export var suspension_stiffness: float = 25000.0 # Spring force in Newtons per meter (N/m)
@export var suspension_damping: float = 3000.0 # Damping force in Ns/m
@export var suspension_rebound_damping: float = 3500.0 # Damping during extension
@export var suspension_max_travel: float = 0.3 # Max travel distance in meters

@export_group("Brakes")
@export var brake_torque: float = 4000.0 # Max braking torque in Newton-meters (Nm)

@export_group("Geometry & Alignment")
@export var camber_degrees: float = 0.0 # Static camber angle
@export var caster_degrees: float = 3.0 # Caster angle for dynamic camber
@export var toe_degrees: float = 0.0 # Static toe angle

@export_group("Suspension Geometry")
# How much the wheel tucks in horizontally as it moves up.
@export var geometry_horizontal_tuck: float = 0.35
# How much the dynamic camber affects the visual wheel angle.
@export var geometry_camber_gain_factor: float = 1.0
# An additional factor to control the geometry's pivot point.
@export var geometry_pivot_offset: float = 0.0
# A final adjustment to the dynamic camber calculation.
@export var geometry_camber_offset_degrees: float = 0.0
# How much the suspension is allowed to travel before incline forces start.
@export var geometry_incline_free_zone: float = 0.2
# The strength of the incline stiffness effect.
@export var geometry_incline_impact_factor: float = 1.5
#endregion

# --- Node References ---
@onready var car: VehicleBody = get_owner()
@onready var wheel_mesh: Marker3D = $Animation/Camber/Wheel # A direct reference to the visual mesh

# --- Internal State Variables ---
var wheel_radius: float = 0.3
var wheel_angular_velocity: float = 0.0
var suspension_force: float = 0.0
var last_suspension_compression: float = 0.0
var hit_position: Vector3 = Vector3.ZERO
var dynamic_camber_degrees: float = 0.0
var axle_y_position: float = 0.0

func _ready():
	var sidewall_height_mm = tyre_width_mm * (tyre_aspect_ratio / 100.0)
	var rim_diameter_mm = rim_diameter_inches * 25.4
	var total_diameter_mm = (sidewall_height_mm * 2.0) + rim_diameter_mm
	wheel_radius = total_diameter_mm / 2000.0

	target_position.y = - (wheel_radius + suspension_max_travel)
	add_exception(car)


func _physics_process(delta: float):
	force_raycast_update()

	if is_colliding():
		hit_position = get_collision_point()

		# In your original script, geometry was calculated based on the visual wheel's
		# position from the PREVIOUS frame. We replicate that by updating visuals/geometry first.
		_update_visuals_and_geometry(delta, true)

		# Now, calculate the definitive suspension force using the collision data.
		suspension_force = _calculate_suspension_force(delta)
		car.apply_force(global_transform.basis.y * suspension_force, position)

		# --- (Brake and Friction logic remains the same) ---
		var brake_application = (car.brakeline * brake_bias) + (car.pedal_controller.get_handbrake() * handbrake_bias)
		var current_brake_torque = brake_torque * clamp(brake_application, 0.0, 1.0)

		var wheel_inertia = 0.5 * pow(wheel_radius, 2) * wheel_mass_kg
		if wheel_inertia > 0:
			var angular_acceleration = - current_brake_torque / wheel_inertia
			wheel_angular_velocity += angular_acceleration * delta

		var ground_velocity = car.linear_velocity + car.angular_velocity.cross(position)
		var local_ground_vel = global_transform.basis.transposed() * ground_velocity
		var surface_speed = wheel_angular_velocity * wheel_radius
		var slip_longitudinal = surface_speed - local_ground_vel.z
		var slip_lateral = - local_ground_vel.x

		var tyre_stiffness = tyre_width_mm * compound_stiffness
		var longitudinal_force = clamp(slip_longitudinal * tyre_stiffness, -suspension_force, suspension_force)
		var lateral_force = clamp(slip_lateral * tyre_stiffness, -suspension_force, suspension_force)
		var friction_force_local = Vector3(lateral_force, 0, longitudinal_force)
		car.apply_force(global_transform.basis * friction_force_local, position)
	else:
		# If in the air, update visuals to the "drooped" state.
		_update_visuals_and_geometry(delta, false)


func _calculate_suspension_force(delta: float) -> float:
	if not is_colliding() or delta == 0:
		last_suspension_compression = 0.0
		return 0.0

	var ray_length = global_position.distance_to(hit_position)
	var local_ground_vel = global_transform.basis.transposed() * (car.linear_velocity + car.angular_velocity.cross(position))

	# --- Incline Calculation (Translated from original) ---
	var car_up_vector = global_transform.basis.y
	var surface_normal = get_collision_normal()
	var incline_factor = (surface_normal - car_up_vector).length()

	if geometry_incline_free_zone < 1.0:
		incline_factor = (incline_factor - geometry_incline_free_zone) / (1.0 - geometry_incline_free_zone)

	incline_factor = clamp(incline_factor * geometry_incline_impact_factor, 0.0, 1.0)

	# --- Compression and Bottom-Out (Translated from original) ---
	var full_travel_dist = wheel_radius + suspension_max_travel
	var compressed_dist = full_travel_dist - ray_length
	var bottom_out_dist = compressed_dist - suspension_max_travel

	if compressed_dist < 0: compressed_dist = 0.0
	if bottom_out_dist < 0: bottom_out_dist = 0.0

	# --- Dynamic Stiffness/Damping (Translated from original) ---
	# On steep inclines, the spring and damper get much stiffer to support the car's weight.
	var bottom_out_stiffness = car.mass * 20.0 # A very high value
	var effective_stiffness = lerp(suspension_stiffness, bottom_out_stiffness, incline_factor)

	var damping_to_use = suspension_rebound_damping if local_ground_vel.y > 0 else suspension_damping
	var effective_damping = lerp(damping_to_use, car.mass * 2.0, incline_factor)

	# --- Final Force Calculation (Translated from original) ---
	var spring_force = compressed_dist * effective_stiffness
	var damping_force = local_ground_vel.y * effective_damping

	var final_force = spring_force - damping_force

	# Add extra bottom-out force if suspension travel is exceeded.
	if bottom_out_dist > 0:
		final_force += bottom_out_dist * (bottom_out_stiffness * 2.0)
		final_force -= local_ground_vel.y * (car.mass * 2.0)

	if final_force < 0:
		final_force = 0.0

	# Store the current compression for the next frame's damping calculation.
	last_suspension_compression = compressed_dist

	return final_force


func _update_visuals_and_geometry(delta: float, on_ground: bool):
	# First, always rotate the wheel mesh.
	wheel_mesh.rotate_x(wheel_angular_velocity * delta)

	if on_ground:
		# --- ON GROUND LOGIC ---
		var desired_world_position = hit_position + (global_transform.basis.y * wheel_radius)
		wheel_mesh.global_position = desired_world_position
	else:
		# --- IN AIR LOGIC ---
		var desired_world_position = global_position + (global_transform.basis.y * target_position.y)
		wheel_mesh.global_position = desired_world_position

	# --- DYNAMIC GEOMETRY (Translated from original) ---
	# This section calculates the dynamic camber and tuck-in based on the wheel's vertical position.
	axle_y_position = wheel_mesh.position.y

	# Calculate camber gain (`g` from original)
	var g = (axle_y_position + geometry_horizontal_tuck) / (abs(position.x) + geometry_pivot_offset + 1.0)
	g /= abs(g) + 1.0 # This is a form of atan-like normalization
	dynamic_camber_degrees = (g * 90.0) - geometry_camber_offset_degrees

	# Calculate horizontal tuck-in (`inned` from original)
	var total_dynamic_camber = abs(dynamic_camber_degrees) + geometry_camber_offset_degrees
	var inned_factor = (total_dynamic_camber / 90.0)
	inned_factor *= inned_factor - (geometry_camber_offset_degrees / 90.0)

	# Apply the geometry changes to the visual mesh and its parent nodes.
	# We use the parent `Camber` node to apply the total camber angle.
	var camber_node = wheel_mesh.get_parent()
	var total_camber_rads = deg_to_rad(camber_degrees + (dynamic_camber_degrees * geometry_camber_gain_factor))

	# The tuck-in is applied to the parent of the camber node.
	var animation_node = camber_node.get_parent()
	animation_node.position.x = - inned_factor * abs(position.x)

	# Ensure rotation is applied correctly based on which side of the car it's on
	camber_node.rotation.z = total_camber_rads * -sign(position.x)
