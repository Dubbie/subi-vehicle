class_name WheelController
extends RayCast3D

#region Export
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

@export_group("Tyre Friction Model")
# How quickly grip falls off after the optimal slip is exceeded.
@export var friction_slip_falloff: float = 1.0
# The base traction factor, affecting the optimal slip angle/ratio.
@export var friction_traction_factor: float = 1.0
# A "magic number" that simulates the non-linear relationship between side-slip
# and lateral force. Higher values make the tyre feel 'sharper' at the limit.
@export var friction_force_rigidity: float = 0.67

@export_group("Tyre Compound")
@export var compound_grip_factor: float = 1.0 # Base grip multiplier
@export var compound_stiffness: float = 1.0 # How much the compound resists deformation

@export_group("Suspension")
@export var suspension_stiffness: float = 25000.0 # Spring force in Newtons per meter (N/m)
@export var suspension_damping: float = 3000.0 # Damping force in Ns/m
@export var suspension_rebound_damping: float = 3500.0 # Damping during extension
@export var suspension_max_travel: float = 0.3 # Max travel distance in meters
@export var suspension_rest_length: float = 0.0

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

@export var incline_area: float = 0.2

@export var a_geometry_3: float = 0.0
@export var a_geometry_4: float = 0.0
#endregion

#region Internal
var dist = 0.0
var w_size = 1.0
var w_size_read = 1.0
var w_weight_read = 0.0
var w_weight = 0.0
var wv = 0.0
var wv_ds = 0.0
var wv_diff = 0.0
var c_tp = 0.0
var effectiveness = 0.0

var angle = 0.0
var snap = 0.0
var absolute_wv = 0.0
var absolute_wv_brake = 0.0
var absolute_wv_diff = 0.0
var output_wv = 0.0
var offset = 0.0
var c_p = 0.0
var wheelpower = 0.0
var wheelpower_global = 0.0
var stress = 0.0
var rolldist = 0.0
var rd = 0.0
var c_camber = 0.0
var cambered = 0.0

var rollvol = 0.0
var sl = 0.0
var skvol = 0.0
var skvol_d = 0.0
var velocity = Vector3(0, 0, 0)
var velocity2 = Vector3(0, 0, 0)
var compress = 0.0
var compensate = 0.0
var axle_position = 0.0

var heat_rate = 1.0
var wear_rate = 1.0

var ground_bump = 0.0
var ground_bump_up = false
var ground_bump_frequency = 0.0
var ground_bump_frequency_random = 1.0
var ground_bump_height = 0.0

var ground_friction = 1.0
var ground_stiffness = 1.0
var fore_friction = 0.0
var fore_stiffness = 0.0
var drag = 0.0
var ground_builduprate = 0.0
var ground_dirt = false
var hitposition = Vector3(0, 0, 0)

var cache_tyrestiffness = 0.0
var cache_friction_action = 0.0

var directional_force = Vector3(0, 0, 0)
var slip_perc = Vector2(0, 0)
var slip_perc2 = 0.0
var slip_percpre = 0.0

var velocity_last = Vector3(0, 0, 0)
var velocity2_last = Vector3(0, 0, 0)
#endregion

# --- Node References ---
@onready var car: VehicleBody = get_owner()
@onready var camber_node: Marker3D = $Animation/Camber
@onready var wheel_marker: Marker3D = $Animation/Camber/Wheel # A direct reference to the visual mesh

@onready var animation_node: Marker3D = $Animation
@onready var geometry_node: Marker3D = $Geometry
@onready var velocity_node: Marker3D = $Velocity
@onready var velocity_step_node: Marker3D = $Velocity/Step
@onready var velocity_2_node: Marker3D = $Velocity2
@onready var velocity_2_step_node: Marker3D = $Velocity2/Step

func _ready() -> void:
	c_tp = tyre_pressure_psi

	w_size = _calculate_wheel_size()
	add_exception(car)

func power() -> void:
	pass

func diffs() -> void:
	pass

func sway() -> void:
	pass

#region Physics
func _physics_process(delta: float) -> void:
	var translation = position
	var cast_to = target_position
	var global_translation = global_position
	var last_translation = position

	# Rotate the wheel, then reset translation
	translation = last_translation

	c_camber = camber_degrees + caster_degrees * rotation.y * float(translation.x > 0.0) - caster_degrees * rotation.y * float(translation.x < 0.0)

	directional_force = Vector3.ZERO

	$Velocity.position = Vector3.ZERO
	$Velocity2.global_position = geometry_node.global_position

	velocity_step_node.global_position = velocity_last
	velocity_2_step_node.global_position = velocity2_last
	velocity_last = velocity_node.global_position
	velocity2_last = velocity_2_node.global_position

	velocity = - velocity_step_node.position / delta
	velocity2 = - velocity_2_step_node.position / delta

	velocity_node.rotation = Vector3.ZERO
	velocity_2_node.rotation = Vector3.ZERO

	# Variables
	var elasticity = suspension_stiffness
	var damping = suspension_damping
	var damping_rebound = suspension_rebound_damping

	# Swaybar setup, modify suspension values based on it later

	if elasticity < 0.0:
		elasticity = 0.0

	if damping < 0.0:
		damping = 0.0

	if damping_rebound < 0.0:
		damping_rebound = 0.0

	if is_colliding():
		var suspforce = _calculate_suspension(elasticity, damping, damping_rebound, velocity.y, abs(cast_to.y), global_translation, get_collision_point())
		compress = suspforce

	# Force
	if is_colliding():
		hitposition = get_collision_point()
		directional_force.y = _calculate_suspension(elasticity, damping, damping_rebound, velocity.y, abs(cast_to.y), global_translation, get_collision_point())
	else:
		geometry_node.position = cast_to

	geometry_node.position.y += w_size

	var inned = (abs(cambered) + a_geometry_4) / 90.0

	inned *= inned - a_geometry_4 / 90.0

	geometry_node.position.x = - inned * translation.x

	camber_node.rotation.z = - (deg_to_rad(-c_camber * float(translation.x < 0.0) + c_camber * float(translation.x > 0.0)) - deg_to_rad(-cambered * float(translation.x < 0.0) + cambered * float(translation.x > 0.0)) * geometry_camber_gain_factor)

	var g

	axle_position = geometry_node.position.y

	g = (geometry_node.position.y + (abs(cast_to.y) - geometry_horizontal_tuck)) / (abs(translation.x) + a_geometry_3 + 1.0)
	g /= abs(g) + 1.0
	cambered = (g * 90.0) - a_geometry_4

	animation_node.position = geometry_node.position

	car.apply_force((velocity_2_node.global_transform.basis.orthonormalized() * Vector3.UP) * directional_force.y, hitposition - car.global_transform.origin)
#endregion

func _calculate_suspension(elasticity: float, damping: float, damping_rebound: float, linearz: float, g_range: float, located: Vector3, hit_located: Vector3) -> float:
	# This function is unchanged from the previous, metric-corrected version.
	# It correctly uses lerp for dynamic stiffness and handles bottoming out.
	geometry_node.global_position = hit_located

	velocity_node.global_transform = _align_axis_to_vector(velocity_node.global_transform, get_collision_normal())
	velocity_2_node.global_transform = _align_axis_to_vector(velocity_2_node.global_transform, get_collision_normal())

	var incline = (get_collision_normal() - global_transform.basis.y).length()
	incline = (incline - geometry_incline_free_zone) / (1.0 - geometry_incline_free_zone) if geometry_incline_free_zone < 1.0 else 0.0
	incline = clamp(incline * geometry_incline_impact_factor, 0.0, 1.0)

	if geometry_node.position.y > -g_range + suspension_max_travel * (1.0 - incline):
		geometry_node.position.y = - g_range + suspension_max_travel * (1.0 - incline)

	var compressed = g_range - located.distance_to(hit_located)
	var bottom_out_compression = compressed - suspension_max_travel
	var spring_compression = compressed - suspension_rest_length
	if spring_compression < 0.0: spring_compression = 0.0
	if bottom_out_compression < 0.0: bottom_out_compression = 0.0

	var bottom_out_stiffness = car.mass * 20.0
	var bottom_out_damping = car.mass * 2.0
	var effective_stiffness = lerp(elasticity, bottom_out_stiffness, incline)
	var damping_to_use = damping if linearz < 0 else damping_rebound
	var effective_damping = lerp(damping_to_use, bottom_out_damping, incline)

	var suspforce = spring_compression * effective_stiffness
	if bottom_out_compression > 0.0:
		suspforce += bottom_out_compression * (bottom_out_stiffness * 2.0)
		suspforce -= linearz * bottom_out_damping

	suspforce -= linearz * effective_damping

	rd = compressed
	if suspforce < 0.0: suspforce = 0.0
	return suspforce

#region Utils
func _align_axis_to_vector(xform: Transform3D, norm: Vector3) -> Transform3D:
	if norm == Vector3.ZERO: return xform # Safety check

	xform.basis.y = norm
	# Rebuild the X and Z axes to be perpendicular to the new Y-axis (the normal)
	# and to each other. This is a robust way to prevent the matrix from becoming invalid.
	xform.basis.x = xform.basis.z.cross(norm).normalized()
	xform.basis.z = xform.basis.x.cross(norm).normalized()
	return xform


func _calculate_wheel_size() -> float:
	return ((abs(int(tyre_width_mm)) * ((abs(int(tyre_aspect_ratio)) * 2.0) / 100.0) + abs(int(rim_diameter_inches)) * 25.4) * 0.001) / 2.0
#endregion