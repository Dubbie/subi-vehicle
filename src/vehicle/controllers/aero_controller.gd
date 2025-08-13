class_name AeroController
extends Node

#region Configuration
@export_group("Aerodynamics")
## The frontal area of the vehicle in square meters (m^2).
@export var frontal_area: float = 2.2
## The drag coefficient (Cd). A lower value is more aerodynamic.
## Typical values for modern cars are between 0.25 and 0.35.
@export var drag_coefficient: float = 0.3
## The lift coefficient (Cl). A NEGATIVE value creates downforce.
## A sports car might have -0.3 to -0.8. A regular car is closer to 0.
@export var lift_coefficient: float = -0.4

@export_group("Stability")
## How much the car resists turning at high speed. It applies a torque opposing yaw.
## This factor is sensitive; good values are typically very small (e.g., 0.01 to 0.1).
@export var yaw_stability_factor: float = 0.05

@export_group("Application Points")
## A Marker3D node indicating where the drag and downforce are applied.
## Placing this above the center of mass can cause the car to pitch forward slightly.
@export var aero_center_point: Marker3D

@export_group("Debug")
@export var debug_mode: bool = true
#endregion

#region State
# A reference to the vehicle's main rigidbody.
var vehicle_controller: VehicleController

# Constants
const AIR_DENSITY: float = 1.225 # kg/m^3
#endregion

func _physics_process(_delta: float):
	# Exit if the controller hasn't been properly initialized.
	if not is_instance_valid(vehicle_controller):
		return

	# Get the vehicle's current velocity and speed.
	var velocity = vehicle_controller.linear_velocity
	var speed = velocity.length()

	# No need to calculate aero forces at very low speeds.
	if speed < 1.0:
		return

	# --- Common Physics Calculation ---
	# This part of the formula is shared between drag and downforce: 0.5 * rho * v^2
	var dynamic_pressure = 0.5 * AIR_DENSITY * speed * speed

	# --- 1. Calculate Drag Force ---
	var drag_magnitude = dynamic_pressure * frontal_area * drag_coefficient
	# Drag force always acts in the opposite direction of the vehicle's velocity.
	var drag_force = - velocity.normalized() * drag_magnitude

	# --- 2. Calculate Downforce ---
	var downforce_magnitude = dynamic_pressure * frontal_area * lift_coefficient
	# Downforce acts downwards relative to the car's orientation.
	# Since lift_coefficient is negative, this force will point downwards into the ground.
	var downforce_vector = vehicle_controller.global_transform.basis.y * downforce_magnitude

	# --- 3. Calculate Yaw Stability Torque ---
	# This torque makes the car harder to turn at high speeds.
	var angular_velocity = vehicle_controller.angular_velocity
	# Get the yaw rate (rotation around the car's local 'up' axis).
	var yaw_rate = vehicle_controller.global_transform.basis.y.dot(angular_velocity)
	# The torque opposes the current yaw rate and scales with speed.
	var stability_torque_magnitude = - yaw_rate * yaw_stability_factor * speed
	var stability_torque = vehicle_controller.global_transform.basis.y * stability_torque_magnitude

	# --- Apply Forces and Torques ---
	# Determine the point to apply the main aero forces.
	var application_point = vehicle_controller.global_position + vehicle_controller.center_of_mass
	if is_instance_valid(aero_center_point):
		application_point = aero_center_point.global_position

	# Apply the forces to the vehicle's rigidbody.
	vehicle_controller.apply_force(drag_force, application_point - vehicle_controller.global_position)
	vehicle_controller.apply_force(downforce_vector, application_point - vehicle_controller.global_position)
	vehicle_controller.apply_torque(stability_torque)

	# --- Debug Drawing ---
	if debug_mode:
		_draw_debug_forces(application_point, drag_force, downforce_vector)

# Public API used by the VehicleController to set itself up.
func initialize(p_vehicle_controller: VehicleController):
	vehicle_controller = p_vehicle_controller

func _draw_debug_forces(point: Vector3, drag: Vector3, downforce: Vector3):
	# Scale forces for better visualization
	var drag_viz = drag / 100.0
	var downforce_viz = downforce / 100.0

	DebugDraw3D.draw_arrow(point, point + drag_viz, Color.BLUE, 0.1)
	DebugDraw3D.draw_arrow(point, point + downforce_viz, Color.GREEN, 0.1)
