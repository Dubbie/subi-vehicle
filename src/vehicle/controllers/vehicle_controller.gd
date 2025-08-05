# VehicleBody.gd
extends RigidBody3D
class_name VehicleBody

enum GearAssistantLevel {MANUAL, SEMI_AUTO, FULL_AUTO}

enum TransmissionType {FULLY_MANUAL, AUTOMATIC, CVT, SEMI_AUTO}

# --- Components ---
@export var pedal_controller: PedalController

# --- Vehicle Configuration ---
@export_group("Chassis")
@export var Weight: float = 900.0 # Mass in kilograms (kg)

@export_group("Limits")
@export var throttle_limit: float = 0.0

@export_group("Engine")
@export var torque_curve: Curve # Assign a Curve resource in the Inspector
@export var idle_rpm: float = 800.0
@export var rpm_limit: float = 7000.0
@export var dead_rpm: float = 100.0 # RPM below which the engine is considered stalled
# Note: limiter_delay is now in seconds, not frames (e.g., 0.07)
@export var limiter_delay: int = 12
@export var throttle_idle: float = 0.25
# NOTE: These values will need retuning as they are now used with 'delta'.
@export var rev_speed: float = 20000.0 # Acts as torque->acceleration multiplier (inverse inertia)
@export var engine_friction: float = 18000.0 # Coefficient for friction that scales with RPM^2
@export var engine_drag: float = 1.0 # Coefficient for linear RPM drag
@export var throttle_response: float = 15.0 # Rate of throttle change per second

@export_group("Braking")
@export var ABS_Enabled: bool = true
# Note: ABS_Pump_Time is now in seconds, not frames (e.g., 0.02)
@export var ABS_Pump_Time: float = 0.02
@export var ABS_Pump_Force: float = 25.0 # Rate of brake pressure change per second

@export_group("Drivetrain")
@export var driveshaft_inertia_factor: float = 150.0 # Corresponds to DSWeight
@export var differential_lock_factor: float = 0.1
@export var diff_coast_locking: float = 0.0
@export var diff_power_locking: float = 0.0
@export var diff_preload: float = 0.0
@export var diff_centre_coast_locking: float = 0.0
@export var diff_centre_power_locking: float = 0.0
@export var diff_centre_preload: float = 0.0
@export var stress_factor: float = 1.0
@export var powered_wheels: Array[WheelController]

@export_group("Transmission")
@export var transmission_type: TransmissionType = TransmissionType.FULLY_MANUAL
@export var gear_ratios: Array[float] = [3.2, 1.894, 1.259, 1.0, 0.8]
@export var reverse_ratio: float = 3.0
@export var final_drive_ratio: float = 3.694
@export var gear_gap: float = 60.0

@export_group("Clutch")
@export var clutch_grip_torque: float = 400.0
@export var clutch_stability_factor: float = 0.5
@export var clutch_elasticity: float = 0.5
@export var clutch_wobble: float = 0.0
@export var clutch_wobble_rate: float = 0.0
@export var clutch_float_reduction: float = 20.0
@export var clutch_gear_ratio_ratio_threshold: float = 200.0
@export var clutch_threshold_stable: float = 0.0

@export_group("Gear Assistant")
## The shift delay, in frames
@export var ga_shift_delay: float = 50
## Assistance level
@export var ga_level: GearAssistantLevel = GearAssistantLevel.FULL_AUTO
## Downshift rpm iteration
@export var ga_downshift_rpm: float = 6000.0
## Upshift rpm iteration
@export var ga_upshift_rpm: float = 6200.0
## Clutch out RPM
@export var ga_clutch_out_rpm: float = 3000.0
## Throttle input allowed after shifting delay (in frames)
@export var ga_throttle_allowed: int = 5

# --- Internal Physics State (Class-Level Variables) ---
var throttle: float = 0.0
var rpm: float = 0.0
var rpmcs: float = 0.0
var wob: float = 0.0
var rpmforce: float = 0.0 # The net rotational acceleration on the engine (in RPM/sec)
var gear: int = 0
var currentstable: float = 0.0
var drivetrain_resistance: float = 0.0 # Total resistance from wheels fed back to the engine.
var ratio: float = 0.0
var stalled: float = 0.0
var gearstress: float = 0.0
var drivewheels_size: float = 0.0
var ds_weight: float = 0.0 # Excuse me what?
var dsweight: float = 0.0 # Excuse me what?
var dsweightrun: float = 0.0
var diff_locked: float = 0.0
var diff_center_locked: float = 0.0
var wv_difference: float = 0.0
var dist: float = 0.0
var stress: float = 0.0

var local_velocity: Vector3 = Vector3.ZERO
var local_angular_velocity: Vector3 = Vector3.ZERO

var limdel: int = 0 # Timer for the rev limiter
var sassistdel: float = 0 # Timer for the shift assist
var sassiststep: int = 0
var ga_speed_influence: float = 0.0
var actualgear: int = 0
var gasrestricted: bool = false
var clutchin: bool = false
var revmatch: bool = false

var shift_up: bool = false
var shift_down: bool = false

var past_velocity: Vector3 = Vector3.ZERO
var g_force: Vector3 = Vector3.ZERO

var brakeline: float = 0.0
var brake_allowed: float = 1.0
var abs_pump_timer: float = 0.0

const GRAVITY_ACCELERATION: float = 9.80665


func _ready():
	mass = Weight
	rpm = idle_rpm

# The main physics loop. All time-dependent calculations now use the 'delta' parameter.
func _physics_process(delta: float):
	controls(delta)

func controls(d: float):
	var gas_input = Input.is_action_pressed("gas")
	var brake_input = Input.is_action_pressed("brake")
	var handbrake_input = Input.is_action_pressed("handbrake")
	var clutch_input = Input.is_action_pressed("clutch")

	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, d)
