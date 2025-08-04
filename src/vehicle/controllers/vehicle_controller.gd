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
@export var ga_shift_delay: int = 50
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
var sassistdel: int = 0 # Timer for the shift assist
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
	# 1. UPDATE VEHICLE STATE
	# ----------------------------------------------------------------------
	local_velocity = global_transform.basis.transposed() * linear_velocity
	local_angular_velocity = global_transform.basis.transposed() * angular_velocity

	# 2. CALL SIMULATION SUB-SYSTEMS
	# ----------------------------------------------------------------------
	aero()

	var acceleration = (linear_velocity - past_velocity) / delta
	g_force = acceleration / GRAVITY_ACCELERATION
	past_velocity = linear_velocity

	controls(delta)

	ratio = 10.0
	sassistdel -= 1

	transmission()
	limits()

	# Steering stuff

	# ABS pump timer, now using delta.
	abs_pump_timer -= delta
	if abs_pump_timer <= 0:
		brake_allowed += ABS_Pump_Force * delta
	else:
		brake_allowed -= ABS_Pump_Force * delta
	brake_allowed = clamp(brake_allowed, 0.0, 1.0)

	brakeline = pedal_controller.get_brake() * brake_allowed

	limdel -= 1

	# Calculate the final throttle value using frame-rate independent interpolation.
	var current_gas_pedal = pedal_controller.get_throttle()
	var target_throttle = current_gas_pedal # Placeholder for TCS factor

	if limdel < 0:
		throttle -= (throttle - (current_gas_pedal / (target_throttle * pedal_controller.get_clutch_engagement() + 1.0))) * throttle_response
	else:
		throttle -= throttle * throttle_response

	if rpm > rpm_limit:
		if throttle > throttle_limit:
			throttle = throttle_limit
			limdel = limiter_delay
	elif rpm < idle_rpm:
		if throttle < throttle_idle:
			throttle = throttle_idle

	# 4. CALCULATE ENGINE TORQUE
	# ----------------------------------------------------------------------
	var torque: float = 0.0
	if torque_curve and rpm > dead_rpm:
		torque = torque_curve.sample(rpm) * throttle

	rpmforce = (rpm / (abs(rpm * abs(rpm)) / (engine_friction) + 1.0))
	if rpm < dead_rpm:
		torque = 0.0
		rpmforce /= 5.0
		stalled = 1.0 - rpm / dead_rpm
	else:
		stalled = 0.0

	rpmforce += (rpm * (engine_drag))
	rpmforce -= torque
	rpm -= rpmforce * rev_speed

	drivetrain()

	print("RPM: %d\tThrottle: %.2f\tGear: %d\tClutch: %.2f" % [rpm, throttle, gear, pedal_controller.get_clutch_engagement()])

# --- Placeholder Methods ---
func aero():
	pass

func controls(d: float):
	var gas_input = Input.is_action_pressed("gas")
	var brake_input = Input.is_action_pressed("brake")
	var handbrake_input = Input.is_action_pressed("handbrake")
	var clutch_input = Input.is_action_pressed("clutch")

	pedal_controller.process_inputs(gas_input, brake_input, handbrake_input, clutch_input, d)

# Gear Assistant
# 0 = Shift delay
# 1 = Level
# 2 = Speed Influece  (not in the new export)
# 3 = Downshift RPM iter
# 4 = Upshift RPM
# 5 = Clutch out RPM
# 6 = throttle input frames

func transmission():
	shift_up = Input.is_action_just_pressed("shift_up")
	shift_down = Input.is_action_just_pressed("shift_down")

	var gas = pedal_controller.get_throttle() > 0.01
	var brake = pedal_controller.get_brake() > 0.01

	if gear > 0:
		ratio = gear_ratios[gear - 1] * final_drive_ratio
	elif gear == -1:
		ratio = reverse_ratio * final_drive_ratio

	if transmission_type == TransmissionType.FULLY_MANUAL:
		# Clutch is handled in the pedal controller now
		if gear > 0:
			ratio = gear_ratios[gear - 1] * final_drive_ratio
		elif gear == -1:
			ratio = reverse_ratio * final_drive_ratio

		if ga_level == GearAssistantLevel.MANUAL:
			if shift_up:
				shift_up = false
				if gear < len(gear_ratios):
					if gearstress < gear_gap:
						actualgear += 1
			if shift_down:
				shift_down = false
				if gear > -1:
					if gearstress < gear_gap:
						actualgear -= 1
		elif ga_level == GearAssistantLevel.SEMI_AUTO:
			if rpm < ga_clutch_out_rpm:
				var irga_ca = (ga_clutch_out_rpm - rpm) / (ga_clutch_out_rpm - idle_rpm)
				pedal_controller.clutch_pedal = irga_ca * irga_ca
				if pedal_controller.clutch_pedal > 1.0:
					pedal_controller.clutch_pedal = 1.0
			else:
				if not gasrestricted and not revmatch:
					clutchin = false

			if shift_up:
				shift_up = false
				if gear < len(gear_ratios):
					if rpm < ga_clutch_out_rpm:
						actualgear += 1
					else:
						if actualgear < 1:
							actualgear += 1
							if rpm > ga_clutch_out_rpm:
								clutchin = false
						else:
							if sassistdel > 0:
								actualgear += 1
							sassistdel = ga_shift_delay / 2
							sassiststep = -4

							clutchin = true
							gasrestricted = true
			elif shift_down:
				shift_down = false
				if gear > -1:
					if rpm < ga_clutch_out_rpm:
						actualgear -= 1
					else:
						if actualgear == 0 or actualgear == 1:
							actualgear -= 1
							clutchin = false
						else:
							if sassistdel > 0:
								actualgear -= 1
							sassistdel = ga_shift_delay / 2
							sassiststep = -2

							clutchin = true
							revmatch = true
							gasrestricted = false
		elif ga_level == GearAssistantLevel.FULL_AUTO:
			var assist_shift_speed = (ga_upshift_rpm / ratio) * ga_speed_influence
			var assist_downshift_speed = (ga_downshift_rpm / abs((gear_ratios[gear - 2] * final_drive_ratio))) * ga_speed_influence

			if gear == 0:
				if gas:
					sassistdel -= 1
					if sassistdel < 0:
						actualgear = 1
				elif brake:
					sassistdel -= 1
					if sassistdel < 0:
						actualgear = -1
				else:
					sassistdel = 144
			elif linear_velocity.length() < 5:
				if not gas and gear == 1 or not brake and gear == -1:
					sassistdel = 144
					actualgear = 0
			if sassiststep == 0:
				if rpm < ga_clutch_out_rpm:
					var irga_ca = (ga_clutch_out_rpm - rpm) / (ga_clutch_out_rpm - idle_rpm)
					pedal_controller.clutch_pedal = irga_ca * irga_ca
					if pedal_controller.clutch_pedal > 1.0:
						pedal_controller.clutch_pedal = 1.0
				else:
					clutchin = false
				if not gear == -1:
					if gear < len(gear_ratios) and linear_velocity.length() > assist_shift_speed:
						sassistdel = ga_shift_delay / 2
						sassiststep = -4

						clutchin = true
						gasrestricted = true
					if gear > 1 and linear_velocity.length() < assist_downshift_speed:
						sassistdel = ga_shift_delay / 2
						sassiststep = -2

						clutchin = true
						gasrestricted = true
						revmatch = true

		if sassiststep == -4 and sassistdel < 0:
			sassistdel = ga_shift_delay / 2
			if gear < len(gear_ratios):
				actualgear += 1
			sassiststep = -3
		elif sassiststep == -3 and sassistdel < 0:
			if rpm > ga_clutch_out_rpm:
				clutchin = false
			if sassistdel < -ga_throttle_allowed:
				sassiststep = 0
				gasrestricted = false
		elif sassiststep == -2 and sassistdel < 0:
			sassiststep = 0
			if gear > -1:
				actualgear -= 1
			if rpm > ga_clutch_out_rpm:
				clutchin = false
			gasrestricted = false
			revmatch = false

		gear = actualgear

func limits():
	pass

func drivetrain():
	rpmcs -= (rpmcs - drivetrain_resistance)
	rpmcs += rpmcs * clutch_elasticity
	rpmcs -= rpmcs * pedal_controller.get_clutch_engagement()
	wob = clutch_wobble * pedal_controller.clutch_pedal
	wob *= ratio * clutch_wobble_rate
	rpmcs -= (rpmcs - drivetrain_resistance) * (1.0 / (wob + 1.0))

	var magic_rpm_number: float = 1.475
	if gear < 0:
		rpm -= rpmcs * rev_speed / magic_rpm_number
	else:
		rpm += rpmcs * rev_speed / magic_rpm_number

	gearstress = (abs(drivetrain_resistance) * stress_factor) * pedal_controller.clutch_pedal
	var stabled = ratio * 0.9 + 0.1
	ds_weight = driveshaft_inertia_factor / stabled

	if drivetrain_resistance > 0.0:
		diff_locked = abs(drivetrain_resistance / ds_weight) * (diff_coast_locking / 100.0) + diff_preload
	else:
		diff_locked = abs(drivetrain_resistance / ds_weight) * (diff_power_locking / 100.0) + diff_preload

	if diff_locked < 0.0:
		diff_locked = 0.0
	elif diff_locked > 1.0:
		diff_locked = 1.0

	if wv_difference > 0.0:
		diff_center_locked = abs(wv_difference) * (diff_centre_coast_locking / 10.0) + diff_centre_preload
	else:
		diff_center_locked = abs(wv_difference) * (diff_centre_power_locking / 10.0) + diff_centre_preload
	if diff_center_locked < 0.0 or len(powered_wheels) < 4:
		diff_center_locked = 0.0
	elif diff_center_locked > 1.0:
		diff_center_locked = 1.0

	var maxd: WheelController = _get_fastest_driven_wheel()
	var what: float = 0.0

	var float_reduction: float = clutch_float_reduction

	if dsweightrun > 0.0:
		float_reduction = clutch_float_reduction / dsweightrun
	else:
		float_reduction = 0.0

	var stabling = - (clutch_gear_ratio_ratio_threshold - ratio * drivewheels_size) * clutch_threshold_stable
	if stabling < 0.0:
		stabling = 0.0

	currentstable = clutch_stability_factor * stabling
	currentstable *= rev_speed / magic_rpm_number

	if dsweightrun > 0.0:
		what = (rpm - (((rpmforce * float_reduction) * pow(currentstable, 1.0)) / (ds_weight / dsweightrun)))
	else:
		what = rpm

	if gear < 0.0:
		dist = maxd.wv + what / ratio
	else:
		dist = maxd.wv - what / ratio

	dist *= (pedal_controller.clutch_pedal * pedal_controller.clutch_pedal)

	if gear == 0:
		dist *= 0.0

	wv_difference = 0.0
	drivewheels_size = 0.0
	for wheel in powered_wheels:
		drivewheels_size += wheel.w_size / len(powered_wheels)
		wheel.c_p = wheel.power_bias
		wv_difference += ((wheel.wv - what / ratio) / (len(powered_wheels))) * (pedal_controller.clutch_pedal * pedal_controller.clutch_pedal)
		if gear < 0:
			wheel.dist = dist * (1 - diff_center_locked) + (wheel.wv + what / ratio) * diff_center_locked
		else:
			wheel.dist = dist * (1 - diff_center_locked) - (wheel.wv - what / ratio) * diff_center_locked
		if gear == 0:
			wheel.dist *= 0.0

	ga_speed_influence = drivewheels_size
	drivetrain_resistance = 0.0
	dsweightrun = dsweight
	stress = 0.0

func _get_fastest_driven_wheel() -> WheelController:
	var val = -10000000000000000000000000000000000.0
	var obj: WheelController = null

	for wheel in powered_wheels:
		val = max(val, abs(wheel.absolute_wv))

		if val == abs(wheel.absolute_wv):
			obj = wheel

	return obj

func _get_slowest_driven_wheel() -> WheelController:
	var val = 10000000000000000000000000000000000.0
	var obj: WheelController = null

	for wheel in powered_wheels:
		val = min(val, abs(wheel.absolute_wv))

		if val == abs(wheel.absolute_wv):
			obj = wheel

	return obj
