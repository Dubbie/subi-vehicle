class_name PedalController
extends Node

## Manages the state and behavior of the vehicle's pedals.
## This node processes raw boolean inputs into gradual analog values,
## simulating the time it takes to press and release a physical pedal.

@export_group("Input Rates")
@export var on_throttle_rate: float = 0.2
@export var off_throttle_rate: float = 0.2

@export var on_brake_rate: float = 0.05
@export var off_brake_rate: float = 0.1

@export var on_handbrake_rate: float = 0.2
@export var off_handbrake_rate: float = 0.2

@export var on_clutch_rate: float = 0.2
@export var off_clutch_rate: float = 0.2

@export_group("Input Limits")
@export var max_throttle: float = 1.0
@export var max_brake: float = 1.0
@export var max_handbrake: float = 1.0
@export var max_clutch: float = 1.0

# Internal variables to store the current state of each pedal.
var throttle_pedal: float = 0.0
var brake_pedal: float = 0.0
var handbrake_pull: float = 0.0
var clutch_pedal: float = 0.0

# This is the main processing function for this controller.
# It takes the current state of inputs and the delta time.
func process_inputs(
	gas_pressed: bool,
	brake_pressed: bool,
	handbrake_pressed: bool,
	clutch_pressed: bool,
	delta: float
	):
	# The original script's rates were balanced around a 60 FPS physics tick.
	# We multiply by (delta * 60) to make the rates frame-rate independent
	# and consistent with the original feel.
	var tick_rate: float = delta * 60.0

	# Process Throttle Input
	if gas_pressed:
		throttle_pedal += on_throttle_rate * tick_rate
	else:
		throttle_pedal -= off_throttle_rate * tick_rate

	# Process Brake Input
	if brake_pressed:
		brake_pedal += on_brake_rate * tick_rate
	else:
		brake_pedal -= off_brake_rate * tick_rate

	# Process Handbrake Input
	if handbrake_pressed:
		handbrake_pull += on_handbrake_rate * tick_rate
	else:
		handbrake_pull -= off_handbrake_rate * tick_rate

	# Process Clutch Input
	if clutch_pressed:
		clutch_pedal += on_clutch_rate * tick_rate
	else:
		clutch_pedal -= off_clutch_rate * tick_rate

	# Clamp the values to ensure they stay within the defined min/max range.
	_apply_limits()


# Internal function to clamp all pedal values.
func _apply_limits():
	throttle_pedal = clamp(throttle_pedal, 0.0, max_throttle)
	brake_pedal = clamp(brake_pedal, 0.0, max_brake)
	handbrake_pull = clamp(handbrake_pull, 0.0, max_handbrake)
	clutch_pedal = clamp(clutch_pedal, 0.0, max_clutch)


# --- Public Getters ---
# These functions allow other scripts to safely access the pedal values.

func get_throttle() -> float:
	return throttle_pedal

func get_brake() -> float:
	return brake_pedal

func get_handbrake() -> float:
	return handbrake_pull

# Returns the clutch pedal position (0.0 = released, 1.0 = fully pressed).
func get_clutch() -> float:
	return clutch_pedal

# Helper function to get the clutch engagement value (1.0 = fully engaged, 0.0 = disengaged).
# This is often more useful for torque and transmission calculations.
func get_clutch_engagement() -> float:
	return 1.0 - clutch_pedal