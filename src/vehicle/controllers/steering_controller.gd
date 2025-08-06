class_name SteeringController
extends Node

@export_group("Input Rates")
## How quickly the steering moves towards the player's input.
@export var steer_rate: float = 3.0
## How quickly the steering returns to center when there is no input.
@export var center_rate: float = 6.0

# Internal variable to store the current steering position.
# -1.0 = Full Left, 0.0 = Center, 1.0 = Full Right
var steer_value: float = 0.0

# This is the main processing function for this controller.
# It takes the current steer axis input and the delta time.
func process_inputs(steer_input: float, delta: float):
	# If the player is providing any steering input...
	if steer_input != 0.0:
		# Move the current steer_value towards the input value.
		steer_value = move_toward(steer_value, steer_input, steer_rate * delta)
	else:
		# If there's no input, move the steer_value back to the center (0.0).
		steer_value = move_toward(steer_value, 0.0, center_rate * delta)

	steer_value = clamp(steer_value, -1.0, 1.0)


# --- Public Getters ---
func get_steer_value() -> float:
	return steer_value