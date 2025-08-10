class_name ChaseCamera
extends Node3D

# --- Camera States ---
enum State {FOLLOW, FREE_LOOK}

# --- EXPORT VARIABLES ---
@export_group("Target")
@export var target: Node3D

@export_group("Camera Behavior")
@export var follow_speed: float = 24.0
@export var rotation_speed: float = 7.0

@export_group("Offsets")
@export var distance: float = 5.0
@export var height: float = 2.0

@export_group("Mouse Look")
@export var mouse_sensitivity: float = 0.004
@export var free_look_timeout: float = 2.0
@export var return_to_follow_speed: float = 5.0

# --- NODES ---
@onready var spring_arm: SpringArm3D = $SpringArm3D

# --- PRIVATE VARIABLES ---
var _current_state: State = State.FOLLOW
var _yaw_offset: float = 0.0
var _pitch_offset: float = 0.0
var _time_since_last_input: float = 0.0

func _ready():
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
	spring_arm.spring_length = distance
	spring_arm.position.y = height
	_time_since_last_input = free_look_timeout

	if is_instance_valid(target):
		global_position = target.global_position

func _input(event: InputEvent):
	if event is InputEventMouseMotion and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		_current_state = State.FREE_LOOK
		_time_since_last_input = 0.0

		# Apply mouse movement to offsets
		_yaw_offset -= event.relative.x * mouse_sensitivity
		_pitch_offset -= event.relative.y * mouse_sensitivity

		# Clamp pitch to prevent flipping
		_pitch_offset = clamp(_pitch_offset, deg_to_rad(-40), deg_to_rad(30))

	if event.is_action_pressed("ui_cancel"):
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED else Input.MOUSE_MODE_CAPTURED

func _physics_process(delta: float):
	if not is_instance_valid(target):
		return

	# Follow target position
	global_position = global_position.lerp(target.global_position, follow_speed * delta)

	# Handle timeout
	_time_since_last_input += delta
	if _time_since_last_input > free_look_timeout:
		_current_state = State.FOLLOW

	# Get target's yaw (ignore pitch and roll for stability)
	var target_yaw = target.global_rotation.y

	# In follow mode, gradually return offsets to zero
	if _current_state == State.FOLLOW:
		_yaw_offset = lerp_angle(_yaw_offset, 0.0, return_to_follow_speed * delta)
		_pitch_offset = lerp(_pitch_offset, 0.0, return_to_follow_speed * delta)

	# Apply target yaw + our yaw offset
	var final_yaw = target_yaw + _yaw_offset
	rotation.y = lerp_angle(rotation.y, final_yaw, rotation_speed * delta)

	# Apply pitch offset to spring arm
	spring_arm.rotation.x = lerp_angle(spring_arm.rotation.x, _pitch_offset, rotation_speed * delta)

	# Ensure basis stays clean
	global_transform.basis = global_transform.basis.orthonormalized()

	# Update spring arm settings
	spring_arm.spring_length = distance
	spring_arm.position.y = height
