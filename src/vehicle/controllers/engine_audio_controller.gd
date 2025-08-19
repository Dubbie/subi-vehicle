class_name EngineSoundController
extends Node3D

## The vehicle controller to which this controller is attached.
@export var vehicle_controller: VehicleController
## The minimum pitch of the sound.
@export var min_pitch: float = 0.5
## The maximum pitch of the sound.
@export var max_pitch: float = 1.5

## The vehicle's actual maximum engine RPM. This is used to scale the sound.
@export var vehicle_max_rpm: float = 8000.0
## The maximum RPM of the provided engine sound recordings.
## If set to 0, it will be auto-detected from the sound_loops array.
@export var sounds_max_rpm: float = 0.0

# In the inspector, you will create and assign EngineSoundLoop resources here.
@export var sound_loops: Array[EngineSoundLoop] = []

# Audio players for crossfading
@onready var on_player_a: AudioStreamPlayer3D = $OnPlayerA
@onready var on_player_b: AudioStreamPlayer3D = $OnPlayerB
@onready var off_player_a: AudioStreamPlayer3D = $OffPlayerA
@onready var off_player_b: AudioStreamPlayer3D = $OffPlayerB

# We'll use these to track which player is handling which part of the crossfade
var _on_players: Array[AudioStreamPlayer3D]
var _off_players: Array[AudioStreamPlayer3D]
var _lower_loop_idx: int = 0
var _upper_loop_idx: int = 1
var _rpm_scale_factor: float = 1.0

func _ready():
	if not vehicle_controller or sound_loops.size() < 2:
		set_process(false)
		return

	sound_loops.sort_custom(func(a, b): return a.rpm < b.rpm)

	var recorded_max_rpm = sounds_max_rpm
	if recorded_max_rpm <= 0:
		recorded_max_rpm = sound_loops.back().rpm

	if vehicle_max_rpm > 0 and recorded_max_rpm > 0:
		_rpm_scale_factor = recorded_max_rpm / vehicle_max_rpm

	# Initial player setup
	_on_players = [on_player_a, on_player_b]
	_off_players = [off_player_a, off_player_b]

	# Pre-assign the first two loops to our players
	_on_players[0].stream = sound_loops[0].on_throttle_sound
	_on_players[1].stream = sound_loops[1].on_throttle_sound
	_off_players[0].stream = sound_loops[0].off_throttle_sound
	_off_players[1].stream = sound_loops[1].off_throttle_sound

	for player in get_children():
		if player is AudioStreamPlayer3D:
			player.volume_db = -80.0
			player.call_deferred("play")

func _process(_delta):
	if not is_instance_valid(vehicle_controller):
		return

	var current_rpm: float = vehicle_controller.get_engine_rpm()
	var throttle_input: float = vehicle_controller.pedal_controller.get_throttle()
	var scaled_rpm = current_rpm * _rpm_scale_factor
	update_engine_sound(scaled_rpm, throttle_input)

func update_engine_sound(rpm: float, throttle: float) -> void:
	# Check if the RPM has crossed a boundary, requiring us to change loops
	if rpm < sound_loops[_lower_loop_idx].rpm and _lower_loop_idx > 0:
		# RPM decreased, we need to shift our loops down
		_lower_loop_idx -= 1
		_upper_loop_idx -= 1

		# The player that was handling the upper loop is now free.
		# Proactively assign it the new lower loop sound.
		_on_players.reverse()
		_off_players.reverse()
		_on_players[0].stream = sound_loops[_lower_loop_idx].on_throttle_sound
		_off_players[0].stream = sound_loops[_lower_loop_idx].off_throttle_sound

	elif rpm > sound_loops[_upper_loop_idx].rpm and _upper_loop_idx < sound_loops.size() - 1:
		# RPM increased, we need to shift our loops up
		_lower_loop_idx += 1
		_upper_loop_idx += 1

		# The player that was handling the lower loop is now free.
		# Proactively assign it the new upper loop sound.
		_on_players.reverse()
		_off_players.reverse()
		_on_players[1].stream = sound_loops[_upper_loop_idx].on_throttle_sound
		_off_players[1].stream = sound_loops[_upper_loop_idx].off_throttle_sound

	# The rest of the logic remains the same, but uses our state variables
	var lower_loop = sound_loops[_lower_loop_idx]
	var upper_loop = sound_loops[_upper_loop_idx]

	# Calculate pitch and crossfade volume
	var lower_pitch = clamp(rpm / lower_loop.rpm, min_pitch, max_pitch)
	var upper_pitch = clamp(rpm / upper_loop.rpm, min_pitch, max_pitch)

	var crossfade_ratio = inverse_lerp(lower_loop.rpm, upper_loop.rpm, rpm)
	crossfade_ratio = clamp(crossfade_ratio, 0.0, 1.0)

	# Constant Power Fade
	var fade_angle = crossfade_ratio * PI / 2.0
	var lower_volume = cos(fade_angle)
	var upper_volume = sin(fade_angle)

	# Blend between on-throttle and off-throttle sounds
	var on_throttle_volume = throttle
	var off_throttle_volume = 1.0 - throttle

	# Apply final pitch and volume to all four players
	_on_players[0].pitch_scale = lower_pitch
	_on_players[0].volume_db = linear_to_db(lower_volume * on_throttle_volume)

	_on_players[1].pitch_scale = upper_pitch
	_on_players[1].volume_db = linear_to_db(upper_volume * on_throttle_volume)

	_off_players[0].pitch_scale = lower_pitch
	_off_players[0].volume_db = linear_to_db(lower_volume * off_throttle_volume)

	_off_players[1].pitch_scale = upper_pitch
	_off_players[1].volume_db = linear_to_db(upper_volume * off_throttle_volume)

	# Forcefully ensure players are always in a playing state.
	for player in get_children():
		if player is AudioStreamPlayer3D and not player.is_playing():
			player.play()