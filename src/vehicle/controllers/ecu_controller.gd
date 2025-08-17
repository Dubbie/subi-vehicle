class_name ECUController
extends Node

@export_group("Rev Limiter")
## The engine speed at which the fuel cut engages. (RPM)
@export var rev_limit_rpm: float = 7000.0
## The engine speed below which fuel is restored after a cut. (RPM)
## This should be lower than rev_limit_rpm to create hysteresis.
@export var rev_resume_rpm: float = 6800.0

var fuel_cut_active: bool = false

## Updates the rev limiter state based on the current engine speed.
## Returns true if the fuel should be cut.
func is_fuel_cut_active(current_rpm: float) -> bool:
	if fuel_cut_active:
		# If fuel is currently cut, check if the RPM has dropped enough to resume.
		if current_rpm < rev_resume_rpm:
			fuel_cut_active = false
	else:
		# If fuel is not cut, check if the RPM has exceeded the limit.
		if current_rpm >= rev_limit_rpm:
			fuel_cut_active = true

	return fuel_cut_active