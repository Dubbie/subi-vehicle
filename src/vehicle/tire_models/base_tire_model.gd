class_name BaseTireModel
extends Resource

# This class defines the interface for all tire models.
# It is meant to be extended, not used directly.

# --- Public Interface ---

# All tire models MUST implement this function.
# It takes the wheel's state and returns the calculated local-space tire forces.
func calculate_forces(_params: TireParams) -> Vector2:
	push_warning("calculate_forces() is not implemented in the extending script.")
	return Vector2.ZERO