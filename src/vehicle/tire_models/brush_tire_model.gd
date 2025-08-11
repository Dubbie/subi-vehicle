class_name EnhancedBrushTireModel
extends BaseTireModel

# Basic friction parameters
@export var peak_longitudinal_friction: float = 1.1
@export var peak_lateral_friction: float = 1.0
@export var sliding_longitudinal_friction: float = 0.8
@export var sliding_lateral_friction: float = 0.7

# Peak slip characteristics
@export var peak_slip_ratio: float = 0.12 # ~12% slip for peak longitudinal force
@export var peak_slip_angle_deg: float = 6.0 # 6 degrees for peak lateral force

# Tire stiffness (force per unit deflection)
@export var longitudinal_stiffness_factor: float = 18.0
@export var lateral_stiffness_factor: float = 17.0

# Shape factors for slip curves
@export var longitudinal_shape_factor: float = 2.0 # Higher = sharper peak
@export var lateral_shape_factor: float = 2.5

# Load sensitivity
@export var load_sensitivity: float = 0.8 # How much load affects peak friction (0-1)
@export var camber_stiffness: float = 0.1 # Effect of camber angle

# Relaxation and transient behavior
@export var relaxation_length: float = 0.3
@export var transient_response: float = 0.8 # How quickly tire responds to slip changes

# Combined slip parameters
@export var ellipse_factor: float = 1.2 # Shape of friction ellipse (1.0 = circle)

# Bristle deflection state - stored per wheel instance
var longitudinal_deflection: float = 0.0
var lateral_deflection: float = 0.0
var previous_lon_slip: float = 0.0
var previous_lat_slip: float = 0.0

func calculate_forces(params: TireParams) -> Vector2:
    # Unpack parameters
    var vertical_load = params.vertical_load
    var lat_slip_rad = params.lat_slip_angle_rad
    var lon_slip_ratio = params.lon_slip_ratio
    var surface_friction = params.surface_friction
    var v_lon = params.longitudinal_velocity
    var delta = params.delta
    var wheel_angular_velocity = params.wheel_angular_velocity
    var wheel_radius = params.wheel_radius

    # Early exit for invalid conditions
    if vertical_load <= 0.0 or delta <= 0.0:
        _reset_deflections()
        return Vector2.ZERO

    # Calculate slip velocities
    var wheel_rim_speed = wheel_angular_velocity * wheel_radius
    var slip_velocity_lon = wheel_rim_speed - v_lon
    var slip_velocity_lat = abs(v_lon) * tan(lat_slip_rad)

    # Load-dependent characteristics
    var load_factor = _calculate_load_factor(vertical_load)
    var effective_peak_lon_friction = peak_longitudinal_friction * load_factor
    var effective_peak_lat_friction = peak_lateral_friction * load_factor

    # Calculate effective stiffness
    var effective_lon_stiffness = vertical_load * longitudinal_stiffness_factor * load_factor
    var effective_lat_stiffness = vertical_load * lateral_stiffness_factor * load_factor

    # Update bristle deflection with relaxation and transient response
    _update_deflections(slip_velocity_lon, slip_velocity_lat, wheel_rim_speed, v_lon, delta)

    # Calculate raw forces from bristle deflection
    var raw_force_lon = longitudinal_deflection * effective_lon_stiffness
    var raw_force_lat = lateral_deflection * effective_lat_stiffness

    # Calculate normalized slip values for curve lookup
    var normalized_lon_slip = abs(lon_slip_ratio) / peak_slip_ratio if peak_slip_ratio > 0 else 0.0
    var normalized_lat_slip = abs(lat_slip_rad) / deg_to_rad(peak_slip_angle_deg) if peak_slip_angle_deg > 0 else 0.0

    # Get force scaling factors from slip curves
    var lon_force_factor = _longitudinal_slip_curve(normalized_lon_slip)
    var lat_force_factor = _lateral_slip_curve(normalized_lat_slip)

    # Apply peak friction limits and curve shapes
    var max_lon_force = vertical_load * effective_peak_lon_friction * surface_friction
    var max_lat_force = vertical_load * effective_peak_lat_friction * surface_friction

    var force_lon = sign(raw_force_lon) * min(abs(raw_force_lon), max_lon_force * lon_force_factor)
    var force_lat = sign(raw_force_lat) * min(abs(raw_force_lat), max_lat_force * lat_force_factor)

    # Apply combined slip limitation (friction ellipse/circle)
    var limited_forces = _apply_combined_slip_limit(force_lon, force_lat, max_lon_force, max_lat_force)
    force_lon = limited_forces.y
    force_lat = limited_forces.x

    # Update deflections to match limited forces (important for next frame)
    if effective_lon_stiffness > 0:
        longitudinal_deflection = force_lon / effective_lon_stiffness
    if effective_lat_stiffness > 0:
        lateral_deflection = force_lat / effective_lat_stiffness

    # Store slip values for next frame's transient response
    previous_lon_slip = lon_slip_ratio
    previous_lat_slip = lat_slip_rad

    # Return forces: Vector2(lateral_force, longitudinal_force)
    return Vector2(force_lat, force_lon)

func _update_deflections(slip_vel_lon: float, slip_vel_lat: float, wheel_speed: float, vehicle_speed: float, delta: float) -> void:
    var relaxation_factor = 0.0
    if relaxation_length > 0:
        # Use the speed at which the tire is moving through the contact patch
        var contact_speed = max(wheel_speed, abs(vehicle_speed))
        if contact_speed > 0.01:
            relaxation_factor = contact_speed / relaxation_length

    # Apply transient response (how quickly tire responds to changes)
    var response_rate = transient_response * relaxation_factor

    # Update deflections with both relaxation and transient effects
    longitudinal_deflection += (slip_vel_lon - response_rate * longitudinal_deflection) * delta
    lateral_deflection += (slip_vel_lat - response_rate * lateral_deflection) * delta

func _calculate_load_factor(vertical_load: float) -> float:
    # Typical tire behavior: friction coefficient decreases slightly with increased load
    var normalized_load = vertical_load / 5000.0 # Normalize around 5000N typical load
    return 1.0 - (1.0 - load_sensitivity) * (normalized_load - 1.0) * 0.1

func _longitudinal_slip_curve(normalized_slip: float) -> float:
    # Modified Pacejka-like curve for longitudinal force
    if normalized_slip <= 1.0:
        # Build-up region: smooth curve to peak
        return normalized_slip * (2.0 - normalized_slip)
    else:
        # Sliding region: drop to sliding friction
        var sliding_ratio = sliding_longitudinal_friction / peak_longitudinal_friction
        var excess_slip = normalized_slip - 1.0
        return sliding_ratio + (1.0 - sliding_ratio) * exp(-longitudinal_shape_factor * excess_slip)

func _lateral_slip_curve(normalized_slip: float) -> float:
    # Modified Pacejka-like curve for lateral force
    if normalized_slip <= 1.0:
        # Build-up region: smooth curve to peak
        return sin(normalized_slip * PI * 0.5) * (2.0 - normalized_slip * 0.3)
    else:
        # Sliding region: drop to sliding friction
        var sliding_ratio = sliding_lateral_friction / peak_lateral_friction
        var excess_slip = normalized_slip - 1.0
        return sliding_ratio + (1.0 - sliding_ratio) * exp(-lateral_shape_factor * excess_slip)

func _apply_combined_slip_limit(force_lon: float, force_lat: float, max_lon: float, max_lat: float) -> Vector2:
    # Friction ellipse for combined slip
    if max_lon <= 0.0 or max_lat <= 0.0:
        return Vector2(force_lat, force_lon)

    # Normalize forces to create ellipse equation
    var normalized_lon = force_lon / max_lon
    var normalized_lat = force_lat / max_lat

    # Elliptical friction limit (ellipse_factor = 1.0 makes it circular)
    var ellipse_value = (normalized_lon * normalized_lon) + (normalized_lat * normalized_lat) / (ellipse_factor * ellipse_factor)

    if ellipse_value > 1.0 and ellipse_value > 0.0:
        var scale_factor = 1.0 / sqrt(ellipse_value)
        force_lon *= scale_factor
        force_lat *= scale_factor

    return Vector2(force_lat, force_lon)

func _reset_deflections() -> void:
    longitudinal_deflection = 0.0
    lateral_deflection = 0.0
    previous_lon_slip = 0.0
    previous_lat_slip = 0.0

# Debug function to visualize slip curves
func get_slip_curve_data(max_slip: float = 2.0, steps: int = 100) -> Dictionary:
    var lon_data = []
    var lat_data = []

    for i in range(steps + 1):
        var slip = (float(i) / steps) * max_slip
        var normalized_lon = slip / peak_slip_ratio
        var normalized_lat = slip / deg_to_rad(peak_slip_angle_deg)

        lon_data.append(Vector2(slip, _longitudinal_slip_curve(normalized_lon)))
        lat_data.append(Vector2(slip, _lateral_slip_curve(normalized_lat)))

    return {
        "longitudinal": lon_data,
        "lateral": lat_data
    }