@tool extends PIDController


func _integrate_state(current_state: Dictionary, delta: float) -> void:
	current_state["value_dot"] += Vector3.DOWN*9.81*delta
	current_state["value"] += current_state["value_dot"]*delta

	var time := Time.get_ticks_msec() / 1000.0
	target_value = 5.0*Vector3(sin(0.25*TAU*time), 0.0, cos(0.25*TAU*time))
