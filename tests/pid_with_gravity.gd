@tool extends PIDController


func _physics_process(delta: float) -> void:
	var time := Time.get_ticks_msec() / 1000.0
	target = 5.0*Vector3(sin(0.25*TAU*time), 0.0, cos(0.25*TAU*time))
