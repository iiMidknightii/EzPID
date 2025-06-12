#include "pid_controller.hpp"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/templates/local_vector.hpp>

using namespace godot;

PIDController::~PIDController() {
    cached_node = nullptr;
    cached_controlled_properties.clear();
}

_ALWAYS_INLINE_ double angle_difference(double p_from, double p_to) {
    double difference = fmod(p_to - p_from, Math_TAU);
    return fmod(2.0 * difference, Math_TAU) - difference;
}

void PIDController::set_p_gain(const Variant &p_p) {
    if (p_p != p_gain) {
        p_gain = p_p;
        state_length_validation = STATE_LENGTH_UNVALIDATED;
    }
}

Variant PIDController::get_p_gain() const {
    return p_gain;
}

void PIDController::set_i_gain(const Variant &p_i) {
    if (p_i != i_gain) {
        i_gain = p_i;
        state_length_validation = STATE_LENGTH_UNVALIDATED;
    }
}

Variant PIDController::get_i_gain() const {
    return i_gain;
}

void PIDController::set_d_gain(const Variant &p_d) {
    if (p_d != d_gain) {
        d_gain = p_d;
        state_length_validation = STATE_LENGTH_UNVALIDATED;
    }
}

Variant PIDController::get_d_gain() const {
    return d_gain;
}

void PIDController::set_value_is_angle(bool p_is_angle) {
    value_is_angle = p_is_angle;
}

bool PIDController::get_value_is_angle() const {
    return value_is_angle;
}

void PIDController::enable_overshoot_prevention(bool p_prevent) {
    overshoot_prevent_enabled = p_prevent;
}

bool PIDController::is_preventing_overshoot() const {
    return overshoot_prevent_enabled;
}

void PIDController::enable_kick_prevention(bool p_prevent) {
    kick_prevent_enabled = p_prevent;
}

bool PIDController::is_preventing_kick() const {
    return kick_prevent_enabled;
}

void PIDController::set_error_accumulation_limit(const Variant &p_limit) {
    if (p_limit != error_accumulation_limit) {
        error_accumulation_limit = p_limit;
        state_length_validation = STATE_LENGTH_UNVALIDATED;
    }
}

Variant PIDController::get_error_accumulation_limit() const {
    return error_accumulation_limit;
}

void PIDController::set_error_accumulation_decay(double p_decay) {
    error_accum_decay = Math::max(p_decay, 0.0);
}

double PIDController::get_error_accumulation_decay() const {
    return error_accum_decay;
}

void PIDController::set_control_output_limit(const Variant &p_limit) {
    if (p_limit != control_limit) {
        control_limit = p_limit;
        state_length_validation = STATE_LENGTH_UNVALIDATED;
    }
}

Variant PIDController::get_control_output_limit() const {
    return control_limit;
}

void PIDController::set_state_length(StateLength p_length) {
    ERR_FAIL_COND(p_length != SCALAR || p_length != VECTOR2 || p_length != VECTOR3 || p_length != VECTOR4);

    if (p_length != state_length) {
        state_length = p_length;
        _update_state_length();
    }
}

auto PIDController::get_state_length() const -> StateLength {
    return state_length;
}

void PIDController::set_target(const Variant &p_target) {
    if (p_target != target) {
        target = p_target;
        state_length_validation = STATE_LENGTH_UNVALIDATED;
    }
}

Variant PIDController::get_target() const {
    return target;
}

void PIDController::set_update_method(UpdateMethod p_method) {
    if (p_method != process_method) {
        ERR_FAIL_COND(p_method < PID_PROCESS_PHYSICS || p_method > PID_PROCESS_MANUAL);

        process_method = p_method;
        _update_process_modes();
    }
}

auto PIDController::get_update_method() const -> UpdateMethod {
    return process_method;
}

void PIDController::set_control_method(ControlMethod p_method) {
    if (p_method != control_method) {
        ERR_FAIL_COND(p_method < PID_CONTROL_AUTO_PROPERTY || p_method > PID_CONTROL_USER_SCRIPT);

        control_method = p_method;
        _update_state_length();
        update_configuration_warnings();
    }
}

auto PIDController::get_control_method() const -> ControlMethod {
    return control_method;
}

void PIDController::set_controlled_node(NodePath p_node) {
    if (p_node != controlled_node) {
        controlled_node = p_node;
        cached_node = nullptr;
        cached_controlled_properties.clear();
        _setup_node_control();
    }
}

NodePath PIDController::get_controlled_node() const {
    return controlled_node;
}

void PIDController::set_controlled_property(const StringName &p_name) {
    if (p_name != controlled_property) {
        controlled_property = p_name;
        _setup_node_control();
    }
}

StringName PIDController::get_controlled_property() const {
    return controlled_property;
}

Variant PIDController::get_error() const {
    return prev_error;
}

Variant PIDController::get_accumulated_error() const {
    return accumulated_error;
}

void PIDController::reset_accumulated_error() {
    accumulated_error = zero;
}

Variant PIDController::calculate_control_output(const Variant &p_value, double p_delta) {
    Variant out;
    if (!_validate_state_length()) {
        return out;
    }

    switch (state_length) {

    case SCALAR: {
        accumulated_error = double(accumulated_error) * Math::exp(-error_accum_decay * p_delta);

        double tmp_value = p_value;
        double tmp_target = target;
        double tmp_prev_value = prev_value;
        double tmp_prev_target = prev_target;
        double tmp_prev_error = prev_error;
        double tmp_accumulated_error = accumulated_error;

        double delta_target, error, delta_error;
        if (value_is_angle) {
            delta_target = angle_difference(tmp_prev_target, tmp_target);
            error = angle_difference(tmp_value, tmp_target);
            delta_error = angle_difference(tmp_prev_error, error);
        } else {
            delta_target = tmp_target - tmp_prev_target;
            error = tmp_target - tmp_value;
            delta_error = error - tmp_prev_error;
        }

        tmp_accumulated_error += error * p_delta;
        double tmp_error_limit = error_accumulation_limit;
        if (tmp_error_limit > 0.0) {
            tmp_accumulated_error = Math::clamp(tmp_accumulated_error, -tmp_error_limit, tmp_error_limit);
        }

        double p = double(p_gain) * error;
        double i = double(i_gain) * tmp_accumulated_error;
        double d = double(d_gain) * (kick_prevent_enabled ? delta_error - delta_target : delta_error) / p_delta;

        double tmp_out = p + i + d;
        double tmp_control_limit = control_limit;
        if (tmp_control_limit > 0.0) {
            tmp_out = Math::clamp(tmp_out, -tmp_control_limit, tmp_control_limit);
        }

        prev_target = target;
        prev_error = error;
        accumulated_error = tmp_accumulated_error;

        out = tmp_out;

    } break;

    case VECTOR2: {
        accumulated_error = Vector2(accumulated_error) * Math::exp(-error_accum_decay * p_delta);

        Vector2 tmp_value = p_value;
        Vector2 tmp_target = target;
        Vector2 tmp_prev_value = prev_value;
        Vector2 tmp_prev_target = prev_target;
        Vector2 tmp_prev_error = prev_error;
        Vector2 tmp_accumulated_error = accumulated_error;

        Vector2 delta_target, error, delta_error;
        if (value_is_angle) {
            delta_target.x = angle_difference(tmp_prev_target.x, tmp_target.x);
            error.x = angle_difference(tmp_value.x, tmp_target.x);
            delta_error.x = angle_difference(tmp_prev_error.x, error.x);
            
            delta_target.y = angle_difference(tmp_prev_target.y, tmp_target.y);
            error.y = angle_difference(tmp_value.y, tmp_target.y);
            delta_error.y = angle_difference(tmp_prev_error.y, error.y);

        } else {
            delta_target = tmp_target - tmp_prev_target;
            error = tmp_target - tmp_value;
            delta_error = error - tmp_prev_error;
        }

        tmp_accumulated_error += error * p_delta;
        Vector2 tmp_error_limit = error_accumulation_limit;
        if (tmp_error_limit.x > 0.0) {
            tmp_accumulated_error.x = Math::clamp(tmp_accumulated_error.x, -tmp_error_limit.x, tmp_error_limit.x);
        }
        if (tmp_error_limit.y > 0.0) {
            tmp_accumulated_error.y = Math::clamp(tmp_accumulated_error.y, -tmp_error_limit.y, tmp_error_limit.y);
        }

        Vector2 p = Vector2(p_gain) * error;
        Vector2 i = Vector2(i_gain) * tmp_accumulated_error;
        Vector2 d = Vector2(d_gain) * (kick_prevent_enabled ? delta_error - delta_target : delta_error) / p_delta;

        Vector2 tmp_out = p + i + d;
        Vector2 tmp_control_limit = control_limit;
        if (tmp_control_limit.x > 0.0) {
            tmp_out.x = Math::clamp(tmp_out.x, -tmp_control_limit.x, tmp_control_limit.x);
        }
        if (tmp_control_limit.y > 0.0) {
            tmp_out.y = Math::clamp(tmp_out.y, -tmp_control_limit.y, tmp_control_limit.y);
        }

        prev_target = target;
        prev_error = error;
        accumulated_error = tmp_accumulated_error;

        out = tmp_out;

    } break;

    case VECTOR3: {
        accumulated_error = Vector3(accumulated_error) * Math::exp(-error_accum_decay * p_delta);

        Vector3 tmp_value = p_value;
        Vector3 tmp_target = target;
        Vector3 tmp_prev_value = prev_value;
        Vector3 tmp_prev_target = prev_target;
        Vector3 tmp_prev_error = prev_error;
        Vector3 tmp_accumulated_error = accumulated_error;

        Vector3 delta_target, error, delta_error;
        if (value_is_angle) {
            delta_target.x = angle_difference(tmp_prev_target.x, tmp_target.x);
            error.x = angle_difference(tmp_value.x, tmp_target.x);
            delta_error.x = angle_difference(tmp_prev_error.x, error.x);
            
            delta_target.y = angle_difference(tmp_prev_target.y, tmp_target.y);
            error.y = angle_difference(tmp_value.y, tmp_target.y);
            delta_error.y = angle_difference(tmp_prev_error.y, error.y);
            
            delta_target.z = angle_difference(tmp_prev_target.z, tmp_target.z);
            error.z = angle_difference(tmp_value.z, tmp_target.z);
            delta_error.z = angle_difference(tmp_prev_error.z, error.z);

        } else {
            delta_target = tmp_target - tmp_prev_target;
            error = tmp_target - tmp_value;
            delta_error = error - tmp_prev_error;
        }

        tmp_accumulated_error += error * p_delta;
        Vector3 tmp_error_limit = error_accumulation_limit;
        if (tmp_error_limit.x > 0.0) {
            tmp_accumulated_error.x = Math::clamp(tmp_accumulated_error.x, -tmp_error_limit.x, tmp_error_limit.x);
        }
        if (tmp_error_limit.y > 0.0) {
            tmp_accumulated_error.y = Math::clamp(tmp_accumulated_error.y, -tmp_error_limit.y, tmp_error_limit.y);
        }
        if (tmp_error_limit.z > 0.0) {
            tmp_accumulated_error.z = Math::clamp(tmp_accumulated_error.z, -tmp_error_limit.z, tmp_error_limit.z);
        }

        Vector3 p = Vector3(p_gain) * error;
        Vector3 i = Vector3(i_gain) * tmp_accumulated_error;
        Vector3 d = Vector3(d_gain) * (kick_prevent_enabled ? delta_error - delta_target : delta_error) / p_delta;

        Vector3 tmp_out = p + i + d;
        Vector3 tmp_control_limit = control_limit;
        if (tmp_control_limit.x > 0.0) {
            tmp_out.x = Math::clamp(tmp_out.x, -tmp_control_limit.x, tmp_control_limit.x);
        }
        if (tmp_control_limit.y > 0.0) {
            tmp_out.y = Math::clamp(tmp_out.y, -tmp_control_limit.y, tmp_control_limit.y);
        }
        if (tmp_control_limit.z > 0.0) {
            tmp_out.z = Math::clamp(tmp_out.z, -tmp_control_limit.z, tmp_control_limit.z);
        }

        prev_target = target;
        prev_error = error;
        accumulated_error = tmp_accumulated_error;

        out = tmp_out;

    } break;

    case VECTOR4: {
        accumulated_error = Vector4(accumulated_error) * Math::exp(-error_accum_decay * p_delta);

        Vector4 tmp_value = p_value;
        Vector4 tmp_target = target;
        Vector4 tmp_prev_value = prev_value;
        Vector4 tmp_prev_target = prev_target;
        Vector4 tmp_prev_error = prev_error;
        Vector4 tmp_accumulated_error = accumulated_error;

        Vector4 delta_target, error, delta_error;
        if (value_is_angle) {
            delta_target.x = angle_difference(tmp_prev_target.x, tmp_target.x);
            error.x = angle_difference(tmp_value.x, tmp_target.x);
            delta_error.x = angle_difference(tmp_prev_error.x, error.x);
            
            delta_target.y = angle_difference(tmp_prev_target.y, tmp_target.y);
            error.y = angle_difference(tmp_value.y, tmp_target.y);
            delta_error.y = angle_difference(tmp_prev_error.y, error.y);
            
            delta_target.z = angle_difference(tmp_prev_target.z, tmp_target.z);
            error.z = angle_difference(tmp_value.z, tmp_target.z);
            delta_error.z = angle_difference(tmp_prev_error.z, error.z);
            
            delta_target.w = angle_difference(tmp_prev_target.w, tmp_target.w);
            error.w = angle_difference(tmp_value.w, tmp_target.w);
            delta_error.w = angle_difference(tmp_prev_error.w, error.w);

        } else {
            delta_target = tmp_target - tmp_prev_target;
            error = tmp_target - tmp_value;
            delta_error = error - tmp_prev_error;
        }

        tmp_accumulated_error += error * p_delta;
        Vector4 tmp_error_limit = error_accumulation_limit;
        if (tmp_error_limit.x > 0.0) {
            tmp_accumulated_error.x = Math::clamp(tmp_accumulated_error.x, -tmp_error_limit.x, tmp_error_limit.x);
        }
        if (tmp_error_limit.y > 0.0) {
            tmp_accumulated_error.y = Math::clamp(tmp_accumulated_error.y, -tmp_error_limit.y, tmp_error_limit.y);
        }
        if (tmp_error_limit.z > 0.0) {
            tmp_accumulated_error.z = Math::clamp(tmp_accumulated_error.z, -tmp_error_limit.z, tmp_error_limit.z);
        }
        if (tmp_error_limit.w > 0.0) {
            tmp_accumulated_error.w = Math::clamp(tmp_accumulated_error.w, -tmp_error_limit.w, tmp_error_limit.w);
        }

        Vector4 p = Vector4(p_gain) * error;
        Vector4 i = Vector4(i_gain) * tmp_accumulated_error;
        Vector4 d = Vector4(d_gain) * (kick_prevent_enabled ? delta_error - delta_target : delta_error) / p_delta;

        Vector4 tmp_out = p + i + d;
        Vector4 tmp_control_limit = control_limit;
        if (tmp_control_limit.x > 0.0) {
            tmp_out.x = Math::clamp(tmp_out.x, -tmp_control_limit.x, tmp_control_limit.x);
        }
        if (tmp_control_limit.y > 0.0) {
            tmp_out.y = Math::clamp(tmp_out.y, -tmp_control_limit.y, tmp_control_limit.y);
        }
        if (tmp_control_limit.z > 0.0) {
            tmp_out.z = Math::clamp(tmp_out.z, -tmp_control_limit.z, tmp_control_limit.z);
        }
        if (tmp_control_limit.w > 0.0) {
            tmp_out.w = Math::clamp(tmp_out.w, -tmp_control_limit.w, tmp_control_limit.w);
        }

        prev_target = target;
        prev_error = error;
        accumulated_error = tmp_accumulated_error;

        out = tmp_out;

    } break;

    }

    return out;
}

void PIDController::advance(double p_delta) {
    ERR_FAIL_COND_MSG(control_method != PID_CONTROL_AUTO_PROPERTY || process_method != PID_PROCESS_MANUAL, 
        "advance() should only be called when controlling a property via manual updates.");
    
    _advance(p_delta);
}

void PIDController::reset_state() {
    prev_value = zero;
    prev_target = zero;
    prev_error = zero;
    accumulated_error = zero;
}

void PIDController::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_update_method", "update_method"), &PIDController::set_update_method);
    ClassDB::bind_method(D_METHOD("get_update_method"), &PIDController::get_update_method);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "update_method", PROPERTY_HINT_ENUM, "Physics,Idle,Physics (Tool),Idle (Tool),Manual,None"), "set_update_method", "get_update_method");
    
    ClassDB::bind_method(D_METHOD("set_control_method", "control_method"), &PIDController::set_control_method);
    ClassDB::bind_method(D_METHOD("get_control_method"), &PIDController::get_control_method);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "control_method", PROPERTY_HINT_ENUM, "Auto Control Node Property,User Defined Script"), "set_control_method", "get_control_method");
    
    ClassDB::bind_method(D_METHOD("set_state_length", "length"), &PIDController::set_state_length);
    ClassDB::bind_method(D_METHOD("get_state_length"), &PIDController::get_state_length);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "state_length", PROPERTY_HINT_ENUM, "Scalar:3,Vector2:5,Vector3:9,Vector4:12"), "set_state_length", "get_state_length");

    ClassDB::bind_method(D_METHOD("set_controlled_node", "node"), &PIDController::set_controlled_node);
    ClassDB::bind_method(D_METHOD("get_controlled_node"), &PIDController::get_controlled_node);
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "controlled_node"), "set_controlled_node", "get_controlled_node");

    ClassDB::bind_method(D_METHOD("set_controlled_property", "property_name"), &PIDController::set_controlled_property);
    ClassDB::bind_method(D_METHOD("get_controlled_property"), &PIDController::get_controlled_property);
    ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "controlled_property"), "set_controlled_property", "get_controlled_property");

    ClassDB::bind_method(D_METHOD("set_target", "target"), &PIDController::set_target);
    ClassDB::bind_method(D_METHOD("get_target"), &PIDController::get_target);
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "target"), "set_target", "get_target");

    ClassDB::bind_method(D_METHOD("set_p_gain", "p_gain"), &PIDController::set_p_gain);
    ClassDB::bind_method(D_METHOD("get_p_gain"), &PIDController::get_p_gain);
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "p_gain", PROPERTY_HINT_LINK), "set_p_gain", "get_p_gain");
    
    ClassDB::bind_method(D_METHOD("set_i_gain", "i_gain"), &PIDController::set_i_gain);
    ClassDB::bind_method(D_METHOD("get_i_gain"), &PIDController::get_i_gain);
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "i_gain", PROPERTY_HINT_LINK), "set_i_gain", "get_i_gain");
    
    ClassDB::bind_method(D_METHOD("set_d_gain", "d_gain"), &PIDController::set_d_gain);
    ClassDB::bind_method(D_METHOD("get_d_gain"), &PIDController::get_d_gain);
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "d_gain", PROPERTY_HINT_LINK), "set_d_gain", "get_d_gain");

    ClassDB::bind_method(D_METHOD("set_value_is_angle", "is_angle"), &PIDController::set_value_is_angle);
    ClassDB::bind_method(D_METHOD("get_value_is_angle"), &PIDController::get_value_is_angle);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "value_is_angle"), "set_value_is_angle", "get_value_is_angle");

    ClassDB::bind_method(D_METHOD("enable_overshoot_prevention", "enable"), &PIDController::enable_overshoot_prevention);
    ClassDB::bind_method(D_METHOD("is_preventing_overshoot"), &PIDController::is_preventing_overshoot);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "prevent_overshoot"), "enable_overshoot_prevention", "is_preventing_overshoot");

    ClassDB::bind_method(D_METHOD("enable_kick_prevention", "enable"), &PIDController::enable_kick_prevention);
    ClassDB::bind_method(D_METHOD("is_preventing_kick"), &PIDController::is_preventing_kick);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "prevent_kick"), "enable_kick_prevention", "is_preventing_kick");
    
    ClassDB::bind_method(D_METHOD("set_control_output_limit", "limit"), &PIDController::set_control_output_limit);
    ClassDB::bind_method(D_METHOD("get_control_output_limit"), &PIDController::get_control_output_limit);
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "control_output_limit", PROPERTY_HINT_LINK), "set_control_output_limit", "get_control_output_limit");

    ClassDB::bind_method(D_METHOD("set_error_accumulation_limit", "limit"), &PIDController::set_error_accumulation_limit);
    ClassDB::bind_method(D_METHOD("get_error_accumulation_limit"), &PIDController::get_error_accumulation_limit);
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "error_accumulation_limit", PROPERTY_HINT_LINK), "set_error_accumulation_limit", "get_error_accumulation_limit");
    
    ClassDB::bind_method(D_METHOD("set_error_accumulation_decay", "decay"), &PIDController::set_error_accumulation_decay);
    ClassDB::bind_method(D_METHOD("get_error_accumulation_decay"), &PIDController::get_error_accumulation_decay);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "error_accumulation_decay", PROPERTY_HINT_RANGE, "0.0,100.0,0.01,or_greater"), "set_error_accumulation_decay", "get_error_accumulation_decay");
    
    ClassDB::bind_method(D_METHOD("get_error"), &PIDController::get_error);
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "error", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_error");

    ClassDB::bind_method(D_METHOD("get_accumulated_error"), &PIDController::get_accumulated_error);
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "accumulated_error", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_accumulated_error");

    ClassDB::bind_method(D_METHOD("reset_accumulated_error"), &PIDController::reset_accumulated_error);
    ClassDB::bind_method(D_METHOD("reset_state"), &PIDController::reset_state);
    ClassDB::bind_method(D_METHOD("calculate_control_output", "state_value", "delta"), &PIDController::calculate_control_output);
    ClassDB::bind_method(D_METHOD("advance", "delta"), &PIDController::advance);

    BIND_ENUM_CONSTANT(PID_PROCESS_PHYSICS);
    BIND_ENUM_CONSTANT(PID_PROCESS_IDLE);
    BIND_ENUM_CONSTANT(PID_PROCESS_PHYSICS_TOOL);
    BIND_ENUM_CONSTANT(PID_PROCESS_IDLE_TOOL);
    BIND_ENUM_CONSTANT(PID_PROCESS_MANUAL);
    BIND_ENUM_CONSTANT(PID_PROCESS_NONE);

    BIND_ENUM_CONSTANT(PID_CONTROL_AUTO_PROPERTY);
    BIND_ENUM_CONSTANT(PID_CONTROL_USER_SCRIPT);

    BIND_ENUM_CONSTANT(SCALAR);
    BIND_ENUM_CONSTANT(VECTOR2);
    BIND_ENUM_CONSTANT(VECTOR3);
    BIND_ENUM_CONSTANT(VECTOR4);
}

bool PIDController::_property_can_revert(const StringName &p_prop) const {
    return VARIABLE_PROPS.has(p_prop);
}

bool PIDController::_property_get_revert(const StringName &p_prop, Variant &r_ret) const {
    if (VARIABLE_PROPS.has(p_prop)) {
        r_ret = zero;
        return true;
    } else if (LIMIT_PROPS.has(p_prop)) {
        switch (state_length) {
            case SCALAR: {
                r_ret = -1.0;
            } break;
            case VECTOR2: {
                r_ret = Vector2(-1.0, -1.0);
            } break;
            case VECTOR3: {
                r_ret = Vector3(-1.0, -1.0, -1.0);
            } break;
            case VECTOR4: {
                r_ret = Vector4(-1.0, -1.0, -1.0, -1.0);
            } break;
            default:
                return false;
        }
        return true;
    } else {
        return false;
    }
}

void PIDController::_validate_property(PropertyInfo &p_prop) const {
    if (AUTO_PROPS.has(p_prop.name)) {
        if (control_method == PID_CONTROL_AUTO_PROPERTY) {
            p_prop.usage = PROPERTY_USAGE_DEFAULT;

            if (p_prop.name == StringName("controlled_property")) {
                PackedStringArray props;
                for (const KeyValue<String, StateLength> &E : cached_controlled_properties) {
                    if (E.value == SCALAR || E.value == VECTOR2 || E.value == VECTOR3 || E.value == VECTOR4) {
                        props.push_back(E.key);
                    }
                }
                p_prop.hint = PROPERTY_HINT_ENUM_SUGGESTION;
                p_prop.hint_string = String(",").join(props);
            }
        } else {
            p_prop.usage = PROPERTY_USAGE_NONE;
        }
    } else if (SCRIPT_PROPS.has(p_prop.name)) {
        p_prop.usage = control_method == PID_CONTROL_USER_SCRIPT ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
    }

    if (VARIABLE_PROPS.has(p_prop.name)) {
        p_prop.type = Variant::Type(state_length);
        if (ANGLE_PROPS.has(p_prop.name) && value_is_angle) {
            p_prop.hint = PROPERTY_HINT_RANGE;
            p_prop.hint_string = "0.0,360.0,0.001,radians_as_degrees";
        }
    }

    if (READ_PROPS.has(p_prop.name) && (process_method == PID_PROCESS_IDLE_TOOL || process_method == PID_PROCESS_PHYSICS_TOOL)) {
        p_prop.usage = PROPERTY_USAGE_READ_ONLY;
    }
}

PackedStringArray PIDController::_get_configuration_warnings() const {
    PackedStringArray out;
    Node *node = const_cast<PIDController *>(this)->_get_controlled_node();

    if (control_method == PID_CONTROL_AUTO_PROPERTY && 
            (node == nullptr || !cached_controlled_properties.has(controlled_property))) {
        out.push_back("To auto control a node's property, you must assign a node to \"controlled_node\" and a valid property name to \"controlled_property\"");
    }

    if (control_method == PID_CONTROL_USER_SCRIPT && !GDVIRTUAL_IS_OVERRIDDEN(_update_state)) {
        out.push_back("To enable manual control, you must override _update_state() in a user defined script.");
    }

    return out;
}

void PIDController::_notification(int p_what) {
    switch (p_what) {
        case NOTIFICATION_READY: {
            _update_process_modes();
            _setup_node_control();
        } break;

        case NOTIFICATION_INTERNAL_PROCESS: {
            _advance(get_process_delta_time());
        } break;

        case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
            _advance(get_physics_process_delta_time());
        } break;
    }
}

bool PIDController::_is_valid_variant(const Variant &p_value) const {
    return p_value.get_type() == Variant::Type(state_length);
}

bool PIDController::_validate_state_length() {
    switch (state_length_validation) {
        case STATE_LENGTH_VALIDATED: 
            return true;

        case STATE_LENGTH_WARNING_EMITTED: 
            return false;

        case STATE_LENGTH_UNVALIDATED: {
            switch (control_method) {
                case PID_CONTROL_AUTO_PROPERTY: {
                    if (_get_controlled_node() != nullptr && cached_controlled_properties.has(controlled_property)) {
                        if (state_length != cached_controlled_properties[controlled_property]) {
                            WARN_PRINT("state_length has been set to a value that doesn't match the controlled property");
                            _update_state_length();
                            return true;
                        }
                    }
                } [[fallthrough]];
                case PID_CONTROL_USER_SCRIPT: {
                    if (!_is_valid_variant(p_gain)) {
                        WARN_PRINT("p_gain was set to an invalid variant type.");
                        state_length_validation = STATE_LENGTH_WARNING_EMITTED;
                        return false;
                    } else if (!_is_valid_variant(i_gain)) {
                        WARN_PRINT("i_gain was set to an invalid variant type.");
                        state_length_validation = STATE_LENGTH_WARNING_EMITTED;
                        return false;
                    } else if (!_is_valid_variant(d_gain)) {
                        WARN_PRINT("d_gain was set to an invalid variant type.");
                        state_length_validation = STATE_LENGTH_WARNING_EMITTED;
                        return false;
                    } else if (!_is_valid_variant(error_accumulation_limit)) {
                        WARN_PRINT("error_accumulation_limit was set to an invalid variant type.");
                        state_length_validation = STATE_LENGTH_WARNING_EMITTED;
                        return false;
                    } else if (!_is_valid_variant(control_limit)) {
                        WARN_PRINT("control_limit was set to an invalid variant type.");
                        state_length_validation = STATE_LENGTH_WARNING_EMITTED;
                        return false;
                    } else if (!_is_valid_variant(target)) {
                        WARN_PRINT("target was set to an invalid variant type.");
                        state_length_validation = STATE_LENGTH_WARNING_EMITTED;
                        return false;
                    } else {
                        state_length_validation = STATE_LENGTH_VALIDATED;
                        return true;
                    }
                } break;
                default:
                    return false;
            }
        } break;

        default: 
            return false;
    }
}

void PIDController::_update_process_modes() {
    switch (process_method) {
        case PID_PROCESS_PHYSICS: {
            set_process_internal(false);
            set_physics_process_internal(!Engine::get_singleton()->is_editor_hint());
        } break;
        case PID_PROCESS_IDLE: {
            set_process_internal(!Engine::get_singleton()->is_editor_hint());
            set_physics_process_internal(false);
        } break;
        case PID_PROCESS_PHYSICS_TOOL: {
            set_process_internal(false);
            set_physics_process_internal(true);
        } break;
        case PID_PROCESS_IDLE_TOOL: {
            set_process_internal(true);
            set_physics_process_internal(false);
        } break;
        case PID_PROCESS_MANUAL: {
            set_process_internal(false);
            set_physics_process_internal(false);
        } break;
        case PID_PROCESS_NONE: {
            set_process_internal(false);
            set_physics_process_internal(false);
        } break;
    }
}

void PIDController::_update_state_length() {
    if (control_method == PID_CONTROL_AUTO_PROPERTY && cached_controlled_properties.has(controlled_property)) {
        state_length = cached_controlled_properties[controlled_property];
    }

    switch (state_length) {
        case SCALAR:    
            zero = 0.0;
            break;
        case VECTOR2: 
            zero = Vector2(0.0, 0.0);
            break;
        case VECTOR3: 
            zero = Vector3(0.0, 0.0, 0.0);
            break;
        case VECTOR4: 
            zero = Vector4(0.0, 0.0, 0.0, 0.0);
            break;
    }

    target = zero;
    reset_state();
    state_length_validation = STATE_LENGTH_VALIDATED;
    notify_property_list_changed();
}

void PIDController::_advance(double p_delta) {
    if (!_validate_state_length()) {
        return;
    }

    switch (control_method) {
        case PID_CONTROL_AUTO_PROPERTY: {
            Node *node = _get_controlled_node();
            if (node == nullptr || !cached_controlled_properties.has(controlled_property)) {
                return;
            }

            // Verlet: x(t + dt) = x(t) + x(t) - x(t - dt) + a(t)dt^2 + O(dt^4)
            #define VERLET(m_type, m_magnitude) \
            m_type value_m1 = prev_value; \
            m_type value_0 = node->get(controlled_property); \
            m_type control_output = calculate_control_output(value_0, p_delta); \
            \
            m_type delta_value = value_0 - value_m1 + control_output * p_delta * p_delta; \
            \
            m_type value_p1; \
            if (overshoot_prevent_enabled && (delta_value)m_magnitude > m_type(prev_error)m_magnitude) { \
                value_p1 = target; \
            } else { \
                value_p1 = value_0 + delta_value; \
            } \
            \
            node->set(controlled_property, value_p1); \
            prev_value = value_0;

            switch (state_length) {
                case SCALAR: {
                    VERLET(double, )

                } break;

                case VECTOR2: {
                    VERLET(Vector2, .length_squared())

                } break;

                case VECTOR3: {
                    VERLET(Vector3, .length_squared())

                } break;

                case VECTOR4: {
                    VERLET(Vector4, .length_squared())

                } break;
            }

            #undef VERLET
        } break;

        case PID_CONTROL_USER_SCRIPT: {
            GDVIRTUAL_REQUIRED_CALL(_update_state, p_delta);
        } break;
    }
}

void PIDController::_setup_node_control() {
    Node *node = get_node_or_null(controlled_node);
    if (node != nullptr && node != cached_node) {
        cached_node = node;

        cached_controlled_properties.clear();
        TypedArray<Dictionary> props = cached_node->get_property_list();
        for (uint64_t idx = 0; idx < props.size(); ++idx) {
            const Dictionary &prop = props[idx];
            auto len = StateLength(int(prop["type"]));
            if (len == SCALAR || len == VECTOR2 || len == VECTOR3 || len == VECTOR4) {
                cached_controlled_properties[prop["name"]] = StateLength(int(prop["type"]));
            }
        }

        _update_state_length();
    }

    update_configuration_warnings();
}

Node *PIDController::_get_controlled_node() {
    if (cached_node == nullptr) {
        _setup_node_control();
    }

    return cached_node;
}