#include "pid_controller.hpp"

using namespace godot;
using namespace ez_pid;

#define EADD(m_a, m_b) _eval_variant(Variant::OP_ADD, m_a, m_b)
#define ESUB(m_a, m_b) _eval_variant(Variant::OP_SUBTRACT, m_a, m_b)
#define EMUL(m_a, m_b) _eval_variant(Variant::OP_MULTIPLY, m_a, m_b)
#define EDIV(m_a, m_b) _eval_variant(Variant::OP_DIVIDE, m_a, m_b)
#define EGT(m_a, m_b) _eval_variant(Variant::OP_GREATER, m_a, m_b)
#define ELT(m_a, m_b) _eval_variant(Variant::OP_LESS, m_a, m_b)

_ALWAYS_INLINE_ double angle_difference(double p_from, double p_to) {
    double difference = fmod(p_to - p_from, Math_TAU);
    return fmod(2.0 * difference, Math_TAU) - difference;
}

void PIDController::set_update_method(ProcessMethod p_method) {
    if (p_method != process_method) {
        process_method = p_method;
        switch (process_method) {
            case ProcessMethod::PID_PROCESS_PHYSICS: {
                set_process_internal(false);
                set_physics_process_internal(true);
            } break;
            case ProcessMethod::PID_PROCESS_IDLE: {
                set_process_internal(true);
                set_physics_process_internal(false);
            } break;
            case ProcessMethod::PID_PROCESS_MANUAL: {
                set_process_internal(false);
                set_physics_process_internal(false);
            }
        }
    }
}

PIDController::ProcessMethod PIDController::get_update_method() const {
    return process_method;
}

void PIDController::set_pid(const Ref<PID> &p_pid) {
    ERR_FAIL_NULL(p_pid);

    if (p_pid != pid) {
        if (pid.is_valid()) {
            pid->hide_state_length(false);
            pid->disconnect("property_list_changed", callable_mp(this, &PIDController::_on_pid_property_list_changed));
        }

        pid = p_pid;
        pid->hide_state_length(true);
        reset();

        pid->connect("property_list_changed", callable_mp(this, &PIDController::_on_pid_property_list_changed));
    }
}

Ref<PID> PIDController::get_pid() const {
    return pid;
}

void PIDController::set_value_is_angle(bool p_is_angle) {
    value_is_angle = p_is_angle;
}

bool PIDController::get_value_is_angle() const {
    return value_is_angle;
}

void PIDController::set_prevent_overshoot(bool p_prevent) {
    preventing_overshoot = p_prevent;
}

bool PIDController::is_preventing_overshoot() const {
    return preventing_overshoot;
}

void PIDController::set_prevent_derivative_kick(bool p_prevent) {
    preventing_derivative_kick = p_prevent;
}

bool PIDController::is_preventing_derivative_kick() const {
    return preventing_derivative_kick;
}

void PIDController::set_error_integration_limit(const Variant &p_limit) {
    if (p_limit != error_sum_limit) {
        error_sum_limit = p_limit;
        if (EGT(error_sum_limit, 0.0)) {
            error_sum = _limit(error_sum, error_sum_limit);
        }
    }
}

Variant PIDController::get_error_integration_limit() const {
    return error_sum_limit;
}

void PIDController::set_control_output_limit(const Variant &p_limit) {
    control_limit = _eval_variant(Variant::OP_MAX, p_limit, 0.0);
}

Variant PIDController::get_control_output_limit() const {
    return control_limit;
}

void PIDController::set_controlled_node(Node *p_node) {
    if (p_node == controlled_node) {
        return;
    }

    controlled_node = p_node;
    cached_properties.clear();

    if (nullptr != controlled_node) {
        cached_properties.clear();
        TypedArray<Dictionary> props = controlled_node->get_property_list();
        for (uint64_t idx = 0; idx < props.size(); ++idx) {
            const Dictionary &prop = props[idx];
            if (Array::make(PID::SCALAR, PID::VECTOR2, PID::VECTOR3, PID::VECTOR4).has(prop["type"])) {
                cached_properties[prop["name"]] = Variant::Type(int(prop["type"]));
            }
        }

        if (!cached_properties.has(controlled_property)) {
            controlled_property = StringName();
        }
    }

    _update_state_length();
}

Node *PIDController::get_controlled_node() const {
    return controlled_node;
}

void PIDController::set_controlled_property(const StringName &p_name) {
    controlled_property = p_name;
    _update_state_length();
}

StringName PIDController::get_controlled_property() const {
    return controlled_property;
}

void PIDController::set_target_value(const Variant &p_val) {
    ERR_FAIL_COND(!pid->is_correct_length(p_val));
    
    target = p_val;
}

Variant PIDController::get_target_value() const {
    return target;
}

Variant PIDController::calculate_control_output(const Variant &p_value, const Variant &p_target, double p_delta) {
    ERR_FAIL_COND_V_MSG(!pid->is_correct_length(p_value) || !pid->is_correct_length(p_target), pid->get_zero(), 
        "Parameter value and target must be the same length as the PID state length.");

    Variant delta_target;
    Variant error;
    Variant delta_error;
    if (value_is_angle) {
        delta_target = _get_angular_difference(target_prev, p_target);
        error = _get_angular_difference(p_value, p_target);
        delta_error = _get_angular_difference(error_prev, error);
    } else {
        delta_target = ESUB(p_target, target_prev);
        error = ESUB(p_target, p_value);
        delta_error = ESUB(error, error_prev);
    }

    Variant p_out = EMUL(pid->get_p_gain(), error);
    Variant i_out = pid->is_integration_enabled() ?  EMUL(pid->get_i_gain(), error_sum) : pid->get_zero();
    
    Variant d_out = EMUL(pid->get_d_gain(), EDIV(preventing_derivative_kick ? ESUB(delta_error, delta_target) : delta_error, p_delta));

    Variant control_output = EADD(EADD(p_out, i_out), d_out);
    if (EGT(control_limit, 0.0)) {
        control_output = _limit(control_output, control_limit);
    }

    target_prev = p_target;
    error_prev = error;
    if (pid->is_integration_enabled()) {
        error_sum = EADD(error_sum, EMUL(error, p_delta));
    }

    return control_output;
}

Variant PIDController::update_state(const Variant &p_value, const Variant &p_target, double p_delta) {
    ERR_FAIL_COND_V_MSG(!pid->is_correct_length(p_value) || !pid->is_correct_length(p_target), pid->get_zero(), 
        "Parameter value and target must be the same length as the PID state length.");

    Variant control_output = calculate_control_output(p_value, p_target, p_delta);

    Variant delta_value_dot = EMUL(control_output, p_delta);
    Variant delta_value = EMUL(EADD(value_dot, delta_value_dot), p_delta);

    Variant value;
    if (preventing_overshoot && _get_magnitude(delta_value) > _get_magnitude(error_prev)) {
        value_dot = pid->get_zero();
        value = p_target;
    } else {
        value_dot = EADD(value_dot, delta_value_dot);
        value = EADD(p_value, delta_value);
    }

    if (_get_magnitude(ESUB(p_target, value)) <= 1e-6) {
        emit_signal("converged");
    }

    return value;
}

void PIDController::reset() {
    Variant zero = pid->get_zero();
    value_dot = zero;
    target = zero;
    target_prev = zero;
    error_prev = zero;
    error_sum = zero;
}


void PIDController::_update_state_length() {
    if (nullptr == controlled_node || !cached_properties.has(controlled_property)) {
        pid->set_state_length(PID::SCALAR);
    } else {
        pid->set_state_length(PID::StateLength(int(cached_properties[controlled_property])));
    }
}

Variant PIDController::_eval_variant(Variant::Operator p_op, const Variant &p_a, const Variant &p_b) const {
    bool is_valid = false;
    Variant out;
    Variant::evaluate(p_op, p_a, p_b, out, is_valid);
    if (!is_valid) {
        out = pid->get_zero();
    }

    return out;
}

Variant PIDController::_get_angular_difference(const Variant &p_value1, const Variant &p_value2) const {
    ERR_FAIL_COND_V(p_value1.get_type() != p_value2.get_type(), pid->get_zero());

    Variant out;
    switch (p_value1.get_type()) {
        case PID::SCALAR: {
            out = angle_difference(p_value1, p_value2);
        } break;
        case PID::VECTOR2: {
            Vector2 val1 = p_value1;
            Vector2 val2 = p_value2;
            
            out = Vector2(angle_difference(val1.x, val2.x), angle_difference(val1.y, val2.y));
        } break;
        case PID::VECTOR3: {
            Vector3 val1 = p_value1;
            Vector3 val2 = p_value2;

            out = Vector3(
                angle_difference(val1.x, val2.x), 
                angle_difference(val1.y, val2.y), 
                angle_difference(val1.z, val2.z));
        } break;
        case PID::VECTOR4: {
            Vector4 val1 = p_value1;
            Vector4 val2 = p_value2;

            out = Vector4(
                angle_difference(val1.x, val2.x), 
                angle_difference(val1.y, val2.y), 
                angle_difference(val1.z, val2.z),
                angle_difference(val1.w, val2.w));
        } break;
        default:
            ERR_FAIL_V_MSG(pid->get_zero(), "Somehow passed in angles are not of correct type.");
    }

    return out;
}

Variant PIDController::_limit(const Variant &p_value, const Variant &p_limit) const {
    double magnitude = _get_magnitude(p_limit);    

    Variant out;
    switch (p_value.get_type()) {
        case PID::SCALAR: {
            double val = double(p_value);
            out = Math::sign(val) * Math::min(Math::abs(val), magnitude);
        } break;
        case PID::VECTOR2: {
            out = Vector2(p_value).limit_length(magnitude);
        } break;
        case PID::VECTOR3: {
            out = Vector3(p_value).limit_length(magnitude);
        } break;
        case PID::VECTOR4: {
            Vector4 val = Vector4(p_value);
            out = val.length() > magnitude ? val.normalized() * magnitude : val;
        } break;
        default:
            ERR_FAIL_V_MSG(pid->get_zero(), "Somehow passed in value is not of correct type.");
    }

    return out;
}

double PIDController::_get_magnitude(const Variant &p_value) const {
    double magnitude = 0;
    switch (p_value.get_type()) {
        case PID::SCALAR: {
            magnitude = p_value;
        } break;
        case PID::VECTOR2: {
            magnitude = Vector2(p_value).length();
        } break;
        case PID::VECTOR3: {
            magnitude = Vector3(p_value).length();
        } break;
        case PID::VECTOR4: {
            magnitude = Vector4(p_value).length();
        } break;
        default:
            ERR_FAIL_V_MSG(pid->get_zero(), "Somehow passed in value is not of correct type.");
    }

    return magnitude;
}

void PIDController::_on_pid_property_list_changed() {
    notify_property_list_changed();
}

void PIDController::_validate_property(PropertyInfo &p_prop) const {
    if (
            p_prop.name == StringName("error_integration_limit") ||
            p_prop.name == StringName("control_output_limit") || 
            p_prop.name == StringName("target_value")
    ) {
        p_prop.type = Variant::Type(pid->get_state_length());

        if (
                (p_prop.name == StringName("error_integration_limit") && !pid->is_integration_enabled()) ||
                (p_prop.name == StringName("target_value") && controlled_node == nullptr) 
        ) {
            p_prop.usage = PROPERTY_USAGE_NONE;
        }
    } 
    
    else if (p_prop.name == StringName("controlled_property")) {
        p_prop.hint_string = String(",").join(cached_properties.keys());
        p_prop.usage = controlled_node == nullptr ? PROPERTY_USAGE_NONE : PROPERTY_USAGE_DEFAULT;
    }
}

void PIDController::_notification(int p_what) {
    switch (p_what) {
        case NOTIFICATION_INTERNAL_PROCESS:
        case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
            if (nullptr == controlled_node || !cached_properties.has(controlled_property)) {
                break;
            }

            double delta = 0.0;
            if (p_what == NOTIFICATION_INTERNAL_PROCESS) {
                delta = get_process_delta_time();
            } else {
                delta = get_physics_process_delta_time();
            }
            Variant node_value = controlled_node->get(controlled_property);

            GDVIRTUAL_CALL(_integrate_state, node_value, delta, node_value);

            controlled_node->set(controlled_property, update_state(node_value, target, delta));

        } break;
    }
}

void PIDController::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_update_method", "update_method"), &PIDController::set_update_method);
    ClassDB::bind_method(D_METHOD("get_update_method"), &PIDController::get_update_method);
    ClassDB::bind_method(D_METHOD("set_pid", "pid"), &PIDController::set_pid);
    ClassDB::bind_method(D_METHOD("get_pid"), &PIDController::get_pid);
    ClassDB::bind_method(D_METHOD("set_value_is_angle", "is_angle"), &PIDController::set_value_is_angle);
    ClassDB::bind_method(D_METHOD("get_value_is_angle"), &PIDController::get_value_is_angle);
    ClassDB::bind_method(D_METHOD("set_prevent_overshoot", "prevent_overshoot"), &PIDController::set_prevent_overshoot);
    ClassDB::bind_method(D_METHOD("is_preventing_overshoot"), &PIDController::is_preventing_overshoot);
    ClassDB::bind_method(D_METHOD("set_prevent_derivative_kick", "prevent_kick"), &PIDController::set_prevent_derivative_kick);
    ClassDB::bind_method(D_METHOD("is_preventing_derivative_kick"), &PIDController::is_preventing_derivative_kick);
    ClassDB::bind_method(D_METHOD("set_error_integration_limit", "integration_limit"), &PIDController::set_error_integration_limit);
    ClassDB::bind_method(D_METHOD("get_error_integration_limit"), &PIDController::get_error_integration_limit);
    ClassDB::bind_method(D_METHOD("set_control_output_limit", "output_limit"), &PIDController::set_control_output_limit);
    ClassDB::bind_method(D_METHOD("get_control_output_limit"), &PIDController::get_control_output_limit);
    ClassDB::bind_method(D_METHOD("set_controlled_node", "node"), &PIDController::set_controlled_node);
    ClassDB::bind_method(D_METHOD("get_controlled_node"), &PIDController::get_controlled_node);
    ClassDB::bind_method(D_METHOD("set_controlled_property", "property_name"), &PIDController::set_controlled_property);
    ClassDB::bind_method(D_METHOD("get_controlled_property"), &PIDController::get_controlled_property);
    ClassDB::bind_method(D_METHOD("set_target_value", "target"), &PIDController::set_target_value);
    ClassDB::bind_method(D_METHOD("get_target_value"), &PIDController::get_target_value);

    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "pid_gains", PROPERTY_HINT_RESOURCE_TYPE, "PID", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_pid", "get_pid");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "prevent_overshoot"), "set_prevent_overshoot", "is_preventing_overshoot");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "prevent_derivative_kick"), "set_prevent_derivative_kick", "is_preventing_derivative_kick");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "value_is_angle"), "set_value_is_angle", "get_value_is_angle");
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "error_integration_limit"), "set_error_integration_limit", "get_error_integration_limit");
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "control_output_limit"), "set_control_output_limit", "get_control_output_limit");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "controlled_node", PROPERTY_HINT_NODE_TYPE), "set_controlled_node", "get_controlled_node");
    ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "controlled_property", PROPERTY_HINT_ENUM_SUGGESTION, ""), "set_controlled_property", "get_controlled_property");
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "target_value"), "set_target_value", "get_target_value");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "update_method", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_CLASS_IS_ENUM, "PIDController.ProcessedMethod"), "set_update_method", "get_update_method");

    ADD_SIGNAL(MethodInfo("converged"));
}