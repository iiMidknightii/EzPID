#include "pid.hpp"


using namespace godot;
using namespace ez_pid;

Variant PID::get_zero() const {
    switch (state_length) {
        case StateLength::SCALAR: {
            return 0;
        } break;
        case StateLength::VECTOR2: {
            return Vector2();
        } break;
        case StateLength::VECTOR3: {
            return Vector3();
        } break;
        case StateLength::VECTOR4: {
            return Vector4();
        } break;
        default: {
            ERR_FAIL_V_MSG(0, "Somehow PID has an enum value outside VECTOR4.");
        }
    }
}

bool PID::is_correct_length(const Variant &p_value) const {
    return p_value.get_type() == state_length;
}

void PID::set_state_length(StateLength p_length) {
    if (p_length != state_length) {
        ERR_FAIL_COND(p_length < SCALAR || p_length > VECTOR4);

        state_length = p_length;
        p_gain = get_zero();
        i_gain = get_zero();
        d_gain = get_zero();
        emit_changed();
        notify_property_list_changed();
    }
}

PID::StateLength PID::get_state_length() const {
    return state_length;
}

// Use a macro to make the identical setget methods for P, I, and D gains
#define MAKE_SETGET_GAIN(m_type)                                                \
void PID::set_##m_type##_gain(const Variant &p_##m_type) {                      \
    if (p_##m_type != m_type##_gain) {                                          \
        ERR_FAIL_COND(!is_correct_length(p_##m_type));                          \
        m_type##_gain = p_##m_type;                                             \
        emit_changed();                                                         \
    }                                                                           \
}                                                                               \
                                                                                \
Variant PID::get_##m_type##_gain() const {                                      \
    return m_type##_gain;                                                       \
}

MAKE_SETGET_GAIN(p)
MAKE_SETGET_GAIN(i)
MAKE_SETGET_GAIN(d)

void PID::enable_integration(bool p_enable) {
    if (p_enable != integration_enabled) {
        integration_enabled = p_enable;
        emit_changed();
        notify_property_list_changed();
    }
}

bool PID::is_integration_enabled() const {
    return integration_enabled;
}

void PID::hide_state_length(bool p_hide) {
    if (p_hide != state_length_hidden) {
        state_length_hidden = p_hide;
        notify_property_list_changed();
    }
}

bool PID::_property_can_revert(const StringName &p_prop) const {
    if (Array::make("p_gain", "d_gain", "i_gain").has(p_prop)) {
        return true;
    }

    return false;
}

bool PID::_property_get_revert(const StringName &p_prop, Variant &r_ret) const {
    if (Array::make("p_gain", "d_gain", "i_gain").has(p_prop)) {
        r_ret = get_zero();
        return true;
    }

    return false;
}


void PID::_validate_property(PropertyInfo &p_prop) const {
    if (Array::make("p_gain", "d_gain", "i_gain").has(p_prop.name)) {
        p_prop.type = Variant::Type(state_length);
        if (p_prop.name == StringName("i_gain")) {
            p_prop.usage = integration_enabled ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_STORAGE;
        }
    } else if (p_prop.name == StringName("state_length")) {
        p_prop.usage = state_length_hidden ? PROPERTY_USAGE_STORAGE : PROPERTY_USAGE_DEFAULT;
    }
}

void PID::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_state_length", "length"), &PID::set_state_length);
    ClassDB::bind_method(D_METHOD("get_state_length"), &PID::get_state_length);
    ClassDB::bind_method(D_METHOD("enable_integration", "enable"), &PID::enable_integration);
    ClassDB::bind_method(D_METHOD("is_integration_enabled"), &PID::is_integration_enabled);
    ClassDB::bind_method(D_METHOD("set_p_gain", "p_gain"), &PID::set_p_gain);
    ClassDB::bind_method(D_METHOD("get_p_gain"), &PID::get_p_gain);
    ClassDB::bind_method(D_METHOD("set_i_gain", "i_gain"), &PID::set_i_gain);
    ClassDB::bind_method(D_METHOD("get_i_gain"), &PID::get_i_gain);
    ClassDB::bind_method(D_METHOD("set_d_gain", "d_gain"), &PID::set_d_gain);
    ClassDB::bind_method(D_METHOD("get_d_gain"), &PID::get_d_gain);

    ADD_PROPERTY(PropertyInfo(Variant::INT, "state_length", PROPERTY_HINT_ENUM, "Scalar:3,Vector2:5,Vector3:9,Vector4:12"), "set_state_length", "get_state_length");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "integration_enabled"), "enable_integration", "is_integration_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "p_gain", PROPERTY_HINT_LINK), "set_p_gain", "get_p_gain");
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "i_gain", PROPERTY_HINT_LINK), "set_i_gain", "get_i_gain");
    ADD_PROPERTY(PropertyInfo(Variant::NIL, "d_gain", PROPERTY_HINT_LINK), "set_d_gain", "get_d_gain");

    BIND_ENUM_CONSTANT(SCALAR);
    BIND_ENUM_CONSTANT(VECTOR2);
    BIND_ENUM_CONSTANT(VECTOR3);
    BIND_ENUM_CONSTANT(VECTOR4);
}