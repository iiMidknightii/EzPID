#ifndef PID_H
#define PID_H

#include <godot_cpp/classes/resource.hpp>

namespace godot::ez_pid {

class PID : public Resource {
    GDCLASS(PID, Resource)

public:
    enum StateLength {
        SCALAR = Variant::FLOAT, 
        VECTOR2 = Variant::VECTOR2,
        VECTOR3 = Variant::VECTOR3,
        VECTOR4 = Variant::VECTOR4,
    };

    Variant get_zero() const;
    bool is_correct_length(const Variant &p_value) const;

    void set_state_length(StateLength p_length);
    StateLength get_state_length() const;

    void set_p_gain(const Variant &p_p);
    Variant get_p_gain() const;

    void set_i_gain(const Variant &p_i);
    Variant get_i_gain() const;

    void set_d_gain(const Variant &p_d);
    Variant get_d_gain() const;

    void enable_integration(bool p_enable);
    bool is_integration_enabled() const;

    void hide_state_length(bool p_hide);

protected:
    bool _property_can_revert(const StringName &p_prop) const;
    bool _property_get_revert(const StringName &p_prop, Variant &r_ret) const;
    void _validate_property(PropertyInfo &p_prop) const;
    static void _bind_methods();

private:
    StateLength state_length = StateLength::SCALAR;

    Variant p_gain = 0;
    Variant i_gain = 0;
    Variant d_gain = 0;

    bool integration_enabled = false;
    bool state_length_hidden = false;
};

}

VARIANT_ENUM_CAST(ez_pid::PID::StateLength);

#endif