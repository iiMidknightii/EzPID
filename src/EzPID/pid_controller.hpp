#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>

namespace godot {

class PIDController : public Node {
    GDCLASS(PIDController, Node)

public:
    
    enum UpdateMethod {
        PID_PROCESS_PHYSICS,
        PID_PROCESS_IDLE,
        PID_PROCESS_PHYSICS_TOOL,
        PID_PROCESS_IDLE_TOOL,
        PID_PROCESS_MANUAL,
        PID_PROCESS_NONE,
    };
    enum ControlMethod {
        PID_CONTROL_AUTO_PROPERTY,
        PID_CONTROL_USER_SCRIPT,
    };
    enum StateLength {
        SCALAR = Variant::FLOAT,
        VECTOR2 = Variant::VECTOR2,
        VECTOR3 = Variant::VECTOR3,
        VECTOR4 = Variant::VECTOR4,
    };

    ~PIDController() override;

    void set_p_gain(const Variant &p_p);
    Variant get_p_gain() const;

    void set_i_gain(const Variant &p_i);
    Variant get_i_gain() const;

    void set_d_gain(const Variant &p_d);
    Variant get_d_gain() const;

    void set_value_is_angle(bool p_is_angle);
    bool get_value_is_angle() const;

    void enable_overshoot_prevention(bool p_prevent);
    bool is_preventing_overshoot() const;

    void enable_kick_prevention(bool p_prevent);
    bool is_preventing_kick() const;

    void set_control_output_limit(const Variant &p_limit);
    Variant get_control_output_limit() const;

    void set_error_accumulation_limit(const Variant &p_limit);
    Variant get_error_accumulation_limit() const;
    
    void set_error_accumulation_decay(double p_decay);
    double get_error_accumulation_decay() const;

    void set_target(const Variant &p_target);
    Variant get_target() const;

    void set_state_length(StateLength p_length);
    StateLength get_state_length() const;

    void set_update_method(UpdateMethod p_method);
    UpdateMethod get_update_method() const;

    void set_control_method(ControlMethod p_method);
    ControlMethod get_control_method() const;

    // Auto controlled properties
    void set_controlled_node(NodePath p_node);
    NodePath get_controlled_node() const;

    void set_controlled_property(const StringName &p_name);
    StringName get_controlled_property() const;
    //

    Variant get_error() const;
    Variant get_accumulated_error() const;
    void reset_accumulated_error();
    void reset_state();

    Variant calculate_control_output(const Variant &p_value, double p_delta);
    void advance(double p_delta);

    GDVIRTUAL1(_update_state, double)
    
    PackedStringArray _get_configuration_warnings() const override;

protected:
    bool _property_can_revert(const StringName &p_prop) const;
    bool _property_get_revert(const StringName &p_prop, Variant &r_ret) const;
    void _validate_property(PropertyInfo &p_prop) const;
    void _notification(int p_what);
    static void _bind_methods();

private:
    using PropertyCache = HashMap<String, StateLength>;

    const Vector<StringName> VARIABLE_PROPS {
        "p_gain",
        "i_gain",
        "d_gain",
        "control_output_limit",
        "error_accumulation_limit",
        "target",
        "error",
        "accumulated_error",
    };
    const Vector<StringName> LIMIT_PROPS {"control_output_limit", "error_accumulation_limit"};
    const Vector<StringName> ANGLE_PROPS {
        "error_accumulation_limit",
        "target",
        "error",
        "accumulated_error",
    };
    const Vector<StringName> AUTO_PROPS {"controlled_node", "controlled_property", "prevent_overshoot"};
    const Vector<StringName> SCRIPT_PROPS {"state_length"};
    const Vector<StringName> READ_PROPS {"error", "accumulated_error"};

    StateLength state_length = SCALAR;
    enum StateLengthValidation {
        STATE_LENGTH_UNVALIDATED, 
        STATE_LENGTH_VALIDATED, 
        STATE_LENGTH_WARNING_EMITTED,
    } state_length_validation = STATE_LENGTH_UNVALIDATED;

    Variant p_gain = 0.0;
    Variant i_gain = 0.0;
    Variant d_gain = 0.0;

    UpdateMethod process_method = PID_PROCESS_PHYSICS;

    bool value_is_angle = false;
    bool overshoot_prevent_enabled = false;
    bool kick_prevent_enabled = false;
    Variant error_accumulation_limit = -1;
    double error_accum_decay = 0;
    Variant control_limit = -1;

    ControlMethod control_method = PID_CONTROL_AUTO_PROPERTY;
    NodePath controlled_node;
    String controlled_property;
    
    Variant zero              = 0.0;
    Variant target            = 0.0;
    Variant prev_value        = 0.0;
    Variant prev_target       = 0.0;
    Variant prev_error        = 0.0;
    Variant accumulated_error = 0.0;
    
    Node *cached_node = nullptr;
    PropertyCache cached_controlled_properties;
    
    bool _is_valid_variant(const Variant &p_value) const;
    bool _validate_state_length();
    void _update_process_modes();
    void _update_state_length();
    void _advance(double p_delta);
    void _setup_node_control();
    Node *_get_controlled_node();
};

}

VARIANT_ENUM_CAST(PIDController::UpdateMethod);
VARIANT_ENUM_CAST(PIDController::ControlMethod);
VARIANT_ENUM_CAST(PIDController::StateLength);