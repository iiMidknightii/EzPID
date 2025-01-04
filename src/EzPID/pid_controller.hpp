#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>
#include "pid.hpp"

namespace godot::ez_pid {

class PIDController : public Node {
    GDCLASS(PIDController, Node)

public:
    enum ProcessMethod {
        PID_PROCESS_PHYSICS,
        PID_PROCESS_IDLE,
        PID_PROCESS_MANUAL,
    };

    void set_update_method(ProcessMethod p_method);
    ProcessMethod get_update_method() const;

    void set_pid(const Ref<PID> &p_pid);
    Ref<PID> get_pid() const;

    void set_value_is_angle(bool p_is_angle);
    bool get_value_is_angle() const;

    void set_prevent_overshoot(bool p_prevent);
    bool is_preventing_overshoot() const;

    void set_prevent_derivative_kick(bool p_prevent);
    bool is_preventing_derivative_kick() const;

    void enable_error_integration_limit(bool p_enable);
    bool is_limiting_error_integration() const;

    void set_error_integration_limit(const Variant &p_limit);
    Variant get_error_integration_limit() const;

    void set_error_integration_decay(double p_decay);
    double get_error_integration_decay() const;

    void enable_control_output_limit(bool p_enable);
    bool is_limiting_control_output() const;

    void set_control_output_limit(const Variant &p_limit);
    Variant get_control_output_limit() const;

    void set_run_in_editor(bool p_run);
    bool get_run_in_editor() const;

    void set_controlled_node(Node *p_node);
    Node *get_controlled_node() const;

    void set_controlled_property(const StringName &p_name);
    StringName get_controlled_property() const;

    void set_target_value(const Variant &p_val);
    Variant get_target_value() const;

    void set_value(const Variant &p_value);
    Variant get_value() const;
    void set_value_dot(const Variant &p_value_dot);
    Variant get_value_dot() const;
    Variant get_error() const;
    Variant get_sum_error() const;

    Variant calculate_control_output(const Variant &p_value, const Variant &p_target, double p_delta);
    Variant update_state(const Variant &p_value, const Variant &p_target, double p_delta);
    void reset();

    GDVIRTUAL1(_integrate_state, double)

protected:
    void _get_property_list(List<PropertyInfo> *p_list) const;
    bool _get(const StringName &p_prop, Variant &r_ret) const;
    bool _set(const StringName &p_prop, const Variant &p_value);
    bool _property_can_revert(const StringName &p_prop) const;
    bool _property_get_revert(const StringName &p_prop, Variant &r_ret) const;
    void _validate_property(PropertyInfo &p_prop) const;
    void _notification(int p_what);
    static void _bind_methods();

private:
    ProcessMethod process_method = ProcessMethod::PID_PROCESS_PHYSICS;

    Ref<PID> pid;

    bool enable_controller = true;
    bool value_is_angle = false;
    bool preventing_overshoot = false;
    bool preventing_derivative_kick = false;
    bool run_in_editor = false;
    bool limiting_error_sum = false;
    Variant error_sum_limit = -1;
    double error_sum_decay = 0;
    bool limiting_control = false;
    Variant control_limit = -1;

    bool is_controlling_node = false;
    Node *controlled_node = nullptr;
    Dictionary cached_properties;
    String controlled_property;

    Variant value = 0;
    Variant value_dot = 0;
    Variant target = 0;
    Variant target_prev = 0;
    Variant error_prev = 0;
    Variant error_sum = 0;

    void _update_state_length();
    Variant _eval_variant(Variant::Operator p_op, const Variant &p_a, const Variant &p_b) const;
    Variant _get_angular_difference(const Variant &p_value1, const Variant &p_value2) const;
    Variant _limit(const Variant &p_value, const Variant &p_limit) const;
    double _get_magnitude(const Variant &p_value) const;
    void _on_pid_property_list_changed();
};

}

VARIANT_ENUM_CAST(ez_pid::PIDController::ProcessMethod);

#endif