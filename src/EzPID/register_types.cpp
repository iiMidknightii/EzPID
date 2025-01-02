#include "register_types.hpp"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "pid_controller.hpp"

void initialize_pid(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

    GDREGISTER_CLASS(ez_pid::PID);
    GDREGISTER_CLASS(ez_pid::PIDController);
}

void uninitialize_pid(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}

extern "C" {
    // Initialization.
    GDExtensionBool GDE_EXPORT ez_pid_init(
            GDExtensionInterfaceGetProcAddress p_get_proc_address, 
            const GDExtensionClassLibraryPtr p_library, 
            GDExtensionInitialization *r_initialization) {
        godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

        init_obj.register_initializer(initialize_pid);
        init_obj.register_terminator(uninitialize_pid);
        init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

        return init_obj.init();
    }

}