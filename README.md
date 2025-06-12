# ![](addons/EzPID/icons/PIDController64.png) EzPID
 A lightweight, fast PID controller for Godot implemented as a C++ GDExtension.

## Installation
### From Godot Asset Library in the Editor
Click the `AssetLib` button at the top of the Godot editor and search for `EzPID`.

 ### From Godot Asset Library Web
 Head over to [the EzPID page on the asset library website](https://godotengine.org/asset-library/asset) and click the download button.  Unzip the download into a location of your choosing.  To put the addon in your project, just copy the "addons" folder into the project directory.

 ### From GitHub.com
 You can download the full repository for EzPID [here](https://github.com/iiMidknightii/EzPID).  You can clone this repository by doing `git clone https://github.com/iiMidknightii/EzPID.git` in the directory of your choosing.  If you want to compile your own binaries this is the best option.  To put the addon in your project, just copy the "addons" folder into the project directory.

## Tutorial
This plugin adds a single node type to Godot: The `PIDController`.  Add a `PIDController` node anywhere in the scene to enable PID control in one of 2 `control_method`s: `PID_CONTROL_AUTO_PROPERTY` and `PID_CONTROL_USER_SCRIPT`.

### Auto Control Mode (`PID_CONTROL_AUTO_PROPERTY`)
When `control_method` is set to this value, you select another node and one if its properties to be controlled by the `PIDController`.  For example, to control the position of a `Node3D` when unknown outside disturbances are affecting it, you set the controller's `controlled_node` property to the node path to the `Node3D` and the `controlled_property` property to `position`.  From there, just select your `target` position and any other parameters for the controller.  

You can have the `PIDController` update the position of the node during `PID_PROCESS_IDLE` (during `_process`), `PID_PROCESS_PHYSICS` (during `_physics_process`), an equivalent TOOL mode (for use in the editor), or `PID_PROCESS_MANUAL` (the position will only update when you specifically call `advance(delta)` in script).

### Script Control Mode (`PID_CONTROL_USER_SCRIPT`)
When `control_method` is set to this value, the `PIDController` node makes no attempt to directly control the state of anything.  Instead, you must extend the `PIDController` with a custom attached script.  Inside that script, you can override `_process`, `_physics_process`, or any other engine callback to do your own state update.  Inside that function, you can update the `target` value, call `calculate_control_output` with a state variable, and use the control output to drive the system however you choose.

For example, to keep a `RigidBody3D` pointing in a specific orientation, you might override the `_physics_process` callback inside a script attached to the `PIDController` or `RigidBody3D` itself.  Inside `_physics_process`, you can set the `target` to the rotation desired, then call the `PIDController`::`calculate__control_output` with the current `rotation` of the rigid body.  The output of the function should be the forces required to keep the rigid body in the chosen direction, so call `RigidBody3D`::`apply_torque` with the output of the controller.

## Latest Release
* 1.1 - Reworked `PIDController` to no longer use any resources for tuning.  Also squashed some bugs around the variant property initialization.

## Contributing
Feel free to leave any feedback, bug reports, and contributions to the repository at [https://github.com/iiMidknightii/EzPID](https://github.com/iiMidknightii/EzPID).
