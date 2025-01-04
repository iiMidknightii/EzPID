# ![](addons/EzPID/icons/PIDController64.png) EzPID
 A lightweight, fast PID controller for Godot implemented as a C++ GDExtension.

 ## Download
 ### From Godot Asset Library in the Editor
 Click the `AssetLib` button at the top of the Godot editor and search for `EzPID`.  From there, you can download it directly into your project.

 ### From Godot Asset Library Web
 Head over to [the EzPID page on the asset library website](https://godotengine.org/asset-library/asset) and click the download button.

 ### From GitHub.com
 You can download the full repository for EzPID [here](https://github.com/iiMidknightii/EzPID).  You can clone this repository by doing `git clone https://github.com/iiMidknightii/EzPID.git` in the directory of your choosing.  If you want to compile your own binaries this is the best option.


 ## Installation
 Once you have the files downloaded, there are a couple paths you could take for installation.  The [addons/EzPID](addons/EzPID/) folder can be directly copied into your project.  It already has the binaries for debug and release builds on Windows and Linux.
 
 If you wish to build the binaries from source, you'd need to also copy the [src](./src/), [doc_classes](./doc_classes/), and [godot-cpp](./godot-cpp/) folders along with the [SConstruct](./Sconstruct) file.  [This page will tell you how to build the extension from source using the `scons` command](https://docs.godotengine.org/en/stable/tutorials/scripting/gdextension/gdextension_cpp_example.html).

The actual GDExtension classes are in the [addons/EzPID/bin](addons/EzPID/bin) binaries and are added to Godot via [addons/EzPID/ez_pid.gdextension](addons/EzPID/ez_pid.gdextension).

## Tutorial
Just add a `PIDController` node to your scene, select a node/property to control, then begin tuning the gains.  You can sellect whether the `PIDController` updates in the process or physics frame.  If what you are trying to control is more complicated or you want finer control on when it updates, you can set the controller to update manually then call the `update_state` method directly.  You can also add custom state integration for the controlled property by overriding the `_integrate_state` method in a script that extends PIDController.  Both the `PIDController` node and the `PID` gains resource have built-in documentation for reference.

> [!TIP]
> You can enable the PID controller in the editor if you want to use it for editor plugins or nodes that need to run in @tool mode.

## Tagged Releases
* 1.0 - intial release, targets godot-4.3

## Contributing
Feel free to leave any feedback, bug reports, and contributions to the repository at [https://github.com/iiMidknightii/EzPID](https://github.com/iiMidknightii/EzPID).