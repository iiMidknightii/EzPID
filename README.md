# ![](addons/EzPID/icons/PIDController64.png) EzPID
 A lightweight, fast PID controller for Godot implemented as a C++ GDExtension.

## Installation
### From Godot Asset Library in the Editor
Click the `AssetLib` button at the top of the Godot editor and search for `EzPID`.  When prompted where to install it, you can select only the folder named "addons".  If you wish to modify or recompile the addon, then you'll need to include the "godot-cpp" and "src" folders along with the "SConstruct" file from this repository.

 ### From Godot Asset Library Web
 Head over to [the EzPID page on the asset library website](https://godotengine.org/asset-library/asset) and click the download button.  Unzip the download into a location of your choosing.  To put the addon in your project, just copy the "addons" folder into the project directory.

 ### From GitHub.com
 You can download the full repository for EzPID [here](https://github.com/iiMidknightii/EzPID).  You can clone this repository by doing `git clone https://github.com/iiMidknightii/EzPID.git` in the directory of your choosing.  If you want to compile your own binaries this is the best option.  To put the addon in your project, just copy the "addons" folder into the project directory.

## Tutorial
Just add a `PIDController` node to your scene, select a node/property to control, then begin tuning the gains.  You can sellect whether the `PIDController` updates in the process or physics frame.  If what you are trying to control is more complicated or you want finer control on when it updates, you can set the controller to update manually then call the `update_state` method directly.  You can also add custom state integration for the controlled property by overriding the `_integrate_state` method in a script that extends PIDController.  Both the `PIDController` node and the `PID` gains resource have built-in documentation for reference.

> [!TIP]
> You can enable the PID controller in the editor if you want to use it for editor plugins or nodes that need to run in @tool mode.

## Latest Release
* 1.0.2 - Fixed build system

## Contributing
Feel free to leave any feedback, bug reports, and contributions to the repository at [https://github.com/iiMidknightii/EzPID](https://github.com/iiMidknightii/EzPID).
