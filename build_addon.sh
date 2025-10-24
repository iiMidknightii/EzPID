#!/usr/bin/bash
rm -f addons.zip

scons
godot --doctool ./ --gdextension-docs
scons target=template_debug
scons target=template_release
scons target=template_debug platform=windows
scons target=template_release platform=windows

rm -f addons/*/bin/~*.TMP
cp LICENSE README.md addons/EzPID

prev_dir=$(basename $PWD)
cd ..
zip -9 -r "$prev_dir/addons.zip" "$prev_dir/addons/"
cd "$prev_dir"
rm -f addons/EzPID/README.md addons/EzPID/LICENSE