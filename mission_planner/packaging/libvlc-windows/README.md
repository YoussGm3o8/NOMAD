libVLC redistributable packaging for NOMAD plugin (Windows)

This folder is intended to contain the native libVLC binaries (DLLs) and the `plugins` folder
that are required for the embedded LibVLC player to work when Mission Planner loads the plugin.

How to populate this folder:
1. Download the matching VLC redistributable (matching the operator machine bitness) from:
   https://www.videolan.org/vlc/

2. Extract/copy the following files and folders into this directory:
   - `libvlc.dll`, `libvlccore.dll`, and any other required DLLs
   - the `plugins` directory from the VLC installation (copy the entire folder)

3. Ensure the resulting tree looks like:
   packaging/libvlc-windows/
     libvlc.dll
     libvlccore.dll
     plugins/...

What the build does with it:
- The build script will copy all files from this folder into `src/bin/Release/` so they sit next to `NOMADPlugin.dll`.
- When you deploy to operator machines, either install VLC or copy these redistributables into the plugin folder next to the DLL.

Security note: Prefer redistributable package provided by VideoLAN. Verify checksums if distributing.
