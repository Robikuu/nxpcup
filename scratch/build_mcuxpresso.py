import subprocess
import os
import shutil

src_dir = r"d:\nxpcup"
dst_workspace = r"D:\robotica\workspace\nxpcup-ciorba_curata"

# Sync source and include files to workspace
for folder in ["include", "source"]:
    s_folder = os.path.join(src_dir, folder)
    d_folder = os.path.join(dst_workspace, folder)
    if os.path.exists(s_folder):
        for f in os.listdir(s_folder):
            if f.endswith(".c") or f.endswith(".h"):
                s_file = os.path.join(s_folder, f)
                d_file = os.path.join(d_folder, f)
                shutil.copy2(s_file, d_file)
                print(f"Synced {folder}/{f}")

tool_bin = r"C:\nxp\MCUXpressoIDE_25.6.136\ide\plugins\com.nxp.mcuxpresso.tools.win32_25.6.0.202501151204\tools\bin"
buildtool_bin = r"C:\nxp\MCUXpressoIDE_25.6.136\ide\plugins\com.nxp.mcuxpresso.tools.win32_25.6.0.202501151204\buildtools\bin"
debug_dir = r"D:\robotica\workspace\nxpcup-ciorba_curata\Debug"

env = os.environ.copy()
env["PATH"] = f"{tool_bin};{buildtool_bin};" + env.get("PATH", "")

print("Running make clean...")
res_clean = subprocess.run(["make", "clean"], cwd=debug_dir, env=env, capture_output=True, text=True)

print("Running make -j8 all...")
res_build = subprocess.run(["make", "-j8", "all"], cwd=debug_dir, env=env, capture_output=True, text=True)
print("Return code:", res_build.returncode)
print("STDOUT:\n", res_build.stdout[-2500:] if len(res_build.stdout) > 2500 else res_build.stdout)
if res_build.stderr:
    print("STDERR:\n", res_build.stderr)

if res_build.returncode == 0:
    src_axf = os.path.join(debug_dir, "nxpcup.axf")
    dst_axf = r"d:\nxpcup\Debug\nxpcup.axf"
    if os.path.exists(src_axf):
        shutil.copy2(src_axf, dst_axf)
        src_map = os.path.join(debug_dir, "nxpcup.map")
        dst_map = r"d:\nxpcup\Debug\nxpcup.map"
        if os.path.exists(src_map):
            shutil.copy2(src_map, dst_map)
        print(f"SUCCESS! Binary copied to {dst_axf} (size: {os.path.getsize(dst_axf)} bytes, modified: {os.path.getmtime(dst_axf)})")
else:
    print("Build FAILED!")
