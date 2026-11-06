"""Build the pinned Pico-ROS libraries with the project's cross compiler."""
from pathlib import Path
import subprocess
import sys

Import("env")

root = Path(env.subst("$PROJECT_DIR"))
pico = root / "thirdparty/picoros"
zenoh = pico / "thirdparty/zenoh-pico"
if not (zenoh / "CMakeLists.txt").exists():
    raise RuntimeError("Run git submodule update --init --recursive first")
generated = Path(env.subst("$BUILD_DIR")) / "zenoh-config"
patched = Path(env.subst("$BUILD_DIR")) / "zenoh-patched"
subprocess.run([sys.executable, str(root / "scripts/patch_zenoh.py"),
                str(zenoh / "src/transport/unicast/rx.c"), str(patched / "rx.c")], check=True)
patched_picoros = Path(env.subst("$BUILD_DIR")) / "picoros-patched"
subprocess.run([sys.executable, str(root / "scripts/patch_picoros.py"),
                str(pico / "src/picoros.c"), str(patched_picoros / "picoros.c")], check=True)
subprocess.run([
    "cmake", "-S", str(zenoh), "-B", str(generated),
    "-DZP_PLATFORM=arduino_esp32", "-DBUILD_EXAMPLES=OFF", "-DBUILD_TESTING=OFF",
    "-DZ_FEATURE_LINK_SERIAL=1", "-DZ_FEATURE_LINK_TCP=0", "-DZ_FEATURE_LINK_UDP_UNICAST=0",
    "-DZ_FEATURE_LINK_UDP_MULTICAST=0", "-DZ_FEATURE_MULTICAST_TRANSPORT=0",
    "-DZ_FEATURE_SCOUTING=0", "-DZ_FEATURE_AUTO_RECONNECT=0",
    "-DFRAG_MAX_SIZE=4096", "-DBATCH_UNICAST_SIZE=2048",
], check=True)
env.Append(CPPPATH=[str(generated / "include"), str(zenoh / "include"),
                    str(pico / "src"), str(pico / "thirdparty/Micro-CDR/include"),
                    str(pico / "thirdparty/config"), str(root / "include")])
env.BuildSources("$BUILD_DIR/zenoh", str(zenoh / "src"), src_filter=[
    "+<*>", "-<system/>", "+<system/common/>", "+<system/socket/>",
    "+<system/arduino/esp32/>", "-<link/transport/tcp/>", "-<link/transport/udp/>",
    "-<transport/unicast/rx.c>",
])
env.BuildSources("$BUILD_DIR/zenoh-patched-objects", str(patched))
env.BuildSources("$BUILD_DIR/picoros", str(pico / "src"),
                 src_filter=["+<picoserdes.c>"])
env.BuildSources("$BUILD_DIR/picoros-patched-objects", str(patched_picoros))
env.BuildSources("$BUILD_DIR/microcdr", str(pico / "thirdparty/Micro-CDR/src/c"))
