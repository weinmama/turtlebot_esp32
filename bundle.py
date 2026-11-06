"""Export the images and addresses selected by PlatformIO for remote flashing."""
import json
from pathlib import Path
import shutil

Import("env")


def bundle(source, target, env):
    output = Path(env.subst("$BUILD_DIR")) / "flash-bundle"
    output.mkdir(exist_ok=True)
    images = list(env.get("FLASH_EXTRA_IMAGES", []))
    images.append((env.subst("$ESP32_APP_OFFSET"), "$BUILD_DIR/${PROGNAME}.bin"))
    manifest = {"chip": env.BoardConfig().get("build.mcu"), "middleware": "picoros",
                "transport": "serial/460800", "images": []}
    for index, (address, filename) in enumerate(images):
        filename = Path(env.subst(str(filename)))
        name = f"{index}-{filename.name}"
        shutil.copyfile(filename, output / name)
        manifest["images"].append({"address": env.subst(str(address)), "file": name})
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Flash bundle: {output}")


env.AddCustomTarget(
    name="firmware_bundle",
    dependencies=["$BUILD_DIR/${PROGNAME}.bin"],
    actions=[bundle],
    title="Exporting remote flash bundle",
    always_build=True,
)
