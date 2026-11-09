"""Build the pinned, camera-buffer ownership fix without installing system files."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SDK_COMMIT = "625279aa1fe903528a818f48746ee47c1b5fefd3"
PATCH = ROOT / "native/realsense_shared_memory/vendor_patches/librealsense-2.59.1-v4l2-buffer-owner.patch"


def run(*args, **kwargs):
    return subprocess.run([str(a) for a in args], check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / ".local_deps/librealsense-2.59.1")
    parser.add_argument("--build", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / ".local_deps/realsense-v4l2-fixed/lib")
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--cmake-arg", action="append", default=[])
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    if not sys.platform.startswith("linux"):
        parser.error("The V4L2 correction is for Linux hosts")
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    source = args.source.resolve()
    build = (args.build or source / "build-v4l2").resolve()
    output = args.output.resolve()
    if not source.exists():
        source.parent.mkdir(parents=True, exist_ok=True)
        run("git", "clone", "--depth", "1", "--branch", "v2.59.1",
            "https://github.com/realsenseai/librealsense.git", source)
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if revision != SDK_COMMIT:
        parser.error("SDK checkout differs from the pinned 2.59.1 revision")
    applied = subprocess.run(["git", "-C", str(source), "apply", "--reverse", "--check", str(PATCH)],
                             capture_output=True).returncode == 0
    if not applied:
        run("git", "-C", source, "apply", "--check", PATCH)
        run("git", "-C", source, "apply", PATCH)
    source_diff = subprocess.check_output(["git", "-C", str(source), "diff", "--binary", "HEAD"])
    if source_diff.replace(b"\r\n", b"\n") != PATCH.read_bytes().replace(b"\r\n", b"\n"):
        parser.error("SDK has additional tracked edits; preserve them and use a separate checkout")
    run(args.cmake, "-S", source, "-B", build,
        "-DCMAKE_BUILD_TYPE=Release", "-DFORCE_RSUSB_BACKEND=OFF",
        "-DBUILD_EXAMPLES=OFF", "-DBUILD_GRAPHICAL_EXAMPLES=OFF",
        "-DBUILD_UNIT_TESTS=OFF", "-DBUILD_PYTHON_BINDINGS=OFF", "-DBUILD_WITH_DDS=OFF",
        "-DCHECK_FOR_UPDATES=OFF", "-DENABLE_AI_ASSISTANT=OFF", "-DENABLE_STATS=OFF",
        *args.cmake_arg)
    run(args.cmake, "--build", build, "--target", "realsense2", "-j", args.jobs)
    libraries = [p for p in build.rglob("librealsense2.so.2.59.1") if p.is_file() and not p.is_symlink()]
    if len(libraries) != 1:
        raise RuntimeError(f"Expected one built SDK library, found {len(libraries)}")
    output.mkdir(parents=True, exist_ok=True)
    library = output / libraries[0].name
    temporary = output / (library.name + ".new")
    shutil.copy2(libraries[0], temporary)
    os.replace(temporary, library)
    for name in ["librealsense2.so.2.59", "librealsense2.so"]:
        link = output / name
        if link.is_symlink() and link.readlink() == Path(library.name):
            continue
        if link.exists() or link.is_symlink():
            raise RuntimeError(f"Refusing to replace an unrelated SDK link: {link}")
        link.symlink_to(library.name)
    report = {"sdk_revision": revision, "patch_sha256": hashlib.sha256(PATCH.read_bytes()).hexdigest(),
              "library": str(library), "library_sha256": hashlib.sha256(library.read_bytes()).hexdigest()}
    (output / "build.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
