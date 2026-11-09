"""Rebuild the checked-in browser converter (development dependency: wasmtime)."""

from pathlib import Path

from wasmtime import wat2wasm


def main() -> None:
    web = Path(__file__).resolve().parents[1] / "web"
    source = web / "jpeg_i420_rgba.wat"
    output = web / "jpeg_i420_rgba.wasm"
    output.write_bytes(wat2wasm(source.read_text(encoding="utf-8")))
    print(f"Built {output.name}: {output.stat().st_size} bytes")


if __name__ == "__main__":
    main()
