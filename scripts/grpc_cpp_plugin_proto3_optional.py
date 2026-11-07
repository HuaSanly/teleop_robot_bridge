#!/usr/bin/env python3

"""Advertise proto3 optional support for the Ubuntu gRPC 1.16 C++ plugin.

The old plugin can generate service code for messages containing optional
fields, but predates CodeGeneratorResponse.supported_features. Newer protoc
versions require that capability bit before accepting the plugin response.
"""

import subprocess
import shutil
import sys


def main() -> int:
    request = sys.stdin.buffer.read()
    plugin = shutil.which("grpc_cpp_plugin")
    if plugin is None:
        print("grpc_cpp_plugin not found in PATH", file=sys.stderr)
        return 127
    result = subprocess.run(
        [plugin, *sys.argv[1:]],
        input=request,
        stdout=subprocess.PIPE,
    )
    if result.returncode != 0:
        return result.returncode

    # CodeGeneratorResponse.supported_features = FEATURE_PROTO3_OPTIONAL.
    sys.stdout.buffer.write(result.stdout + b"\x10\x01")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
