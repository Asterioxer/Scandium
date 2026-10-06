#!/usr/bin/env python3
"""Deterministic verification entry point for agent-assisted changes."""
import argparse
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def run(command, cwd=ROOT):
    print("$", " ".join(command))
    subprocess.run(command, cwd=cwd, check=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--api", action="store_true", help="also verify the Python live-service API")
    args = parser.parse_args()

    build = ROOT / "build"
    run(["cmake", "-S", ".", "-B", str(build), "-DSCANDIUM_BUILD_TESTS=ON"])
    run(["cmake", "--build", str(build), "--parallel"])
    run(["ctest", "--test-dir", str(build), "--output-on-failure"])
    run([str(build / "scandium_collision_comparison")])
    run([str(build / "scandium_navigation_comparison")])

    if args.api:
        service = ROOT / "services" / "game_api"
        run(["python", "-m", "pytest"], cwd=service)

    print("Scandium verification passed.")

if __name__ == "__main__":
    main()
