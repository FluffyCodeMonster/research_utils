# FT 20/4/26
# This file contains utility code relating to SITL

import subprocess
from contextlib import contextmanager
from pathlib import Path


# Start SITL as system process
def start_sitl(
    ardupilot_bin_path, cwd, aircraft_params_filepath, home_coords, init_yaw
):
    print("Starting SITL...")
    # Have to give default values so that it thinks it's calibrated
    args = [
        str(ardupilot_bin_path),
        "--model",
        "plane",
        "--home",
        f"{','.join([str(v) for v in home_coords])},{init_yaw}",
        "--wipe",
        "--defaults",
        str(aircraft_params_filepath),
    ]
    proc = subprocess.Popen(args, cwd=cwd)
    print("Done")
    return proc


# Create a context manager for running the experiment in SITL
# Starts SITL and terminates process after simulation
# cwd: directory from which SITL is run. A logs folder will be created in this directory.
@contextmanager
def sitl_experiment(
    ardupilot_bin_path: Path,
    cwd: Path,
    aircraft_params_file: str,
    home_coords: tuple[float, float, float],
    init_yaw: float,
):

    proc = start_sitl(ardupilot_bin_path, cwd, aircraft_params_file, home_coords, init_yaw)

    try:
        yield
    except Exception as e:
        print(f"An exception occurred: {e}")

        # Terminate (kill: https://docs.python.org/3/library/subprocess.html#module-subprocess) the SITL process
        print("Ending SITL process")
        proc.terminate()
