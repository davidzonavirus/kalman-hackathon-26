"""Reproduces the whole analysis from a clean checkout:  python analysis/run_all.py [--fast]

--fast runs the Monte Carlo with fewer trials (for a quick check; final numbers need the full run).
Runtime of the full run on a laptop: ~1-2 h, dominated by 07_mc_velocity (100,000 trials through
32,649 filter events each).
"""
import os
import runpy
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
STEPS = ["01_inventory.py", "02_replay_validation.py", "03_distance_analysis.py", "04_velocity_analysis.py",
         "05_noise_characterization.py", "06_mc_distance.py", "07_mc_velocity.py", "08_story_figures.py",
         "09_summary.py", "10_build_slides.py", "11_build_briefing.py"]

if __name__ == "__main__":
    sys.path.insert(0, HERE)
    os.chdir(HERE)
    if "--fast" in sys.argv:
        os.environ["GSK_MC_FAST"] = "1"
    for s in STEPS:
        t = time.time()
        print(f"=== {s}", flush=True)
        runpy.run_path(os.path.join(HERE, s), run_name="__main__")
        print(f"=== {s} done in {time.time() - t:.0f} s", flush=True)
