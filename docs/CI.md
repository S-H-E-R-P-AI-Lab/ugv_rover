# Continuous integration

Two GitHub Actions jobs check every pull request to `main` and every push to `main` (both
also run on demand from the Actions tab). They run in parallel. They only build and test;
nothing is deployed.

| Job | Workflow | Runs on | Time (measured) |
|---|---|---|---|
| **Quick check** | [quick-check.yml](../.github/workflows/quick-check.yml) | Every pull request, every push to `main` | About 1.5 minutes |
| **Build and test** | [ci.yml](../.github/workflows/ci.yml) | Pull requests that change more than docs, every push to `main` | About 7 minutes, 5 of them installing ROS 2 |

Both check out without the Git LFS files (`lfs: false`): the speech and vision model weights
are only copied at build time, and LFS bandwidth is limited. On failure, both upload `log/`
and the test results as an artifact (`quick-check-logs` or `ci-logs`).

### Quick check

Runs in the official `ros:humble-ros-base` container, so ROS is already installed.

| Step | What |
|---|---|
| Install | `shellcheck`, `xacro` and `depthai_descriptions` (the OAK camera macros the description includes), then `rosdep install` of the buildtool and test dependencies of `ugv_description` and `ugv_webots` (the ament linters, pytest, launch_testing and so on). |
| Lint shell scripts | `shellcheck -S warning scripts/*.sh` |
| Build | `colcon build` of `ugv_description` and `ugv_webots` only (`--base-paths` hides the other packages from colcon). |
| Test | `colcon test` on `ugv_webots`: flake8, pep257, copyright, xmllint and the unit tests. The sim smoke test skips itself (no Webots). |
| Parse the robot description | `xacro` on `ugv_rover.xacro`, `rasp_rover.xacro` and `ugv_beast.xacro`; any error fails the job. |

### Build and test

Installs everything on a fresh `ubuntu-22.04` runner, like a new team laptop.

| Step | What |
|---|---|
| Lint shell scripts | `shellcheck -S warning scripts/*.sh` |
| Install | [`scripts/setup_ubuntu.sh --no-build --no-upgrade --no-lfs`](../scripts/setup_ubuntu.sh), so it also tests the setup script. Then `rosdep install` must resolve every key (only `cmake_modules` is skipped, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md)). |
| Build | The whole workspace with `colcon build`, using ccache (restored from the latest earlier run, saved after each run). |
| Test | `colcon test` on our own package `ugv_webots`, as in Quick check. Upstream and vendored packages are built, not tested. |

Pull requests that only change `docs/**` or `**/*.md` skip it (`paths-ignore`). Pushes to
`main` always run it. Quick check is a separate workflow because `paths-ignore` skips a whole
workflow, and a required check has to report on every pull request.

### Required check

The plan is to make **Quick check** the required status check for merging into `main`, and
keep **Build and test** informational: it still runs and shows its result, but does not block
a merge. Neither is required yet. To set it up: Settings, Branches, the branch protection
rule for `main` (or Settings, Rules, Rulesets), turn on **Require status checks to pass
before merging** and add **Quick check** (only that one).

## What CI does not run

* **The simulator.** Webots, its world assets (downloaded from GitHub on first start) and
  CPU rendering made a CI sim job slow (about 20 minutes) and flaky, so the sim smoke test
  runs locally only. Run it before merging changes to `ugv_webots`, the worlds or the robot
  model (below).
* **arm64.** The robot's Jetson builds on the robot.

## Running the checks locally

From the workspace root, with ROS sourced:

```bash
colcon build
source install/setup.bash
colcon test --packages-select ugv_webots --event-handlers console_direct+   # linters, unit tests, sim smoke test
colcon test-result --verbose
shellcheck -S warning scripts/*.sh
xacro src/ugv_main/ugv_description/urdf/bases/ugv_rover.xacro > /dev/null   # also rasp_rover, ugv_beast
```

The sim smoke test ([`test_sim_smoke.py`](../src/ugv_main/ugv_webots/test/test_sim_smoke.py))
starts the empty arena headless, checks that `/clock` advances, every topic of the sensor
contract publishes with the right frame, the TF tree is complete, and the robot drives and
turns; after shutdown it checks no Webots process is left. It finds Webots through
`WEBOTS_HOME`, `/usr/local/webots` or `webots` on the `PATH` (in WSL the setup script sets
`WEBOTS_HOME="/mnt/c/Program Files/Webots"`). `UGV_SIM_TEST=0` skips it, `UGV_SIM_TEST=1`
forces it, `UGV_TEST_ARTIFACTS=/some/dir` saves a summary and the camera frames. Only one
simulation can run at a time. Only the smoke test:
`colcon test --packages-select ugv_webots --pytest-args -k sim_smoke`.

## When CI fails

1. Open the failed job from the pull request's **Checks** tab. Search the log for `Failed`
   or `error:`.
2. Download `ci-logs` from the run's **Summary** page for the colcon logs and `pytest.xml`.
3. Reproduce with the commands above. Lint failures name the file and line (lines wrap at
   99 characters).
4. A download or apt mirror error is not your change: **Re-run failed jobs**. If it repeats,
   open an issue.
