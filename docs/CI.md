# Continuous integration

One GitHub Actions workflow, [CI](../.github/workflows/ci.yml), checks every pull request to
`main` and every push to `main` (and runs on demand from the Actions tab). It only builds and
tests; nothing is deployed. It takes about 10 minutes, most of it installing ROS 2.

| Step | What |
|---|---|
| Check out | Without the Git LFS files (`lfs: false`): the speech and vision model weights are only copied at build time, and LFS bandwidth is limited. |
| Lint shell scripts | `shellcheck -S warning scripts/*.sh` |
| Install | [`scripts/setup_ubuntu.sh --no-build --no-upgrade --no-lfs`](../scripts/setup_ubuntu.sh) on a fresh `ubuntu-22.04` runner, so every pull request also tests the setup script. Then `rosdep install` must resolve every key (only `cmake_modules` is skipped, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md)). Shared setup: [`.github/actions/ros-workspace`](../.github/actions/ros-workspace/action.yml). |
| Build | The whole workspace with `colcon build`, using ccache (keyed on every `package.xml` and `CMakeLists.txt`). |
| Test | `colcon test` on our own package `ugv_webots`: flake8, pep257, copyright, xmllint and the unit tests. The sim smoke test skips itself (no Webots on the runner). Upstream and vendored packages are built, not tested. |
| On failure | Uploads `log/` and the test results as the `ci-logs` artifact. |

`main` requires this check (**Build and test**) to pass before merging.

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
