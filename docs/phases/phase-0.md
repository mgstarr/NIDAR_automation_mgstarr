# Phase 0 – Environment and Repository

Nov 2, 2026 · @Mahima

> Markdown export of the PDF *Phase 0 – Environment and Repository*.

**Goal:** an identical, reproducible development setup (Docker dev container with ROS 2
Jazzy + Gazebo Harmonic) and an empty but buildable workspace.

## How these phase documents work

- The master plan is `docs/PLAN.md` (export of *AirMouse Navigation – Phased
  Implementation Plan*). It holds the fixed facts, interface contract, file structure, coding
  standards and parameters. Phase documents reference it instead of repeating it.
- Save each phase document as `docs/phases/phase-N.md`.
- Every phase ends by writing `docs/handoff/phase-N.md` and tagging `phase-N`. The
  next phase starts by reading that note. That note is the bridge between phases.

## Before you start (entry checklist)

- [x] ~~Host: Ubuntu 22.04 with the NVIDIA driver working (`nvidia-smi` prints the RTX 4060)~~
- [ ] Docker installed, user in the `docker` group
- [ ] NVIDIA Container Toolkit installed (`docker run --rm --gpus all ubuntu nvidia-smi` works)
- [ ] VS Code with the Dev Containers extension
- [ ] Repository folder open in VS Code; `docs/PLAN.md` and `docs/phases/` saved in it
- [ ] If an earlier native scaffold exists in the repo, keep it; this phase wraps it in the
  container

## Working rules (all phases)

1. Read `docs/PLAN.md` and this file first.
2. Explain the approach for the phase (files, functions, what each does and how). Wait for
   approval.
3. Build one module at a time with its tests, then pause for review.
4. Ask before using any value not in `docs/PLAN.md`. Never change contract names or
   parameter names without approval.
5. Explain unfamiliar terms briefly the first time they appear.

## Tasks

1. `.devcontainer/` with a `Dockerfile` and `devcontainer.json`:
   - base image Ubuntu 24.04 + ROS 2 Jazzy desktop (e.g. `osrf/ros:jazzy-desktop-full`);
   - install Gazebo Harmonic, `ros_gz`, `rtabmap_ros`, `robot_localization`,
     `teleop_twist_keyboard`, numpy, scipy, pytest;
   - GPU access and display forwarding (X11 or Wayland), so Gazebo and RViz windows
     open on the host;
   - also runnable without VS Code through a `docker compose` file.
2. Workspace skeleton exactly as in *PLAN.md › Repository structure*:
   `airmouse_description`, `airmouse_bringup`, `airmouse_nav`; empty modules with
   docstrings; empty YAML configs in `airmouse_bringup/config/`.
3. Test setup: pytest runs `src/airmouse_nav/test/`; one placeholder test per `core/`
   module.
4. `.gitignore`: `build/ install/ log/ __pycache__/ .pytest_cache/ *.db *.db3
   *.mcap .vscode/`. Default ament linters skipped.
5. Root `README.md`: open the container, build, test.

## Tests

- `colcon build` succeeds inside the container
- `pytest` passes
- `ros2 pkg list` shows the three packages

## Gazebo check

Inside the container, `gz sim shapes.sdf` opens a window and `rviz2` opens. This proves
GPU and display forwarding work.

## Exit criteria

- [ ] Container builds and opens from VS Code ("Reopen in Container") and from `docker
  compose`
- [ ] Gazebo and RViz windows open from inside the container
- [ ] `colcon build` and `pytest` pass
- [ ] Handoff note written, committed, tagged `phase-0`

## Handoff to Phase 1

Write `docs/handoff/phase-0.md` with:

- exact commands to open the container, build and test;
- the image name and installed package versions (ROS 2, Gazebo, rtabmap_ros);
- any deviation from this document and why;
- open issues.

Phase 1 receives: a working container, a buildable empty workspace, and passing
placeholder tests.

**Start prompt for Phase 1** (paste into Claude Code):

```
Read docs/PLAN.md, docs/phases/phase-1.md and docs/handoff/phase-0.md.
Run the Phase 1 entry checklist and report the result.
Then explain your approach for Phase 1 and wait for my approval.
```
