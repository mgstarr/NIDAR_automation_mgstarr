# AirMouse Navigation

Autonomous navigation for the NIDAR AirMouse drone: it explores an unknown indoor maze,
builds a live 2D map, makes sure its camera sees every room, returns to the entry and
lands. Everything is developed and tested in Gazebo first.

- Master plan (single source of truth): [docs/PLAN.md](docs/PLAN.md)
- Phase documents: [docs/phases/](docs/phases/) · Handoff notes: [docs/handoff/](docs/handoff/)

This folder is both the git root and the ROS 2 workspace root.

## 1. One-time host setup

Ubuntu with an NVIDIA GPU, plus:

- Docker Engine, with your user in the `docker` group (log out and in after adding it);
- NVIDIA Container Toolkit (`docker run --rm --gpus all ubuntu nvidia-smi` must work);
- VS Code with the **Dev Containers** extension (optional; plain `docker compose` also works).

## 2. Open the dev container

The container is Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic, with GPU access and
windows forwarded to your screen. The repo is mounted at `/ws` inside it.

**VS Code:** open this folder, then *Ctrl+Shift+P → Dev Containers: Reopen in Container*.
The first build downloads about 5 GB.

**Without VS Code:**

```bash
xhost +SI:localuser:$(id -un)        # allow the container to open windows (once per login)
echo RENDER_GID=$(getent group render | cut -d: -f3) > .devcontainer/.env   # host GPU group id
docker compose -f .devcontainer/docker-compose.yml up -d --build
docker compose -f .devcontainer/docker-compose.yml exec dev bash
# when done:
docker compose -f .devcontainer/docker-compose.yml down
```

## 3. Build and test (inside the container)

```bash
cd /ws
colcon build --symlink-install
source install/setup.bash
pytest src/airmouse_nav/test -v
ros2 pkg list | grep airmouse        # shows the three packages
```

## 4. Check graphics

```bash
gz sim shapes.sdf     # a Gazebo window with a few shapes
rviz2                 # an RViz window
```

## Packages

| Package | Contents |
|---|---|
| `src/airmouse_description` | drone model, maze worlds, maze generator |
| `src/airmouse_bringup` | all launch files, all parameter files, RViz layout |
| `src/airmouse_nav` | navigation algorithms (`core/`), ROS 2 nodes, flight-controller bridges, tests |
| `tools/` | offline scripts: map scoring, plotting, benchmarks |
