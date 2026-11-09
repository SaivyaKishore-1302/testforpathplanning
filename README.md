# testforpathplanning
Formula Student cone planner for the simulation handoff. The original MATLAB
hybrid planner combines Delaunay gates, boundary matching, temporary boundary
hypotheses, and corridor-clipped Voronoi routes. Physical cone positions are
preserved for obstacle checks.

## Run on Linux

Install MATLAB (the source targets R2021a or newer) and put its `matlab`
executable on PATH. The planner uses base MATLAB. From the repository root:

```sh
matlab -batch "addpath('matlab'); disp(planConeFrame(fileread('examples/straight.json')))"
```

For the existing interactive MATLAB simulator, run `addpath('matlab'); main`
from MATLAB and choose **Start / Resume**.

## Connect a simulator

Call `planConeFrame` with a JSON string. It returns a JSON string so a
simulator adapter can use the same interface. Keep MATLAB running for repeated
calls, for example through the Python MATLAB Engine matching your installed
MATLAB release:

```python
import json
from pathlib import Path
import matlab.engine

engine = matlab.engine.start_matlab()
engine.addpath(str(Path("matlab").resolve()), nargout=0)
request = Path("examples/straight.json").read_text()
plan = json.loads(engine.planConeFrame(request, nargout=1))
print(plan["valid"], plan["path_xy"])
```

- `pose` is `[x_m, y_m, yaw_rad]`, with positive yaw counterclockwise.
- `cones` contains rows `[x_m, y_m, colour, variance_m2]`, with blue = 1
  and yellow = 2. Use a local metric map and keep cone coordinates and pose
  in the same frame. This adapter takes a planar map snapshot. Orange and
  unknown cones need separate handling before connecting a complete system.
- Optional `speed_mps` defaults to zero. Optional `previous_path` takes the
  last valid world-frame path as rows of `[x_m, y_m]` for continuity. Only
  supply it while fresh and in the same frame.
- The result includes `valid`, `status`, `method`, `path_xy` in the input
  frame, `confidence` and `speed_limit_mps`. If the plan is invalid, the
  adapter returns a zero speed limit. The returned speed limit is a planner
  hint, subject to the simulator controller's limits.

The planner defaults and footprint rollout are from the existing bicycle-model
simulator. Before testing the POC in closed loop, set its measured footprint
and clearance parameters and adapt the feasibility rollout and follower to
four-wheel skid steering. The repository supplies MATLAB source and a headless
interface. Gazebo world, ROS nodes and skid-steer wheel control are future
integration work.
