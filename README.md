# Human Tracker Node (`find_human.cpp`)

This node implements a two–phase pipeline to:

1. **Decide whether two known humans have moved** from their original map locations.
2. **Search the warehouse** and **estimate the new locations** of any humans that moved, using the global costmap and laser scans.

It is designed to run on top of the professor’s `Navigator` library (for `GoToPose` and `Spin` actions) and uses `map`, `global_costmap`, `scan`, and `amcl_pose` topics.

---

## High-Level Behavior

### Phase 1 – Check if the Humans Moved

**Goal:** From two known map locations for Human 0 and Human 1, decide if each one is still there or has moved.

1. **Wait for `/map` and `/amcl_pose`:**  
   State: `WAIT_FOR_MAP_AND_POSE`  
   - Once both are available, the robot goes to the **map origin (0,0)**.

2. **Go to map origin:**  
   State: `GOING_TO_ORIGIN`  
   - Uses `Navigator::GoToPose()` to move to (0,0).
   - When the origin is reached, the robot:
     - Computes which of the two humans is closer.
     - Calls that one the **“first”** human and the other the **“second”** human.

3. **Check first human:**  
   States: `GOING_TO_FIRST_HUMAN` → `CHECKING_FIRST_HUMAN`  
   - The robot moves to a **vantage point** 1 m “behind” the human’s original position.
   - For ~3 seconds:
     - Reads the latest `LaserScan`.
     - Uses geometry + robot pose + known human position to predict where in the scan that human should appear (range + angle).
     - Looks in a small angular **window** around that direction for a scan range that matches the expected distance within a tolerance (`RANGE_TOL`).
   - If at least one “good” sample is found → **human is STILL**.  
     If none are found → **human MOVED**.

4. **Check second human:**  
   States: `GOING_TO_SECOND_HUMAN` → `CHECKING_SECOND_HUMAN`  
   - Same logic as above, but for the second human.

5. **Phase 1 summary:**
   - Logs whether each human is `"STILL"` or `"MOVED"`.
   - If **no humans moved**, the node logs a summary and goes to `DONE` (Phase 2 is skipped).
   - If **at least one human moved**, it proceeds to Phase 2 (`WAIT_FOR_COSTMAP`).

---

### Phase 2 – Search the Warehouse and Estimate New Locations

**Goal:** Use the global costmap and laser scans to find **free-space costmap blobs** that likely correspond to moved humans, then estimate their new coordinates.

1. **Wait for `/global_costmap/costmap`:**  
   State: `WAIT_FOR_COSTMAP`  
   - Once the global costmap is available, the node:
     - Clears any old extra points.
     - Resets the search waypoint index.
     - Starts the search route.

2. **Search route waypoints:**
   - The node defines **17 hard-coded waypoints** (`search_waypoints_`) that cover the warehouse corridors and open areas (top left, top right, center, bottom, etc.).
   - For each waypoint:
     - State: `SEARCH_GOING_TO_WAYPOINT`
       - Sends a `GoToPose` to the current waypoint.
       - While moving, **every `LaserScan` is processed** to collect candidate extra points.
     - State: `SEARCH_SPINNING_AT_WAYPOINT`
       - Once the waypoint is reached, the robot performs a **360° spin** using `Navigator::Spin(2π)`.
       - During the spin, every incoming `LaserScan` is also processed.
   - After spinning at a waypoint, the node advances to the next waypoint until all are done.

3. **Collecting candidate “moved-human” points from scans**

   This logic runs inside `scanCallback()` **only during Phase 2 search states**:

   - Transform each valid laser hit from the laser frame into the **map frame** using TF (`tf_buffer_`).
   - For each point `(x,y)` in the map frame:
     1. **Check map occupancy (`/map`):**
        - Convert `(x,y)` to map indices `(mx,my)`.
        - If outside the map, or the cell is not **free** (`occ != 0`), reject the point.
     2. **Check costmap (`/global_costmap/costmap`):**
        - Convert `(x,y)` to costmap indices `(cmx,cmy)`.
        - If outside costmap, reject.
        - If cost ≤ 0 (free), reject.
     3. **Check distance to static obstacles:**
        - Around the map cell `(mx,my)`, look in a radius of ~0.8 m for any static occupied cells in `/map`.
        - If it’s within that band, it’s probably just the **normal inflation ring** around shelves/walls → reject.
     4. If all checks pass, the point is a **candidate extra point** and is appended to `extra_points_`.

   Intuition:
   - **Free in `/map`** but **occupied in costmap** and **not near walls/shelves** = a “floating” blob of costmap in free space ⇒ likely a **human-sized obstacle**.

4. **Clustering extra points:**

   Once all waypoints + spins are done, the node runs `clusterExtraPointsAndLog()`:

   - Performs a simple **radius-based clustering** (e.g., 0.8 m radius) on `extra_points_`.
   - For each cluster:
     - Calculates centroid `(cx, cy)` and number of points.
     - Logs clusters with their centroid and point count.
   - Filters out tiny clusters (configurable via `MIN_CLUSTER_POINTS`).

5. **Matching clusters to moved humans:**

   - Only humans previously marked as `moved` in Phase 1 are considered.
   - For each moved human and each cluster:
     - Compute the distance from the human’s **original** position to the cluster centroid.
     - Keep only candidates within a distance window `[MIN_MOVE_DIST, MAX_MOVE_DIST]`.
   - Build a list of `(human_idx, cluster_idx, distance)` candidates and sort by distance (closest first).
   - Greedy matching:
     - Assign each cluster to at most **one** human.
     - Each moved human gets at most **one** closest cluster.
   - For each moved human with a matched cluster:
     - Log a final estimate like:  
       `ESTIMATE: Human i new location ≈ (x=..., y=...) (moved D m from original (x0,y0), using cluster k).`

---

## Topics Used

**Subscriptions**

- `/map` – `nav_msgs/OccupancyGrid`  
  Static map, used to:
  - Determine free vs. occupied cells.
  - Identify static obstacles and their inflation band.

- `/global_costmap/costmap` – `nav_msgs/OccupancyGrid`  
  Global navigation costmap, used to:
  - Find costmap blobs that don’t align with static map obstacles.

- `/scan` – `sensor_msgs/LaserScan`  
  2D laser scan, used to:
  - Check presence/absence of humans at their original spots (Phase 1).
  - Collect extra points during search (Phase 2).

- `/amcl_pose` – `geometry_msgs/PoseWithCovarianceStamped`  
  Robot pose in the map frame, used for:
  - Computing expected distance/angle to humans.
  - Transforming scan hits into global coordinates.

**Actions (through `Navigator`)**

- `Navigator::GoToPose(goal_pose)`  
  - Used to:
    - Go to (0,0).
    - Move to vantages near humans.
    - Follow search waypoints.

- `Navigator::Spin(angle)`  
  - Used to perform **360° spins** at each search waypoint during Phase 2.

---

## Internal State Machine

`State` enum:

- `WAIT_FOR_MAP_AND_POSE`
- `GOING_TO_ORIGIN`
- `GOING_TO_FIRST_HUMAN`
- `CHECKING_FIRST_HUMAN`
- `GOING_TO_SECOND_HUMAN`
- `CHECKING_SECOND_HUMAN`
- `WAIT_FOR_COSTMAP`
- `SEARCH_GOING_TO_WAYPOINT`
- `SEARCH_SPINNING_AT_WAYPOINT`
- `DONE`

The `controlLoop()` timer (200 ms) drives transitions based on:

- `Navigator::IsTaskComplete()`
- Timers for Phase 1 sampling windows
- Availability of `/map`, `/amcl_pose`, `/global_costmap/costmap`

---

## Debug Logging

The node writes a CSV file in the current working directory:

- **File:** `human_tracker_debug.csv`
- **Columns:**
  - `time` – ROS time in seconds
  - `label` – event label (e.g., `PHASE1_GOAL_SENT`, `SEARCH_SPIN_DONE`, etc.)
  - `state` – internal state string
  - `wp_or_human_idx` – waypoint index or human index depending on context
  - `goal_x`, `goal_y` – goal coordinates when applicable
  - `amcl_x`, `amcl_y`, `amcl_yaw` – robot’s pose at time of log

This is useful for offline analysis (e.g., plotting routes, checking timing, correlating with bag data).

---

## How to Run

1. Build the Professor's 'navigation' package inside of MRTP
2. In a different shell run this command to start Gazebo and RViz:
   ```bash
   ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=true
3. Ensure the following are running (they will be after running the previous command):
   - Map server publishing `/map`
   - AMCL publishing `/amcl_pose`
   - Nav2 (or similar) publishing `/global_costmap/costmap` and wired into `Navigator`
   - Laser scan on `/scan`
4. In the same terminal where you built 'navigation' go inside of Ros2-Robot-Navigation and build 'nav' package that contains `find_human.cpp`.
5. Launch the node:
   ```bash
   ros2 run nav find_human_node
6. Once it's running, amcl_pose will be waiting for an update in order to start receiving messages through that topic, do the following:
   - In RViz press '2D Estimate Pose'
   - Click where to robot is located.
   - **This is enough to send an update and the node will start running.**
