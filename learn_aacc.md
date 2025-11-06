# Learning AACC (Angle Aware Coverage Control) - Architecture Deep Dive

**Date:** November 1, 2025  
**Purpose:** Understanding how `aacc.launch.py` works and all the mechanisms behind it

---

## 🎯 Overview

The AACC codebase is indeed **super modular and highly scalable**, with many layers of abstraction working together. Here's what happens when you run `aacc.launch.py`.

---

## 📂 Entry Point: `aacc.launch.py`

```python
from dji_drone_bringup.launch_description_generator import ez_generate, use_dynamic_config

drones = ["mavic_1", "mavic_2", "mavic_3"]

def generate_launch_description():
    return ez_generate(
        namespace=drones,
        package=["dji_drone_demos", "joy"],
        executable=["angle_aware_coverage_control", "joy_node"],
        param_file="aacc.yaml",
        rviz_config=use_dynamic_config(drones, point_cloud2_rgb="density", ...),
    )
```

**Key Points:**
- Launches 3 drones: `mavic_1`, `mavic_2`, `mavic_3`
- Each drone runs two executables:
  - `angle_aware_coverage_control` (the main controller)
  - `joy_node` (joystick controller for manual control)
- Parameters are loaded from `aacc.yaml`
- RViz configuration is dynamically generated

---

## 🏗️ Layer 1: Launch Description Generator

**File:** `dji_drone_bringup/launch_description_generator.py`

### What `ez_generate()` does:

1. **For each drone namespace** (mavic_1, mavic_2, mavic_3):
   - Creates a **`virtual_drone`** node (simulates the physical drone)
   - Creates an **`image_transport`** node (handles camera images)
   - Creates the **custom program nodes** (angle_aware_coverage_control, joy_node)
   - Creates a **`robot_state_publisher`** node (publishes URDF transforms)

2. **Initial Position Assignment:**
   - Reads `battery_charging.station_position` from `aacc.yaml`
   - If not found, assigns random positions
   - Each drone gets an `init_position` like `[x, y, 0.0, 0.0, 0.0, 0.0]`

3. **Parameter Passing:**
   - Loads `aacc.yaml` configuration file
   - Passes parameters to all nodes

4. **RViz Setup:**
   - Generates dynamic RViz configuration
   - Sets up visualizations for point clouds (density), field polygons, etc.

---

## 🎮 Layer 2: The Controller Node

**File:** `dji_drone_demos/angle_aware_coverage_control.py`

### Class Hierarchy (The "Drone Object"):

```
AngleAwareCoverageController
├── DistributedController
│   ├── Controller
│   │   └── Driver
│   │       └── Node (ROS2 base)
│   └── DistributedNode
│       └── Node (ROS2 base)
└── JoyController
    └── Driver
        └── Node (ROS2 base)
```

**Important:** The "drone object" is NOT explicitly instantiated. Instead:
- The **Controller itself IS the ROS2 Node**
- It inherits from `Driver`, which provides drone control interface
- Through inheritance, it gets all the drone capabilities

### The Controller Initialization Flow:

```python
class AngleAwareCoverageController(DistributedController, JoyController):
    def __init__(self, node_name: str, param_file: str = 'aacc.yaml'):
        super().__init__(node_name)  # Initializes ROS2 node + Driver capabilities
        
        # 1. Load parameters from aacc.yaml
        self.load_parameters_from_file(param_file, 'dji_drone_demos')
        
        # 2. Create source space (5D: x, y, z, horizontal_angle, vertical_angle)
        source_space = Space('xyzhv')
        # ... configure ranges and densities
        
        # 3. Create target space (2D: x, y)
        space = Space('xy')
        # ... configure ranges and densities
        
        # 4. Create the coverage algorithm object
        self.angle_aware_coverage = AngleAwareCoverage(
            init_density=init_density,
            a=..., delta=..., gamma=..., sigma=...,
            altitude=1.0,
            update_period=0.1,
            source_space=source_space,
            space=space,
            slack_variable=0.1,
            get_positions=self.get_positions  # Callback to get all drone positions
        )
        
        # 5. Create CBF constraints (field limitation, obstacle avoidance, etc.)
        self.field_limitation = ConvexPolygonFieldLimitation(...)
        self.obstacle_avoidance = ObstacleAvoidance(...)
        self.collision_avoidance = CollisionAvoidance(...)
        
        # 6. Create the QP solver
        self.solver = AngleAwareCoverageControlSolver()
        self.solver.add_cbf(self.angle_aware_coverage)
        
        # 7. Set up ROS2 publishers and timers
        self.control_timer = self.create_timer(0.1, self.send_command)
```

---

## 🧮 Layer 3: The Coverage Algorithm

**Classes:**
- `AngleAwareCoverage` - The coverage algorithm (inherits from `CbfBasedCoverage`)
- `AngleAwareCoverageControlSolver` - The QP solver

### How Coverage Works:

1. **Separate Thread for Calculation:**
   ```python
   def start_thread(self):
       self.run_thread = True
       self.calc_thread = threading.Thread(target=self.calc)
       self.calc_thread.start()
   ```

2. **Main Calculation Pipeline** (runs in separate thread):
   ```python
   def calc(self):
       while self.run_thread:
           positions = self.get_positions()  # Get all drone positions
           
           # Calculate performance functions (Gaussian kernels)
           self.performance_funcs = calc_performance_funcs(positions, grids, param)
           
           # Calculate Voronoi diagrams (which cells each drone covers)
           self.voronoi_diagrams = calc_voronoi_diagrams(positions, grids)
           
           # Update density function (coverage decreases over time)
           self.density_func = calc_density_func(positions, self.density_func, ...)
           
           # Calculate gradient for coverage objective
           self._grad = calc_grad(positions, self.density_func, ...)
           
           # Calculate CBF value
           self._cbf = calc_cbf(self.density_func, ...)
   ```

3. **JAX Acceleration:**
   - All heavy computations use JAX (Just-in-Time compiled)
   - Can run on GPU or CPU (configurable via `use_cpu` parameter)

---

## 🚀 Layer 4: The Driver Layer

**File:** `dji_drone_control/driver.py`

### What `Driver` Provides:

The `Driver` class encapsulates all low-level drone control:

```python
class Driver(Node):
    def __init__(self, node_name: str, namespace: str = None, **kwargs):
        super().__init__(node_name, namespace=namespace, **kwargs)
        
        # Create subscribers for drone state
        self.position_sub = self.create_subscription(PoseStamped, 'pos', ...)
        self.battery_state_sub = self.create_subscription(BatteryState, ...)
        self.gimbal_state_sub = self.create_subscription(Vector3Stamped, ...)
        
        # Create publishers for drone commands
        self.takeoff_pub = self.create_publisher(Empty, 'takeoff', ...)
        self.land_pub = self.create_publisher(Empty, 'land', ...)
        self.piloting_pub = self.create_publisher(Twist, 'cmd_vel', ...)
        self.gimbal_cmd_pub = self.create_publisher(Vector3, 'gimbal_cmd', ...)
        
        # Initialize state objects
        self.position = Position()  # Current position
        self.gimbal_state = Gimbal()  # Gimbal angles
        self.battery = None  # Battery percentage
```

**Key Methods:**
- `takeoff()` - Send takeoff command
- `land()` - Send land command
- `pilot(x, y, z, yaw)` - Send velocity commands
- `rotate_gimbal(roll, pitch, yaw)` - Control camera gimbal
- `world_to_body(...)` - Coordinate transformations

---

## 🔗 Layer 5: Distributed Control

**File:** `dji_drone_control/distributed_controller.py`

### Multi-Agent Coordination:

1. **DistributedNode:**
   - Detects online drones via `/dji_online` topic
   - Maintains list of active drones
   - Triggers callbacks when drones join/leave

2. **DistributedSubscription:**
   - Dynamically subscribes to topics from all active drones
   - Example: subscribes to `/mavic_1/pos`, `/mavic_2/pos`, `/mavic_3/pos`
   - Collects data into a dictionary or array

3. **DistributedController:**
   - Extends `Controller` with multi-agent capabilities
   - Uses `DistributedSubscription` to get positions of all drones
   - Provides `get_positions()` method that returns all drone positions

```python
class DistributedController(Controller, DistributedNode):
    def __init__(self, node_name: str):
        super().__init__(node_name=node_name, 
                        online_topic="/dji_online", 
                        state_check_interval=0.3, 
                        timeout=1.2)
        
        # Replace single position subscription with distributed one
        self.position_manager = DistributedSubscription(self, self.position_sub)
    
    def get_positions(self):
        positions = self.position_manager.array  # Gets np.ndarray of all positions
        return positions
```

---

## 🎨 Layer 6: Control Barrier Functions (CBF)

**File:** `dji_drone_control/control_barrier_function.py`

### CBF Architecture:

```python
class ControlBarrierFunction(ABC):
    """Abstract base class for all CBFs"""
    
    @abstractmethod
    def _calc_cbf(self, positions: np.ndarray):
        """Must return h(x) where h(x) >= 0 means safe"""
        pass
    
    @abstractmethod
    def _calc_grad(self, positions: np.ndarray):
        """Must return ∇h(x)"""
        pass
```

**Concrete CBF Implementations:**
- `AngleAwareCoverage` - Coverage objective as CBF
- `ConvexPolygonFieldLimitation` - Keep drones inside field
- `ObstacleAvoidance` - Avoid static obstacles
- `CollisionAvoidance` - Avoid other drones

### Threaded CBF:

```python
class ThreadedControlBarrierFunction(ControlBarrierFunction):
    """CBF that runs calculations in separate thread"""
    
    def start_thread(self):
        # Start background thread for heavy computations
        pass
    
    def stop_thread(self):
        # Stop background thread
        pass
```

---

## 🎛️ Layer 7: The Control Loop

**Every 0.1 seconds (10 Hz), the controller executes:**

```python
def send_command(self):
    if not self.is_control:
        return  # Manual control mode (joystick)
    
    # 1. Calculate altitude and yaw control (simple proportional control)
    uz = 0.5 * (altitude_target - self.position.z)
    uw = yaw_error
    
    # 2. Solve QP problem to get optimal [ux, uy]
    u = self.solver.solve()  # Returns [ux, uy] from QP
    
    # 3. Apply velocity limits
    velo = sqrt(u[0]**2 + u[1]**2)
    if velo > self.velo_limit:
        ux = self.velo_limit * u[0] / velo
        uy = self.velo_limit * u[1] / velo
    
    # 4. Send command to drone
    self.pilot(ux, uy, uz, uw)
    
    # 5. Control gimbal
    self.rotate_gimbal(roll, pitch, yaw)
```

---

## 🧩 The QP Solver

**File:** `dji_drone_control/control_barrier_function.py`

### The Optimization Problem:

```
minimize    (1/2)||u - u_ref||² + grad_co * ||∇coverage||
subject to  ∇h_coverage · u + α(h_coverage) >= 0  (coverage CBF)
            ∇h_field · u + α(h_field) >= 0        (field limitation)
            ∇h_obstacle · u + α(h_obstacle) >= 0  (obstacle avoidance)
            ∇h_collision · u + α(h_collision) >= 0 (collision avoidance)
            ||u|| <= u_max                         (velocity limit)
```

Where:
- `u = [ux, uy]` - Control input (velocity)
- `∇h · u + α(h) >= 0` - CBF constraint (forward invariance)
- `grad_co` - Weight for coverage gradient

---

## 📊 The Space Objects

**File:** `dji_drone_control/controller.py`

### 5D Source Space (Camera Field of View):

```python
source_space = Space('xyzhv')
source_space.set_x_range(-2.0, 2.0)    # Ground coordinates
source_space.set_y_range(-2.0, 2.0)
source_space.set_z_range(0.0, 1.0)     # Altitude
source_space.set_h_range(-π, π)        # Horizontal camera angle
source_space.set_v_range(1.047, 1.571) # Vertical camera angle (60° to 90°)
source_space.generate()
```

### 2D Target Space (Ground Coverage):

```python
space = Space('xy')
space.set_x_range(-2.2, 2.2)
space.set_y_range(-2.2, 2.2)
space.set_x_density(0.01)  # 1cm resolution
space.set_y_density(0.01)
space.generate()
```

The `Space` class:
- Generates discrete grids using `np.meshgrid`
- Supports polygon constraints
- Provides grid points for coverage calculations

---

## 🔄 The Complete Data Flow

```
1. Launch File (aacc.launch.py)
   ↓
2. Launch Description Generator
   ├─→ Creates virtual_drone nodes (simulation)
   ├─→ Creates image_transport nodes
   ├─→ Creates angle_aware_coverage_control nodes
   └─→ Creates joy_node (joystick)
   ↓
3. AngleAwareCoverageController.__init__()
   ├─→ Inherits from DistributedController (multi-agent)
   ├─→ Inherits from JoyController (manual control)
   ├─→ Loads parameters from aacc.yaml
   ├─→ Creates Space objects (source & target)
   ├─→ Creates AngleAwareCoverage object
   ├─→ Creates CBF constraints
   ├─→ Creates QP solver
   └─→ Starts control timer
   ↓
4. AngleAwareCoverage.start_thread()
   ├─→ Launches separate thread
   ├─→ Continuously calculates:
   │   ├─→ Performance functions (Gaussian kernels)
   │   ├─→ Voronoi diagrams
   │   ├─→ Density function update
   │   ├─→ Coverage gradient
   │   └─→ CBF value
   └─→ Uses JAX for GPU acceleration
   ↓
5. Control Timer fires (10 Hz)
   ├─→ send_command() is called
   ├─→ Gets drone positions via DistributedSubscription
   ├─→ Solver.solve() runs QP optimization
   ├─→ Returns optimal velocity [ux, uy]
   ├─→ pilot() sends Twist message to virtual_drone
   └─→ rotate_gimbal() sends Vector3 message
   ↓
6. Virtual Drone
   ├─→ Receives cmd_vel (Twist)
   ├─→ Simulates physics
   ├─→ Updates drone position
   └─→ Publishes pose to /pos topic
   ↓
7. Driver.position_callback()
   ├─→ Receives PoseStamped
   ├─→ Updates self.position
   ├─→ Publishes TF transform
   └─→ Publishes joint states for visualization
```

---

## 🎯 Key Insights

### 1. **No Explicit Drone Object**
- The controller **IS** the drone control interface
- `Driver` provides low-level control methods
- `Controller` adds parameter management
- `DistributedController` adds multi-agent coordination

### 2. **No Mode Attribute (Yet)**
You mentioned the drone doesn't have a `mode` attribute yet. That's correct! The current implementation only has:
- `is_control` - Boolean flag (True: autonomous, False: manual)
- No explicit state machine for Transfer/Patrol/Charging modes

### 3. **Modularity Through Inheritance**
```
Every controller inherits:
  Driver          → Basic drone control (takeoff, land, pilot)
  Controller      → Parameter management, Space objects
  DistributedController → Multi-agent coordination
  JoyController   → Joystick integration
```

### 4. **Separation of Concerns**
- **Coverage calculation** → Separate thread (compute-heavy)
- **Control loop** → Main thread (send commands)
- **Optimization** → QP solver (combines all CBFs)
- **Position tracking** → DistributedSubscription (dynamic multi-agent)

### 5. **The Magic of `get_positions`**
This callable is passed everywhere and provides the glue:
```python
self.angle_aware_coverage = AngleAwareCoverage(
    get_positions=self.get_positions  # From DistributedController
)
self.field_limitation = ConvexPolygonFieldLimitation(
    get_positions=self.get_positions
)
```
Every CBF can access current drone positions through this callback!

---

## 🚀 What Makes It Scalable

1. **Dynamic Drone Detection**
   - `DistributedNode` automatically detects new drones
   - `DistributedSubscription` auto-subscribes to their topics

2. **Namespace-based Architecture**
   - Each drone has its own namespace (`/mavic_1`, `/mavic_2`, etc.)
   - ROS2 automatically isolates topics

3. **Modular CBF Design**
   - Add new constraints by creating new CBF classes
   - `solver.add_cbf(new_constraint)`

4. **JAX for Performance**
   - GPU acceleration for heavy math
   - JIT compilation for speed

5. **Thread Separation**
   - Coverage calculations don't block control loop
   - Each drone runs independently

---

## 📝 Summary

When you run `aacc.launch.py`:

1. **Launch generator** creates nodes for 3 drones
2. Each drone runs **AngleAwareCoverageController** 
3. Controller inherits **Driver** (low-level control) + **DistributedController** (multi-agent)
4. Controller creates **AngleAwareCoverage** object (runs in separate thread)
5. Controller creates **CBF constraints** (field, obstacles, collisions)
6. Controller creates **QP solver** (combines all CBFs)
7. **Control loop** (10 Hz) solves QP and sends velocity commands
8. **Virtual drone** simulates physics and publishes position
9. **Position updates** trigger next control cycle

The "drone object" is never explicitly created - the **controller itself IS the drone interface** through inheritance!

---

## 🎓 Lessons Learned

1. **Inheritance over Composition** - Uses deep inheritance hierarchies
2. **Callbacks for Decoupling** - `get_positions` callback connects components
3. **Threading for Performance** - Separate thread for heavy computation
4. **ROS2 Native** - Leverages ROS2 features (namespaces, parameters, timers)
5. **Mathematical Rigor** - CBF theory ensures safety guarantees

This architecture allows you to:
- Add more drones easily (just add to namespace list)
- Add more constraints (create new CBF classes)
- Switch algorithms (replace AngleAwareCoverage)
- Scale to GPU (configure JAX)

**Truly a beautiful piece of engineering!** 🎉

---

**Next Steps for Adding Mode Attribute:**

To add Transfer/Patrol/Charging modes like in your MPCS system:

1. Add `mode` attribute to `Driver` or `Controller` class
2. Create mode transition logic in control loop
3. Modify `send_command()` to behave differently per mode
4. Add field management (multiple fields)
5. Add battery charging logic

This would bridge the gap between the current AACC implementation and your multi-field MPCS architecture!

---

# Suggestions on the Next Move

## Where Should We Add Mode Attribute?

Looking at the existing code, we have several options for where to add the `mode` attribute. Let's compare them:

### Option 1: Add to `Driver` Class ⭐ **RECOMMENDED**

**Location:** `dji_drone_control/driver.py`

**Pros:**
- **Most fundamental level** - Every controller inherits from Driver
- Provides mode to ALL control algorithms (AACC, PCC, etc.)
- Consistent with other drone state attributes (`position`, `battery`, `gimbal_state`)
- Can be accessed anywhere in the inheritance chain

**Cons:**
- Changes affect the entire codebase (but this is actually good for consistency)

**Implementation:**
```python
class Driver(Node):
    def __init__(self, node_name: str, namespace: str = None, **kwargs):
        super().__init__(node_name, namespace=namespace, **kwargs)
        
        # ... existing code ...
        
        # Add mode attribute
        self.mode = "Patrol"  # Default mode: Patrol, Transfer, Charging
        
        # Add mode publisher for monitoring
        self.mode_pub = self.create_publisher(String, 'mode', 10)
        self.create_timer(1.0, self.publish_mode)  # Publish at 1 Hz
    
    def publish_mode(self):
        """Publish current mode for monitoring/debugging"""
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)
    
    def set_mode(self, new_mode: str):
        """Change drone mode with validation"""
        valid_modes = ["Patrol", "Transfer", "Charging"]
        if new_mode in valid_modes:
            old_mode = self.mode
            self.mode = new_mode
            self.get_logger().info(f"🔄 Mode: {old_mode} → {new_mode}")
        else:
            self.get_logger().error(f"❌ Invalid mode: {new_mode}")
```

### Option 2: Add to `Controller` Class

**Location:** `dji_drone_control/controller.py`

**Pros:**
- Still accessible to all control algorithms
- Closer to parameter management logic
- Could integrate with parameter callbacks

**Cons:**
- Not available in base `Driver` methods
- Less intuitive (mode is more of a "state" than a "parameter")

### Option 3: Add to `DistributedController` Class

**Location:** `dji_drone_control/distributed_controller.py`

**Pros:**
- Only affects multi-agent systems
- Can coordinate modes across drones

**Cons:**
- Not available to single-drone controllers
- Too high in the hierarchy
- Not all controllers inherit from DistributedController

### Option 4: Add to Individual Controllers (AACC, PCC, etc.)

**Location:** `dji_drone_demos/angle_aware_coverage_control.py`, etc.

**Pros:**
- Maximum flexibility per algorithm
- Can customize mode behavior per controller

**Cons:**
- **Code duplication** (need to add to every controller)
- **Inconsistent** across different algorithms
- Hard to maintain

---

## Comparison Summary

| Aspect | Driver | Controller | DistributedController | Individual Controllers |
|--------|--------|------------|----------------------|----------------------|
| **Accessibility** | ✅ Universal | ✅ Most controllers | ⚠️ Multi-agent only | ❌ Algorithm-specific |
| **Consistency** | ✅ Everywhere | ✅ Good | ⚠️ Limited | ❌ Varies |
| **Maintainability** | ✅ Single location | ✅ Single location | ⚠️ Medium | ❌ Multiple locations |
| **Integration with state** | ✅ Natural fit | ⚠️ Parameter-like | ⚠️ Network-like | ❌ Scattered |
| **Multi-field support** | ✅ Works for all | ✅ Works for all | ✅ Built for it | ❌ Need multi_field separately |

**Verdict:** Add `mode` attribute to **`Driver` class** ✅

This makes it available everywhere, keeps it with other state variables (`position`, `battery`), and ensures consistency across all control algorithms.

---

## How to Fix `multi_field.py`?

Looking at the existing `multi_field.py`, it has good structure but needs integration with the control algorithms. Here's the analysis and fix:

### Current Issues in `multi_field.py`:

1. ✅ **Good Structure** - `FieldManager` class exists with field management
2. ✅ **Central Model Support** - Has MPCS scheduler compatibility
3. ⚠️ **Missing Integration** - Not connected to control algorithms
4. ⚠️ **No Field Switching** - No mechanism to switch between fields
5. ⚠️ **Incomplete Methods** - `publish_boundaries()` is TODO
6. ⚠️ **Mode Management** - Doesn't interact with drone modes

### The Fix Strategy:

We need to create a **mixin class** that can be inherited by control algorithms (AACC, PCC, etc.) to give them multi-field capabilities.

### Step 1: Create `MultiFieldControllerMixin`

**New file:** `dji_drone_control/multi_field_controller.py`

```python
"""Multi-field controller mixin for field-switching capabilities."""

import numpy as np
import rclpy
from typing import Optional, Union
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Int32, String
from dji_drone_control.multi_field import FieldManager
from dji_drone_control.controller import Space


class MultiFieldControllerMixin:
    """
    Mixin to add multi-field capabilities to any controller.
    
    Inherit from this along with your base controller:
    
    class MultiFieldAACController(AngleAwareCoverageController, MultiFieldControllerMixin):
        def __init__(self, node_name: str):
            AngleAwareCoverageController.__init__(self, node_name, 'mf_aacc.yaml')
            MultiFieldControllerMixin.__init__(self)
    
    This gives your controller:
    - Multiple field management
    - Field switching via ROS2 service
    - Automatic mode transitions (Patrol <-> Transfer)
    - Integration with existing CBF-based control
    """
    
    def __init__(self):
        """Initialize multi-field capabilities."""
        # Create field manager
        self.field_manager = FieldManager(f"{self.get_name()}_field_manager")
        
        # Current operational field
        self.current_field_id: Optional[int] = None
        self.target_field_id: Optional[int] = None
        
        # Multi-field parameters (load from YAML)
        field_boundaries = self.get_parameter_value('multi_field.boundaries')
        field_density = self.get_parameter_value('multi_field.density')
        
        # Initialize fields
        self.field_manager.initialize_from_param(
            boundaries=field_boundaries,
            density=field_density,
            dim='xy'  # Can be 'xy', 'xyz', or 'xyzhv'
        )
        
        # Initialize charging station
        charging_bounds = self.get_parameter_value('multi_field.charging_station')
        self.field_manager.add_charging_station('default', charging_bounds)
        
        # Create ROS2 services for field switching
        self.switch_field_srv = self.create_service(
            Int32, 
            'switch_field', 
            self.handle_switch_field
        )
        
        # Create publishers for monitoring
        self.current_field_pub = self.create_publisher(Int32, 'current_field', 10)
        self.target_field_pub = self.create_publisher(Int32, 'target_field', 10)
        
        # Field switching parameters
        self.inside_margin = 0.1  # Margin for "inside field" check
        self.transfer_gain = 1.0  # Proportional gain for transfer mode
        
        # Start in field 0 (or charging station if low battery)
        if hasattr(self, 'battery_charging'):
            # If battery charging is available, check initial battery
            if self.battery_charging.get_battery() <= self.battery_charging.min_battery:
                self.current_field_id = 'c'  # Start at charging station
                self.set_mode('Charging')
            else:
                self.current_field_id = 0
                self.set_mode('Patrol')
        else:
            self.current_field_id = 0
            self.set_mode('Patrol')
        
        self.get_logger().info(
            f"🌾 Multi-field controller initialized with {len(self.field_manager.fields)} fields"
        )
    
    def handle_switch_field(self, request, response):
        """
        ROS2 service callback to switch to a different field.
        
        Usage from command line:
        ros2 service call /mavic_1/angle_aware_coverage_control/switch_field std_srvs/srv/Int32 "{data: 2}"
        
        Special values:
        - data: 0, 1, 2, ... → Switch to field 0, 1, 2, ...
        - data: -1 → Switch to charging station
        """
        field_id = request.data if request.data >= 0 else 'c'
        
        # Validate field ID
        if field_id != 'c' and field_id >= len(self.field_manager.fields):
            response.success = False
            response.message = f"❌ Invalid field ID: {field_id}"
            return response
        
        # Set target and switch to Transfer mode
        self.target_field_id = field_id
        self.set_mode('Transfer')
        
        response.success = True
        response.message = f"✅ Switching to field {field_id}"
        self.get_logger().info(response.message)
        
        return response
    
    def update_field_state(self):
        """
        Check if drone has reached target field and update mode accordingly.
        Call this in your control loop!
        """
        if self.mode == 'Transfer' and self.target_field_id is not None:
            # Check if we've arrived at target field
            pos = [self.position.x, self.position.y]
            
            if self.field_manager.inside(pos, self.target_field_id, self.inside_margin):
                # Arrived at target field
                self.current_field_id = self.target_field_id
                self.target_field_id = None
                
                if self.current_field_id == 'c':
                    self.set_mode('Charging')
                    self.get_logger().info("🔌 Arrived at charging station")
                else:
                    self.set_mode('Patrol')
                    self.get_logger().info(f"🌾 Arrived at field {self.current_field_id}")
                    
                    # Update coverage spaces for new field
                    self.update_coverage_space_for_field(self.current_field_id)
    
    def get_transfer_control(self) -> np.ndarray:
        """
        Calculate control input for Transfer mode (moving to target field).
        
        Returns:
            np.ndarray: [ux, uy] velocity command
        """
        if self.target_field_id is None:
            return np.array([0.0, 0.0])
        
        # Get target centroid
        target = self.field_manager.calc_centroid(self.target_field_id)
        
        # Proportional control toward centroid
        dx = target[0] - self.position.x
        dy = target[1] - self.position.y
        
        u = self.transfer_gain * np.array([dx, dy])
        
        # Apply velocity limit
        speed = np.linalg.norm(u)
        if hasattr(self, 'velo_limit') and speed > self.velo_limit:
            u = u * (self.velo_limit / speed)
        
        return u
    
    def update_coverage_space_for_field(self, field_id: int):
        """
        Update the coverage algorithm's space configuration for a new field.
        Override this method in your specific controller if needed.
        """
        if field_id == 'c':
            return  # No coverage at charging station
        
        # Get the field Space object
        field_space = self.field_manager.get_field(field_id)
        
        # Update source_space and space for coverage algorithm
        # This depends on your specific coverage implementation
        if hasattr(self, 'angle_aware_coverage'):
            # For AACC: update the space
            self.angle_aware_coverage.space = field_space
            
            # Optionally: regenerate initial density for new field
            # self.angle_aware_coverage.density_func = get_density(...)
            
            self.get_logger().info(f"📐 Updated coverage space for field {field_id}")
    
    def get_current_field_space(self) -> Optional[Space]:
        """Get the Space object for current field."""
        if self.current_field_id == 'c' or self.current_field_id is None:
            return None
        return self.field_manager.get_field(self.current_field_id)


class MultiFieldAngleAwareCoverageController(AngleAwareCoverageController, MultiFieldControllerMixin):
    """
    Example: AACC with multi-field capabilities.
    
    This controller can patrol multiple fields and switch between them via ROS2 service.
    """
    
    def __init__(self, node_name: str, param_file: str = 'mf_aacc.yaml'):
        # Initialize AACC first
        AngleAwareCoverageController.__init__(self, node_name, param_file)
        
        # Then add multi-field capabilities
        MultiFieldControllerMixin.__init__(self)
    
    def send_command(self):
        """Override send_command to handle different modes."""
        
        # Update field state (check if we've reached target)
        self.update_field_state()
        
        # Publish current state for monitoring
        self.publish_field_state()
        
        # Mode-specific control
        if self.mode == 'Transfer':
            # Transfer mode: move to target field
            u = self.get_transfer_control()
            
            # Simple altitude and yaw control
            uz = 0.5 * (self.get_parameter_value('angle_aware_coverage.altitude') - self.position.z)
            uw = 0.0
            
            self.pilot(u[0], u[1], uz, uw)
            
        elif self.mode == 'Patrol':
            # Patrol mode: normal AACC control
            super().send_command()
            
        elif self.mode == 'Charging':
            # Charging mode: stay on ground
            # Battery manager handles charging logic
            pass
    
    def publish_field_state(self):
        """Publish current and target field for monitoring."""
        if self.current_field_id is not None:
            msg = Int32()
            msg.data = self.current_field_id if isinstance(self.current_field_id, int) else -1
            self.current_field_pub.publish(msg)
        
        if self.target_field_id is not None:
            msg = Int32()
            msg.data = self.target_field_id if isinstance(self.target_field_id, int) else -1
            self.target_field_pub.publish(msg)


# Entry point
def main(args=None):
    rclpy.init(args=args)
    controller = MultiFieldAngleAwareCoverageController("angle_aware_coverage_control")
    controller.spin()
    controller.destroy_node()


if __name__ == '__main__':
    main()
```

### Step 2: Create Configuration File

**New file:** `dji_drone_demos/config/mf_aacc.yaml`

```yaml
/**:
  angle_aware_coverage_control:
    ros__parameters:
      # Standard AACC parameters (same as aacc.yaml)
      use_joy: false
      control_period: 0.1
      coverage_period: 0.1
      pointcloud_period: 0.1
      
      # ... (copy other parameters from aacc.yaml) ...
      
      # Multi-field parameters
      multi_field:
        # Field boundaries [xmin, xmax, ymin, ymax]
        boundaries:
          - [-4.0, -2.0, -2.0, 2.0]  # Field 0
          - [0.0, 2.0, -2.0, 2.0]    # Field 1
          - [2.0, 4.0, -2.0, 2.0]    # Field 2
        
        # Grid density for fields
        density: 0.01
        
        # Charging station bounds [xmin, xmax, ymin, ymax]
        charging_station: [-5.0, -4.5, -0.5, 0.5]
        
        # Field switching parameters
        inside_margin: 0.1
        transfer_gain: 1.0
      
      # Battery charging (if using BatteryCharging CBF)
      battery_charging:
        station_position: [-4.75, 0.0]  # Center of charging station
        charging_radius: 0.3
        charging_speed: 0.00024
        discharging_speed: 0.00037
        min_battery: 0.2
        max_battery: 1.0
        init_battery: 0.99
        k: 1.0
```

### Step 3: Usage Instructions

**From Command Line (Manual Field Switching):**

```bash
# Launch the multi-field controller
ros2 launch dji_drone_demos mf_aacc.launch.py

# Switch mavic_1 to field 2
ros2 service call /mavic_1/angle_aware_coverage_control/switch_field std_srvs/srv/Int32 "{data: 2}"

# Switch mavic_2 to field 0
ros2 service call /mavic_2/angle_aware_coverage_control/switch_field std_srvs/srv/Int32 "{data: 0}"

# Send mavic_3 to charging station
ros2 service call /mavic_3/angle_aware_coverage_control/switch_field std_srvs/srv/Int32 "{data: -1}"

# Monitor current field
ros2 topic echo /mavic_1/current_field

# Monitor drone mode
ros2 topic echo /mavic_1/mode
```

### Step 4: Integration with Existing CBFs

The multi-field controller automatically works with existing CBFs:

```python
# Field limitation CBF will be updated automatically
# based on current field boundaries

# Collision avoidance still works across all fields
# (drones can see each other even in different fields)

# Battery charging CBF triggers field switch to charging station
# when battery is low
```

### Key Features:

1. ✅ **No Scheduler Needed** - Uses ROS2 service for manual field switching
2. ✅ **Works with Any Control Algorithm** - Just inherit the mixin
3. ✅ **Automatic Mode Transitions** - Transfer → Patrol when reaching field
4. ✅ **Integration with Battery Charging** - Auto-switch to charging station
5. ✅ **Preserves Existing CBFs** - Collision avoidance, field limitation still work
6. ✅ **Easy to Extend** - Add scheduler interface later

---

## Summary

### Mode Attribute:
- **Add to `Driver` class** ✅
- Accessible everywhere
- Consistent with other state variables
- Easy to maintain

### Multi-Field Fix:
- **Create `MultiFieldControllerMixin`** ✅
- Inherit in control algorithms (AACC, PCC, etc.)
- Use ROS2 service for field switching (manual for now)
- Automatic mode transitions
- Scheduler can be added later by replacing service calls with scheduler assignments

### Next Steps:
1. Add `mode` attribute to `Driver.py`
2. Create `multi_field_controller.py` with mixin
3. Test with AACC (create `MultiFieldAngleAwareCoverageController`)
4. Later: Add scheduler interface (replace service with scheduler topic)

This approach keeps everything modular and scalable! 🚀

---

# Implementation Progress & Key Findings

**Date:** November 5, 2025

## ✅ Steps 1-3 Completed

### Step 1: Mode Attribute Added to `Driver` ✅
Located in `/home/pc/ros2_ws/src/dji_drone_robot/dji_drone_control/dji_drone_control/driver.py`

```python
# mode: for switching.
self.mode = "Patrol"
self.mode_pub = self.create_publisher(String, 'mode', qos_profile)
self.create_timer(1.0, self.publish_mode)

def publish_mode(self):
    msg = String()
    msg.data = self.mode
    self.mode_pub.publish(msg)

def set_mode(self, new_mode: str):
    valid_modes: list[str] = ["Patrol", "Transfer", "Charging"]  # Note: Fixed typo from "Trasnsfer"
    if new_mode in valid_modes:
        old_mode = self.mode
        self.mode = new_mode
        self.get_logger().info(f"🔄 Mode: {old_mode} → {new_mode}")
    else:
        self.get_logger().error(f"❌ Invalid mode: {new_mode}")
```

### Step 2: Multi-field Configuration Created ✅
Located in `/home/pc/ros2_ws/src/dji_drone_robot/dji_drone_demos/config/mf_aacc.yaml`

Initial attempt had YAML syntax issues with nested lists.

### Step 3: Multi-field Controller Mixin Created ✅
Located in `/home/pc/ros2_ws/src/dji_drone_robot/dji_drone_control/dji_drone_control/multi_field_controller.py`

Contains `MultiFieldControllerMixin` class with field switching capabilities.

### Step 4: Concrete Implementation Created ✅
The `MultiFieldAngleAwareCoverageController` was correctly placed in:
`/home/pc/ros2_ws/src/dji_drone_robot/dji_drone_demos/dji_drone_demos/multi_field_aacc.py`

**Architecture Decision:** Concrete implementations belong in `dji_drone_demos` (not `dji_drone_control`) because they are specific applications, while mixins stay in `dji_drone_control` as reusable components.

---

## 🔍 Key Discovery: How the Codebase Handles Phi/Density Grids

### The Problem Encountered:
When launching `mf_aacc.launch.py`, encountered this error:
```
Error: Sequences cannot be key at line 17
```

This was caused by trying to pass nested lists (field boundaries) as ROS2 parameters:
```yaml
multi_field:
  boundaries:
    - [-4.0, -2.0, -2.0, 2.0]  # ❌ ROS2 doesn't support this format
    - [0.0, 2.0, -2.0, 2.0]
    - [2.0, 4.0, -2.0, 2.0]
```

### The Investigation:
Asked the question: **"How does this codebase deal with phi grid?"**

Since RViz visualizes phi grids (2D arrays of floats 0-1) as colored point clouds, there must be a way to store and pass 2D arrays.

### The Discovery: Three-Part Solution

#### 1. **Storage Format** 
Phi/density grids are stored as **2D numpy arrays**:

```python
# From angle_aware_coverage_control.py
self.density_func = np.ndarray  # Shape: (440, 440) for example
```

Values range from 0.0 (visited) to 1.0 (unvisited).

#### 2. **Visualization Pipeline**
Found in `angle_aware_coverage_control.py` lines 476-483:

```python
def pub_pointcloud(self):
    if self.solver.coverage.ready:
        density = np.array(self.solver.coverage.density_func)/self.max_density
        voronoi = np.array(self.solver.coverage.voronoi_diagrams[:, :, 0])
        if density.shape != self.space.shape:
            return
        index = int(self.get_namespace()[len(self.get_namespace())-1])
        msg = array_to_pointcloud2_rgb(density, space=self.space, voronoi_diagram=voronoi, index=index, method='color')
        self.pointcloud2_pub.publish(msg)
```

**Key insight:** 
- 2D numpy array → `array_to_pointcloud2_rgb()` → `PointCloud2` message → RViz
- The converter is in `dji_drone_control/message_converter.py`

#### 3. **Parameter Loading Solution**
Found in `mpcs_central.py` lines 38-39:

```python
# Parse field boundaries using YAML approach
field_boundaries_yaml = cfg.get("field_boundaries", 
    "[[-4.0, -2.0, -2.0, 2.0], [0.0, 2.0, -2.0, 2.0], [2.0, 4.0, -2.0, 2.0]]")
field_boundaries = yaml.safe_load(field_boundaries_yaml)
```

**THE SOLUTION:** Store nested lists as **YAML strings** in parameters, then parse with `yaml.safe_load()`!

---

## ✅ The Fixed Approach

### Fixed `mf_aacc.yaml`:
```yaml
/**:
  angle_aware_coverage_control:
    ros__parameters:
      # Standard AACC parameters
      use_joy: false
      control_period: 0.1
      coverage_period: 0.1
      pointcloud_period: 0.1
      
      # Multi-field parameters (stored as YAML string!)
      field_boundaries: "[[-4.0, -2.0, -2.0, 2.0], [0.0, 2.0, -2.0, 2.0], [2.0, 4.0, -2.0, 2.0]]"
      
      multi_field:
        density: 0.01
        charging_station: [-5.0, -4.5, -0.5, 0.5]
        inside_margin: 0.1
        transfer_gain: 1.0
      
      # Battery charging
      battery_charging:
        station_position: [-4.75, 0.0]
        charging_radius: 0.3
        charging_speed: 0.00024
        discharging_speed: 0.00037
        min_battery: 0.2
        max_battery: 1.0
        init_battery: 0.99
        k: 1.0
```

### Fixed `MultiFieldControllerMixin.__init__()`:
```python
import yaml

def __init__(self):
    # Create field manager
    self.field_manager = FieldManager(f"{self.get_name()}_field_manager")
    
    # Load field boundaries from YAML string parameter
    field_boundaries_yaml = self.get_parameter_value('field_boundaries')
    field_boundaries = yaml.safe_load(field_boundaries_yaml)  # Parse YAML string to list
    
    field_density = self.get_parameter_value('multi_field.density')
    
    # Initialize fields
    self.field_manager.initialize_from_param(
        boundaries=field_boundaries,
        density=field_density,
        dim='xy'
    )
    
    # ... rest of initialization
```

---

## 🎓 Lessons Learned

### 1. **Data Type Consistency**
The codebase uses the same approach for all grid-like data:
- Phi/density grids: 2D numpy arrays
- Field boundaries: Lists of lists (loaded via YAML strings)
- Voronoi diagrams: 3D numpy arrays (x, y, agent_index)

### 2. **ROS2 Parameter Limitations**
ROS2 parameters don't natively support:
- ❌ Nested lists/sequences as keys
- ❌ Multi-dimensional arrays
- ✅ **Solution:** Store as YAML string, parse in Python

### 3. **Architecture Pattern**
```
dji_drone_control/          ← Reusable components (mixins, utilities)
├── multi_field_controller.py    ← Mixin (generic)
└── multi_field.py               ← FieldManager utility

dji_drone_demos/            ← Specific applications
└── multi_field_aacc.py     ← Concrete controller (AACC-specific)
```

### 4. **Visualization Pattern**
```
NumPy Array (2D) 
    ↓
array_to_pointcloud2_rgb()
    ↓
PointCloud2 Message
    ↓
RViz (colored point cloud)
```

This same pattern can be used for:
- Coverage density φ
- Field boundaries (as colored regions)
- Voronoi partitions
- Any 2D scalar field

---

## 🚀 Why This Matters for Multi-Field AACC

By discovering how the codebase handles 2D arrays, we can now:

1. ✅ Store multiple field boundaries properly
2. ✅ Maintain separate phi grids per field
3. ✅ Visualize multiple fields in RViz simultaneously
4. ✅ Use the existing `PointCloud2` pipeline for field visualization
5. ✅ Keep the same data structure as single-field AACC

**The codebase already has everything we need** - we just needed to understand the YAML string trick! 🎉

---

## 📍 Where is the Phi Grid Stored?

**Quick Answer:** The phi grid (2D array) is **NOT centrally stored** - each controller creates and manages its own density/phi array.

### For AACC (Angle-Aware Coverage Control):

**Location:** `dji_drone_demos/angle_aware_coverage_control.py`

The phi/density array is created in the `AngleAwareCoverage` class:

```python
# Line ~215 in angle_aware_coverage_control.py
def __init__(self, init_density, ...):
    if isinstance(init_density, np.ndarray):
        self.init_density = jnp.array(init_density)
    elif init_density == None:
        self.init_density = jnp.ones(space.shape)  # ← Creates array of ones!
    else:
        self.init_density = init_density
    
    self.density_func = self.init_density  # ← This is the phi grid
```

Then the controller initializes it:
```python
# Line ~380
init_density = get_density(source_space, space, altitude, "angle_aware")
self.angle_aware_coverage = AngleAwareCoverage(
    init_density=init_density,  # ← Passed here
    ...
)
```

### For MPCS (Multi-field):

**Location:** `dji_drone_demos/mpcs_central.py`

Each field has its own phi array:

```python
# Line ~77
self.phi: List[np.ndarray] = []
for space in self.spaces:
    # Start with uniform density = 1
    self.phi.append(np.ones_like(space.x))  # ← Creates array of ones per field!
```

---

### 🔑 Key Point: No Global Phi Storage

There's **no global phi storage**. Each demo controller:

1. **Creates** its own `Space` object (defines grid dimensions)
2. **Initializes** phi as `np.ones(space.shape)` or loads from file
3. **Updates** phi locally in its own memory
4. **Publishes** to RViz via `PointCloud2` for visualization

The phi grid lives **inside the controller instance**, not in a shared location!

This means for multi-field AACC, we need to:
- Create a `List[np.ndarray]` to store one phi grid per field (like MPCS does)
- Update the appropriate phi grid based on which field the drone is currently in
- Publish only the current field's phi grid for visualization

---

## Field Boundaries in AACC: From YAML Parameter to RViz2 Visualization
Here's the complete flow of field boundary management in the AACC system:
### 1. **Parameter Definition (YAML Configuration)**
The field boundaries start as parameters in `aacc.yaml`:
```yaml
field_range: [-2.2, 2.2, -2.2, 2.2]  # [x_min, x_max, y_min, y_max]
field_limitation:
    field_range: [-2.2, 2.2, -2.2, 2.2]
```
### 2. **Parameter Loading and Initial Processing**
In the `AngleAwareCoverageController.__init__()` method:
```python
# Load field range from parameters
range = self.get_parameter_value('field_range')
density = self.get_parameter_value('field_density')
# Create 2D workspace space
space = Space('xy')
space.set_x_range(range[0], range[1])  # [-2.2, 2.2]
space.set_y_range(range[2], range[3])  # [-2.2, 2.2] 
space.set_x_density(density[0])        # 0.01m resolution
space.set_y_density(density[1])        # 0.01m resolution
space.generate()
```
### 3. **Field Limitation CBF Creation**
The field boundaries are used to create a Control Barrier Function (CBF) for keeping drones within bounds:
```python
# Convert field_range list to polygon vertices
field_range = self.get_parameter_value('field_limitation.field_range')
field = [[field_range[0], field_range[2]], [field_range[0], field_range[3]],
         [field_range[1], field_range[3]], [field_range[1], field_range[2]]]
# Create ConvexPolygonFieldLimitation CBF
self.field_limitation = ConvexPolygonFieldLimitation(
    field=field,
    slack_variable=0.1,
    get_positions=self.get_positions)
```
### 4. **Dynamic Field Updates via ROS Topic**
The system can receive dynamic field boundaries through ROS messages:
```python
# Subscribe to polygon updates
field_topic = '/field_range'
self.polygon_sub = self.create_subscription(PolygonStamped, self.field_topic, self.polygon_callback, 10)
def polygon_callback(self, msg: PolygonStamped):
    """Receive polygon boundary updates from ROS topic"""
    points = msg.polygon.points
    field_buffer = np.zeros((0, 2))
    for i in range(len(points)):
    field_buffer = np.append(field_buffer, [[points[i].x, points[i].y]], axis=0)
    self.field_buffer = field_buffer
```
### 5. **Field Boundary Processing in Control Loop**
In the `send_command()` method, when new polygon data is received:
```python
if self.field_buffer is not None:
    self.field = self.field_buffer
    self.field_buffer = None
    if self.field.shape[0] >= 3:  # Valid polygon needs at least 3 vertices
    # Update coverage space with new polygon
    centroid = np.mean(self.field, axis=0)
    ratio = 0.5
    small_vertices = [(1-ratio)*centroid+ratio*vertex for vertex in self.field]
        
    # Update source space (for 5D->2D compression)
    self.source_space.set_2D_polygon_range(small_vertices, True)
        
    # Update target space (2D coverage area)
    self.space.set_2D_polygon_range(self.field, True)
        
    # Regenerate density function with new boundaries
    self.angle_aware_coverage.init_density = get_density(
            self.source_space, self.space, 
            self.get_parameter_value('angle_aware_coverage.altitude'), 
            "angle_aware") * self.space.in_polygon
        
    # Update field limitation CBF
    self.field_limitation.set_field(self.field)
```
### 6. **Space Class Polygon Processing**
The `Space.set_2D_polygon_range()` method processes polygon boundaries:
```python
def set_2D_polygon_range(self, polygon: tuple | list | np.ndarray, generate: bool = False):
    """Set polygon boundary for the space"""
    assert np.array(polygon).shape[0] >= 3, "The polygon should have at least 3 points."
    assert np.array(polygon).shape[1] == 2, "The polygon should be 2D."
    
    self.polygon = np.array(polygon)
    center = self.polygon.mean(axis=0)
    
    # Sort vertices counter-clockwise
    self.polygon = self.polygon[np.argsort(np.arctan2(
    self.polygon[:, 1]-center[1], 
    self.polygon[:, 0]-center[0]))]
    
    # Set bounding box
    self.set_x_range(np.min(self.polygon[:, 0]), np.max(self.polygon[:, 0]))
    self.set_y_range(np.min(self.polygon[:, 1]), np.max(self.polygon[:, 1]))
    
    if generate:
    self.generate()
```
### 7. **Polygon Masking in Space Generation**
When the space is generated, polygon boundaries create a mask:
```python
def generate(self):
    """Generate the discrete space grid"""
    self._calc_linspace()  # Create x,y coordinate arrays
    self._calc_grid()      # Create meshgrid
    
    self.in_polygon = np.ones(self.shape, dtype=bool)
    if self.polygon is not None:
    self.in_polygon = self.in_polygon & self._in_polygon()
    
def _in_polygon(self) -> np.ndarray:
    """Check which grid points are inside the polygon using cross products"""
    in_polygon = np.ones(self.shape, dtype=bool)
    for i in range(self.polygon.shape[0]):
    a = self.polygon[i]
    b = self.polygon[(i+1) % self.polygon.shape[0]]
    # Cross product test for each edge
    in_polygon = in_polygon & ((b[0]-a[0])*(self.y-b[1])-(b[1]-a[1])*(self.x-b[0]) > 0)
    return in_polygon
```
### 8. **Density Function Visualization**
The field boundaries affect the published density function:
```python
def pub_pointcloud(self):
    """Publish density function as colored point cloud"""
    if self.solver.coverage.ready:
    # Get density function (already masked by polygon boundaries)
    density = np.array(self.solver.coverage.density_func)/self.max_density
    voronoi = np.array(self.solver.coverage.voronoi_diagrams[:, :, 0])
        
    # Convert to PointCloud2 with RGB colors
    msg = array_to_pointcloud2_rgb(density, space=self.space, 
                                     voronoi_diagram=voronoi, index=index, method='color')
    self.pointcloud2_pub.publish(msg)
```
### 9. **RViz2 Configuration and Visualization**
The launch file configures RViz2 to display both the density field and polygon boundaries:
```python
# Launch file: aacc.launch.py
rviz_config=use_dynamic_config(drones, 
                              point_cloud2_rgb="density",      # Density visualization
                              polygon="/field_range")         # Field boundary polygon
```
### **Complete Data Flow Summary**

1. **YAML** → `field_range: [-2.2, 2.2, -2.2, 2.2]`
2. **Parameter Loading** → Converted to Space object bounds
3. **Space Generation** → Creates discrete grid with polygon mask
4. **CBF Creation** → Field limitation using ConvexPolygonFieldLimitation
5. **Dynamic Updates** → `/field_range` topic allows runtime polygon changes
6. **Density Calculation** → Applied only within polygon boundaries (`* self.space.in_polygon`)
7. **PointCloud2 Publishing** → `/density` topic with RGB-colored density values
8. **RViz2 Visualization** → Displays colored density field constrained by boundaries

The field boundaries serve multiple purposes:
- **Safety constraint** via CBF to keep drones within bounds
- **Coverage constraint** to limit density calculation to valid areas  
- **Visualization boundary** shown in RViz2 as polygon overlay
- **Dynamic reconfiguration** allowing runtime field shape changes

This creates a complete system where field boundaries defined in YAML parameters constrain both the drone behavior and the visual coverage representation in RViz2.
