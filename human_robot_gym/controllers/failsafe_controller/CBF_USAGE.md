# CBF Safety Controller Usage Guide

This guide explains how to use the CBF (Control Barrier Function) safety controller as an alternative to the SARA shield in the human-robot-gym framework.

## Overview

The CBF safety controller implements Control Barrier Functions to ensure safe robot operation in human environments. Unlike the SARA shield which uses reachability analysis, the CBF controller uses optimization-based safety filtering to maintain safety constraints while minimizing deviation from desired control inputs.

## Key Features

- **Real-time Safety Filtering**: Uses quadratic programming to filter unsafe control inputs
- **Configurable Class K Functions**: Supports linear, quadratic, exponential, logarithmic, and tanh functions
- **Multiple Solver Options**: Supports both QPOASES (conic) and IPOPT (nonlinear) solvers
- **Robot Agnostic**: Works with different robot kinematic models (Panda, Schunk, etc.)
- **Distance-based Constraints**: Maintains minimum distance to human obstacles

## Installation

1. Ensure CasADi is installed in your environment:
```bash
pip install casadi
```

2. The CBF controller is automatically available when you import human-robot-gym:
```python
from human_robot_gym import CBFFailsafeController
```

## Basic Usage

### Importing the Controller

```python
from human_robot_gym.controllers.failsafe_controller.failsafe_controller.cbf_controller import CBFFailsafeController
```

### Controller Configuration

The CBF controller accepts the same basic parameters as the SARA shield controller, with additional CBF-specific options:

```python
controller_config = {
    'type': 'CBF_FAILSAFE',
    'robot_name': 'panda',
    'safety_type': 'CBF',
    'min_distance': 0.2,              # Minimum distance to obstacles (meters)
    'class_k_type': 'linear',         # Type of Class K function
    'class_k_scale': 1.0,             # Scaling factor for Class K function  
    'opti_solver': 'qpoases',         # Optimization solver ('qpoases' or 'ipopt')
    'control_sample_time': 0.004,     # Control loop sample time
    'kp': 50,                         # PD controller gains
    'damping_ratio': 1,
}
```

### Environment Integration

To use the CBF controller in an environment, you can specify it in the controller configuration:

```python
import human_robot_gym

# Create environment with CBF safety controller
env = human_robot_gym.make(
    'ReachHuman-v0',
    robots=['Panda'],
    controller_configs={
        'type': 'CBF_FAILSAFE',
        'safety_type': 'CBF',
        'min_distance': 0.2,
        'class_k_type': 'linear',
        'class_k_scale': 1.0,
        'opti_solver': 'qpoases'
    }
)
```

## Configuration Parameters

### Safety Parameters

- `min_distance` (float): Minimum allowed distance to human obstacles (default: 0.2m)
- `safety_type` (str): Must be set to "CBF" for CBF controller
- `class_k_type` (str): Type of Class K function - options:
  - `'linear'`: κ(x) = α·x (fastest, good for most cases)
  - `'quadratic'`: κ(x) = α·x²
  - `'exponential'`: κ(x) = α·exp(x)
  - `'logarithmic'`: κ(x) = α·log(x+1)
  - `'tanh'`: κ(x) = α·tanh(x)
- `class_k_scale` (float): Scaling factor α for Class K function (default: 1.0)

### Optimization Parameters

- `opti_solver` (str): Optimization solver to use:
  - `'qpoases'`: Conic solver, faster for simple constraints (recommended)
  - `'ipopt'`: Nonlinear solver, more robust for complex constraints
- `control_sample_time` (float): Sample time for control loop (default: 0.004s)

### Robot Parameters

- `robot_name` (str): Name of robot for kinematic model ('panda', 'schunk', etc.)
- `base_pos` (list): Robot base position [x, y, z]
- `base_orientation` (list): Robot base orientation quaternion [x, y, z, w]

## Human Measurement Interface

The CBF controller receives human position data through the same interface as the SARA shield:

```python
# In your environment step function
human_positions = get_human_joint_positions()  # Your human tracking function
controller.set_human_measurement(human_positions, sim_time)
```

The `human_measurement` should be a list of 3D positions representing human joints or key points.

## Safety Monitoring

Check if the CBF controller intervened:

```python
# After calling run_controller()
is_safe = controller.get_safety()  # True if no intervention, False if CBF intervened
intervention_occurred = not is_safe
```

## Performance Comparison

| Feature | SARA Shield | CBF Controller |
|---------|-------------|----------------|
| Safety Method | Reachability Analysis | Optimization-based |
| Computational Cost | High | Medium |
| Real-time Performance | Good | Excellent |
| Constraint Types | Complex geometric | Distance-based |
| Tuning Complexity | High | Low |
| Mathematical Foundation | Forward Reachable Sets | Control Barrier Functions |

## Troubleshooting

### Common Issues

1. **Optimization Solver Fails**
   - Try switching from 'qpoases' to 'ipopt'
   - Check that min_distance is reasonable (not too small)
   - Ensure human measurements are valid

2. **Controller Too Conservative**
   - Decrease class_k_scale
   - Try 'linear' instead of 'quadratic' or 'exponential'
   - Increase min_distance slightly

3. **Controller Not Intervening**
   - Check human measurement data is being provided
   - Verify min_distance is appropriate for your scenario
   - Increase class_k_scale

### Performance Optimization

- Use 'qpoases' solver for best performance
- Use 'linear' Class K function for fastest computation
- Minimize the number of CBF constraints when possible

## Example: Complete Setup

```python
import numpy as np
import human_robot_gym

# Environment configuration
env_config = {
    'robots': ['Panda'],
    'controller_configs': {
        'type': 'CBF_FAILSAFE',
        'safety_type': 'CBF',
        'robot_name': 'panda',
        'min_distance': 0.15,
        'class_k_type': 'linear',
        'class_k_scale': 2.0,
        'opti_solver': 'qpoases',
        'control_sample_time': 0.004,
        'kp': 100,
        'damping_ratio': 1.0,
    },
    'has_renderer': True,
    'has_offscreen_renderer': False,
    'control_freq': 250,
}

# Create environment
env = human_robot_gym.make('ReachHuman-v0', **env_config)

# Training/testing loop
obs = env.reset()
for step in range(1000):
    action = np.random.uniform(-1, 1, env.action_dim)  # Replace with your policy
    obs, reward, done, info = env.step(action)
    
    # Check if CBF intervened
    if hasattr(env, 'robots') and hasattr(env.robots[0].controller, 'get_safety'):
        is_safe = env.robots[0].controller.get_safety()
        if not is_safe:
            print(f"Step {step}: CBF safety intervention occurred")
    
    if done:
        obs = env.reset()

env.close()
```

This completes the basic usage guide for the CBF safety controller. The controller provides a powerful alternative to SARA shield with different performance characteristics and tuning options.