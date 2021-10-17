# cartpole_description
ROS2 package containing models, simulations and visualizations for the cartpole robot<br />

<img src="https://user-images.githubusercontent.com/59701038/136147195-ec06f238-6a73-469b-a54e-6566bb9d3a1a.png" width="420">

# Package Organization
```
cartpole_description - The package
    |
    |-cartpole_description - Python package nested inside ROS2 package
    |
    |-launch - Launch files used to run package Nodes
    |
    |-meshes - stl meshes of robot used for urdf model
    | 
    |-resource - Ignore this
    |
    |-robot
    |   |
    |   |-gazebo_models/cartpole - cartpole model for Gazebo simulations
    |   |
    |   |-onshape_converts - files generated from onshape_to_robot converter
    |   |
    |   |-urdf - cartpole urdf files for RViz
    |
    |-rviz - RViz configuration files
    |
    |-worlds - Gazebo simulation worlds
```

# Usage
After building and sourcing workspace `dev_ws`, Try running some examples:

## Gazebo Simulation:
`ros2 launch cartpole_description cartpole_world.launch.py`

## RViz Visualization with dummy data:
1. Launch Nodes to publish joint data:  
`ros2 launch cartpole_description cartpole_rviz.launch.py`
2. In another terminal, run RViz to visualize the robot:  
`rviz2 -d ~/dev_ws/install/cartpole_description/share/cartpole_description/rviz/cartpole.rviz`

# Rebuilding the Robot Using Onshape-to-Robot

The cartpole robot designs can be found on Onshape: [link](https://cad.onshape.com/documents/62fb4288b389f749f09c2484/w/3734bab7afe5eddf677d00fd/e/02dadcf645139f052b5a51c4?renderMode=0&uiState=615d305416b0f06691e0c0a6)

New iterations of the robot from Onshape must be converted to urdf and sdf file formats using [onshape-to-robot](https://onshape-to-robot.readthedocs.io/en/latest/index.html) to enable visualization and simulation:

```
cd robot/onshape_converts
cd sdf      # or urdf
onshape-to-robot config.json
```

After the conversion, move the files to their respective directories:
* urdf files goes to `urdf/`
* stl files goes to `meshes/`. Gazebo models will also need their own copy of stls files in their own `robot/gazebo_models/<model_name>/meshes/` folder
* copy sdf files into the `robot/gazebo_models/<model_name>` folder

The urdf files should work right away for RViz. But the sdf files require substantial tweaking for it to work in Gazebo. Consult the existing sdf file for guidance. After modifying the sdf file, copy everything in `<model>...</model>` into the world file at `worlds/cartpole_sim.world`
