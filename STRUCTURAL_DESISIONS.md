# Collision checking

because we want to use nvblox for environment representation, by operating it in occupancy mode we do not have the esdf information for the cumotion planning, so we operate it in static_tsdf mode. this results to be able to utilize cumotion instances for different robots but for the ompl pipelines planners the plans cannot see collisions because there is no integration with this data structure in the planning scene pipeline. We could use the octomap sensors for the moveit planning scene with the pointcloud topic from the cameras but there would be inconsistances between the 2 systems that handle the environment representation.

We result in having ompl planners only for small helper plans , without inspection of collisions ( will be used only for testing and rearly ) and cumotion with nvblox for the main planning and execution.

this is why the sensors_3d.yaml file is commented out in the bringup.launch.py file, because we do not want to use the octomap for the planning scene.

# access to robots from isaac container

because the isaac-ros-cli mounts the folder docker/cumotion_ws in the container as we have specified
in order to have there the packages of robot description for the cumotion to access them we will create an symbolic link from the src/robots to the docker/cumotion_ws/src/robots
that will be persistant from git

update: this approach does not work in the container cause of bad context, we will use volumns for mapping the files to the container. Because the isaac ros cli is not supporting of custom volumns we have opend an issue in their github for maybe adding this functionality
