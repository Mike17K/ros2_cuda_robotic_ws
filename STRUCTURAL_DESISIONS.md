# Collision checking

because we want to use nvblox for environment representation, by operating it in occupancy mode we do not have the esdf information for the cumotion planning, so we operate it in static_tsdf mode. this results to be able to utilize cumotion instances for different robots but for the ompl pipelines planners the plans cannot see collisions because there is no integration with this data structure in the planning scene pipeline. We could use the octomap sensors for the moveit planning scene with the pointcloud topic from the cameras but there would be inconsistances between the 2 systems that handle the environment representation.

We result in having ompl planners only for small helper plans , without inspection of collisions ( will be used only for testing and rearly ) and cumotion with nvblox for the main planning and execution.

this is why the sensors_3d.yaml file is commented out in the bringup.launch.py file, because we do not want to use the octomap for the planning scene.
