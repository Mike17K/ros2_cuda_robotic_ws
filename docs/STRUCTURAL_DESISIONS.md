# Collision checking

because we want to use nvblox for environment representation, by operating it in occupancy mode we do not have the esdf information for the cumotion planning, so we operate it in static_tsdf mode. this results to be able to utilize cumotion instances for different robots but for the ompl pipelines planners the plans cannot see collisions because there is no integration with this data structure in the planning scene pipeline. We could use the octomap sensors for the moveit planning scene with the pointcloud topic from the cameras but there would be inconsistances between the 2 systems that handle the environment representation.

We result in having ompl planners only for small helper plans , without inspection of collisions ( will be used only for testing and rearly ) and cumotion with nvblox for the main planning and execution.

this is why the sensors_3d.yaml file is commented out in the bringup.launch.py file, because we do not want to use the octomap for the planning scene.

# main working workspace goes for the total operation inside the docker ws

because the rviz planning needs the cumotion moveit plugin, and also cumotion needs robot description files, instead of having duplicate packages for robots descriptions and moveit config its more optimal to be on the same docker workspace. now the catch if the gz sim for some reason does not work there i should be launching the workcell gz sim from the src and the rest of bringup seperate from the container to be spawned ( so not posible all in one launch but ok )

we will make the docker cumotion ws as the main working workspace, keeping same structure for more later different docker containers
