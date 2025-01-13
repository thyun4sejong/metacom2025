import carb
import omni
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import find_nucleus_server
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.ros2_bridge import Ros2Bridge
from omni.isaac.navigation import NavigationInterface
from omni.isaac.core.utils.prims import create_prim
import rospy
from geometry_msgs.msg import PoseStamped

# Initialize the Isaac Sim headless simulation app
simulation_app = SimulationApp({"headless": True})

# ROS 2 Node initialization
rospy.init_node("isaac_sim_navigation_example")

# Create a ROS 2 publisher to send goal poses
goal_publisher = rospy.Publisher("/goal_pose", PoseStamped, queue_size=10)

# Set up the simulation world
def setup_world():
    world = World(stage_units_in_meters=1.0)

    # Find the Nucleus server for accessing assets
    nucleus_server = find_nucleus_server()
    if not nucleus_server:
        carb.log_error("Failed to find Nucleus server. Make sure Nucleus is running.")
        simulation_app.close()
        exit()

    # Add the environment to the stage
    environment_path = nucleus_server + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    add_reference_to_stage(environment_path, "/World/Environment")

    # Add a robot (e.g., Carter) to the stage
    robot_path = nucleus_server + "/Isaac/Robots/Carter/carter.usd"
    add_reference_to_stage(robot_path, "/World/Robot")

    # Initialize the ROS 2 Bridge
    ros2_bridge = Ros2Bridge()

    # Initialize the navigation interface
    nav_interface = NavigationInterface(robot_path="/World/Robot")

    # Return the world and interfaces
    return world, ros2_bridge, nav_interface

# Simulation loop
def run_simulation(world, ros2_bridge, nav_interface):
    # Start the simulation
    simulation_app.update()
    world.reset()

    # Publish a goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 3.0
    goal_pose.pose.orientation.w = 1.0
    rospy.sleep(1.0)
    goal_publisher.publish(goal_pose)
    rospy.loginfo("Published goal pose to /goal_pose")

    # Run the simulation loop
    while simulation_app.is_running():
        simulation_app.update()
        world.step(render=True)
        ros2_bridge.update()
        nav_interface.update()

    # Close the simulation
    simulation_app.close()

if __name__ == "__main__":
    try:
        world, ros2_bridge, nav_interface = setup_world()
        run_simulation(world, ros2_bridge, nav_interface)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down simulation.")
        simulation_app.close()
