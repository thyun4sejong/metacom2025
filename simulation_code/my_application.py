from omni.isaac.kit import SimulationApp

# Initialize Isaac Sim in headless mode
simulation_app = SimulationApp({"headless": True})

# Import required modules
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
import omni.isaac.core.utils.nucleus as nucleus

# Load the ROS 2 Navigation stage (adjust path if necessary)
stage_url = "omniverse://localhost/NVIDIA/Assets/Isaac/Robots/ROS2Navigation/navigation_example.usd"
open_stage(stage_url)

# Initialize world and step the simulation
world = World()
for _ in range(500):  # Run for a certain number of steps
    world.step(render=False)

print("ROS2 Navigation example completed!")

# Cleanup
simulation_app.close()
