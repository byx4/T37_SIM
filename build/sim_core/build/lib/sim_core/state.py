class CarState:
    """Shared state for the simulated vehicle."""
    def __init__(self):
        # Initialize position, orientation (yaw), and velocity
        self.x = 0.0  # X position in global map frame (meters)
        self.y = 0.0  # Y position in global map frame (meters)
        self.yaw = 0.0  # Orientation (heading) in radians, 0 = facing along +X axis in map frame
        self.v = 0.0  # forward velocity in m/s

# Create a single global CarState instance (to be shared by simulation nodes)
car = CarState()