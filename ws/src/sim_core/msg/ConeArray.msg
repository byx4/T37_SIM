# Cone detections for one timestep
std_msgs/Header header  # stamp = time detections were computed; frame_id="base_link"
sim_msgs/Cone[] cones   # may be empty; max ~50
