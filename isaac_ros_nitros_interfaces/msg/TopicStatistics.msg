# Statistics data for a ROS Node

string node_name # Name of the ROS Node associated with the topic
string node_namespace # Namspace of the ROS topic
string topic_name # Name of the topic
bool is_subscriber # true if subscriber and false if publisher
float32 frame_rate # Windowed mean frame rate
# jitter is the difference between the time between msgs(dt)
# calculated from fps specified in NITROS statistics ROS param
# and measured using system clock
# Units is microseconds
# mean absolute jitter
int32 mean_abs_jitter
# maximum absolute jitter
int32 max_abs_jitter
