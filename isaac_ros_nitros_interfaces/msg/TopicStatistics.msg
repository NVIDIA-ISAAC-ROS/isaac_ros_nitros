# Statistics data for a ROS Node

string node_name # Name of the ROS Node associated with the topic
string node_namespace # Namspace of the ROS topic
string topic_name # Name of the topic
bool is_subscriber # true if subscriber and false if publisher
float32 frame_rate_node # Windowed mean frame rate calculated using the node clock
float32 frame_rate_msg # Windowed mean frame rate calculated using the message timestamp
# jitter is the difference between the time between msgs(dt)
# calculated from fps specified in NITROS statistics ROS param
# and measured using node clock or the message timestamps
# Units is microseconds
# mean absolute jitter calculated using the node clock
int32 mean_abs_jitter_node
# mean absolute jitter calculated using the message timestamp
int32 mean_abs_jitter_msg
# maximum absolute jitter calculated using the node clock
int32 max_abs_jitter_node
# maximum absolute jitter calculated using the message timestamp
int32 max_abs_jitter_msg
# number of messages outside the jitter tolerance using the node clock
uint64 num_jitter_outliers_node
# number of messages outside the jitter tolerance using the message timestamp
uint64 num_jitter_outliers_msg
# number of messages with non-increasing msg times
uint64 num_non_increasing_msg
