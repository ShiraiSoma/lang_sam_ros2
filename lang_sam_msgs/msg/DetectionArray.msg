# index_mask: mono16画像。0=背景、値 i+1 が detections[i] に対応するインデックスマスク
std_msgs/Header header
Detection[] detections
sensor_msgs/Image index_mask
