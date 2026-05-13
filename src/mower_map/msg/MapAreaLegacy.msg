# The message in this exact shape has been persisted in legacy map.bag files.
# It can't be changed without breaking conversion to JSON, so we save this copy for reading the .bag
# and use an extended MapArea.msg for internal communication.
string name
geometry_msgs/Polygon area
geometry_msgs/Polygon[] obstacles
