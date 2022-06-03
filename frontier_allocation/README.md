# frontier_allocation

This package contain two nodes:

1. frontier_detector: this node uses two passes computer vision based connected-component labeling to detect all the frontiers on the current map
2. frontier_allocator: This node allows to select a frontier as a goal for the navigation stack, two allocation strategies are implemented: random frontier and nearest frontier

These codes are from "https://github.com/lxsang/ROS-packages". A small adaptation has been add to the frontier allocation node to avoid the two robot to move to the same frontier
