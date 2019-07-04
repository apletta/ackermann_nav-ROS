# TODO
### List of additions/changes to be made to current package. This is so developers and users can quickly see what still needs to be worked on.
- node algorithms
  - you may want to change variables reading in subscribed topic data to names more specific/intuitive for your algorithm
- allow ackermann_controller to change topics from parameter file setting
- refactor Object to Obstacle (ex. ObjectMsg and ObjectStateMsg to ObstacleMsg and ObstacleStateMsg)
  - .msg files
  - include statements
  - message types within nodes
- read parameters into nodes
  - put parameters into namespaces for their node so they don't get overwritten in case other param files use the same parameter names
