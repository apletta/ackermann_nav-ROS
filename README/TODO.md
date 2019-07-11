# TODO
### List of additions/changes to be made to current package. This is so developers can see what still needs to be worked on and users know what functionality is not included yet.
- node algorithms
  - you may want to change variables reading in subscribed topic data to names more specific/intuitive for your algorithm
- allow ackermann_controller to change topics from parameter file setting
- refactor Object to Obstacle (ex. ObjectMsg and ObjectStateMsg to ObstacleMsg and ObstacleStateMsg)
  - .msg files
  - include statements
  - message types within nodes
- read parameters into nodes
  - put parameters into namespaces for their node so they don't get overwritten in case other param files use the same parameter names
- change output of local planner from velocity to acceleration to simplify control commands (want controller to only apply changes, not calculate as it has no state information) !ackermann_nav_structure shows this but NOT in code yet!
- add heading field to goal message type !ackermann_nav_structure shows this but NOT in code yet!
