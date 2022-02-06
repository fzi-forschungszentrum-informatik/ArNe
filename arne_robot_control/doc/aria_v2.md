# Tips and tricks for the Aria V2 robot
This is a collection of robot specific experiences and best practices.

## Proper joint configuration
Although nominally independent of its joint configuration (some joints are even
continuous), AriaV2 seems to require its `Joint_2` to be in a special range to
avoid the `ARM OVERLOADED` error.
When operating on a table top or similar, good ranges seem to be from `0.5` to
`2.0` radians. Being greater means that we steer the robot with a *flipped
shoulder*, which reproducible causes the overload error. So, make sure to
operate the robot on *the right* side of this joint range.


