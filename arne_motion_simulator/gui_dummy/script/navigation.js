// Manage the connection to ROS using the specified port.
// On the Linux side, an instance of rosbridge_server will listen on the port and
// forward the incoming messages as ROS topics into the ROS pipeline.

var ros = new ROSLIB.Ros({
url : 'ws://localhost:8001'
});

ros.on('connection', function() {
console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
console.log('Connection to websocket server closed.');
});

// Setup interfaces to the two ROS topics for control.
// The 6D pose of the gripper is controlled with a twist and grasping is
// controlled with an opening speed (negative closes the gripper).

var motion_command = new ROSLIB.Topic({
        ros : ros,
        name : '/motion_control_input',
        messageType : 'geometry_msgs/Twist'
});

var gripper_command = new ROSLIB.Topic({
        ros : ros,
        name : '/gripper_control_input',
        messageType : 'std_msgs/Float64'
});

// Setup constant messages that get published on each iteration
var motion_msg = new ROSLIB.Message({
        linear : {
                x : 0.0,
                y : 0.0,
                z : 0.0
        },
        angular : {
                x : 0.0,
                y : 0.0,
                z : 0.0
        }
});

var gripper_msg = new ROSLIB.Message({
        data : 0.0
});

// Helper function to implement a rate-based publishing
function sleep(milliseconds) {
   return new Promise(resolve => setTimeout(resolve, milliseconds));
};

// Publish cyclically to create a continuous motion in the simulator.
async function send() {
        while (true)
        {
                await sleep(20);
                var t = new Date();
                t = t.getTime() / 1000;

                // Move in somewhat spherical motion
                motion_msg.linear.x = 10 * Math.sin(0.1 * t);
                motion_msg.linear.y = 10 * Math.cos(0.2 * t);
                motion_msg.linear.z = 10 * Math.sin(0.5 * t);
                motion_msg.angular.x = 2 * Math.sin(0.1 * t);
                motion_msg.angular.y = 2 * Math.cos(0.2 * t);
                motion_msg.angular.z = 2 * Math.sin(0.5 * t);
                motion_command.publish(motion_msg);

                // Periodically open and close the gripper
                gripper_msg.data = 5 * Math.sin(t);
                gripper_command.publish(gripper_msg);

                console.log("sent");
        }
}

send();
