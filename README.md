First-Order Boustrophedon Navigator (Lawnmower pattern) using ROS2:
Name: Chakradhar Reddy
Student ID: 1236312313
1.	Introduction
This note describes how to tune a PD controller and optimize the boustrophedon (lawnmower) coverage pattern using ROS2. The intention was to make motion better by reducing cross-track error, smoothing out the motion, and covering the area well. Tuning was done right away with rqt_reconfigure, and the last parameters were picked based on full tests and performance checks.
2. Controller Tuning
2.1 Objective
The PD controller was adjusted to guarantee accurate motion control, decreasing cross-track error and allowing for fluid movement and good cornering.
2.2 Problems and Fixes
Problem 1: High Cross-Track Error During Initial Testing
When I first tested the controller, I noticed that the robot was veering off its intended path. The cross-track error was unacceptably high. This issue was primarily caused by the low proportional gains (Kp_linear and Kp_angular), which weren’t strong enough to make quick corrections to the robot’s position.
Solution:
To fix this, I gradually increased the Kp_linear to 10.0 and Kp_angular to 5.0. With these higher values, the robot was able to correct its path much more effectively. The result was that the robot started following the trajectory more accurately, and the cross-track error dropped significantly. This improvement in correction led to smoother, more controlled motion.
Problem 2: Motion Stability Issues with Default Settings
Motion was jerky and oscillatory with the original settings. After a short distance, the robot started to wobble and exhibit oscillations after moving smoothly at first. This issue was especially apparent when making abrupt turns.
Solution: I adjusted the derivative gains (Kd_linear and Kd_angular) and kept them at 0.1 to counteract the oscillations. This modification made sure the robot moved steadily and smoothly in both straight lines and turns by reducing oscillations.
Challenge 3: Having Trouble Getting a Smooth Corner
The robot responded slowly to turns because the default Kp_angular value was too low. This resulted in abrupt turns that lacked the desired smoothness.
By increasing Kp_angular to 5.0, I improved the robot's responsiveness during turns. This adjustment allowed for smoother and more controlled cornering, improving the overall trajectory following performance.
 2.3 Final Tuned Parameters
	PD Controller Parameters
•	self.Kp_linear = 10.0   # Proportional gain for linear velocity
•	self.Kd_linear = 0.1    # Derivative gain for linear velocity
•	self.Kp_angular = 5.0   # Proportional gain for angular velocity
•	self.Kd_angular = 0.1   # Derivative gain for angular velocity
2.3 Tuning Methodology
•	Initial Values: The controller started with Kp_linear = 1.0, Kd_linear = 0.1, Kp_angular = 1.0, and Kd_angular = 0.1.
•	Incremental Adjustments:
o	Increased Kp_linear to 10.0 for stronger corrections.
o	Increased Kp_angular to 5.0 for better turning response.
o	Maintained Kd_linear and Kd_angular at 0.1 to avoid oscillations.
•	Testing & Optimization:
Values were adjusted iteratively while watching cross-track error and motion stability.
2.4 Performance Improvements
•	Reduced Cross-Track Error: Better following the path.
•	Smoother Motion: Minimal oscillations, cutting down on jerky moves.
•	Improved Cornering Performance: More sensitivity in bends because of bigger Kp_angular
•	Final Average Cross-Track Error: 0.279, indicating significant improvement in path accuracy.
________________________________________
3. Boustrophedon (Lawnmower) Pattern Optimization
3.1 Objective
The boustrophedon style was improved to help coverage good and full while cutting down on overlap and gaps.
3.2 Final Tuned Parameters
	Lawnmower Pattern Parameters
•	self.spacing = 0.5  # Spacing between coverage lines
•	self.waypoints = self.generate_waypoints()
•	self.current_waypoint = 0
3.3 Tuning Methodology
•	Initial Spacing: Default 1.0 spacing caused coverage gaps.
•	Incremental Adjustments:
                Reduced spacing to 0.5; increased coverage.
                Analysed effectiveness of patterns with different values.
•	Testing & Optimization:
	Ensured minimal gaps while maintaining efficiency.
3.4 Performance Improvements
•	Higher Coverage Efficiency: Reduced spacing resulted in better coverage.
•	Pattern Completeness: All waypoints were reached with minimal redundancy.
________________________________________
4. Cross-Track Error Handling
To analyse controller performance, a cross-track error buffer was implemented:
# Cross-track error storage
	self.cross_track_errors = deque(maxlen=1000)  # Store last 1000 errors
•	This allowed real-time monitoring of deviations and validated tuning effectiveness.
•	The final computed average cross-track error was 0.279, indicating a well-tuned control system.
________________________________________
5. Conclusion
Tuning the PD controller and optimizing the lawnmower pattern in ROS2 resulted in:
•	Reduced cross-track error with improved motion control.Smooth transitions and enhanced cornering.
•	Improved transition smoothness and cornering.
•	Efficient area coverage with minimal gaps; final average cross-track error of 0.279 indicative of excellent path-following capabilities.
Much needed improvements to ensure a reliable and effective navigation system for autonomous coverage applications..
