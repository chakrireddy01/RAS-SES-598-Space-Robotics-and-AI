# Cart-Pole Optimal Control Using ROS2

## Cart-Pole Optimal Control Assignment

### Using ROS2

**Name:** Chakradhar Reddy  
**Student ID:** 1236312313  

## Introduction
I adjusted and examined an LQR controller for a cart-pole system impacted by earthquakes as part of this project. Despite outside disturbances, the objective is to retain the cart inside its physical bounds while preserving the stability of the pendulum. I will learn how to manage dynamic systems under erratic influences by modeling seismic disturbances. Space robotics, lunar landings, and orbital debris cleaning missions are just a few practical uses for the abilities learned here, such as optimum control and disturbance rejection.

## 1.1 LQR Tuning Approach Objective

### Analysis of the Existing Q and R Matrices
```python
# State cost matrix Q (default values)
Q = np.diag([1.0, 1.0, 10.0, 10.0])  # [x, x_dot, theta, theta_dot]

# Control cost R (default value)
R = np.array([[0.1]])  # Control effort cost
```
- The Q matrix prioritized pendulum stability above cart location and velocity by giving more weight to the pendulum angle (θ) and angular velocity (θ_dot).
- To reduce the use of excessive force, the R matrix regulated the effort put forward.

### Updated LQR Parameters
```python
self.Q = np.diag([1.0, 3.0, 1.0, 3.0])  # State cost
self.R = np.array([[0.1]])  # Control cost
```
- **Increased weight on cart velocity (Q[1,1]):** Aids in better control of cart motion.
- **Less weight on the pendulum angle (Q[2,2]):** Permits small variations without requiring drastic adjustments.
- **Increased weight on angular velocity (Q[3,3]):** Reduces excessive cart movement while improving pendulum stabilization.
- **Control effort (R) remains constant** to balance stability and force application.

### Final Tuned Parameters
- **Cart Position (x) →** Q(0,0) (remained constant at 1.0)
- **Cart velocity (ẋ) →** Q(1,1) (increased from 1.0 → 3.0)
- **Pendulum Angle (θ) →** Q(2,2) (Decreased from 10.0 → 1.0)
- **Pendulum Angular Velocity (θ) →** Q(3,3) (Decreased from 10.0 → 3.0)

## Tuning Methodology
### Methodology for Tuning
I utilized an iterative process to carefully tune the LQR controller, concentrating on enhancing stability, cart displacement, and control effort while making sure the system is resilient to disruptions.

#### Step 1: Analysis of Baseline Performance
- Used the system's default Q and R settings to simulate it:
  ```python
  [1.0, 1.0, 10.0, 10.0] = np.diag(Q)
  [[0.1]] = np.array(R)
  ```
- It was noted that the controller overreacted to pendulum aberrations and that the cart moved excessively.

#### Step 2: Modifying the Q Matrix for Cart Control and Stability
- **Increase in Q(1,1) (cart velocity) from 1.0 to 3.0** keeps the cart from moving too much and keeps it within ±2.5m bounds.
- **Q(2,2) (Pendulum Angle) decreased from 10.0 to 1.0.** Permits little deviations without applying undue force.
- **Pendulum Angular Velocity (Q(3,3)) decreased from 10.0 to 3.0.** Guarantees more gradual stability without drastic adjustments.
- **R (Control Effort) remained constant at 0.1.** Preserves equilibrium between energy efficiency and stability.

## Performance Improvements
### Performance Assessment and Testing
Several simulations were run, and the outcomes before and after tuning were compared.

#### Key Performance Metrics
- **Cart displacement:** Ensured it didn't go over ±2.5 meters.
- **Pendulum stability:** Measured angle deviation over time.
- **Control Effort:** Guaranteed force application with minimal energy use.
- **Less cart movement and smoother pendulum stability** were noted, although disruptions were still successfully rejected.

## Analysis of Pendulum and Cart Dynamics
### Time vs. Cart Input
- This graph depicts the control force applied to the cart over time.
- High spikes signify when the system maintains stability by applying strong corrective pressures.
- A steadier and energy-efficient reaction results from clipped forces, which prevent excessive corrections.

**Important Points to Note:**
- The controller actively modifies the applied force in response to disturbances.
- Force values are clipped to avoid unnecessary oscillations.
- The system responds in a balanced manner without applying undue force.

### Cart Position vs. Time
- The cart's movement over time, expressed in meters, is displayed in this graph.
- Greater control with less movement is indicated by smaller variances.
- An overly aggressive controller is suggested by excessive cart movement.

**Important Points to Note:**
- Cart displacement is successfully limited by the adjusted LQR controller.
- The cart moves more steadily and precisely than it did in the original configuration.
- Even in the face of disruptions, the system keeps a constant course.

### Time vs. Pendulum Angle
- This graph shows the pendulum's tilt (in degrees) as it attempts to stay upright over time.
- Minimal oscillations at 0° (vertical position) are indicative of a stable pendulum.
- Instability is indicated by large oscillations.
- Smooth variations indicate a control system that is optimized.

**Important Points to Note:**
- Excessive oscillations are decreased by the adjusted LQR settings.
- Effective balance is ensured because the pendulum stays within allowable deviation bounds.
- Extreme corrections are not necessary for the system to recover from external corrections.

## Conclusion
- **Cart Stability:** The cart stayed inside the physical boundaries of ±2.5m.
- **Reduced Unnecessary Movement:** Increasing Q(1,1) reduced cart movement.
- **Pendulum Stability:** Minimal oscillations with the pendulum remaining upright.
- **Balanced Force Application:** Moderate force was applied, preventing excessive energy consumption.
- **Disturbance Handling:** The system effectively absorbed seismic disturbances without loss of control.

## Graphs and Photographs
- **cart_pole_output_image**
- **Cart Input vs Time**
- **Cart Position vs Time**
- **Pendulum Angle vs Time**
-  ![image](https://github.com/user-attachments/assets/55e56a65-a04f-49be-be7d-638452d6c275)
![image](https://github.com/user-attachments/assets/55347bcf-fcd1-4a7d-b818-4131d6074875)
CART INPUT VS TIME:![image](https://github.com/user-attachments/assets/b030565c-740c-41e1-becc-45b5227c9bbd)
![image](https://github.com/user-attachments/assets/aeec77a5-d28a-4333-9cd3-5e9eaf702167)
CART POSITION VS TIME ![image](https://github.com/user-attachments/assets/f0d76cd2-c106-4637-a12f-ff69e6d47583)
![image](https://github.com/user-attachments/assets/f56ac329-c7b6-4e95-a9c8-7f13f7159a0c)
Pendulum angle vs time : ![image](https://github.com/user-attachments/assets/ac1d947b-e60f-4c83-bddb-2d18a5918ccd)
![image](https://github.com/user-attachments/assets/5899e453-13ab-4045-975f-5e122be66df9)
