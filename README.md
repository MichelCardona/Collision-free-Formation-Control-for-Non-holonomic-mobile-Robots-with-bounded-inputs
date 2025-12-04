# Collision-free-Formation-Control-for-Non-holonomic-mobile-Robots-with-bounded-inputs
This project consist in a formation control scheme for differential mobile robots. Using four Turtlebot3 Waffle Pi systems, ROS for communication and MATLAB for the control schemes.

The formation control of mobile agents is a widely studied topic in robotics due to its broad range of potential applications across multiple fields. One notable example is its use in transporting large or bulky loads, where mobile agents can organize themselves into a formation adapted to the geometry of the load, optimizing the transportation process. In the military domain, managing the formations of troops and vehicles is crucial in combat scenarios. Formation control would allow unmanned vehicles to maintain strategic arrangements, thereby reducing the risk of human casualties. Additionally, in creative industries such as film or theater, formation control can be a valuable tool for organizing performances or coordinating parades, improving both precision and visual aesthetics. Overall, this approach offers numerous applications that could significantly impact industrial and creative sectors.

The control strategy described here is specifically designed for mobile robots with holonomic constraints. These robots exhibit movement limitations inherent to their mechanical design, as is the case with the Turtlebot3 Waffle Pi units used in this work.

To model and control these robots, a linearized model approach was adopted, since it is not necessary to directly control the robots’ orientation to achieve the desired formation. This simplifies the control system design by allowing the focus to be placed on positional coordination among the robots.

The control method is based on weighted graphs to define and manage the desired formations. In a weighted graph, each edge (connection between vertices) has an associated value or weight, which may represent distances, times, capacities, or any relevant metric. In this context:

The agents (robots) are represented by the vertices of the graph.

The edges represent the desired distances between agents.

The weights assigned to the edges correspond to the distances the system aims to maintain between robots within the formation.

The designed controller takes these weights and decomposes them into their X and Y axis components separately, enabling the calculation and adjustment of the robots’ relative positions with high precision to achieve the desired configuration.

Since the robots’ positions are critical for the controller, a motion-tracking camera system is used to obtain real-time positional data. This system provides accurate and reliable information regarding each robot’s location, allowing the controller to make efficient adjustments synchronized across all agents.

Overall, this weighted-graph-based formation control approach, combined with linearized models and advanced tracking systems, enables effective coordination of mobile robots with holonomic constraints, such as the Turtlebot3 Waffle Pi.


<img width="437" height="291" alt="image" src="https://github.com/user-attachments/assets/6dd89a52-f9e8-4899-bfd9-72f30c56cae7" />


$$ w_ij (x)=-1/f(k_X )   csch^2⁡〖((f(x_i-x_j )-f(δ))/k_X )+1〗$$

$$
\dot{X} =
\begin{pmatrix}
0 & I \\
-K_x & -K_v
\end{pmatrix}
X
+
\begin{pmatrix}
0 \\
I
\end{pmatrix}
u
$$

