# State Space System Model

---

## What is a State Space Controller

State space representation can relate constitutive equations between multi-domain systems (e.g. mechanical, fluid, electrical, etc.) in a simplified set of linear equations.  

In classical controls, a PID controller is often used to perform a closed-loop response utilizing a single input reference and a single output control (SISO).  State space control is a modern control technique that allows for multi-input and multi-output (MIMO) control.  

Because state space controls uses multiple inputs, it can be more expensive to implement due to the need for more sensors.  However, in practice, an observer may be developed to estimate the input response without the need of physical hardware making this option more affordable.

## Steps for developing a state space controller:

1) Design the state space equations:
   $$
   \vec{\dot{x}} = \tilde{A}\vec{x} + \tilde{B}\vec{u} \\
   \vec{{y}} = \tilde{C}\vec{x} + \tilde{D}\vec{u}
   $$

2. Determine desired outputs
3. Check for stability
4. Check for observability
5. Check for controllability
6. Develop Controllability Matrix
7. Develop Observability Matrix
8. Test with matlab

## Why use closed-loop controller?

A closed-loop controller has the ability to perform processes accurately in an imperfect system.  For instance, an inverted pendulum is an unstable system due to the force of gravity.  However, this can be rectified by developing a controller that can read the angular position of the pendulum and then perform an action to straighten the pendulum.  

In the case of the standing desk, it is important that the motors operate in a controlled manner such that the desk surface remains flat during travel.  If a motor is slower than the other, this causes an issue.  With state space control, we can monitor the speed of each motor at the same time and adjust the voltage level to each motor independently in order to compensate for different external factors affecting the system dynamics.

## Why not use a PID controller?

Because the system has two motors, two PID controllers would need to be created.  This approach can also work.  However, the relationship between the two motors is severed.  Another benefit of state space control is that many outputs can be monitored at the same time and can be linked together easily through linear systems.  For instance, the angular rotation of the desk surface can be calculated directly from the state space relations.  





