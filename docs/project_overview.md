# 3D Missile–Aircraft Pursuit Simulation (Toy IR Heat-Seeker)

This project implements a **3D missile–aircraft engagement** in MATLAB using a simplified infrared (IR) heat-seeker model. The goal is to simulate how a missile pursues a maneuvering target aircraft in 3D space, subject to:

- Missile speed and turn-rate limits  
- A toy IR seeker with **noise** and **lag**  
- A scripted 3D aircraft trajectory with turns and pitch maneuvers  
- A finite **kill radius** for intercept

The script produces a 3D animation of the engagement, showing both the aircraft and the missile trajectories until either:

- The missile reaches the target's kill radius (intercept), or  
- The simulation time runs out.

This is a **didactic / toy model**, not a real weapon system. It is meant for learning about relative motion, seeker behavior, and constrained guidance in 3D.
