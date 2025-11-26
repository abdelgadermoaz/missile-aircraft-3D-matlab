# Simulation Setup & Parameters

This document summarizes the main parameters used in the simulation and how they map to the MATLAB script.

---

## 1. Time Settings

```matlab
dt      = 0.01;   % time step [s]
t_final = 60;     % total sim time [s]
t       = 0:dt:t_final;
