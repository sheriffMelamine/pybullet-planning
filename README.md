# pybullet-planning

This is a **forked repository**. For a detailed overview of the repository files and installation instructions, please refer to the original repository: [pybullet_planning](https://github.com/caelan/pybullet_planning).

---

## Heterogeneous Multi-Robot System

This project focuses on developing a **heterogeneous multi-robot system** for **collaborative long-horizon task planning**.  
The objective is to enable coordinated task execution among robots with different capabilities within a shared simulation environment.

<p align="center">
  <img src="images/hetero.gif" alt="Heterogeneous Multi-Robot Simulation" width="600"/>
</p>

---

### Test Environment: Single Execution

In this setup, a set of tasks are done by the robots one by one, and none of the robots can work concurrently. As the timestep of simulation is controlled by the task itself (i.e. when robots are idle, the simulation will pause), it does not provide practical simulation scenario. 

To run the test simulation of the system environment with such task execution, execute the following command in the `pybullet_planning` directory:

```bash
python3 -m hetero_env_single
```
---

### Test Environment: Concurrent Execution via Python Generator Implementation

In this setup, both robots can do tasks in parallel, though it is not true sync, as the robots' steps are iterated in a loop separately. By using python generators, it was possible to create this parallel task scheduling mechanism. Here, the simulation time control is less compared to the single execution, though it is still somewhat controlled by the main loop and the logic structure.

To run the test simulation of the system environment with such task execution, execute the following command in the `pybullet_planning` directory:

```bash
python3 -m hetero_env_gencon
```
---

### Test Environment: Concurrent Execution using `asyncio`

In this setup, both robots do task in parallel asynchronously, and here the simulation stepping is fully neutral of the control of the task itself, as it runs independently. The physics simulation, and the individual robot tasks all are handled by `asyncio`, and the main loop is also free of logic structure of the task which was necessary in previous case. Therefore, this is more practical simulation environment compared to the previous two.

To run the test simulation of the system environment with such task execution, execute the following command in the `pybullet_planning` directory:
```bash
python3 -m hetero_env_asyncio
```
---
*Additional Modules to be added later...*
