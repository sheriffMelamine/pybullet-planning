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

### Test Environment: Asynchronous Execution

In this setup, a set of tasks are done by the robots one by one, and none of the robots can work concurrently.   

To run the test simulation of the system environment with asynchronous task execution, execute the following command in the `pybullet_planning` directory:

```bash
python3 -m hetero_env_async
```
---

### Test Environment: Fake Synchronous Execution

In this setup, both robots can do tasks in parallel, though it is not true sync, as the robots' steps are iterated in a loop separately. By using python generators, it was possible to create this fake parallel mechanism.   

To run the test simulation of the system environment with such synchronous task execution, execute the following command in the `pybullet_planning` directory:

```bash
python3 -m hetero_env_fsync
```
---
*Additional Modules to be added later...*
