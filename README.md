# pybullet-planning

This is a **forked repository**. For a detailed overview of the repository structure and installation instructions, please refer to the original repository:  
➡️ [pybullet-planning](https://github.com/caelan/pybullet-planning)

---

## Heterogeneous Multi-Robot System

This project explores a **heterogeneous multi-robot framework** for **collaborative long-horizon task planning** and execution.  
The system enables coordinated workflows between robots with different capabilities in a unified PyBullet simulation environment.

<p align="center">
  <img src="images/hetero.gif" alt="Heterogeneous Multi-Robot Simulation" width="600"/>
</p>

---
### Installation:

```bash
git clone --recursive https://github.com/sheriffMelamine/pybullet-planning.git
```

### Requirements:

In order to test the codes, install python modules with their appropriate versions:

```bash
pip install -r requirements.txt
```
The robots (Franka and PR2) used in this project are using **ikfast** for solving inverse kinematics. For that:

```bash
source setup_ikfast.sh
```

---

### ✅ Test Environment: Single Execution

In this configuration, tasks are executed **strictly one after another**, meaning only one robot is active at any given time.  
The simulation timestep is controlled by the task execution itself — when robots are idle, the physics engine effectively pauses.  
This leads to a **deterministic but not time-realistic** simulation, making it useful for verifying logic flow and debugging foundational behaviors, but not ideal for practical collaborative scenarios.

This mode represents the simplest execution structure and establishes baseline behavior before introducing concurrency.

To run:

```bash
python3 -m hetero_env_single
````

---

### ✅ Test Environment: Concurrent Execution via Python Generator Implementation

In this mode, both robots can operate **in parallel**, but concurrency is achieved through Python generators (`yield` / `yield from`), creating a cooperative multitasking structure.
Each robot progresses in small steps as control is yielded back and forth, resulting in **interleaved task execution**.
While this achieves a form of concurrency, the main loop still influences timing and task switching, and therefore the simulation is not fully independent of task control flow.

This approach offers more dynamic behavior than the single-execution version and demonstrates a clear improvement in realism and responsiveness, though it still maintains some logical coupling to the scheduling loop.

To run:

```bash
python3 -m hetero_env_gencon
```

---

### ✅ Test Environment: Concurrent Execution using `asyncio`

This mode provides **fully asynchronous execution** using Python’s `asyncio` framework.
Each robot, along with the physics simulation, runs as an independent asynchronous task.
This decouples simulation time from task logic — robots operate concurrently without needing to yield control manually, and the simulation continues to progress regardless of individual task scheduling.

This design enables more realistic and scalable multi-robot interaction, where timing, motion, and task overlap more closely resemble real robotic systems.
It is the most flexible and practical setup among the three, well-suited for complex multi-robot workflows and long-horizon coordinated planning.

To run:

```bash
python3 -m hetero_env_asyncio
```

---

### ✅ Test Environment: Implementing Behavior Tree with `asyncio`

Utilizing the `asyncio` based concurrent simulation, the implementaion of **behavior trees** for high level planning is added. For that, `py_trees` module is used for creating behavior trees for individual robots and defining their plans. In this code, behavior tree is run in a separate thread in background, while the simulation and the robot motion and calculations are coordinated in `asyncio` based concurrent simulation. 

In other words, the **low-level motion** and the **high-level planning** are kept *independent*, where they can communicate using **shared environment variables**.

To run:

```bash
python3 -m hetero_env_bt_async
```

---
*More modules & demos will be added soon…*



