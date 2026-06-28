# SW-UAV Project Instructions

This document provides permanent instructions for Gemini when working on this project.

Always follow these instructions unless the user explicitly overrides them.

---

# Project Overview

SW-UAV is an autonomous UAV ground control application written in Python.

The application communicates with UAVs using the Python SDK (MAVSDK), performs mission planning, executes autonomous missions, and visualizes telemetry and maps through a Qt GUI.

Primary capabilities include:

* UAV connection and telemetry
* Mission planning
* Waypoint navigation
* Coverage Path Planning (CPP)
* Obstacle avoidance
* Return to Launch (RTL)
* Landing
* Interactive map visualization
* UAV status monitoring

Safety and reliability are always more important than adding new features.

---

# Technologies

Programming Language

* Python 3.x

Frameworks

* MAVSDK-Python
* PySide6 / Qt
* Folium
* OpenCV (if used)
* NumPy
* Shapely
* asyncio

Simulation

* PX4 SITL
* Gazebo

ROS2 is ONLY used to launch and support the PX4 + Gazebo simulation environment.

ROS is NOT the core architecture of this project.

Do not redesign the project around ROS nodes, topics, services or actions unless explicitly requested.

---

# Project Structure

Important directories:

* src/
* config/
* assets/
* docs/
* matlab/
* cmd/

Do not reorganize the folder structure unless explicitly requested.

---

# Coding Style

Follow PEP8.

Use

* snake_case for variables and functions
* PascalCase for classes
* UPPER_CASE for constants

Prefer:

* type hints
* dataclasses when appropriate
* clear variable names
* modular functions

Avoid overly complex functions.

---

# Modification Rules

When modifying code:

* Prefer editing existing files.
* Do not create new modules unless necessary.
* Preserve the current architecture.
* Do not rename public APIs.
* Do not modify configuration files unless required.
* Keep changes as small as possible.
* Do not rewrite working code unnecessarily.

---

# GUI Rules

The GUI is built using Qt.

When modifying GUI code:

* Preserve existing layouts.
* Avoid breaking signal-slot connections.
* Keep the UI responsive.
* Never block the UI thread with long-running tasks.

Use asynchronous tasks or worker threads when needed.

---

# Async Programming

This project uses asyncio.

Prefer asynchronous code over blocking code.

Avoid:

* time.sleep()
* blocking loops

Prefer:

* await
* asyncio.sleep()
* async tasks

---

# UAV Safety Rules

Safety has the highest priority.

Never remove or bypass:

* connection checking
* health checking
* battery checking
* GPS validation
* flight mode verification
* emergency stop logic
* failsafe logic
* timeout handling

Never arm the UAV automatically unless requested.

Never remove safety checks simply to make the code run.

---

# Navigation Rules

When implementing navigation:

Prioritize

* stable flight
* smooth trajectories
* safe waypoint transitions
* obstacle avoidance
* predictable UAV behavior

Avoid aggressive control logic.

---

# Coverage Path Planning

When modifying CPP algorithms:

Prioritize

* complete area coverage
* minimum overlap
* minimum unnecessary turning
* smooth waypoint generation
* computational efficiency

Preserve algorithm correctness over optimization.

---

# Error Handling

Always handle exceptions properly.

Never ignore exceptions with empty except blocks.

Log meaningful error messages whenever possible.

---

# Verification Workflow

After modifying Python code:

1. Run the affected program if possible.
2. Read all error messages.
3. Identify the root cause.
4. Fix only the necessary code.
5. Run the program again.
6. Repeat until no obvious runtime errors remain.

Do not stop after only generating code.

---

# Before Completing a Task

Before considering a task finished:

* Ensure the code is syntactically correct.
* Verify imports.
* Remove unused code if introduced.
* Check for obvious runtime errors.
* Explain every modified file.
* Report any remaining issues honestly.

Do not claim success unless the code has been reasonably verified.

---

# Things to Avoid

Never:

* remove safety features
* change project architecture without request
* introduce unnecessary dependencies
* duplicate existing functionality
* hardcode configuration values
* break backward compatibility without explanation

---

# Response Style

When modifying code:

1. Explain the problem.
2. Explain the proposed solution.
3. Modify the code.
4. Verify the result if possible.
5. Clearly report any remaining limitations.

If information is missing, ask instead of guessing.

Accuracy is more important than speed.

Also read:

- docs/PROJECT_ARCHITECTURE.md
- docs/CODING_RULES.md