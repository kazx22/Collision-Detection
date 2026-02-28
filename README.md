# Collision Detection using Spatial Partitioning (C++ Physics Simulation)

A 2D particle physics simulation demonstrating **collision detection optimisation using spatial partitioning** compared against a traditional **brute force approach**.

Built using **C++ and OpenGL (GLUT)**.

Repository:
https://github.com/kazx22/Collision-Detection

---

## Project Overview

This project simulates multiple moving particles ("blobs") inside a physics world where:

- Particles move under gravity
- Particle–particle collisions occur
- Particles collide with static platforms
- Collision contacts are resolved using a physics engine

The main goal of this project is to explore **efficient collision detection techniques**.

---

## Collision Detection Approaches

### 1. Quadrant-Based Spatial Partitioning (Optimised)

The simulation space is divided into four regions:

Top Left	Top Right
Bottom Left	Bottom Right

Particles are assigned to quadrants based on their position.

Collision checks are performed only between particles inside the same region.

This significantly reduces unnecessary comparisons.

#### Benefits

- Faster collision detection
- Reduced computation cost
- Better scalability with more particles

---

### 2. Brute Force Collision Detection (Reference)

A brute force implementation is included in the source code as a commented section.

This method checks every particle against every other particle.

Example logic:


for each particle i
for each particle j


#### Drawbacks

- O(N²) complexity
- Performance decreases as particle count increases

This version is kept for comparison and learning purposes.

---

## Physics Simulation

Each simulation frame performs:

1. Physics update
2. Particle integration
3. Quadrant reassignment
4. Collision detection
5. Contact resolution
6. Rendering

---

## Features

- Particle physics simulation
- Real-time collision detection
- Platform collision handling
- Gravity and damping effects
- Spatial partitioning optimisation
- OpenGL visualisation

---

## Technologies Used

- C++
- OpenGL
- GLUT
- Custom Physics Engine
- Vector Mathematics

---

## Core Components

Main classes used in the project:

- Particle
- ParticleWorld
- ParticleContact
- ParticleCollision
- Platform
- BlobDemo

---

## How to Build

Requirements:

- C++ Compiler
- OpenGL
- GLUT

Example compilation:

```bash
g++ main.cpp -lglut -lGL -lGLU
Learning Goals

This project explores:

Collision detection algorithms

Spatial optimisation techniques

Real-time simulation systems

Physics-based programming

Future Improvements

Possible future extensions:

QuadTree spatial partitioning

Broad-phase and narrow-phase detection

3D collision systems

GPU acceleration

Author

Kazi Alif

GitHub:
https://github.com/kazx22
