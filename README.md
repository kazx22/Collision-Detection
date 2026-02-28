Collision Detection using Particle Physics (Blob Demo)

This project demonstrates a 2D particle collision detection and response system built using C++ and OpenGL (GLUT). The simulation creates multiple moving particles ("blobs") that interact with each other and with static platforms inside a bounded physics world.

Repository:
https://github.com/kazx22/Collision-Detection

Overview

The simulation models a particle-based physics environment where:

Circular particles move under gravity

Particle–particle collisions are detected and resolved

Particles collide with static platforms

A physics engine handles contact generation and penetration resolution

The project explores improving collision detection performance using spatial partitioning compared with a traditional brute force approach.

Key Features

Real-time particle physics simulation

Particle-to-particle collision detection

Platform collision handling

Gravity and damping simulation

OpenGL visualization

Quadrant-based spatial partitioning

Brute force collision detection for comparison

Particle System

Each blob particle contains:

Position

Velocity

Acceleration

Radius

Mass

Damping

Particles are managed and updated through a physics world (ParticleWorld).

Platform Collision

Platforms are implemented as 2D line segments.

They act as static collision surfaces and generate contacts when particles intersect with them.

Collision handling includes:

Endpoint collision detection

Projection-based line collision detection

Collision Detection Methods

This project includes two different collision detection approaches.

Spatial Partitioning (Quadrant-Based Detection)

The simulation space is divided into four quadrants:

Top Left     | Top Right
-------------|-----------
Bottom Left  | Bottom Right

Particles are assigned to quadrants depending on their position during every update step.

Collision checks are only performed between particles inside the same quadrant.

Advantages

Reduces unnecessary collision checks

Improves performance

Scales better as particle count increases

Instead of checking every particle against all others, collisions are limited to local regions.

Quadrants are rebuilt every frame after particle movement.

Brute Force Collision Detection

A brute force implementation is also included in the source code as a commented reference.

This method checks collisions between every possible particle pair:

for each particle i
    for each particle j
Advantages

Simple implementation

Always detects collisions correctly

Disadvantages

O(N²) computational complexity

Performance decreases rapidly with more particles

This version is kept for educational comparison with the optimized spatial partitioning method.

Physics Simulation Flow

Each frame performs the following steps:

Physics update

Particle position integration

Quadrant reassignment

Collision contact generation

Collision resolution

Rendering using OpenGL

Visualization

Rendering is done using:

OpenGL

GLUT

Particles are rendered as spheres with different colors, while platforms are rendered as line segments.

Technologies Used

C++

OpenGL

GLUT

Custom Physics Engine

Vector Mathematics

Main Classes

Particle

ParticleWorld

ParticleContact

ParticleCollision

Platform

BlobDemo

How to Run

Requirements:

C++ Compiler

OpenGL

GLUT

Example compilation:

g++ main.cpp -lglut -lGL -lGLU
Learning Objectives

This project explores:

Collision detection algorithms

Spatial partitioning optimization

Physics-based simulations

Real-time graphics programming

Future Improvements

Possible extensions include:

QuadTree spatial partitioning

Broad-phase and narrow-phase collision systems

GPU acceleration

3D collision simulation

Author

Kazi Alif
https://github.com/kazx22
