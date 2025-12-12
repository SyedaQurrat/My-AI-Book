---
id: 1
title: Generate Physical AI Humanoid Robotics Textbook Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.constitution as a Principal Curriculum Designer and AI Architect. I need you to generate the `.specify/memory/constitution.md` file for our Hackathon Project. **Project Name:** Physical AI & Humanoid Robotics Textbook (Panaversity) **Context:** We are building an AI-Native Textbook using Docusaurus that teaches students how to control Humanoid Robots using ROS 2, NVIDIA Isaac Sim, and VLA (Vision-Language-Action) models. The project includes a RAG Chatbot, Hardware Personalization, and Urdu Translation. **Requirements to Enforce in Constitution:** 1.  **Core Philosophy:** "Embodied Intelligence" - All code must bridge digital AI with physical hardware (Jetson Orin, RTX GPU). 2.  **Hardware Awareness:** Content must explicitly handle "Sim-to-Real" transfer. Always verify if code is for Simulation (Gazebo) or Edge (Jetson). 3.  **Tech Stack:** ROS 2 (Humble), Python (rclpy), NVIDIA Isaac Sim, Docusaurus (Markdown), FastAPI (Backend), Neon Postgres (Auth). 4.  **Bonus Features Constraints:**     -   Must support Better-Auth for hardware profiling (Signup/Signin).     -   Must support Urdu Translation buttons in UI.     -   Must support RAG Chatbot integration. **Output:** Generate a rigorous `constitution.md` with: -   Core Principles -   Quality Standards (Academic Tone, Code Safety) -   Constraints (Markdown format, 4 Modules coverage) -   Success Criteria (Based on Hackathon Points System)
labels: ["project-setup", "constitution", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
null
---

## Prompt

as a Principal Curriculum Designer and AI Architect. I need you to generate the `.specify/memory/constitution.md` file for our Hackathon Project.

**Project Name:** Physical AI & Humanoid Robotics Textbook (Panaversity)

**Context:**
We are building an AI-Native Textbook using Docusaurus that teaches students how to control Humanoid Robots using ROS 2, NVIDIA Isaac Sim, and VLA (Vision-Language-Action) models. The project includes a RAG Chatbot, Hardware Personalization, and Urdu Translation.

**Requirements to Enforce in Constitution:**
1.  **Core Philosophy:** "Embodied Intelligence" - All code must bridge digital AI with physical hardware (Jetson Orin, RTX GPU).
2.  **Hardware Awareness:** Content must explicitly handle "Sim-to-Real" transfer. Always verify if code is for Simulation (Gazebo) or Edge (Jetson).
3.  **Tech Stack:** ROS 2 (Humble), Python (rclpy), NVIDIA Isaac Sim, Docusaurus (Markdown), FastAPI (Backend), Neon Postgres (Auth).
4.  **Bonus Features Constraints:**
    -   Must support Better-Auth for hardware profiling (Signup/Signin).
    -   Must support Urdu Translation buttons in UI.
    -   Must support RAG Chatbot integration.

**Output:**
Generate a rigorous `constitution.md` with:
-   Core Principles
-   Quality Standards (Academic Tone, Code Safety)
-   Constraints (Markdown format, 4 Modules coverage)
-   Success Criteria (Based on Hackathon Points System)

## Response snapshot

Generated the initial project constitution at `.specify/memory/constitution.md`.

## Outcome

- ✅ Impact: Successfully generated the project constitution based on user requirements.
- 🧪 Tests: No automated tests were run for constitution generation.
- 📁 Files: Generated one file: `.specify/memory/constitution.md`
- 🔁 Next prompts: Review generated constitution; proceed with plan generation.
- 🧠 Reflection: Initial constitution generated. Next steps involve generating plan and spec templates and ensuring consistency with this constitution.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): initial-constitution-generation
- Next experiment (smallest change to try): N/A
