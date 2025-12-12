# Physical AI & Humanoid Robotics Textbook (Panaversity) Constitution

<!--
Sync Impact Report:
Version change: N/A (initial) → 1.0.0
Modified principles:
  - N/A (initial)
Added sections:
  - Core Principles
  - Quality Standards
  - Constraints
  - Success Criteria
  - Governance
Removed sections: N/A
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: N/A
-->

## Core Principles

### I. Embodied Intelligence
All code MUST bridge digital AI with physical hardware (Jetson Orin, RTX GPU). This principle ensures that our AI solutions are not purely theoretical but are designed for practical application in humanoid robotics.

### II. Hardware Awareness & Sim-to-Real Transfer
Content MUST explicitly handle "Sim-to-Real" transfer. All code and documentation MUST clearly verify if the implementation is for Simulation (NVIDIA Isaac Sim, Gazebo) or Edge hardware (Jetson Orin). This ensures robust development and deployment across different environments.

## Quality Standards

### I. Academic Tone
All content and documentation MUST maintain a rigorous academic tone, ensuring clarity, accuracy, and adherence to scientific principles in the fields of AI and robotics.

### II. Code Safety
All code MUST prioritize safety, especially concerning physical robot control. Implementations MUST include robust error handling, validation, and fail-safes. Security best practices, including secure authentication (Better-Auth) and data handling, MUST be followed.

## Constraints

### I. Tech Stack Adherence
All development MUST adhere to the specified technology stack:
-   **Robotics Framework:** ROS 2 (Humble) with Python (rclpy)
-   **Simulation:** NVIDIA Isaac Sim
-   **Documentation Platform:** Docusaurus (utilizing Markdown for content)
-   **Backend:** FastAPI
-   **Authentication/Database:** Neon Postgres (for Better-Auth)

### II. Content Format & Coverage
Textbook content MUST be delivered in Markdown format, compatible with Docusaurus. The curriculum MUST cover 4 distinct modules relating to Physical AI and Humanoid Robotics.

### III. Bonus Feature Integration
The project MUST support the following bonus features:
-   **Hardware Personalization:** Integration with Better-Auth for user signup/signin and hardware profiling.
-   **Localization:** User interface MUST include Urdu Translation buttons.
-   **AI Interaction:** Seamless integration with a RAG Chatbot.

## Success Criteria

### I. Hackathon Points System Alignment
Project success will be measured directly against the official Hackathon Points System. All features and quality standards MUST contribute to maximizing the score, with particular emphasis on functionality, innovation, and completeness.

## Governance

This Constitution supersedes all other project practices and documentation. Amendments require a formal review process, documentation of rationale, explicit approval by project leadership, and a clear migration plan for any affected systems or processes. All pull requests and code reviews MUST explicitly verify compliance with these principles. Justification for increased complexity MUST always be provided.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
