---
description: "Task list for Module 1 - The Robotic Nervous System (ROS 2) implementation"
---

# Tasks: Module 1 - The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/001-ros2-module-1/`
**Prerequisites**: plan.md (architecture & tech stack), spec.md (3 user stories + acceptance scenarios)
**Organization**: Tasks grouped by user story to enable independent content authoring and chatbot integration
**Testing**: Content validation tests based on acceptance scenarios (BDD); integration tests for chatbot accuracy

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1, US2, US3) - only for user story phase tasks
- **Exact file paths** included for all tasks

## Path Conventions

- **Book content**: `docs/docs/modules/01-ros2-fundamentals/`
- **Code examples**: `docs/docs/modules/01-ros2-fundamentals/code-examples/`
- **Diagrams**: `docs/docs/modules/01-ros2-fundamentals/diagrams/`
- **Backend**: `backend/src/`, `backend/tests/`
- **Chatbot frontend**: `chatbot-frontend/src/`, `chatbot-frontend/tests/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization for Docusaurus book site, backend service, and chatbot component

- [x] T001 Create docs/ directory structure per plan.md: docs/docs/modules/01-ros2-fundamentals/{_category_.json, code-examples/, diagrams/}
- [x] T002 Initialize Docusaurus 3.x project with `npx create-docusaurus@latest frontend_ai_book classic`
- [x] T003 [P] Create docusaurus.config.js with site metadata, dark mode, markdown plugins (admonitions, code tabs)
- [x] T004 [P] Create docs/docs/introduction.md (Docusaurus intro page explaining book structure)
- [x] T005 [P] Create docs/sidebars.js with navigation for Module 1 chapters
- [x] T006 Create backend/ directory structure per plan.md: backend/src/{main.py, models/, services/, api/}, backend/tests/{unit/, integration/}
- [x] T007 [P] Initialize FastAPI project with requirements.txt (FastAPI, python-dotenv, pydantic, aiohttp, qdrant-client, openai)
- [x] T008 [P] Create backend/.env.example with placeholders (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL)
- [x] T009 [P] Initialize chatbot-frontend/ with `npm init -y && npm install react typescript @types/react @types/node`
- [x] T010 Create _category_.json in docs/docs/modules/01-ros2-fundamentals/ with chapter group metadata (label, position, collapsible)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY content authoring or chatbot work

**⚠️ CRITICAL**: No user story work can begin until this phase completes

- [ ] T011 Create backend/src/config.py with environment variable loading, Qdrant client initialization, OpenAI client setup
- [ ] T012 [P] Create backend/src/models/query.py with ChatbotQuery, ChatbotResponse, Citation Pydantic models
- [ ] T013 [P] Create backend/src/models/citation.py with Citation model (section_id, chapter_title, heading, line_range, confidence_score)
- [ ] T014 [P] Create backend/src/api/middleware.py with CORS setup, request logging, error handling
- [ ] T015 [P] Create backend/src/api/routers/chatbot.py with route definitions (POST /chatbot/query stub, POST /chatbot/feedback stub)
- [ ] T016 Create backend/src/main.py FastAPI app initialization with routers and middleware
- [ ] T017 [P] Create backend/src/services/embedding_service.py with embed_text(text) → embedding vector function (OpenAI API)
- [ ] T018 [P] Create backend/src/services/context_service.py with store_conversation(), retrieve_context() functions
- [ ] T019 [P] Create backend/src/services/rag_service.py stub with query_rag_index(), ingest_chapter() function signatures
- [ ] T020 [P] Create backend/tests/conftest.py with pytest fixtures (mock Qdrant, mock OpenAI)
- [ ] T021 [P] Create chatbot-frontend/src/components/ChatWindow.tsx component skeleton with message state, input handler
- [ ] T022 [P] Create chatbot-frontend/src/services/chatbotApi.ts with fetchChatbotResponse() function stub
- [ ] T023 [P] Create chatbot-frontend/src/hooks/useChatbot.ts custom hook for state + API integration
- [ ] T024 Create docs/docs/modules/01-ros2-fundamentals/index.md (redirect to chapter 1 or overview)
- [ ] T025 Create RAG indexing system design doc in specs/001-ros2-module-1/data-model.md (Section entity, indexing metadata, validation rules)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Learn ROS 2 Fundamentals and Purpose (Priority: P1) 🎯 MVP

**Goal**: Students read Chapter 1 and understand what ROS 2 is, its purpose in humanoid robotics, and key differences from ROS 1

**Independent Test**: Students can summarize ROS 2 core purpose (2-3 sentences), describe 3+ ROS 1 vs ROS 2 differences, explain why humanoid robots use ROS 2

### Content Authoring for User Story 1

- [x] T026 [US1] Create outline for Chapter 1 in docs/docs/modules/01-ros2-fundamentals/01-introduction.md with sections: (1) What is ROS 2? (2) ROS 1 vs ROS 2, (3) Why Humanoid Robots Need ROS 2, (4) ROS 2 in Physical AI Ecosystem
- [x] T027 [P] [US1] Write Chapter 1 Section 1: "What is ROS 2?" (300-400 words, define middleware, pub/sub, DDS, hardware abstraction)
- [x] T028 [P] [US1] Write Chapter 1 Section 2: "ROS 1 vs ROS 2" (400-500 words, compare master-dependent vs masterless, real-time support, multi-platform, DDS)
- [x] T029 [P] [US1] Write Chapter 1 Section 3: "Why Humanoid Robots Need ROS 2" (250-350 words, distributed systems, real-time constraints, hardware diversity)
- [x] T030 [P] [US1] Write Chapter 1 Section 4: "ROS 2 in Physical AI Ecosystem" (200-300 words, relationship to AI agents, example architectures)
- [ ] T031 [US1] Create 2 diagrams for Chapter 1: (1) ROS 2 architecture (nodes, DDS layer, middleware) in docs/docs/modules/01-ros2-fundamentals/diagrams/ros2-architecture.svg, (2) ROS 1 vs ROS 2 comparison table (embedded in markdown or SVG)
- [ ] T032 [US1] Embed diagrams in Chapter 1 markdown with proper alt text and captions
- [ ] T033 [US1] Create comprehension quiz in Chapter 1 (3-5 questions on ROS 2 purpose, ROS 1 vs 2, humanoid use case) - embed as markdown admonition with answer hints

### RAG Indexing for User Story 1

- [ ] T034 [US1] Tag each section in Chapter 1 markdown with unique IDs: 1.1 (What is ROS 2?), 1.2 (ROS 1 vs ROS 2), 1.3 (Why Humanoid?), 1.4 (Physical AI)
- [ ] T035 [US1] Create metadata in Chapter 1 frontmatter (YAML): chapter_id, created_date, word_count, section_ids
- [ ] T036 [US1] Write RAG indexing pipeline test in backend/tests/integration/test_chapter1_indexing.py (load Chapter 1, split by sections, verify section count = 4, verify all section IDs present)
- [ ] T037 [US1] Implement backend/src/services/rag_service.py::ingest_chapter(chapter_id, markdown_content) to split Chapter 1 by H2 headings, generate embeddings, store in Qdrant with section metadata

### Chatbot Integration for User Story 1

- [ ] T038 [US1] Create backend/tests/integration/test_ros2_fundamentals_queries.py with 5 test queries: "What is ROS 2?", "How is ROS 2 different from ROS 1?", "Why do humanoid robots use ROS 2?", "What is DDS?", "How does ROS 2 help AI agents?"
- [ ] T039 [US1] Implement backend/src/services/rag_service.py::query_rag_index(query: str) to embed query, search Qdrant top 3 sections, build prompt context, call OpenAI GPT-4, extract citations
- [ ] T040 [US1] Implement backend/src/api/routers/chatbot.py::POST /chatbot/query endpoint with response: {answer, citations[], retrieval_metadata}
- [ ] T041 [US1] Test chatbot endpoint locally: query "What is ROS 2?", verify response includes chapter 1 citation with section_id
- [ ] T042 [P] [US1] Implement chatbot-frontend/src/components/ChatWindow.tsx with message rendering, input field, send button (basic UI)
- [ ] T043 [P] [US1] Implement chatbot-frontend/src/services/chatbotApi.ts::fetchChatbotResponse(query) to call backend /chatbot/query endpoint
- [ ] T044 [US1] Integrate CitationDisplay component in ChatWindow.tsx to render citations with chapter/section/line range

### Validation for User Story 1

- [ ] T045 [US1] Content validation: 4 sections written, 2 diagrams embedded, quiz questions present, total word count 1200-1500 words
- [ ] T046 [US1] Acceptance test: 3 manual student reviews - Can they summarize ROS 2 purpose? Can they list 3 ROS 1 vs 2 differences? Can they explain why humanoid robots use ROS 2?
- [ ] T047 [US1] Chatbot accuracy: 5 test queries answered correctly (>85% confidence score), citations point to correct section, no hallucinations

**Checkpoint**: User Story 1 complete - Chapter 1 authored, indexed, and chatbot integration working

---

## Phase 4: User Story 2 - Understand Core ROS 2 Communication Patterns (Priority: P1)

**Goal**: Students read Chapter 2 and learn nodes, topics, services with concrete humanoid robot examples

**Independent Test**: Students can design 2 system architectures (sensor subscription, actuator command), write pseudocode for a node, explain topic vs service differences

### Content Authoring for User Story 2

- [x] T048 [US2] Create outline for Chapter 2 in docs/docs/modules/01-ros2-fundamentals/02-core-concepts.md with sections: (1) Nodes Explained, (2) Topics: Pub/Sub Patterns, (3) Services: Request/Response, (4) Communication Patterns in Humanoid Systems
- [x] T049 [P] [US2] Write Chapter 2 Section 1: "Nodes Explained" (300-400 words, define nodes, rclpy node structure, node lifecycle)
- [x] T050 [P] [US2] Write Chapter 2 Section 2: "Topics: Pub/Sub Patterns" (400-500 words, explain pub/sub, message queues, examples: /imu, /camera/rgb, /joint_state for humanoid robots)
- [x] T051 [P] [US2] Write Chapter 2 Section 3: "Services: Request/Response" (350-450 words, explain services, call-response semantics, examples: /robot/get_gripper_position, /robot/set_velocity)
- [x] T052 [P] [US2] Write Chapter 2 Section 4: "Communication Patterns in Humanoid Systems" (300-400 words, sensor fusion topology, control loops, real-time constraints, 100Hz polling example)
- [ ] T053 [US2] Create 3 diagrams for Chapter 2: (1) Node architecture (multiple nodes communicating) in SVG, (2) Pub/Sub topology (sensor subscribers, command publishers) in SVG, (3) Service call flow in SVG
- [ ] T054 [US2] Embed diagrams in Chapter 2 with alt text; include practical humanoid robot examples (IMU → perception node, control node → joint commands)
- [ ] T055 [US2] Create system design exercise in Chapter 2: "Design a node architecture for a humanoid robot that reads IMU sensor and sends velocity commands. Draw the topic/service graph." (with solution template)
- [ ] T056 [US2] Create classification exercise: "For each scenario, choose topic or service: (a) Publish 100Hz camera frames, (b) Request joint position once, (c) Stream lidar data, (d) Send emergency stop signal"

### RAG Indexing for User Story 2

- [ ] T057 [US2] Tag each section in Chapter 2: 2.1 (Nodes), 2.2 (Topics), 2.3 (Services), 2.4 (Patterns in Humanoid)
- [ ] T058 [US2] Create Chapter 2 YAML frontmatter with section metadata, link diagrams in frontmatter
- [ ] T059 [US2] Write RAG indexing test in backend/tests/integration/test_chapter2_indexing.py (verify 4 sections indexed, 3 diagram references captured)
- [ ] T060 [US2] Re-index Qdrant with Chapter 2 content via ingest_chapter() (now 8 total sections across Chapters 1 + 2)

### Chatbot Integration for User Story 2

- [ ] T061 [US2] Create test queries for Chapter 2: "What is a ROS 2 node?", "Explain topics vs services", "Why use topics for sensor data?", "How fast can ROS 2 nodes communicate?", "Design a humanoid control system using topics and services"
- [ ] T062 [US2] Test chatbot: Query "Explain topics vs services", verify response cites Chapter 2 sections 2.2 and 2.3 with correct line ranges
- [ ] T063 [US2] Test chatbot accuracy on Chapter 2 content: 5+ test queries, >85% correct responses with valid citations

### Validation for User Story 2

- [ ] T064 [US2] Content validation: 4 sections written, 3 diagrams embedded, 2 exercises present, 1500-1800 words total
- [ ] T065 [US2] Acceptance test: Student design exercise - can they sketch a node architecture for sensor fusion + control? Can they classify scenarios correctly?
- [ ] T066 [US2] Chatbot accuracy: 5 test queries answered with citations pointing to Chapter 2 sections, no off-topic responses

**Checkpoint**: User Story 2 complete - Chapter 2 authored, indexed, chatbot answering communication pattern questions

---

## Phase 5: User Story 3 - Bridge Python Agents to Humanoid Robots via rclpy (Priority: P1)

**Goal**: Students learn rclpy, write working Python nodes, understand URDF, and design end-to-end systems

**Independent Test**: Students write a Python rclpy node (subscriber + publisher), understand URDF links/joints/sensors, outline system design connecting AI agent to robot

### Content Authoring for User Story 3

- [x] T067 [US3] Create outline for Chapter 3 in docs/docs/modules/01-ros2-fundamentals/03-python-integration.md with sections: (1) Introduction to rclpy, (2) Writing Subscriber Nodes, (3) Writing Publisher Nodes, (4) Understanding URDF, (5) System Design: AI Agent → Robot
- [x] T068 [P] [US3] Write Chapter 3 Section 1: "Introduction to rclpy" (250-350 words, rclpy client library, installation, imports, basic node creation)
- [x] T069 [P] [US3] Write Chapter 3 Section 2: "Writing Subscriber Nodes" (400-500 words, callback pattern, QoS settings, error handling with code examples)
- [x] T070 [P] [US3] Write Chapter 3 Section 3: "Writing Publisher Nodes" (400-500 words, publishing messages, topic naming conventions, timing with code examples)
- [x] T071 [P] [US3] Write Chapter 3 Section 4: "Understanding URDF" (400-500 words, XML format, links (rigid bodies), joints (DOF), sensors, kinematics with example humanoid snippets)
- [x] T072 [P] [US3] Write Chapter 3 Section 5: "System Design: AI Agent → Robot" (350-450 words, end-to-end architecture, agent decision loop → rclpy nodes → robot actuators, latency considerations)
- [ ] T073 [US3] Create Python code examples and save to docs/docs/modules/01-ros2-fundamentals/code-examples/:
  - [ ] T073a [P] [US3] docs/docs/modules/01-ros2-fundamentals/code-examples/ros2-node-subscriber.py (listen to /imu topic, print messages, 40-50 lines)
  - [ ] T073b [P] [US3] docs/docs/modules/01-ros2-fundamentals/code-examples/ros2-node-publisher.py (publish commands to /joint_velocity topic, 40-50 lines)
  - [ ] T073c [P] [US3] docs/docs/modules/01-ros2-fundamentals/code-examples/ros2-sensor-to-command.py (subscriber + publisher combined, reads sensor, processes, publishes command, 60-80 lines)
- [ ] T074 [P] [US3] Create URDF example docs/docs/modules/01-ros2-fundamentals/code-examples/humanoid-robot-simple.urdf (simple humanoid with torso, 2 arms, 2 legs, 5-10 joints, 50-80 lines)
- [ ] T075 [US3] Embed code examples in Chapter 3 markdown using code tabs (rclpy subscriber, publisher, combined; URDF example) with syntax highlighting
- [ ] T076 [US3] Create 3 diagrams: (1) rclpy node lifecycle (init → run → callback → shutdown) in SVG, (2) URDF tree structure for humanoid robot in SVG, (3) AI agent → rclpy nodes → robot control loop in SVG
- [ ] T077 [US3] Create hands-on exercise in Chapter 3: "Write a Python node that subscribes to /joint_state, publishes velocity commands to /joint_velocity. What QoS settings would you use for real-time control?" (with solution template)
- [ ] T078 [US3] Create URDF parsing exercise: "Examine the humanoid URDF. How many joints does it have? What type of joints? How would you command the gripper?" (with solution)

### Code Examples Validation for User Story 3

- [ ] T079 [US3] Test Python code examples: ros2-node-subscriber.py runs without syntax errors (pylint check in backend/tests/)
- [ ] T080 [US3] Test Python code examples: ros2-node-publisher.py runs without syntax errors
- [ ] T081 [US3] Test Python code examples: ros2-sensor-to-command.py combines sub + pub correctly (no race conditions, proper cleanup)
- [ ] T082 [US3] Validate URDF: humanoid-robot-simple.urdf is well-formed XML, loads in urdfpy or robot_state_publisher (via test script)

### RAG Indexing for User Story 3

- [ ] T083 [US3] Tag Chapter 3 sections: 3.1 (rclpy intro), 3.2 (subscribers), 3.3 (publishers), 3.4 (URDF), 3.5 (system design)
- [ ] T084 [US3] Create Chapter 3 YAML frontmatter; link code examples and diagrams
- [ ] T085 [US3] Write RAG indexing test in backend/tests/integration/test_chapter3_indexing.py (verify 5 sections indexed, code examples referenced, URDF example captured)
- [ ] T086 [US3] Re-index Qdrant with all 3 chapters (total 12 sections across Chapters 1, 2, 3; code examples and diagrams indexed as metadata)

### Chatbot Integration for User Story 3

- [ ] T087 [US3] Create test queries for Chapter 3: "How do I write a rclpy subscriber?", "What is URDF?", "How do I connect an AI agent to a robot?", "Show me a code example of publishing commands", "What QoS settings should I use for real-time control?"
- [ ] T088 [US3] Test chatbot: Query "Show me a rclpy code example", verify response includes code snippet from code-examples/ with citation to Chapter 3
- [ ] T089 [US3] Test chatbot accuracy on Chapter 3: 5+ queries, >85% correct responses with citations pointing to Chapter 3 sections + code examples

### Validation for User Story 3

- [ ] T090 [US3] Content validation: 5 sections written, 3 diagrams embedded, 3 code examples + 1 URDF + 2 exercises, 2000-2500 words total
- [ ] T091 [US3] Code validation: All Python examples syntactically correct, URDF valid XML, no hard-coded credentials/secrets
- [ ] T092 [US3] Acceptance test: Student writes working rclpy node (sub + pub), understands URDF structure, can outline end-to-end system design
- [ ] T093 [US3] Chatbot accuracy: 5 test queries with code examples, >85% accurate, citations correct

**Checkpoint**: User Story 3 complete - All 3 chapters authored, indexed, chatbot fully functional with code examples and citations

---

## Phase 6: Chatbot Accuracy & Observability

**Purpose**: Ensure chatbot meets success criterion SC-005 (>90% accuracy) and observability requirements

- [ ] T094 Create comprehensive test query set (50+ questions) derived from chapter acceptance scenarios in backend/tests/integration/test_chatbot_accuracy.py
- [ ] T095 [P] Run chatbot accuracy test: Execute all 50+ queries, measure accuracy, citation correctness, confidence scores
- [ ] T096 [P] Implement user feedback endpoint backend/src/api/routers/chatbot.py::POST /chatbot/feedback (thumbs up/down, user comments)
- [ ] T097 [P] Implement feedback logging in backend/src/services/rag_service.py to track which queries get positive/negative feedback
- [ ] T098 Analyze accuracy results: If <90%, refine OpenAI prompt in backend/src/services/rag_service.py to improve response quality and citation accuracy
- [ ] T099 Create observability dashboard (docs/monitoring.md) showing: chatbot query count, avg response time, accuracy rate, top N questions, user feedback summary

---

## Phase 7: Frontend Build & Deployment

**Purpose**: Build Docusaurus static site and deploy to GitHub Pages; prepare backend and chatbot frontend for deployment

### Docusaurus Build & GitHub Pages

- [ ] T100 Configure GitHub Pages in docusaurus.config.js (baseUrl, org, project settings)
- [ ] T101 [P] Create GitHub Actions workflow in .github/workflows/deploy-docusaurus.yml (build on push to main, deploy to GitHub Pages)
- [ ] T102 Build Docusaurus locally: `cd docs && npm run build` and verify build succeeds (no markdown errors, all diagrams embedded)
- [ ] T103 [P] Test Docusaurus site locally: `cd docs && npm run start` and verify all 3 chapters render correctly, diagrams display, code examples visible
- [ ] T104 Verify Docusaurus sidebar navigation works: Click through all 3 chapters, quizzes/exercises load
- [ ] T105 Test chatbot widget embedded in Docusaurus: Verify ChatWindow component renders in a chapter page, can send queries
- [ ] T106 Deploy to GitHub Pages: Push to main branch, verify GitHub Actions builds and deploys, site live at https://[owner].github.io/Hackathon-I/

### Backend Deployment Preparation

- [ ] T107 [P] Create Dockerfile for FastAPI backend (Python 3.10 base, requirements.txt, expose port 8000)
- [ ] T107 [P] Create requirements.txt for backend with all dependencies pinned to versions (FastAPI, Qdrant client, OpenAI, etc.)
- [ ] T108 Create backend deployment guide in specs/001-ros2-module-1/quickstart.md: "Deploy FastAPI to Vercel" with environment variable setup
- [ ] T109 [P] Create .github/workflows/test-backend.yml: pytest on push, run backend integration tests
- [ ] T110 Test backend locally: `cd backend && python -m pytest tests/integration/ -v` and verify all tests pass

### Chatbot Frontend Build

- [ ] T111 Build chatbot React app: `cd chatbot-frontend && npm run build` and verify build succeeds
- [ ] T112 Test chatbot UI locally: `cd chatbot-frontend && npm start` and verify ChatWindow component renders, can type and submit queries
- [ ] T113 Test chatbot-to-backend integration: Query from chatbot UI should reach backend /chatbot/query endpoint (verify via console/network tab)

---

## Phase 8: Documentation & Quickstart

**Purpose**: Comprehensive documentation for future contributors and deployment guide

- [ ] T114 Write specs/001-ros2-module-1/quickstart.md covering: (1) How to add a new chapter, (2) How to embed code examples, (3) How to add diagrams, (4) How to test chapter content, (5) How to deploy
- [ ] T115 [P] Write specs/001-ros2-module-1/data-model.md documenting: Chapter/Section/CodeExample/Diagram/Conversation entities, validation rules, indexing strategy
- [ ] T116 [P] Write specs/001-ros2-module-1/contracts/chatbot-api.md documenting POST /chatbot/query and POST /chatbot/feedback endpoints (request/response schemas)
- [ ] T117 [P] Write specs/001-ros2-module-1/contracts/rag-indexing-api.md documenting POST /rag/reindex-chapter endpoint (schema, error handling)
- [ ] T118 Create README for docs/ folder explaining Docusaurus structure, how to run locally, build, and deploy
- [ ] T119 Create README for backend/ folder explaining FastAPI setup, environment variables, running tests, deployment options

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements, security hardening, and comprehensive validation

- [ ] T120 [P] Security audit: Verify no API keys in docs/, backend/ Git history (use git-secrets)
- [ ] T121 [P] Security: Verify environment variables template (.env.example) has no real secrets, only placeholders
- [ ] T122 [P] Code cleanup: Run pylint on backend/src/, fix any high-priority linting errors
- [ ] T123 [P] Documentation: Update any outdated comments in code, ensure docstrings present for public functions
- [ ] T124 Performance audit: Check chatbot response times, verify <1s p90 latency (measure via backend logging)
- [ ] T125 Accessibility: Test Docusaurus site with screen reader (VoiceOver/NVDA), verify alt text on all images/diagrams
- [ ] T126 Cross-browser testing: Verify Docusaurus site renders correctly in Chrome, Firefox, Safari, Edge
- [ ] T127 Mobile testing: Verify chatbot widget and book content responsive on mobile devices
- [ ] T128 Run final validation: All 3 chapters complete, all acceptance scenarios pass, chatbot accuracy >90%, site deployed and live

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup - CRITICAL BLOCKER for all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - Independent from US2/US3
- **User Story 2 (Phase 4)**: Depends on Foundational + US1 (optional, for integration) - Independent from US3
- **User Story 3 (Phase 5)**: Depends on Foundational - Independent from US1/US2
- **Chatbot Accuracy (Phase 6)**: Depends on all 3 user stories (Phases 3, 4, 5)
- **Frontend Build & Deployment (Phase 7)**: Depends on Phases 3, 4, 5 (content complete)
- **Documentation (Phase 8)**: Depends on Phases 1-7 (to document actual structure)
- **Polish (Phase 9)**: Depends on all previous phases

### Within Each User Story

- Content outline → write sections → embed diagrams → create exercises → RAG indexing → chatbot integration → validation

### Parallel Opportunities

**Phase 1 (Setup) - All marked [P]**:
- T003, T004, T005, T006, T007, T008, T009 can run in parallel (different files/components)

**Phase 2 (Foundational) - Mixed parallelization**:
- T012-T023 can run in parallel (different modules: models, services, components)

**Phases 3, 4, 5 (User Stories) - All independent**:
- User Story 1 (T026-T047) can run fully in parallel with US2 and US3 if team has capacity
- Within each story:
  - Content writing tasks (T027-T030, T049-T052, T068-T072) can run in parallel
  - Code example/diagram tasks can run in parallel

**Phase 6 & 7**: Can run in parallel after all user stories complete

---

## Parallel Example: User Story 3

```bash
# All content writing tasks can run in parallel (different sections):
Task T069: Write rclpy intro section
Task T070: Write subscriber section
Task T071: Write publisher section
Task T072: Write URDF section
Task T073: Write system design section

# All code examples can run in parallel (different files):
Task T073a: Write subscriber example
Task T073b: Write publisher example
Task T073c: Write combined example
Task T074: Write URDF example

# All diagram tasks can run in parallel (different diagrams):
Task T076: Create rclpy lifecycle diagram
Task T076: Create URDF tree diagram
Task T076: Create control loop diagram
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2 + 3)

**Rationale**: All 3 user stories are P1 (foundational) and independent. For a book, delivering all 3 chapters together makes pedagogical sense.

1. **Complete Phase 1**: Setup (10 tasks, ~2-3 days)
2. **Complete Phase 2**: Foundational infrastructure (15 tasks, ~3-4 days)
3. **Complete Phases 3-5 in parallel**: All 3 user stories authoring/integration (60+ tasks, ~5-7 days if team of 3)
4. **Complete Phase 6**: Chatbot accuracy validation (6 tasks, ~1-2 days)
5. **Complete Phase 7**: Build & deploy (15+ tasks, ~2-3 days)
6. **Complete Phases 8-9**: Documentation & polish (20+ tasks, ~2-3 days)

**Total**: MVP complete in 2-3 weeks with team of 3

### Single Developer Sequential Approach

1. Phases 1-2 (Setup + Foundational) - 4-5 days
2. Phase 3 (User Story 1) - 5-7 days
3. Phase 4 (User Story 2) - 5-7 days
4. Phase 5 (User Story 3) - 5-7 days
5. Phases 6-9 (Validation + deployment + polish) - 5-7 days

**Total**: 4-5 weeks solo

### Team Parallelization (3+ developers)

1. **Team together**: Phases 1-2 (Setup + Foundational) - 2-3 days
2. **Split team**:
   - Developer A: Phase 3 (User Story 1)
   - Developer B: Phase 4 (User Story 2)
   - Developer C: Phase 5 (User Story 3)
   - Backend specialist: Phase 6 (Chatbot accuracy)
3. **Team together**: Phases 7-9 (Deployment + documentation + polish) - 2-3 days

**Total**: 2-3 weeks with team

---

## Task Metrics

- **Total tasks**: 128 tasks
- **Phase 1 (Setup)**: 10 tasks
- **Phase 2 (Foundational)**: 15 tasks
- **Phase 3 (US1)**: 22 tasks (content + RAG + chatbot + validation)
- **Phase 4 (US2)**: 19 tasks (content + RAG + chatbot + validation)
- **Phase 5 (US3)**: 27 tasks (content + code examples + URDF + RAG + chatbot + validation)
- **Phase 6 (Chatbot accuracy)**: 6 tasks
- **Phase 7 (Deployment)**: 14 tasks
- **Phase 8 (Documentation)**: 6 tasks
- **Phase 9 (Polish)**: 9 tasks

---

## Notes

- [P] tasks = different files/modules, can run in parallel (no conflicts)
- [Story] label (US1, US2, US3) maps task to user story for traceability
- Each user story is independently completable and testable at its checkpoint
- Verify content meets acceptance scenarios from spec.md before marking complete
- Commit after each task or logical group (e.g., after all diagrams for a chapter)
- Stop at any checkpoint to validate story independently and get feedback
- Constitution principles guide all work: Content-First (chapters are source of truth), Test-First (validation at each step), Modular (each component independent)
