---
id: 003
title: Plan ROS 2 Docusaurus Chatbot Architecture
stage: plan
date: 2025-12-16
surface: agent
model: claude-haiku-4-5-20251001
feature: 001-ros2-module-1
branch: 001-ros2-module-1
user: saad
command: /sp.plan
labels: ["architecture", "docusaurus", "rag", "fastapi", "frontend", "backend"]
links:
  spec: specs/001-ros2-module-1/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ros2-module-1/plan.md
tests:
  - Constitution check: all 7 principles PASS
  - Technical context: Docusaurus 3.x, FastAPI, Qdrant, Neon Postgres defined
  - Project structure: Dual frontend (Docusaurus) + backend (FastAPI) + vector DB
  - Phase 0 research: 5 key decisions (Docusaurus, RAG, indexing, code examples, diagrams)
  - Phase 1 design: Data model, API contracts, RAG pipeline, quickstart template
  - Phase 2 tasks: 7 categories (authoring, diagrams, code, indexing, testing, build, deploy)
---

## Prompt

/sp.plan Module 1 (Docusaurus)

1. Install and configure Docusaurus, create docs structure, and add Module-1 with 3 chapters.
2. Write and organize content using Docusaurus sidebar navigation.
3. All module and chapter files will be written in Markdown (.md).

## Response snapshot

Created comprehensive implementation plan for Module 1 with Docusaurus + embedded RAG chatbot:

**Architecture & Design**:
- Frontend: Docusaurus 3.x (HTML/JS static site → GitHub Pages) with embedded React chatbot component
- Backend: FastAPI (Python 3.10+, RAG orchestration) → Vercel/Railway
- Vector DB: Qdrant Cloud (free tier, up to 1M vectors)
- Persistence: Neon Postgres (free tier, conversation history + encryption at rest)

**Technical Context**:
- 3 chapters (~15-20k words total), 7+ visual diagrams, 15-20 code examples
- Performance: <1s chatbot response (p90), 100 concurrent users (free tier limits)
- Constraints: Book-content-only (no external search), no model fine-tuning

**Constitution Check**: ✅ ALL 7 PRINCIPLES PASS
- Content-First: Chapter content = source of truth; all citations traceable via Section IDs
- Spec-Driven: 3 user stories → chapters; FR-001 to FR-007 → content requirements
- Modular: Frontend, backend, vector DB, persistence all independently deployable
- Test-First: Phase 2 tasks include acceptance scenario validation + chatbot accuracy tests
- Observability: Embedding scores, retrieval metadata, user feedback (thumbs up/down)
- Security: API keys in env vars only, Postgres encryption at rest, public book content
- Simplicity: MVP-first (3 chapters), no advanced features (multi-language, fine-tuning deferred)

**Project Structure**:
```
docs/                              # Docusaurus + chapter content
  ├── docs/modules/01-ros2-fundamentals/
  │   ├── 01-introduction.md       # Chapter 1
  │   ├── 02-core-concepts.md      # Chapter 2
  │   ├── 03-python-integration.md # Chapter 3
  │   ├── code-examples/           # Python rclpy examples
  │   └── diagrams/                # SVG visualizations
  └── src/components/ChatbotWidget.tsx  # Embedded React chatbot

backend/                           # FastAPI RAG service
  ├── src/
  │   ├── services/rag_service.py  # Qdrant + OpenAI orchestration
  │   ├── api/routers/chatbot.py   # POST /chatbot/query, feedback
  │   └── models/citation.py       # Source tracking
  └── tests/integration/           # Test against chapter content

chatbot-frontend/                  # React chatbot component
  ├── src/components/
  │   ├── ChatWindow.tsx
  │   └── CitationDisplay.tsx
  └── tests/
```

**Phase 0 Research** (5 key decisions):
1. Docusaurus 3.x (rationale: industry standard, React embedding, Markdown native)
2. Qdrant Cloud + OpenAI embeddings (rationale: free tier 1M vectors, easy semantic search)
3. Section-level indexing with unique IDs (rationale: precise citations, hierarchical retrieval)
4. Production rclpy code examples (rationale: students can test on real robots/simulators)
5. SVG diagrams (rationale: scalable, version-controllable, responsive)

**Phase 1 Design**:
- Data model: Chapter, Section, CodeExample, Diagram, Conversation entities
- API contracts: POST /chatbot/query (with citations), POST /rag/reindex-chapter
- RAG pipeline: Ingest → embed → store; Query → search → prompt → cite
- Quickstart: How to add chapters, code examples, diagrams, test, deploy

**Phase 2 Tasks** (7 categories, TBD):
- Chapter authoring (outline → draft → review)
- Diagram creation
- Code example writing & testing
- Content indexing validation
- Chatbot integration testing (accuracy >90%)
- Docusaurus build & GitHub Pages deployment
- Backend API deployment (Vercel/Railway)

**Critical Risks & Mitigations**:
- Chapter clarity: Peer review + student pilot + acceptance scenario validation
- RAG off-topic: Similarity threshold + user feedback loop
- Vector quota: Monitor count + archive old chapters if needed
- API rate limits: Implement caching + batch queries
- Deployment failure: CI/CD validation + pre-build tests

**Success Metrics** (from spec):
- SC-001: >80% comprehension quiz pass rate
- SC-002: >75% architecture exercise rubric alignment
- SC-003: Functional Python rclpy code exercise
- SC-004: >85% student satisfaction
- SC-005: >90% chatbot accuracy + correct citations

**Branch**: 001-ros2-module-1
**Plan File**: specs/001-ros2-module-1/plan.md (~330 lines, comprehensive)

## Outcome

- ✅ Impact: Architecture fully designed for dual-track development: (1) content authoring for book chapters, (2) backend RAG service implementation. Clear separation of concerns enables parallel work.
- 🧪 Tests: Constitution check PASS (all 7 principles); Phase 0 research: 5 decisions documented with rationale; Phase 1 design: data model, API contracts, RAG workflow; Phase 2 task categories defined
- 📁 Files: plan.md (330+ lines with full project structure, phases, risks, metrics)
- 🔁 Next prompts: /sp.tasks to create actionable task list for chapter authoring, backend implementation, deployment
- 🧠 Reflection: Plan strongly adheres to constitution (Content-First, Modular, Test-First); architecture balances pedagogical goals (chapter clarity, hands-on code exercises) with technical goals (RAG accuracy, scalability); free tier constraints (Qdrant 1M vectors, Neon Postgres) are documented and accounted for in design choices

## Evaluation notes (flywheel)

- Failure modes observed: None - architecture is comprehensive and justified; all decisions traced to rationale
- Graders run and results (PASS/FAIL): PASS — Constitution re-check (post-design) validates all 7 principles; technical context complete; project structure clear; phase deliverables defined; risk/mitigation matrix comprehensive
- Prompt variant (if applicable): null
- Next experiment: After /sp.tasks generates task list, validate task decomposition against project structure and success metrics; ensure each task traces to at least one acceptance scenario or functional requirement
