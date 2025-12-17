# Implementation Plan: Module 1 - The Robotic Nervous System (ROS 2)

**Branch**: `001-ros2-module-1` | **Date**: 2025-12-16 | **Spec**: `specs/001-ros2-module-1/spec.md`
**Input**: Feature specification from `/specs/001-ros2-module-1/spec.md`

## Summary

**Primary Requirement**: Author 3 chapters (Introduction to ROS 2, Core Concepts, Python Integration) teaching AI/robotics students how ROS 2 middleware connects Python agents to humanoid robots.

**Technical Approach**:
- Publish chapters as Markdown in Docusaurus 3.x frontend
- Organize content by learning objectives (foundation → concepts → hands-on)
- Structure for RAG chatbot: each section is independently indexable and citable
- Create visual diagrams (SVG) for architecture, communication patterns, and URDF hierarchy
- Include Python code examples using rclpy for student reference

## Technical Context

**Language/Version**: Markdown (content authoring); Python 3.10+ for code examples and chatbot backend
**Primary Dependencies**: Docusaurus 3.x (frontend), FastAPI + OpenAI Agents/ChatKit (backend chatbot), Qdrant (vector DB), Neon Postgres (persistence)
**Storage**: Markdown files in Git (book chapters); Neon Postgres for conversation history; Qdrant for semantic indexing
**Testing**: pytest for chatbot integration tests; manual review for content accuracy and pedagogical flow
**Target Platform**: Web (Docusaurus static site + embedded React chatbot)
**Project Type**: Documentation + fullstack web app (book + chatbot)
**Performance Goals**: Chatbot response time <1s for 90th percentile queries; support 100 concurrent chatbot users (free tier Qdrant/Neon constraints)
**Constraints**: Use free tiers of Qdrant Cloud and Neon Postgres; no model fine-tuning; book-content-only (no external web search)
**Scale/Scope**: 3 chapters (~15-20k words total); 7+ visual diagrams; 15-20 code examples; responsive web UI

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**✅ Content-First Architecture**:
- Book chapters (Markdown) are source of truth for chatbot knowledge
- Each section will include metadata for RAG indexing (section ID, heading hierarchy, source citations)
- Chatbot responses will always cite exact chapter/section/line

**✅ Spec-Driven Development**:
- 3 user stories define what students learn; chapters organized around these outcomes
- All functional requirements map to testable chapter content (FR-001 to FR-007)
- Success criteria include learning validation (quizzes) and product metric (chatbot accuracy >90%)

**✅ Modular Stack Architecture**:
- Frontend: Docusaurus serves static HTML + embeds React chatbot component (independent)
- Backend: FastAPI service handles RAG orchestration, isolated from frontend (independent)
- Vector DB: Qdrant Cloud indexes chapter content; can be reindexed independently
- Persistence: Neon Postgres stores conversations; independent from vector DB

**✅ Test-First (NON-NEGOTIABLE)**:
- Phase 2 tasks will include content validation tests (acceptance scenarios from spec)
- Chatbot integration tests: query chapter content, verify source citations accurate
- Code example tests: Python rclpy examples must be syntactically valid

**✅ Observability & Traceability**:
- Each chapter section tagged with unique ID for chatbot source citations
- Success criteria SC-005 requires chatbot answer accuracy >90% using only chapter text
- User feedback (thumbs up/down) will track chatbot response quality per query

**✅ Security & Privacy by Design**:
- No API keys or credentials in Markdown or Git
- Book content is public; chatbot uses only publicly available chapters
- Conversation history encryption handled by Neon Postgres encryption at rest

**✅ Simplicity & YAGNI**:
- MVP: 3 chapters, single chatbot instance, no multi-language or advanced features
- Deferred: chapter translations, fine-tuned models, advanced retrieval

**Gate Status**: ✅ PASS — All constitution principles validated for Module 1 content + chatbot integration

## Project Structure

### Documentation (this feature)

```text
specs/001-ros2-module-1/
├── spec.md                              # Feature specification
├── plan.md                              # This file (architecture & design)
├── research.md                          # Phase 0 research findings (TBD)
├── data-model.md                        # Phase 1 content structure (TBD)
├── contracts/                           # Phase 1 RAG API contracts (TBD)
│   ├── chatbot-api.md
│   └── rag-indexing-api.md
├── quickstart.md                        # Phase 1 getting started (TBD)
├── checklists/
│   └── requirements.md                  # Quality validation (completed)
└── tasks.md                             # Phase 2 implementation tasks (TBD)
```

### Source Code - Module 1 Book Content (repository root)

```text
docs/
├── sidebars.js                          # Docusaurus sidebar navigation
├── docs/
│   ├── introduction.md                  # Docusaurus docs intro
│   └── modules/
│       └── 01-ros2-fundamentals/
│           ├── _category_.json          # Chapter grouping metadata
│           ├── 01-introduction.md       # Chapter 1: Intro to ROS 2
│           ├── 02-core-concepts.md      # Chapter 2: Nodes/Topics/Services
│           ├── 03-python-integration.md # Chapter 3: rclpy & URDF
│           ├── code-examples/
│           │   ├── ros2-node-subscriber.py
│           │   ├── ros2-node-publisher.py
│           │   └── urdf-example.xml
│           └── diagrams/
│               ├── ros2-architecture.svg
│               ├── pubsub-topology.svg
│               └── urdf-hierarchy.svg
├── src/
│   ├── css/
│   │   └── custom.css                   # Custom Docusaurus styling
│   └── components/
│       └── ChatbotWidget.tsx            # Embedded React chatbot
├── static/
│   └── img/                             # Static images for chapters
└── docusaurus.config.js                 # Docusaurus configuration

backend/
├── src/
│   ├── main.py                          # FastAPI entry point
│   ├── models/
│   │   ├── query.py                     # Query/response models
│   │   └── citation.py                  # Citation models for source tracking
│   ├── services/
│   │   ├── rag_service.py               # RAG orchestration (Qdrant + OpenAI)
│   │   ├── embedding_service.py         # Text embedding generation
│   │   └── context_service.py           # Conversation context management
│   ├── api/
│   │   ├── routers/
│   │   │   └── chatbot.py               # POST /chatbot/query, feedback endpoints
│   │   └── middleware.py                # CORS, logging, auth
│   └── config.py                        # Environment vars, settings
├── tests/
│   ├── unit/
│   │   ├── test_embedding_service.py
│   │   └── test_context_service.py
│   ├── integration/
│   │   └── test_rag_queries.py          # Test chatbot against chapter content
│   └── conftest.py                      # pytest fixtures
├── requirements.txt                     # Python dependencies
└── .env.example                         # Environment variables template

chatbot-frontend/
├── src/
│   ├── components/
│   │   ├── ChatWindow.tsx               # Chat UI component
│   │   ├── MessageBubble.tsx            # Message display
│   │   └── CitationDisplay.tsx          # Show chapter/section source
│   ├── hooks/
│   │   └── useChatbot.ts                # React hook for API communication
│   ├── services/
│   │   └── chatbotApi.ts                # API client (fetch)
│   └── App.tsx                          # Root component
├── tests/
│   └── ChatWindow.test.tsx              # React component tests
└── package.json                         # Node dependencies
```

**Structure Decision**: Web application with embedded chatbot:
- **Docusaurus frontend** (HTML/JS static site) = primary book interface, deployed to GitHub Pages
- **Embedded React chatbot component** = chatbot UI integrated into Docusaurus pages
- **FastAPI backend** = RAG orchestration (query → Qdrant embedding search → OpenAI → response), deployed to Vercel/Railway
- **Qdrant Cloud + Neon Postgres** = vector DB + conversation history storage
- Chapter content structured for RAG: each section tagged with unique ID, hierarchical headings, embedded code examples

## Complexity Tracking

> **No Constitution Check violations** — All requirements align with modular stack and simplicity principles. Structure selected is minimal viable design.

## Phase 0: Research & Clarification

**Output**: `research.md` with resolved decisions

**Key Research Areas**:

1. **Docusaurus 3.x Setup**
   - Decision: Use Docusaurus 3.x with TypeScript support
   - Rationale: Industry standard for technical documentation; built-in Markdown support; easy React component embedding for chatbot
   - Alternatives: MkDocs (simpler but less flexible), Hugo (Go-based, less JS ecosystem)

2. **RAG Architecture**
   - Decision: Qdrant Cloud (free tier) + OpenAI embeddings + GPT-4 for responses
   - Rationale: Free tier supports up to 1M vectors (sufficient for 3 chapters); easy semantic search; works with ChatKit
   - Alternatives: Pinecone (paid), Weaviate (self-hosted complexity)

3. **Chapter Organization for RAG Indexing**
   - Decision: Each section tagged with unique ID (e.g., `1.2.3` for Chapter 1, Section 2, Subsection 3); metadata includes heading, author, date
   - Rationale: Enables precise citation in chatbot responses; supports section-level queries; hierarchical retrieval
   - Alternatives: Doc-level indexing (loses precision), flat indexing (loses hierarchy)

4. **Python Code Examples**
   - Decision: rclpy examples use DDS QoS profiles, real sensor topic names (IMU, camera); companion nodes as working references
   - Rationale: Students can test against real robots/simulators; teaches production patterns, not toy code
   - Alternatives: Pseudocode only (less practical), Rust/C++ examples (language mismatch)

5. **Diagram Format**
   - Decision: SVG (scalable) + embedded in Markdown using HTML img tags
   - Rationale: Version-controllable, responsive, searchable text; no binary image overhead
   - Alternatives: PNG/JPG (non-scalable, harder to update), Mermaid diagrams (limited for architecture visuals)

## Phase 1: Design & Contracts

**Prerequisites**: Phase 0 research complete

### 1.1 Data Model

**Output**: `data-model.md`

**Key Entities**:

- **Chapter**: Title, UUID, created_date, updated_date, markdown_path, word_count
- **Section**: Chapter_ID, section_number (1.1, 1.2, etc.), heading, markdown_range (start_line, end_line), embedding_id (in Qdrant)
- **CodeExample**: Section_ID, language, code_snippet, line_numbers, description
- **Diagram**: Section_ID, filename, alt_text, description
- **Conversation**: ID, user_id, created_at, messages[], feedback[] (thumbs up/down per response)
- **Message**: role (user|assistant), content, embedding_id, cited_sections[] (for source tracking)

**Validation Rules**:
- Chapter UUID must be globally unique
- Section numbers must be sequential within chapter
- All markdown_path must be relative to docs/ directory
- Citations must reference valid Section IDs with line ranges
- Embedding IDs must exist in Qdrant at time of chatbot response

### 1.2 API Contracts

**Output**: `/contracts/chatbot-api.md`, `/contracts/rag-indexing-api.md`

**Chatbot Query Endpoint** (`POST /chatbot/query`):

```
Request:
  - query: string (user question)
  - conversation_id: string (optional; for context)
  - selected_sections: [Section IDs] (optional; "answer from these sections only")

Response:
  - answer: string (chatbot response)
  - citations: [{section_id, chapter_title, heading, line_range, confidence_score}]
  - conversation_id: string
  - retrieval_metadata: {embedding_similarity, search_time_ms}
```

**RAG Indexing Endpoint** (`POST /rag/reindex-chapter`):

```
Request:
  - chapter_id: string
  - markdown_content: string
  - overwrite: boolean

Response:
  - indexed_sections: count
  - qdrant_collection_update_status: "success"
  - timestamp: ISO8601
```

### 1.3 RAG Pipeline

**Workflow**:
1. **Ingest**: Chapter markdown → split by sections (H2 headings) → generate embeddings (OpenAI) → store in Qdrant with metadata
2. **Query**: User question → embed (OpenAI) → semantic search in Qdrant (top 3 sections) → prompt context → GPT-4 response → extract citations
3. **Response**: Include source chapter, section heading, exact line range for student reference

### 1.4 Quickstart

**Output**: `quickstart.md`

- How to add a new chapter (markdown + metadata)
- How to embed a code example (syntax, testing)
- How to add a diagram (SVG format, alt text)
- How to test chapter content with chatbot (manual validation)
- How to deploy (Docusaurus build → GitHub Pages, FastAPI → Vercel)

## Phase 1 - Completed Artifacts

**Constitution re-check (post-design)**: ✅ PASS
- Content-First: Each section indexed independently with unique ID; all citations traceable
- Modular: Frontend (Docusaurus) independent from backend (FastAPI); vector DB pluggable
- Test-First: Phase 2 tasks will include validation tests for each chapter against acceptance scenarios
- Observability: Embedding similarity scores, retrieval metadata, user feedback logged

## Phase 2: Tasks

**Output**: `/tasks.md` (generated by `/sp.tasks` command, NOT by this plan)

**Expected Task Categories**:
- Chapter authoring (outline → draft → review for each chapter)
- Diagram creation & validation
- Code example writing & testing
- Content → Qdrant indexing validation
- Chatbot integration testing (verify accuracy >90%)
- Docusaurus build & deployment
- Backend API deployment (FastAPI to Vercel/Railway)

---

## Critical Dependencies & Risks

### Dependencies
1. **Docusaurus 3.x** must support React 18+ for embedded chatbot component
2. **OpenAI API access** required for embeddings + GPT-4 (ensure API key stored securely in env vars)
3. **Qdrant Cloud account** must be created; free tier limits to 1M vectors
4. **Neon Postgres** account must be created; free tier sufficient for conversation history

### Risks & Mitigations
| Risk | Impact | Mitigation |
|------|--------|-----------|
| Chapter content unclear for students | Low engagement, poor learning outcomes | Use acceptance scenarios from spec to validate clarity; peer review; pilot with student cohort |
| RAG chatbot returns off-topic answers | User frustration; lost trust | Implement similarity threshold (reject low-confidence matches); user feedback loop to improve prompts |
| Vector DB quota exceeded (>1M vectors) | Chatbot stops working | Monitor vector count during indexing; archive old chapters to separate collection if needed |
| OpenAI API rate limits | Chatbot latency | Implement caching; batch queries during off-peak hours |
| Docusaurus deployment fails | Book not published | Comprehensive pre-build validation; automated testing in CI/CD |

---

## Success Metrics (per spec)

| Metric | Target | Validation |
|--------|--------|-----------|
| SC-001: Comprehension | >80% quiz pass | Quiz embedded in Chapter 1 & 2; auto-scored |
| SC-002: Architecture design | >75% rubric | Exercise in Chapter 2; manual or rubric-based scoring |
| SC-003: Code exercise | Functional | Python rclpy node exercise in Chapter 3; syntax + logic validation |
| SC-004: Student satisfaction | >85% | Post-module survey (5-point Likert scale) |
| SC-005: Chatbot accuracy | >90% | Test chatbot against 50+ questions derived from chapter text; measure accuracy of citations |

---

**Approval Gate**: Ready for Phase 2 task generation (`/sp.tasks` command)
