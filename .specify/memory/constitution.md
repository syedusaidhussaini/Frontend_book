# AI-Spec-Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Content-First Architecture
The book content (Markdown) is the source of truth for all chatbot knowledge. Every chatbot response MUST be traceable to specific book sections. No knowledge outside the published book is acceptable. Book updates automatically propagate to the RAG vector database within 24 hours.

### II. Spec-Driven Development (SDD)
All features start with a specification that defines objectives, success criteria, constraints, and acceptance tests. Implementation follows from approved specs. Architectural decisions are documented in ADRs linked from specs. No code is written before spec approval.

### III. Modular Stack Architecture
Each component (frontend, backend, vector DB, auth) is independently deployable and testable:
- **Frontend**: Docusaurus static site with embedded React chatbot component
- **Backend**: FastAPI microservice for RAG orchestration, with OpenAI Agents/ChatKit
- **Vector DB**: Qdrant Cloud (free tier) for semantic search over book content
- **Persistence**: Neon Serverless Postgres for conversation history and metadata

Each module has its own test suite, CI/CD, and deployment pipeline.

### IV. Test-First (NON-NEGOTIABLE)
Red-Green-Refactor cycle enforced: Write tests → Get approval → Tests fail → Implement → Tests pass.
Integration tests required for: chatbot queries against book content, API endpoints, vector DB indexing, user authentication.

### V. Observability & Traceability
All chatbot responses include:
- Source citations (exact book section + page/line reference)
- Confidence scores for semantic matches
- Query embedding and retrieval metadata
- User feedback loops (thumbs up/down for response quality)

Structured logging and metrics dashboards track chatbot accuracy, latency, and error rates.

### VI. Security & Privacy by Design
- Authentication: OpenAI API key management via environment variables (never committed)
- User data: Conversation history stored securely in Neon Postgres with encryption at rest
- Access control: Public book content via GitHub Pages; private admin panel for book updates
- No sensitive data (API keys, credentials) in Markdown files or Git history

### VII. Simplicity & YAGNI
Start with the minimum viable book (chapters only, no advanced features). Add complexity only when proven necessary:
- MVP: Single chatbot instance per book
- Constraint: Book content ONLY (no external web search)
- Future (deferred): Multi-language support, custom fine-tuning, advanced filtering

## Technology Stack Requirements

**Mandatory**:
- Frontend: Docusaurus 3.x, React, TypeScript
- Backend: FastAPI (Python 3.10+), OpenAI Agents/ChatKit SDK
- Vector DB: Qdrant Cloud (free tier, up to 1M vectors)
- Database: Neon Serverless Postgres (free tier)
- Deployment: GitHub Pages (static frontend), Vercel/Railway (serverless backend)
- Format: Markdown for all content; YAML for metadata

**Forbidden**:
- Model fine-tuning (use foundation models only)
- Mobile-specific apps (responsive web only)
- Third-party search engines or scrapers
- Hardcoded API keys in code

## Development Workflow

1. **Feature Request** → Spec written and approved (see `/sp.specify`)
2. **Specification Phase** → Architecture planned, ADR created if needed (see `/sp.plan`)
3. **Task Generation** → Testable tasks defined with acceptance criteria (see `/sp.tasks`)
4. **Red Phase** → Tests written, fail-fast validation
5. **Green Phase** → Minimal implementation to pass tests
6. **Refactor Phase** → Code polish, documentation, performance optimization
7. **Code Review** → All PRs reviewed against constitution principles
8. **Deployment** → Automated builds and deploys to staging; manual promotion to production

Every PR MUST reference a spec and at least one PHR (Prompt History Record).

## Governance

- **Constitution Authority**: This constitution supersedes all other practices. Conflicts resolved in favor of these principles.
- **Amendment Process**: Changes to principles require documented rationale, impact analysis on existing specs/tasks, and ADR if architecturally significant. User approval required before implementation.
- **Compliance Verification**: All PRs/commits checked via CI against spec alignment, test coverage (>80%), and principle adherence.
- **Review Cadence**: Constitution reviewed quarterly or when triggered by major architectural changes.
- **Guidance Reference**: See `CLAUDE.md` for Spec-Driven Development workflow and runtime guidance for agents.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
