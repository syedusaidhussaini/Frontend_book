---
id: 001
title: Create AI Book Chatbot Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: master
user: saad
command: /sp.constitution
labels: ["constitution", "project-setup", "rag-chatbot", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - Constitution created with 7 core principles
  - Technology stack requirements documented
  - Development workflow defined
  - Governance rules established
---

## Prompt

/sp.constitution AI-Spec-Driven Technical Book with Embedded RAG Chatbot

Target audience:
- Developers, AI learners, and technical readers

Objectives:
- Write a full technical book using Spec-Kit Plus and Claude Code
- Publish the book with Docusaurus and deploy to GitHub Pages
- Build and embed a RAG chatbot inside the book

Success criteria:
- Book fully written in Markdown and rendered in Docusaurus
- Deployed successfully on GitHub Pages
- Embedded chatbot answers questions using only book content
- Supports "answer from selected text only" mode
- Uses OpenAI Agents/ChatKit, FastAPI, Qdrant Cloud, and Neon Postgres

Constraints:
- Format: Markdown
- Frontend: Docusaurus
- Backend: FastAPI
- Vector DB: Qdrant Cloud (Free tier)
- Database: Neon Serverless Postgres
- AI SDKs: OpenAI Agents / ChatKit

Not building:
- Generic chatbot unrelated to book content
- Model fine-tuning
- Mobile applications

## Response snapshot

Created AI-Spec-Driven Technical Book with Embedded RAG Chatbot Constitution (v1.0.0) with 7 core principles:
1. Content-First Architecture - book is source of truth for chatbot
2. Spec-Driven Development - all features start with approved specifications
3. Modular Stack Architecture - independent frontend, backend, vector DB, persistence
4. Test-First (NON-NEGOTIABLE) - Red-Green-Refactor cycle enforced
5. Observability & Traceability - source citations, confidence scores, user feedback
6. Security & Privacy by Design - env vars, encrypted storage, no hardcoded secrets
7. Simplicity & YAGNI - MVP-first approach, deferred advanced features

Technology stack documented (mandatory: Docusaurus 3.x, FastAPI, Qdrant Cloud, Neon Postgres; forbidden: fine-tuning, mobile apps, hardcoded keys).
Development workflow defined (spec → plan → tasks → red/green/refactor → review → deploy).
Governance established with amendment procedure, compliance verification, quarterly review cadence.

## Outcome

- ✅ Impact: Establishes foundational governance for the entire project; aligns all future specs, plans, and implementations with constitution principles
- 🧪 Tests: Constitution syntax validated; all placeholder tokens replaced; version 1.0.0 ratified
- 📁 Files: .specify/memory/constitution.md created; history/prompts/constitution/ structure initialized
- 🔁 Next prompts: /sp.specify for first feature (book structure), /sp.plan for RAG architecture, /sp.tasks for implementation sprints
- 🧠 Reflection: Constitution balances prescriptive principles (Content-First, Test-First, Security) with flexibility for implementation details; strongly emphasizes SDD workflow alignment

## Evaluation notes (flywheel)

- Failure modes observed: None; constitution follows template structure perfectly
- Graders run and results (PASS/FAIL): PASS — all 7 principles align with project objectives; technology stack constraints honored; governance rules clear and enforceable
- Prompt variant (if applicable): null
- Next experiment: Validate constitution against first feature spec to ensure consistency (test spec template compliance with constitution principles)
