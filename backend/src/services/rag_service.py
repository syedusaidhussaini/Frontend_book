"""
RAG (Retrieval-Augmented Generation) Service
Handles semantic search over book content and LLM integration
"""

import logging
import re
from typing import List, Optional, Dict, Tuple
from models.query import Citation, RetrievalMetadata
import os
from pathlib import Path

logger = logging.getLogger(__name__)


def split_markdown_by_sections(content: str) -> List[Dict[str, str]]:
    """
    Split markdown content by H2 headings to create searchable sections
    """
    sections = []

    # Split by H2 headings (##)
    h2_pattern = r'^##\s+(.+)$'
    lines = content.split('\n')

    current_section = {'title': 'Introduction', 'content': []}

    for line in lines:
        if re.match(h2_pattern, line.strip()):
            # Save previous section
            if current_section['content']:
                sections.append({
                    'title': current_section['title'],
                    'content': '\n'.join(current_section['content']).strip()
                })

            # Start new section
            title = re.match(h2_pattern, line.strip()).group(1)
            current_section = {'title': title, 'content': [line]}
        else:
            current_section['content'].append(line)

    # Add the last section
    if current_section['content']:
        sections.append({
            'title': current_section['title'],
            'content': '\n'.join(current_section['content']).strip()
        })

    return sections


class RAGService:
    """Service for RAG operations (retrieve + generate)"""

    def __init__(self):
        """Initialize RAG service with Qdrant and OpenAI clients"""
        # TODO: Initialize Qdrant client
        # TODO: Initialize OpenAI client

        # For now, load the Module 2 content into memory
        self.indexed_content = self._load_module2_content()
        logger.info("RAG Service initialized with Module 2 content")

    def _load_module2_content(self) -> List[Dict]:
        """
        Load Module 2 content into memory for basic search functionality
        This is a temporary implementation until Qdrant is set up
        """
        module2_path = Path("../../frontend_ai_book/docs/modules/02-digital-twin")
        all_sections = []

        if module2_path.exists():
            md_files = list(module2_path.glob("*.md"))

            for file_path in md_files:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Skip frontmatter (content between ---)
                lines = content.split('\n')
                content_without_frontmatter = []
                in_frontmatter = False
                for line in lines:
                    if line.strip() == '---':
                        in_frontmatter = not in_frontmatter
                        continue
                    if not in_frontmatter:
                        content_without_frontmatter.append(line)

                content = '\n'.join(content_without_frontmatter)

                # Split content into sections
                sections = split_markdown_by_sections(content)

                # Create section entries with IDs
                for i, section in enumerate(sections):
                    section_id = f"mod2-{file_path.stem.replace('-', '_')}-{i+1}"
                    section_entry = {
                        'id': section_id,
                        'chapter': file_path.stem,
                        'title': section['title'],
                        'content': section['content'],
                        'file_path': str(file_path)
                    }
                    all_sections.append(section_entry)

        logger.info(f"Loaded {len(all_sections)} sections from Module 2")
        return all_sections

    def _load_module3_content(self) -> List[Dict]:
        """
        Load Module 3 content into memory for basic search functionality
        This is a temporary implementation until Qdrant is set up
        """
        module3_path = Path("../../frontend_ai_book/docs/modules/03-isaac-ecosystem")
        all_sections = []

        if module3_path.exists():
            md_files = list(module3_path.glob("*.md"))

            for file_path in md_files:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Skip frontmatter (content between ---)
                lines = content.split('\n')
                content_without_frontmatter = []
                in_frontmatter = False
                for line in lines:
                    if line.strip() == '---':
                        in_frontmatter = not in_frontmatter
                        continue
                    if not in_frontmatter:
                        content_without_frontmatter.append(line)

                content = '\n'.join(content_without_frontmatter)

                # Split content into sections
                sections = split_markdown_by_sections(content)

                # Create section entries with IDs
                for i, section in enumerate(sections):
                    section_id = f"mod3-{file_path.stem.replace('-', '_')}-{i+1}"
                    section_entry = {
                        'id': section_id,
                        'chapter': file_path.stem,
                        'title': section['title'],
                        'content': section['content'],
                        'file_path': str(file_path)
                    }
                    all_sections.append(section_entry)

        logger.info(f"Loaded {len(all_sections)} sections from Module 3")
        return all_sections

    async def query_rag_index(
        self,
        query: str,
        selected_sections: Optional[List[str]] = None,
        top_k: int = 3,
    ) -> Tuple[str, List[Citation], RetrievalMetadata]:
        """
        Query the RAG index and generate response.

        Process:
        1. Embed the user query
        2. Search Qdrant for top_k similar sections
        3. Build prompt context from retrieved sections
        4. Call GPT-4 to generate response
        5. Extract and format citations

        Args:
            query: User question
            selected_sections: Optional list of section IDs to restrict search
            top_k: Number of top sections to retrieve

        Returns:
            Tuple of (answer, citations, retrieval_metadata)
        """
        try:
            logger.info(f"RAG query: {query[:50]}...")

            # TODO: Implement
            # 1. embed_query = await self.embedding_service.embed_text(query)
            # 2. results = await self.qdrant_client.search(embedding, top_k)
            # 3. if selected_sections: filter results to only selected_sections
            # 4. context = format_search_results(results)
            # 5. answer = await self.call_llm(query, context)
            # 6. citations = extract_citations(answer, results)

            # Placeholder
            return (
                "Placeholder answer",
                [],
                RetrievalMetadata(
                    embedding_similarity=0.0,
                    search_time_ms=0,
                    num_contexts_retrieved=0,
                ),
            )

        except Exception as e:
            logger.error(f"Error querying RAG index: {str(e)}", exc_info=e)
            raise

    async def ingest_chapter(self, chapter_id: str, markdown_content: str) -> Dict:
        """
        Ingest a new chapter into the RAG index.

        Process:
        1. Split markdown by H2 sections
        2. Tag each section with unique ID (1.1, 1.2, etc.)
        3. Generate embeddings for each section
        4. Store in Qdrant with metadata
        5. Store in Postgres for reference

        Args:
            chapter_id: Chapter identifier
            markdown_content: Full markdown content of chapter

        Returns:
            Dict with indexing results
        """
        try:
            logger.info(f"Ingesting chapter {chapter_id}")

            # TODO: Implement
            # 1. sections = split_markdown_by_headings(markdown_content)
            # 2. for section in sections:
            #       section_id = generate_id(chapter_id, section.index)
            #       embedding = await embedding_service.embed_text(section.content)
            #       await qdrant_client.upload_point(section_id, embedding, metadata)
            # 3. await postgres.store_chapter_metadata(chapter_id, sections)

            return {
                "status": "success",
                "chapter_id": chapter_id,
                "sections_indexed": 0,
            }

        except Exception as e:
            logger.error(f"Error ingesting chapter: {str(e)}", exc_info=e)
            raise


# Global service instance
rag_service = RAGService()
