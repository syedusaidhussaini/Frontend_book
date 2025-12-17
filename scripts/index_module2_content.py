"""
Script to index Module 2 content for the RAG system
This script will parse the Module 2 markdown files and prepare them for chatbot integration
"""
import os
import re
from typing import List, Dict, Tuple
from pathlib import Path

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

def index_module2_content():
    """
    Index all Module 2 content for RAG system
    """
    # Define the path to Module 2 content
    module2_path = Path("frontend_ai_book/docs/modules/02-digital-twin")

    # Get all markdown files in Module 2
    md_files = list(module2_path.glob("*.md"))

    all_sections = []

    for file_path in md_files:
        print(f"Processing {file_path.name}...")

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

    print(f"\nIndexed {len(all_sections)} sections from Module 2")

    # Print sample entries
    print("\nSample indexed sections:")
    for i, section in enumerate(all_sections[:3]):
        print(f"  {i+1}. ID: {section['id']}")
        print(f"     Chapter: {section['chapter']}")
        print(f"     Title: {section['title']}")
        print(f"     Content preview: {section['content'][:100]}...")
        print()

    return all_sections

if __name__ == "__main__":
    indexed_sections = index_module2_content()
    print(f"Successfully indexed {len(indexed_sections)} sections for Module 2 chatbot integration")