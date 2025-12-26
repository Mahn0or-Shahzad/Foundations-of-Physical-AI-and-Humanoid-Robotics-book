"""
Document Loader for Physical AI Book
Loads markdown files from ../docs and prepares them for embedding
"""

import os
from pathlib import Path
from typing import List, Dict
import re


class Document:
    """Represents a document chunk with metadata"""
    def __init__(self, content: str, metadata: Dict):
        self.content = content
        self.metadata = metadata

    def __repr__(self):
        return f"Document(content_length={len(self.content)}, metadata={self.metadata})"


def load_docs() -> List[Document]:
    """
    Load all markdown files from ../docs directory.

    Returns:
        List of Document objects with content and metadata
    """
    docs = []
    docs_path = Path(__file__).parent.parent / "docs"

    if not docs_path.exists():
        raise FileNotFoundError(f"Docs directory not found: {docs_path}")

    print(f"Loading documents from: {docs_path}")

    # Walk through all markdown files
    for md_file in docs_path.rglob("*.md"):
        try:
            # Read file content
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract metadata from frontmatter
            metadata = extract_frontmatter(content)

            # Get relative path for reference
            relative_path = md_file.relative_to(docs_path)
            metadata['file_path'] = str(relative_path)
            metadata['full_path'] = str(md_file)

            # Remove frontmatter from content
            clean_content = remove_frontmatter(content)

            # Create document chunks (split by sections)
            chunks = split_into_chunks(clean_content, metadata)
            docs.extend(chunks)

            print(f"  ✓ Loaded: {relative_path} ({len(chunks)} chunks)")

        except Exception as e:
            print(f"  ✗ Error loading {md_file}: {e}")

    print(f"\nTotal documents loaded: {len(docs)}")
    return docs


def extract_frontmatter(content: str) -> Dict:
    """
    Extract YAML frontmatter from markdown file.

    Args:
        content: Raw markdown content

    Returns:
        Dictionary of frontmatter metadata
    """
    metadata = {}

    # Match frontmatter between --- delimiters
    frontmatter_pattern = r'^---\s*\n(.*?)\n---\s*\n'
    match = re.match(frontmatter_pattern, content, re.DOTALL)

    if match:
        frontmatter_text = match.group(1)

        # Parse simple YAML (key: value pairs)
        for line in frontmatter_text.split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                metadata[key.strip()] = value.strip()

    return metadata


def remove_frontmatter(content: str) -> str:
    """
    Remove YAML frontmatter from markdown content.

    Args:
        content: Raw markdown with frontmatter

    Returns:
        Clean markdown content
    """
    frontmatter_pattern = r'^---\s*\n.*?\n---\s*\n'
    clean_content = re.sub(frontmatter_pattern, '', content, flags=re.DOTALL)
    return clean_content.strip()


def split_into_chunks(content: str, base_metadata: Dict, chunk_size: int = 2000) -> List[Document]:
    """
    Split markdown content into semantic chunks.

    Strategy:
    - Split by ## headings (sections)
    - If section too large, split by paragraphs
    - Maintain context by including heading in chunk

    Args:
        content: Markdown content (without frontmatter)
        base_metadata: Metadata from frontmatter
        chunk_size: Target characters per chunk

    Returns:
        List of Document objects
    """
    chunks = []

    # Split by level 2 headings (##)
    sections = re.split(r'\n## ', content)

    for i, section in enumerate(sections):
        if not section.strip():
            continue

        # Add ## back except for first section
        if i > 0:
            section = '## ' + section

        # Extract section heading
        heading_match = re.match(r'##\s+(.+)', section)
        section_title = heading_match.group(1) if heading_match else "Introduction"

        # Create metadata for this chunk
        chunk_metadata = base_metadata.copy()
        chunk_metadata['section'] = section_title
        chunk_metadata['chunk_index'] = i

        # If section is small enough, use as-is
        if len(section) <= chunk_size:
            chunks.append(Document(content=section, metadata=chunk_metadata))
        else:
            # Split large sections by paragraphs
            paragraphs = section.split('\n\n')
            current_chunk = ""

            for paragraph in paragraphs:
                if len(current_chunk) + len(paragraph) <= chunk_size:
                    current_chunk += paragraph + '\n\n'
                else:
                    if current_chunk:
                        chunks.append(Document(content=current_chunk.strip(), metadata=chunk_metadata))
                    current_chunk = paragraph + '\n\n'

            # Add remaining content
            if current_chunk.strip():
                chunks.append(Document(content=current_chunk.strip(), metadata=chunk_metadata))

    return chunks


if __name__ == "__main__":
    # Test the document loader
    print("=" * 60)
    print("Testing Document Loader")
    print("=" * 60)
    print()

    docs = load_docs()

    # Show sample
    if docs:
        print("\nSample document:")
        print(f"Content preview: {docs[0].content[:200]}...")
        print(f"Metadata: {docs[0].metadata}")
