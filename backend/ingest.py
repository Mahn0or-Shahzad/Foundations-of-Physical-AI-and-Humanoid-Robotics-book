"""
Document Ingestion Pipeline
Orchestrates loading docs, generating embeddings, and storing in Qdrant
"""

from document_loader import load_docs, Document
from embeddings import EmbeddingsManager
from typing import List
import time


def ingest_book_content():
    """
    Main ingestion pipeline:
    1. Load all markdown files from ../docs
    2. Generate embeddings for each chunk
    3. Store in Qdrant vector database
    """
    print("=" * 60)
    print("Physical AI Book Content Ingestion Pipeline")
    print("=" * 60)
    print()

    # Step 1: Load documents
    print("Step 1: Loading markdown documents...")
    print("-" * 60)
    documents = load_docs()
    print(f"\n✓ Loaded {len(documents)} document chunks\n")

    if not documents:
        print("✗ No documents found. Check ../docs directory.")
        return

    # Step 2: Initialize embeddings manager
    print("Step 2: Initializing embeddings manager...")
    print("-" * 60)
    try:
        embeddings_manager = EmbeddingsManager()
        print()
    except Exception as e:
        print(f"\n✗ Failed to initialize: {e}")
        print("Check your .env file and API keys.")
        return

    # Step 3: Generate embeddings
    print("Step 3: Generating embeddings...")
    print("-" * 60)
    print("⚠ This may take several minutes depending on document count...")

    try:
        # Extract text content from documents
        texts = [doc.content for doc in documents]

        # Batch processing (OpenAI allows up to 2048 texts per request)
        batch_size = 100
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            print(f"  Processing batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1} ({len(batch)} texts)...")

            batch_embeddings = embeddings_manager.generate_embeddings(batch)
            all_embeddings.extend(batch_embeddings)

            # Rate limiting (avoid API throttling)
            time.sleep(0.5)

        print(f"\n✓ Generated {len(all_embeddings)} embeddings\n")

    except Exception as e:
        print(f"\n✗ Error generating embeddings: {e}")
        return

    # Step 4: Store in Qdrant
    print("Step 4: Storing embeddings in Qdrant...")
    print("-" * 60)

    try:
        embeddings_manager.store_embeddings_in_qdrant(documents, all_embeddings)
        print()
    except Exception as e:
        print(f"\n✗ Error storing embeddings: {e}")
        return

    # Success summary
    print("=" * 60)
    print("✓ Ingestion Complete!")
    print("=" * 60)
    print(f"  Documents processed: {len(documents)}")
    print(f"  Embeddings generated: {len(all_embeddings)}")
    print(f"  Vector store: Qdrant ({embeddings_manager.qdrant_host})")
    print(f"  Collection: {embeddings_manager.collection_name}")
    print()
    print("Your RAG chatbot is ready to answer questions!")


if __name__ == "__main__":
    ingest_book_content()
