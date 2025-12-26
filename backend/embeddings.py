"""
Embeddings Generation and Vector Store Management
Handles OpenAI embeddings and Qdrant storage
"""

import os
from typing import List, Dict
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from openai import OpenAI

load_dotenv()


class EmbeddingsManager:
    """Manages embeddings generation and vector store operations"""

    def __init__(self):
        """Initialize OpenAI client and Qdrant client"""
        # OpenAI configuration
        self.openai_api_key = os.getenv('OPENAI_API_KEY')
        if not self.openai_api_key or self.openai_api_key == 'your_free_openai_api_key':
            raise ValueError("OPENAI_API_KEY not configured in .env file")

        self.openai_client = OpenAI(api_key=self.openai_api_key)
        self.embedding_model = "text-embedding-3-small"  # 1536 dimensions, cost-effective

        # Qdrant configuration
        self.qdrant_host = os.getenv('QDRANT_HOST', 'http://localhost:6333')
        self.qdrant_api_key = os.getenv('QDRANT_API_KEY')
        self.collection_name = "physical_ai_book"

        # Initialize Qdrant client
        if self.qdrant_api_key and self.qdrant_api_key != 'your_qdrant_free_api_key':
            # Cloud instance with API key
            self.qdrant_client = QdrantClient(
                url=self.qdrant_host,
                api_key=self.qdrant_api_key
            )
        else:
            # Local instance (no API key)
            self.qdrant_client = QdrantClient(url=self.qdrant_host)

        print(f"✓ EmbeddingsManager initialized")
        print(f"  OpenAI model: {self.embedding_model}")
        print(f"  Qdrant host: {self.qdrant_host}")

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using OpenAI API.

        Args:
            texts: List of text strings to embed

        Returns:
            List of embedding vectors (each is 1536-dimensional)
        """
        print(f"Generating embeddings for {len(texts)} texts...")

        try:
            # Call OpenAI embeddings API
            response = self.openai_client.embeddings.create(
                model=self.embedding_model,
                input=texts
            )

            # Extract embedding vectors
            embeddings = [item.embedding for item in response.data]

            print(f"  ✓ Generated {len(embeddings)} embeddings")
            return embeddings

        except Exception as e:
            print(f"  ✗ Error generating embeddings: {e}")
            raise

    def create_collection(self):
        """
        Create Qdrant collection for storing book embeddings.
        Creates collection with proper vector dimensions.
        """
        print(f"Creating Qdrant collection: {self.collection_name}")

        try:
            # Check if collection exists
            collections = self.qdrant_client.get_collections().collections
            collection_names = [col.name for col in collections]

            if self.collection_name in collection_names:
                print(f"  ⚠ Collection '{self.collection_name}' already exists")
                return

            # Create collection with vector configuration
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=1536,  # text-embedding-3-small dimensions
                    distance=Distance.COSINE  # Cosine similarity
                )
            )

            print(f"  ✓ Collection created successfully")

        except Exception as e:
            print(f"  ✗ Error creating collection: {e}")
            raise

    def store_embeddings_in_qdrant(self, documents: List, embeddings: List[List[float]]):
        """
        Store document embeddings in Qdrant vector database.

        Args:
            documents: List of Document objects (from document_loader)
            embeddings: List of embedding vectors (from generate_embeddings)
        """
        print(f"Storing {len(documents)} embeddings in Qdrant...")

        if len(documents) != len(embeddings):
            raise ValueError(f"Mismatch: {len(documents)} documents but {len(embeddings)} embeddings")

        try:
            # Ensure collection exists
            self.create_collection()

            # Prepare points for insertion
            points = []
            for idx, (doc, embedding) in enumerate(zip(documents, embeddings)):
                point = PointStruct(
                    id=idx,  # Unique ID
                    vector=embedding,  # 1536-dim vector
                    payload={
                        'content': doc.content,
                        'metadata': doc.metadata
                    }
                )
                points.append(point)

            # Batch upsert to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            print(f"  ✓ Stored {len(points)} embeddings successfully")

        except Exception as e:
            print(f"  ✗ Error storing embeddings: {e}")
            raise

    def search_similar(self, query: str, top_k: int = 5) -> List[Dict]:
        """
        Search for similar documents using semantic similarity.

        Args:
            query: User's question
            top_k: Number of results to return

        Returns:
            List of relevant documents with scores
        """
        print(f"Searching for: '{query}' (top {top_k} results)")

        try:
            # Generate embedding for query
            query_embedding = self.generate_embeddings([query])[0]

            # Search Qdrant
            results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k
            )

            # Format results
            similar_docs = []
            for result in results:
                similar_docs.append({
                    'content': result.payload['content'],
                    'metadata': result.payload['metadata'],
                    'score': result.score
                })

            print(f"  ✓ Found {len(similar_docs)} relevant documents")
            return similar_docs

        except Exception as e:
            print(f"  ✗ Error searching: {e}")
            raise


if __name__ == "__main__":
    # Test the embeddings manager
    print("=" * 60)
    print("Testing Embeddings Manager")
    print("=" * 60)
    print()

    try:
        manager = EmbeddingsManager()

        # Test embedding generation (with sample text)
        sample_texts = ["This is a test document about ROS 2."]
        embeddings = manager.generate_embeddings(sample_texts)
        print(f"\nSample embedding dimensions: {len(embeddings[0])}")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        print("\nMake sure to configure .env with valid API keys!")
