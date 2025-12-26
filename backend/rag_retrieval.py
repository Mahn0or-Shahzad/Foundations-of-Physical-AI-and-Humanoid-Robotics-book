"""
RAG Retrieval Logic
Handles context retrieval from Qdrant vector store based on user questions
"""

import os
from typing import List, Dict, Optional
from dotenv import load_dotenv
from embeddings import EmbeddingsManager

load_dotenv()


class RAGRetriever:
    """Manages retrieval of relevant context for answering user questions"""

    def __init__(self):
        """Initialize embeddings manager for retrieval operations"""
        self.embeddings_manager = EmbeddingsManager()
        print("✓ RAGRetriever initialized")

    def retrieve_relevant_context(
        self,
        user_question: str,
        selected_text: Optional[str] = None,
        top_k: int = 5
    ) -> Dict:
        """
        Retrieve relevant context for answering user question.

        Strategy:
        - If selected_text provided: Search for similar content to the selection
        - Else: Search based on user question across full book

        Args:
            user_question: The question asked by the user
            selected_text: Optional highlighted/selected text from the page
            top_k: Number of relevant chunks to retrieve (default: 5)

        Returns:
            Dictionary containing:
            {
                'context_chunks': List of relevant text chunks,
                'metadata': List of metadata for each chunk (file, section, etc.),
                'scores': Similarity scores for each chunk,
                'retrieval_mode': 'selected_text' or 'full_search'
            }
        """
        print(f"\n{'='*60}")
        print("RAG Retrieval Process")
        print(f"{'='*60}")
        print(f"User question: {user_question}")

        if selected_text:
            print(f"Selected text: {selected_text[:100]}...")
            return self._retrieve_from_selected_text(
                user_question, selected_text, top_k
            )
        else:
            print("Mode: Full book search")
            return self._retrieve_from_full_book(user_question, top_k)

    def _retrieve_from_selected_text(
        self,
        user_question: str,
        selected_text: str,
        top_k: int
    ) -> Dict:
        """
        Retrieve context based on selected/highlighted text.

        Strategy:
        1. Generate embedding for selected_text
        2. Search Qdrant for nearest neighbors
        3. Return chunks similar to the selection (contextual expansion)

        This helps when user highlights a concept and asks "explain more"

        Args:
            user_question: User's question
            selected_text: Highlighted text from the page
            top_k: Number of results

        Returns:
            Dictionary with context chunks and metadata
        """
        print("\nRetrieval mode: SELECTED TEXT")
        print(f"  Searching for content similar to selection...")

        try:
            # TODO: Generate embedding for selected_text
            # selected_embedding = self.embeddings_manager.generate_embeddings([selected_text])[0]

            # TODO: Search Qdrant for similar chunks
            # results = self.embeddings_manager.qdrant_client.search(
            #     collection_name=self.embeddings_manager.collection_name,
            #     query_vector=selected_embedding,
            #     limit=top_k
            # )

            # Placeholder response
            print("  ⚠ Using placeholder results (not yet implemented)")

            return {
                'context_chunks': [
                    f"[Placeholder] Context related to selected text: {selected_text[:200]}..."
                ],
                'metadata': [
                    {
                        'file_path': 'placeholder.md',
                        'section': 'Placeholder Section',
                        'retrieval_source': 'selected_text'
                    }
                ],
                'scores': [0.95],
                'retrieval_mode': 'selected_text'
            }

        except Exception as e:
            print(f"  ✗ Error in selected text retrieval: {e}")
            raise

    def _retrieve_from_full_book(
        self,
        user_question: str,
        top_k: int
    ) -> Dict:
        """
        Retrieve context by searching the full book content.

        Strategy:
        1. Generate embedding for user_question
        2. Search Qdrant across all book chunks
        3. Return most semantically similar chunks

        Args:
            user_question: User's question
            top_k: Number of results

        Returns:
            Dictionary with context chunks and metadata
        """
        print("\nRetrieval mode: FULL BOOK SEARCH")
        print(f"  Searching entire book for: '{user_question}'")

        try:
            # TODO: Generate embedding for user question
            # question_embedding = self.embeddings_manager.generate_embeddings([user_question])[0]

            # TODO: Search Qdrant
            # results = self.embeddings_manager.qdrant_client.search(
            #     collection_name=self.embeddings_manager.collection_name,
            #     query_vector=question_embedding,
            #     limit=top_k
            # )

            # TODO: Format results
            # context_chunks = [result.payload['content'] for result in results]
            # metadata = [result.payload['metadata'] for result in results]
            # scores = [result.score for result in results]

            # Placeholder response
            print("  ⚠ Using placeholder results (not yet implemented)")

            return {
                'context_chunks': [
                    f"[Placeholder] Relevant context for question: {user_question}"
                ],
                'metadata': [
                    {
                        'file_path': 'placeholder.md',
                        'section': 'Placeholder Section',
                        'retrieval_source': 'full_search'
                    }
                ],
                'scores': [0.92],
                'retrieval_mode': 'full_search'
            }

        except Exception as e:
            print(f"  ✗ Error in full book search: {e}")
            raise

    def format_context_for_llm(self, retrieval_result: Dict) -> str:
        """
        Format retrieved context chunks for LLM prompt.

        Args:
            retrieval_result: Dictionary from retrieve_relevant_context()

        Returns:
            Formatted string with context and sources
        """
        context_chunks = retrieval_result['context_chunks']
        metadata = retrieval_result['metadata']

        # Build formatted context
        formatted = "Relevant Context from Book:\n\n"

        for idx, (chunk, meta) in enumerate(zip(context_chunks, metadata), 1):
            source = f"{meta.get('file_path', 'unknown')} - {meta.get('section', 'unknown section')}"
            formatted += f"[Source {idx}: {source}]\n"
            formatted += f"{chunk}\n\n"
            formatted += "-" * 60 + "\n\n"

        return formatted


if __name__ == "__main__":
    # Test the RAG retriever
    print("=" * 60)
    print("Testing RAG Retriever")
    print("=" * 60)
    print()

    try:
        retriever = RAGRetriever()

        # Test 1: Full book search
        print("\nTest 1: Full book search")
        result1 = retriever.retrieve_relevant_context(
            user_question="What is ROS 2?",
            top_k=3
        )
        print(f"\nRetrieved {len(result1['context_chunks'])} chunks")
        print(f"Mode: {result1['retrieval_mode']}")

        # Test 2: Selected text search
        print("\n" + "=" * 60)
        print("\nTest 2: Selected text search")
        result2 = retriever.retrieve_relevant_context(
            user_question="Explain this in more detail",
            selected_text="ROS 2 is middleware for distributed robotic systems",
            top_k=3
        )
        print(f"\nRetrieved {len(result2['context_chunks'])} chunks")
        print(f"Mode: {result2['retrieval_mode']}")

        # Test 3: Format for LLM
        print("\n" + "=" * 60)
        print("\nTest 3: Format context for LLM")
        formatted = retriever.format_context_for_llm(result1)
        print(f"\nFormatted context:\n{formatted[:300]}...")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        print("\nMake sure .env is configured and Qdrant is running!")
