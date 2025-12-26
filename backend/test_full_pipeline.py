"""
Full RAG Pipeline Test
Tests the complete flow: Question → Retrieval → LLM Generation → Answer
"""

from rag_retrieval import RAGRetriever
from llm_generator import LLMGenerator


def test_full_rag_pipeline():
    """
    Test complete RAG pipeline end-to-end.

    Flow:
    1. User asks question
    2. Retrieve relevant context from Qdrant
    3. Format context for LLM
    4. Generate answer with OpenAI
    5. Return answer with sources
    """
    print("=" * 60)
    print("Full RAG Pipeline Test")
    print("=" * 60)
    print()

    try:
        # Initialize components
        print("Step 1: Initializing RAG components...")
        retriever = RAGRetriever()
        generator = LLMGenerator()
        print()

        # Test scenarios
        test_cases = [
            {
                "question": "What is ROS 2 and why is it used in robotics?",
                "selected_text": None,
                "description": "General question (full book search)"
            },
            {
                "question": "Can you explain this concept in more detail?",
                "selected_text": "Topics implement the publish-subscribe pattern for continuous data streams.",
                "description": "Question with selected text (contextual expansion)"
            },
            {
                "question": "How do I set up Gazebo for humanoid simulation?",
                "selected_text": None,
                "description": "Specific technical question"
            }
        ]

        for idx, test_case in enumerate(test_cases, 1):
            print("\n" + "=" * 60)
            print(f"Test Case {idx}: {test_case['description']}")
            print("=" * 60)

            user_question = test_case['question']
            selected_text = test_case['selected_text']

            # Step 2: Retrieve context
            print("\n[Step 2] Retrieving relevant context...")
            retrieval_result = retriever.retrieve_relevant_context(
                user_question=user_question,
                selected_text=selected_text,
                top_k=5
            )
            print(f"  Retrieved {len(retrieval_result['context_chunks'])} chunks")
            print(f"  Mode: {retrieval_result['retrieval_mode']}")

            # Step 3: Format context
            print("\n[Step 3] Formatting context for LLM...")
            formatted_context = retriever.format_context_for_llm(retrieval_result)
            print(f"  Formatted context: {len(formatted_context)} characters")

            # Step 4: Generate answer
            print("\n[Step 4] Generating answer with LLM...")
            llm_result = generator.generate_answer(
                context_text=formatted_context,
                user_question=user_question,
                selected_text=selected_text
            )
            print(f"  Model used: {llm_result['model']}")
            print(f"  Tokens used: {llm_result['tokens_used']}")
            print(f"  Sources: {llm_result['sources']}")

            # Step 5: Display result
            print("\n[Step 5] Final Answer:")
            print("-" * 60)
            print(llm_result['answer'])
            print("-" * 60)

        print("\n" + "=" * 60)
        print("✓ All test cases completed!")
        print("=" * 60)
        print("\nNote: Using placeholder results (API calls commented out)")
        print("\nTo activate full pipeline:")
        print("  1. Run: python ingest.py")
        print("  2. Uncomment TODO sections in rag_retrieval.py")
        print("  3. Uncomment TODO sections in llm_generator.py")
        print("  4. Uncomment pipeline in main.py")

    except Exception as e:
        print(f"\n✗ Pipeline test failed: {e}")
        print("\nChecklist:")
        print("  - Is .env configured with valid API keys?")
        print("  - Is virtual environment activated?")
        print("  - Are dependencies installed?")
        print("  - Is Qdrant running (if using local instance)?")


if __name__ == "__main__":
    test_full_rag_pipeline()
