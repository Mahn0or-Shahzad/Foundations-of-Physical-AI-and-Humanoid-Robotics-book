"""
Test script for RAG retrieval functions
Tests the retrieval logic without calling OpenAI (uses placeholders)
"""

from rag_retrieval import RAGRetriever


def test_full_book_search():
    """Test retrieval with full book search (no selected text)"""
    print("\n" + "=" * 60)
    print("Test 1: Full Book Search")
    print("=" * 60)

    retriever = RAGRetriever()

    result = retriever.retrieve_relevant_context(
        user_question="What is ROS 2 and why is it important for robotics?",
        selected_text=None,
        top_k=5
    )

    print("\nRetrieval Results:")
    print(f"  Mode: {result['retrieval_mode']}")
    print(f"  Chunks retrieved: {len(result['context_chunks'])}")
    print(f"  Scores: {result['scores']}")
    print(f"\nFirst chunk preview:")
    print(f"  {result['context_chunks'][0][:200]}...")
    print(f"\nMetadata:")
    for meta in result['metadata']:
        print(f"  - {meta}")


def test_selected_text_search():
    """Test retrieval with selected text context"""
    print("\n" + "=" * 60)
    print("Test 2: Selected Text Search")
    print("=" * 60)

    retriever = RAGRetriever()

    selected_text = """
    ROS 2 is an open-source middleware framework providing the communication
    infrastructure, tools, and libraries necessary to build complex robotic systems.
    """

    result = retriever.retrieve_relevant_context(
        user_question="Can you explain this concept in more detail?",
        selected_text=selected_text,
        top_k=3
    )

    print("\nRetrieval Results:")
    print(f"  Mode: {result['retrieval_mode']}")
    print(f"  Chunks retrieved: {len(result['context_chunks'])}")
    print(f"  Scores: {result['scores']}")
    print(f"\nFirst chunk preview:")
    print(f"  {result['context_chunks'][0][:200]}...")


def test_context_formatting():
    """Test formatting context for LLM prompt"""
    print("\n" + "=" * 60)
    print("Test 3: Context Formatting for LLM")
    print("=" * 60)

    retriever = RAGRetriever()

    # Get sample retrieval result
    result = retriever.retrieve_relevant_context(
        user_question="What are topics in ROS 2?",
        top_k=2
    )

    # Format for LLM
    formatted = retriever.format_context_for_llm(result)

    print("\nFormatted context for LLM prompt:")
    print("-" * 60)
    print(formatted)


if __name__ == "__main__":
    print("=" * 60)
    print("RAG Retrieval Test Suite")
    print("=" * 60)

    try:
        test_full_book_search()
        test_selected_text_search()
        test_context_formatting()

        print("\n" + "=" * 60)
        print("✓ All tests completed!")
        print("=" * 60)
        print("\nNote: Currently using placeholder results.")
        print("To enable actual retrieval:")
        print("  1. Run: python ingest.py (to populate Qdrant)")
        print("  2. Uncomment TODO sections in rag_retrieval.py")

    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        print("\nMake sure:")
        print("  - .env is configured with valid API keys")
        print("  - Virtual environment is activated")
        print("  - Dependencies are installed (pip install -r requirements.txt)")
