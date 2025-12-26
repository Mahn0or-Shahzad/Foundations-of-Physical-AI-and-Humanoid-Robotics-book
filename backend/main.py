"""
RAG Chatbot Backend for Physical AI Book
FastAPI server providing chat endpoint for book content Q&A
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import os
from dotenv import load_dotenv
from rag_retrieval import RAGRetriever
from llm_generator import LLMGenerator

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Book RAG Chatbot",
    description="Question-answering system for AI in Motion book content",
    version="1.0.0"
)

# Configure CORS for frontend communication (HF + Vercel)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # allow all origins (safe for testing)
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize RAG components (singleton pattern for efficiency)
try:
    retriever = RAGRetriever()
    generator = LLMGenerator()
    print("✓ RAG components initialized successfully")
except Exception as e:
    print(f"⚠ Warning: RAG components not initialized: {e}")
    print("  The /chat endpoint will use placeholder mode.")
    print("  Make sure .env is configured and Qdrant is accessible.")
    retriever = None
    generator = None

# Request/Response models
class ChatRequest(BaseModel):
    user_question: str
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    sources: Optional[list] = None
    model: Optional[str] = None

# Health check endpoint
@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "status": "online",
        "service": "Physical AI Book RAG Chatbot",
        "version": "1.0.0"
    }

# Main chat endpoint
@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Process user question and return AI-generated answer using RAG pipeline.

    Pipeline:
    1. Retrieve relevant context from Qdrant vector store
    2. Format context with sources
    3. Generate answer using OpenAI LLM
    4. Return answer with source citations

    Args:
        request: ChatRequest with user_question and optional selected_text

    Returns:
        ChatResponse with answer, sources, and model info
    """
    try:
        user_question = request.user_question
        selected_text = request.selected_text

        # Check if RAG components are initialized
        if retriever is None or generator is None:
            return ChatResponse(
                answer="[Error] RAG system not initialized. Please configure .env and ensure Qdrant is accessible.",
                sources=None,
                model=None
            )

        # Step 1: Retrieve relevant context from Qdrant
        print(f"\n[/chat] Processing question: {user_question[:50]}...")

        retrieval_result = retriever.retrieve_relevant_context(
            user_question=user_question,
            selected_text=selected_text,
            top_k=5  # Retrieve top 5 most relevant chunks
        )

        print(f"  Retrieved {len(retrieval_result['context_chunks'])} chunks (mode: {retrieval_result['retrieval_mode']})")

        # Step 2: Format context for LLM prompt
        formatted_context = retriever.format_context_for_llm(retrieval_result)
        print(f"  Context formatted: {len(formatted_context)} characters")

        # Step 3: Generate answer using OpenAI
        llm_result = generator.generate_answer(
            context_text=formatted_context,
            user_question=user_question,
            selected_text=selected_text
        )

        print(f"  Answer generated: {llm_result['tokens_used']} tokens, model: {llm_result['model']}")

        # Step 4: Return complete response
        return ChatResponse(
            answer=llm_result['answer'],
            sources=llm_result['sources'],
            model=llm_result['model']
        )

    except Exception as e:
        print(f"  ✗ Error in /chat endpoint: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing question: {str(e)}"
        )

# Run with: uvicorn main:app --reload --port 8000
if __name__ == "__main__":
    import uvicorn
    import os

    port = int(os.environ.get("PORT", 7860))  # HF uses 7860
    uvicorn.run(app, host="0.0.0.0", port=port)
