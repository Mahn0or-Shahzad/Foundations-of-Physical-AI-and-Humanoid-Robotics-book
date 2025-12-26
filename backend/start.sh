#!/bin/bash
# Quick start script for Linux/Mac

echo "Starting Physical AI Book RAG Chatbot Backend..."
echo ""

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
    echo ""
fi

# Activate virtual environment
source venv/bin/activate

# Install dependencies if needed
echo "Checking dependencies..."
pip install -q -r requirements.txt

# Start FastAPI server
echo ""
echo "Starting FastAPI server on http://localhost:8000"
echo "API docs available at http://localhost:8000/docs"
echo ""
echo "Press Ctrl+C to stop the server"
echo ""

uvicorn main:app --reload --port 8000
