"""
Test script for RAG chatbot API
"""

import requests
import json

BASE_URL = "http://localhost:8000"

def test_health_check():
    """Test the root health check endpoint"""
    print("Testing health check endpoint...")
    response = requests.get(f"{BASE_URL}/")
    print(f"Status: {response.status_code}")
    print(f"Response: {json.dumps(response.json(), indent=2)}")
    print()

def test_chat_endpoint():
    """Test the chat endpoint with a sample question"""
    print("Testing chat endpoint...")

    # Test 1: Simple question
    payload = {
        "user_question": "What is ROS 2?",
    }
    response = requests.post(f"{BASE_URL}/chat", json=payload)
    print(f"Status: {response.status_code}")
    print(f"Question: {payload['user_question']}")
    print(f"Answer: {response.json()['answer']}")
    print()

    # Test 2: Question with selected text
    payload = {
        "user_question": "Explain this concept in more detail",
        "selected_text": "ROS 2 is middleware for distributed robotic systems"
    }
    response = requests.post(f"{BASE_URL}/chat", json=payload)
    print(f"Status: {response.status_code}")
    print(f"Question: {payload['user_question']}")
    print(f"Selected text: {payload['selected_text']}")
    print(f"Answer: {response.json()['answer']}")
    print()

if __name__ == "__main__":
    print("=" * 60)
    print("Physical AI Book RAG Chatbot - API Test")
    print("=" * 60)
    print()

    try:
        test_health_check()
        test_chat_endpoint()
        print("✓ All tests passed!")
    except requests.exceptions.ConnectionError:
        print("✗ Error: Cannot connect to backend server")
        print("Make sure the server is running: uvicorn main:app --reload --port 8000")
    except Exception as e:
        print(f"✗ Error: {e}")
