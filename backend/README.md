# RAG Chatbot Backend

Backend API for the Physical AI Book chatbot, providing question-answering over book content.

## Setup

### 1. Create Virtual Environment

```bash
cd backend
python -m venv venv

# Activate (Windows)
venv\Scripts\activate

# Activate (Linux/Mac)
source venv/bin/activate
```

### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

### 3. Configure Environment Variables

Copy `.env.example` to `.env` and fill in your credentials:

```bash
cp .env.example .env
```

Edit `.env` and add your actual keys:

```
# Get from https://platform.openai.com/api-keys
OPENAI_API_KEY=sk-your-actual-openai-key

# Get from https://neon.tech/ (free tier)
NEON_DB_URL=postgresql://user:pass@ep-xxx.region.aws.neon.tech/dbname

# Get from https://qdrant.tech/ (free cloud tier)
QDRANT_API_KEY=your-actual-qdrant-key
QDRANT_HOST=https://xxx.cloud.qdrant.io
```

**Free Tier Resources**:
- **OpenAI**: Free $5 credit for new accounts
- **Neon**: Free tier with 512 MB storage
- **Qdrant Cloud**: Free tier with 1GB cluster

### 4. Run Server

```bash
# Development mode (auto-reload)
uvicorn main:app --reload --port 8000

# Or using Python directly
python main.py
```

Server runs at: `http://localhost:8000`

API docs: `http://localhost:8000/docs`

## API Endpoints

### GET /
Health check endpoint

**Response**:
```json
{
  "status": "online",
  "service": "Physical AI Book RAG Chatbot",
  "version": "1.0.0"
}
```

### POST /chat
Process user question and return answer

**Request**:
```json
{
  "user_question": "What is ROS 2?",
  "selected_text": "Optional context from highlighted text"
}
```

**Response**:
```json
{
  "answer": "ROS 2 is a middleware framework..."
}
```

## Document Ingestion

Before the chatbot can answer questions, you need to ingest the book content:

### Run Ingestion Pipeline

```bash
# Activate virtual environment first
venv\Scripts\activate  # Windows

# Run ingestion script
python ingest.py
```

**What it does**:
1. Loads all markdown files from `../docs`
2. Splits content into semantic chunks (~2000 chars)
3. Generates embeddings using OpenAI API
4. Stores in Qdrant vector database

**Expected output**:
```
Step 1: Loading markdown documents...
  ✓ Loaded: intro.md (5 chunks)
  ✓ Loaded: module1-ros2/ros2-architecture.md (12 chunks)
  ...
✓ Loaded 150 document chunks

Step 2: Initializing embeddings manager...
✓ EmbeddingsManager initialized

Step 3: Generating embeddings...
  Processing batch 1/2 (100 texts)...
  Processing batch 2/2 (50 texts)...
✓ Generated 150 embeddings

Step 4: Storing embeddings in Qdrant...
✓ Stored 150 embeddings successfully

✓ Ingestion Complete!
```

**Cost Estimate**: ~$0.02 for 150 chunks (OpenAI text-embedding-3-small)

---

## Development Status

- [x] FastAPI skeleton
- [x] Basic endpoints
- [x] CORS configuration
- [x] Document ingestion pipeline
- [x] Vector store integration
- [ ] LLM integration for answer generation
- [ ] RAG retrieval implementation in /chat endpoint
- [ ] Conversation history support
- [ ] Source citation in responses

## Architecture

```
User Question
     ↓
FastAPI /chat endpoint
     ↓
document_loader.py → Load docs from ../docs
     ↓
embeddings.py → Generate OpenAI embeddings
     ↓
Qdrant → Store/search vectors
     ↓
OpenAI GPT-4 → Generate answer
     ↓
Response with sources
```
