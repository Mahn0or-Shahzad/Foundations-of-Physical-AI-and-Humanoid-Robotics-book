# Quick Start Guide - AI in Motion Book with RAG Chatbot

## Test Everything (No API Costs)

### 1. Start Backend (Placeholder Mode)

```bash
cd backend
python -m venv venv
venv\Scripts\activate  # Windows
pip install -r requirements.txt
python main.py
```

**Expected**: Server runs on http://localhost:8000

### 2. Start Frontend

```bash
cd frontend
npm install
npm run start
```

**Expected**: Site runs on http://localhost:3000/AI-in-Motion---Foundations-of-Physical-AI-and-Humanoid-Robotics/

### 3. Test Chatbot UI

1. Open browser to frontend URL
2. Click 💬 button (bottom-right)
3. Chat panel opens
4. Type: "What is ROS 2?"
5. See response: "[Error] RAG system not initialized..."

**✓ Success!** Frontend and backend communicate (placeholder mode, $0 cost)

---

## Activate Real Chatbot (Optional)

### Prerequisites

- OpenAI API key (free $5 credit): https://platform.openai.com/
- Qdrant Cloud (free 1GB): https://qdrant.tech/

### Steps

1. **Configure .env**:
   ```bash
   cd backend
   cp .env.example .env
   # Edit .env with real API keys
   ```

2. **Run ingestion** (~$0.02):
   ```bash
   python ingest.py
   ```

3. **Uncomment TODOs**:
   - `rag_retrieval.py`: Lines 54-62, 114-125
   - `llm_generator.py`: Lines 65-80

4. **Restart backend**:
   ```bash
   python main.py
   ```

5. **Test real answers**:
   - Open chat widget
   - Ask: "What is ROS 2?"
   - Get real answer with sources!

**Cost**: $0.02 setup + $0.0004 per query

---

## Project Structure

```
AI in Motion/
├── backend/               # RAG chatbot API
│   ├── main.py           # FastAPI server
│   ├── rag_retrieval.py  # Context retrieval
│   ├── llm_generator.py  # Answer generation
│   ├── ingest.py         # One-time setup
│   └── .env              # API keys
│
├── frontend/             # Docusaurus book site
│   ├── src/
│   │   ├── components/
│   │   │   └── ChatWidget/  # Chatbot UI
│   │   └── theme/
│   │       └── Root.js   # Global wrapper
│   └── docusaurus.config.js
│
└── docs/                 # Book content (markdown)
    ├── intro.md
    ├── module1-ros2/
    ├── module2-digital-twin/
    └── ...
```

---

## What You Built

✅ **Docusaurus Book Site**
- Custom homepage
- All MDX errors fixed
- Responsive design
- Light/dark theme

✅ **RAG Chatbot Backend**
- Document ingestion pipeline
- OpenAI embeddings
- Qdrant vector store
- LLM answer generation
- Complete /chat API

✅ **Chat Widget Frontend**
- Floating button on all pages
- Auto-captures selected text
- Real-time Q&A
- Source citations
- Mobile-friendly

**Your intelligent Physical AI book is ready!** 📚🤖
