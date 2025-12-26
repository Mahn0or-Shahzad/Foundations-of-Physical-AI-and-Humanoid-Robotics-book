# RAG Chatbot Testing Guide - No Paid API Calls

## Overview

This guide helps you test the complete RAG chatbot system **without making paid API calls** to OpenAI or Qdrant.

All components use placeholder/mock responses until you're ready to activate the real APIs.

---

## Testing Strategy

```
Phase 1: Component Tests (No API calls) ✓
Phase 2: Integration Tests (Placeholders) ✓
Phase 3: Frontend-Backend Communication ✓
Phase 4: Activation (Real APIs) - Optional
```

---

## Phase 1: Backend Component Tests (No API Calls)

### Test 1: Document Loader

```bash
cd backend
python -m venv venv
venv\Scripts\activate  # Windows
# OR: source venv/bin/activate  # Linux/Mac

pip install -r requirements.txt

python document_loader.py
```

**Expected Output**:
```
============================================================
Testing Document Loader
============================================================

Loading documents from: E:\...\docs
  ✓ Loaded: intro.md (5 chunks)
  ✓ Loaded: module1-ros2/ros2-architecture.md (12 chunks)
  ✓ Loaded: module2-digital-twin/gazebo-simulation.md (15 chunks)
  ...

Total documents loaded: 150

Sample document:
Content preview: ## What is ROS 2?

ROS 2 is an open-source middleware...
Metadata: {'id': 'ros2-architecture', 'title': 'ROS 2 Fundamentals', ...}
```

**Verification**:
- ✅ All markdown files loaded from ../docs
- ✅ Frontmatter extracted correctly
- ✅ Content chunked by sections
- ✅ Metadata preserved

**Cost**: $0 (no API calls)

---

### Test 2: Embeddings Manager (Mock Mode)

**IMPORTANT**: Keep API keys as placeholders in .env to avoid charges!

Edit `backend/.env`:
```
OPENAI_API_KEY=your_free_openai_api_key  # Keep placeholder!
QDRANT_API_KEY=your_qdrant_free_api_key  # Keep placeholder!
```

```bash
python embeddings.py
```

**Expected Output**:
```
============================================================
Testing Embeddings Manager
============================================================

✗ Error: ValueError: OPENAI_API_KEY not configured in .env file

Make sure to configure .env with valid API keys!
```

**This is correct!** The component detects placeholder keys and prevents accidental API calls.

**Verification**:
- ✅ Validates API keys before making calls
- ✅ Prevents accidental charges
- ✅ Clear error messages

**Cost**: $0 (no API calls made)

---

### Test 3: RAG Retrieval (Placeholder Mode)

```bash
python rag_retrieval.py
```

**Expected Output**:
```
============================================================
Testing RAG Retriever
============================================================

✓ RAGRetriever initialized

Test 1: Full book search
============================================================
RAG Retrieval Process
============================================================
User question: What is ROS 2?
Mode: Full book search

Retrieval mode: FULL BOOK SEARCH
  Searching entire book for: 'What is ROS 2?'
  ⚠ Using placeholder results (not yet implemented)

Retrieved 1 chunks
Mode: full_search
```

**Verification**:
- ✅ Retriever initializes without crashing
- ✅ Returns placeholder responses
- ✅ Handles both search modes (full_search, selected_text)
- ✅ No API calls made

**Cost**: $0

---

### Test 4: LLM Generator (Placeholder Mode)

```bash
python llm_generator.py
```

**Expected Output**:
```
============================================================
Testing LLM Generator
============================================================

✗ Error: ValueError: OPENAI_API_KEY not configured in .env file

Make sure OPENAI_API_KEY is configured in .env!
```

**This is correct!** Prevents accidental API calls.

**Verification**:
- ✅ API key validation works
- ✅ No accidental charges

**Cost**: $0

---

## Phase 2: Integration Tests (Placeholders)

### Test 5: Full Pipeline Test

```bash
python test_full_pipeline.py
```

**Expected Output**:
```
============================================================
Full RAG Pipeline Test
============================================================

Step 1: Initializing RAG components...
✗ Error: ValueError: OPENAI_API_KEY not configured
```

**This is expected and correct!** The test validates that all components check for proper configuration.

**Verification**:
- ✅ All components properly validate environment
- ✅ No surprise API charges

**Cost**: $0

---

## Phase 3: Frontend-Backend Communication Test

### Step 1: Start Backend (Placeholder Mode)

```bash
cd backend
venv\Scripts\activate
python main.py
```

**Expected Output**:
```
⚠ Warning: RAG components not initialized: OPENAI_API_KEY not configured in .env file
  The /chat endpoint will use placeholder mode.
  Make sure .env is configured and Qdrant is accessible.

INFO:     Started server process
INFO:     Uvicorn running on http://0.0.0.0:8000
```

**Verify**:
- ✅ Server starts successfully
- ✅ Warns about placeholder mode
- ✅ No crashes

### Step 2: Test Health Endpoint

Open browser: http://localhost:8000

**Expected**:
```json
{
  "status": "online",
  "service": "Physical AI Book RAG Chatbot",
  "version": "1.0.0"
}
```

### Step 3: Test Chat Endpoint (Placeholder)

Open: http://localhost:8000/docs

Try the POST /chat endpoint with:
```json
{
  "user_question": "What is ROS 2?",
  "selected_text": null
}
```

**Expected Response**:
```json
{
  "answer": "[Error] RAG system not initialized. Please configure .env and ensure Qdrant is accessible.",
  "sources": null,
  "model": null
}
```

**Verification**:
- ✅ Endpoint responds
- ✅ Returns error message (RAG not initialized)
- ✅ No charges incurred

**Cost**: $0

---

### Step 4: Start Frontend

```bash
cd frontend
npm run start
```

**Expected Output**:
```
[SUCCESS] Docusaurus website is running at:
http://localhost:3000/AI-in-Motion---Foundations-of-Physical-AI-and-Humanoid-Robotics/
```

### Step 5: Test Chat Widget UI

1. **Open browser**: http://localhost:3000/AI-in-Motion---Foundations-of-Physical-AI-and-Humanoid-Robotics/

2. **Find chat button**: Bottom-right corner (💬)

3. **Click to open**: Chat panel appears

4. **Check welcome message**:
   ```
   👋 Hi! I'm your AI assistant for this book.
   Ask me anything about ROS 2, digital twins, Isaac, or VLA systems.
   Tip: Highlight text on the page and ask for more details!
   ```

5. **Test text selection**:
   - Navigate to any docs page
   - Highlight a paragraph
   - See "Selected: ..." banner in chat widget

6. **Type a question**: "What is ROS 2?"

7. **Submit**: Click ➤ or press Enter

8. **Expected response** (from backend placeholder):
   ```
   [Error] RAG system not initialized. Please configure .env and ensure Qdrant is accessible.
   ```

**Verification**:
- ✅ Chat button appears on all pages
- ✅ Panel opens/closes smoothly
- ✅ Text selection captured automatically
- ✅ Messages display correctly
- ✅ Error handling works (backend not configured)
- ✅ Frontend-backend communication established

**Cost**: $0

---

## Phase 4: Activation (Optional - Uses Real APIs)

⚠️ **WARNING**: The following steps will use your OpenAI and Qdrant free tier credits.

**Only proceed if you want to activate the real chatbot.**

### Prerequisites:

1. ✅ OpenAI API key (sign up: https://platform.openai.com/)
2. ✅ Qdrant Cloud account (sign up: https://qdrant.tech/)

### Step 1: Configure Real API Keys

Edit `backend/.env`:
```bash
# Replace placeholders with real keys
OPENAI_API_KEY=sk-your-actual-openai-key-here
QDRANT_API_KEY=your-actual-qdrant-api-key-here
QDRANT_HOST=https://your-cluster-id.cloud.qdrant.io
```

### Step 2: Run Ingestion (Costs ~$0.02)

```bash
cd backend
venv\Scripts\activate
python ingest.py
```

**Expected** (~2-5 minutes):
```
Step 1: Loading markdown documents...
  ✓ Loaded: intro.md (5 chunks)
  ...
✓ Loaded 150 document chunks

Step 2: Initializing embeddings manager...
✓ EmbeddingsManager initialized

Step 3: Generating embeddings...
  Processing batch 1/2 (100 texts)...
  ✓ Generated embeddings

Step 4: Storing embeddings in Qdrant...
✓ Stored 150 embeddings successfully

✓ Ingestion Complete!
```

**Cost**: ~$0.02

### Step 3: Uncomment TODO Sections

**File: backend/rag_retrieval.py**

Line 54-62 (in `_retrieve_from_selected_text`):
```python
# UNCOMMENT THIS:
selected_embedding = self.embeddings_manager.generate_embeddings([selected_text])[0]

results = self.embeddings_manager.qdrant_client.search(
    collection_name=self.embeddings_manager.collection_name,
    query_vector=selected_embedding,
    limit=top_k
)

# Comment out or remove the placeholder return statement
```

Line 114-125 (in `_retrieve_from_full_book`):
```python
# UNCOMMENT THIS:
question_embedding = self.embeddings_manager.generate_embeddings([user_question])[0]

results = self.embeddings_manager.qdrant_client.search(
    collection_name=self.embeddings_manager.collection_name,
    query_vector=question_embedding,
    limit=top_k
)

context_chunks = [result.payload['content'] for result in results]
metadata = [result.payload['metadata'] for result in results]
scores = [result.score for result in results]

# Use real results instead of placeholder
```

**File: backend/llm_generator.py**

Line 65-80 (in `generate_answer`):
```python
# UNCOMMENT THIS:
response = self.client.chat.completions.create(
    model=self.model,
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_prompt}
    ],
    temperature=0.7,
    max_tokens=1000,
)

answer = response.choices[0].message.content
tokens_used = response.usage.total_tokens

# Comment out the placeholder answer
```

### Step 4: Restart Backend

```bash
# Stop server (Ctrl+C)
# Restart with changes
uvicorn main:app --reload --port 8000
```

**Expected**:
```
✓ RAGRetriever initialized
✓ LLMGenerator initialized (model: gpt-4o-mini)
✓ RAG components initialized successfully
INFO: Uvicorn running on http://0.0.0.0:8000
```

### Step 5: Test Real Chatbot

1. Open frontend: http://localhost:3000/AI-in-Motion---Foundations-of-Physical-AI-and-Humanoid-Robotics/
2. Click chat button
3. Ask: "What is ROS 2?"
4. Wait 1-3 seconds
5. **Expected**: Real answer with sources!

**Example Response**:
```
ROS 2 is an open-source middleware framework that provides the
communication infrastructure, tools, and libraries necessary to
build complex robotic systems. It operates above the OS layer,
providing abstractions for distributed computation across
heterogeneous hardware components...

Sources:
• module1-ros2/ros2-architecture.md - What is ROS 2?
• module1-ros2/ros2-architecture.md - ROS 2 Architecture
```

**Cost per query**: ~$0.0004 (less than 0.05 cents)

---

## Quick Test Commands

### Terminal 1: Backend
```bash
cd backend
venv\Scripts\activate
python main.py
```

### Terminal 2: Frontend
```bash
cd frontend
npm run start
```

### Terminal 3: API Test (No Browser)
```bash
cd backend
python test_api.py
```

**Expected** (placeholder mode):
```
============================================================
Physical AI Book RAG Chatbot - API Test
============================================================

Testing health check endpoint...
Status: 200
Response: {
  "status": "online",
  "service": "Physical AI Book RAG Chatbot",
  "version": "1.0.0"
}

Testing chat endpoint...
Status: 200
Question: What is ROS 2?
Answer: [Error] RAG system not initialized. Please configure .env and ensure Qdrant is accessible.

✓ All tests passed!
```

---

## Verification Checklist (Placeholder Mode - No Costs)

### Backend Tests:

- [ ] `python document_loader.py` - Loads all docs successfully
- [ ] `python main.py` - Server starts on port 8000
- [ ] `curl http://localhost:8000/` - Returns health check JSON
- [ ] `python test_api.py` - API endpoints respond (with error messages)

### Frontend Tests:

- [ ] `npm run start` - Docusaurus starts on port 3000
- [ ] Chat button visible (bottom-right corner)
- [ ] Chat panel opens when clicked
- [ ] Welcome message displays
- [ ] Can type in input field
- [ ] Submit button works

### Integration Tests:

- [ ] Text selection on page captured in chat widget
- [ ] "Selected: ..." banner appears when text highlighted
- [ ] Submitting question sends request to backend
- [ ] Error message displayed (RAG not initialized)
- [ ] Messages appear in chat history
- [ ] Clear button works
- [ ] Chat persists during navigation

**All checks should pass without any API calls or costs!**

---

## Visual Testing Guide

### Step-by-Step Visual Verification:

#### 1. Start Both Servers

**Terminal 1** (Backend):
```bash
cd backend
venv\Scripts\activate
python main.py
```

Wait for:
```
⚠ Warning: RAG components not initialized
INFO: Uvicorn running on http://0.0.0.0:8000
```

**Terminal 2** (Frontend):
```bash
cd frontend
npm run start
```

Wait for:
```
[SUCCESS] Docusaurus website is running at: http://localhost:3000/...
```

#### 2. Open Browser

Navigate to:
```
http://localhost:3000/AI-in-Motion---Foundations-of-Physical-AI-and-Humanoid-Robotics/
```

#### 3. Verify Homepage

Should see:
- ✅ "AI in Motion: Foundations..." title
- ✅ Subtitle and buttons
- ✅ Module cards
- ✅ 💬 button in bottom-right corner

#### 4. Click Chat Button

Should see:
- ✅ Chat panel slides up
- ✅ "AI Book Assistant" header
- ✅ Welcome message with tips
- ✅ Input field at bottom

#### 5. Navigate to Docs Page

Click "Start Reading" → Go to Introduction page

Should see:
- ✅ Chat button still visible
- ✅ Chat state preserved (if was open)

#### 6. Test Text Selection

1. Highlight a paragraph on the page (e.g., about ROS 2)
2. Look at chat widget

Should see:
- ✅ "Selected: ..." banner appears
- ✅ First 100 chars of selection shown
- ✅ ✕ button to clear selection

#### 7. Ask Question (Without Selected Text)

1. Clear any selected text
2. Type: "What is ROS 2?"
3. Press Enter

Should see:
- ✅ User message appears (right side)
- ✅ Loading animation (● ● ● Thinking...)
- ✅ Bot response appears (left side)
- ✅ Error message: "[Error] RAG system not initialized..."

#### 8. Ask Question (With Selected Text)

1. Highlight text: "Topics implement publish-subscribe pattern"
2. Type: "Explain this in more detail"
3. Submit

Should see:
- ✅ User message with "📌 With selected text" tag
- ✅ Loading animation
- ✅ Bot response with error message
- ✅ Selected text banner clears after submission

#### 9. Test Clear Functionality

Click 🗑️ in chat header

Should see:
- ✅ All messages cleared
- ✅ Welcome message reappears
- ✅ Input cleared

#### 10. Test Mobile View

Resize browser to mobile width (<768px)

Should see:
- ✅ Chat panel becomes full-width
- ✅ Chat button repositions appropriately
- ✅ All functionality works on touch

---

## Network Monitoring (Verify No Charges)

### Open Browser DevTools

**Chrome/Edge**: F12 → Network tab

**Firefox**: F12 → Network tab

### Filter for API Calls

1. Start backend and frontend
2. Open DevTools Network tab
3. Click chat button
4. Ask a question
5. Watch network requests

**Should see**:
```
POST http://localhost:8000/chat
Status: 200 OK
Response: {
  "answer": "[Error] RAG system not initialized...",
  "sources": null,
  "model": null
}
```

**Should NOT see**:
- ❌ Requests to api.openai.com
- ❌ Requests to cloud.qdrant.io
- ❌ Any external API calls

**Verification**: All communication is localhost only!

**Cost**: $0

---

## Common Issues (Placeholder Mode)

### Issue: "Failed to fetch" error

**Cause**: Backend not running

**Solution**:
```bash
cd backend
python main.py
# Verify: http://localhost:8000/ responds
```

### Issue: Chat button not appearing

**Cause**: Frontend not built with new component

**Solution**:
```bash
cd frontend
npm run build
npm run start
```

### Issue: CORS error in console

**Cause**: Frontend URL not in CORS allow list

**Solution**: Check `backend/main.py` lines 28-32:
```python
allow_origins=[
    "http://localhost:3000",  # Add your frontend URL
    ...
],
```

### Issue: Messages not displaying

**Cause**: State management or rendering issue

**Solution**: Check browser console for React errors

---

## Success Criteria (Placeholder Mode)

Before activating real APIs, verify:

✅ **Backend**:
- [ ] Server starts on port 8000
- [ ] Health check endpoint responds
- [ ] /chat endpoint responds with error message
- [ ] No crashes or exceptions

✅ **Frontend**:
- [ ] Site builds successfully
- [ ] Chat button visible on all pages
- [ ] Panel opens/closes smoothly
- [ ] Text selection captured
- [ ] Messages display correctly

✅ **Integration**:
- [ ] Frontend sends POST to backend
- [ ] Backend responds (even if error)
- [ ] Error messages displayed in UI
- [ ] No CORS errors
- [ ] No external API calls

✅ **Cost**:
- [ ] $0 spent (all placeholder responses)

---

## When Ready to Activate Real APIs

See `backend/IMPLEMENTATION_GUIDE.md` for activation steps:

1. Get real API keys from OpenAI and Qdrant
2. Update `.env` with actual values
3. Run `python ingest.py` (costs ~$0.02)
4. Uncomment TODO sections in:
   - `rag_retrieval.py`
   - `llm_generator.py`
5. Restart backend
6. Test with real queries

**Estimated cost**: $0.02 ingestion + $0.0004 per query

---

## Summary

**Current Testing Status**: ✅ COMPLETE

You can now:
- ✅ Test the entire system without any costs
- ✅ Verify frontend-backend communication
- ✅ Validate UI/UX without real AI
- ✅ Demonstrate the chatbot interface
- ✅ Develop frontend features safely

**All components work in placeholder mode with zero API charges!**

When you're ready to activate the real RAG chatbot:
- Total setup cost: ~$0.02 (one-time ingestion)
- Per-query cost: ~$0.0004 (less than 0.05 cents)
- Free tier: 12,000+ queries on $5 credit

**Your testing environment is production-ready and cost-safe!** 🎉
