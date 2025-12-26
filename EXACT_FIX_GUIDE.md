# EXACT FIX: Docusaurus + Vercel → HF Spaces FastAPI Connection

## ROOT CAUSE IDENTIFIED ✅

Your Hugging Face Space is returning **404 errors**. This means:
1. The Space is NOT running or is sleeping
2. The Space URL might be wrong
3. The Space hasn't been deployed correctly

## Step-by-Step Fix

### STEP 1: Verify HF Space is Running

1. Go to: https://huggingface.co/spaces/mahishahzad/textbook
2. Check the status at the top:
   - 🟢 **Running** = Good
   - 🔴 **Building** = Wait
   - ⚪ **Sleeping** = Click "Wake up"
   - ❌ **Runtime error** = Check logs

3. If Space exists and shows "Running", open it and check if you see your FastAPI app

### STEP 2: Check Space Files

Your HF Space needs these files in the root:

**File: `app.py`** (NOT `main.py`)
```python
"""
RAG Chatbot Backend for Physical AI Book
FastAPI server providing chat endpoint for book content Q&A
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import os

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Book RAG Chatbot",
    description="Question-answering system for AI in Motion book content",
    version="1.0.0"
)

# CRITICAL: CORS Configuration for Vercel Frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://foundations-of-physical-ai-and-huma.vercel.app",
        "http://localhost:3000",
        "*"  # Remove this in production
    ],
    allow_credentials=False,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"],
)

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
    Process user question and return AI-generated answer.
    """
    try:
        user_question = request.user_question
        
        # TODO: Replace with your actual RAG logic
        # For now, return a simple response to test connection
        return ChatResponse(
            answer=f"I received your question: {user_question}. Backend is working! (RAG system needs to be connected)",
            sources=["Test source"],
            model="test-model"
        )
        
    except Exception as e:
        print(f"Error: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing question: {str(e)}"
        )
```

**File: `requirements.txt`**
```
fastapi==0.104.1
uvicorn[standard]==0.24.0
pydantic==2.5.0
python-dotenv==1.0.0
```

**File: `README.md`** (HF Spaces needs this)
```
---
title: Physical AI Book Chatbot
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: gradio
sdk_version: 4.0.0
app_file: app.py
pinned: false
---

# Physical AI Book RAG Chatbot Backend
```

### STEP 3: Set Environment Variables in HF Space

1. Go to: https://huggingface.co/spaces/mahishahzad/textbook
2. Click **Settings** tab
3. Scroll to **Repository secrets**
4. Add:
   - `OPENAI_API_KEY` = your key
   - `QDRANT_API_KEY` = your key
   - `QDRANT_HOST` = your Qdrant URL

### STEP 4: Update Frontend Environment Variable

**In Vercel Dashboard:**

1. Go to your frontend project: https://vercel.com/dashboard
2. Select your project
3. Settings → Environment Variables
4. Add NEW variable:
   - **Name:** `REACT_APP_API_URL`
   - **Value:** `https://mahishahzad-textbook.hf.space`
   - **Environment:** Production, Preview, Development (select all)
5. Click **Save**
6. Go to **Deployments** tab → Click "..." → **Redeploy**

### STEP 5: Updated Frontend Code (Already Correct)

Your `ChatWidget/index.js` is correct at line 53:
```javascript
const apiUrl = process.env.REACT_APP_API_URL || 'http://localhost:8000';
```

This will use HF Space URL in production, localhost in development.

### STEP 6: Test Connection

**Test 1: Backend Health**
```bash
curl https://mahishahzad-textbook.hf.space/
```
Should return: `{"status":"online",...}`

**Test 2: Backend Chat**
```bash
curl -X POST https://mahishahzad-textbook.hf.space/chat \
  -H "Content-Type: application/json" \
  -d '{"user_question":"test"}'
```
Should return: `{"answer":"...","sources":[...],"model":"..."}`

**Test 3: Frontend**
1. Visit: https://foundations-of-physical-ai-and-huma.vercel.app/
2. Open chat widget
3. Ask: "What is physical AI?"
4. Should get response

## Common Issues & Fixes

### Issue: HF Space Shows 404
**Cause:** Space doesn't exist or wrong URL
**Fix:** Check https://huggingface.co/spaces/mahishahzad/textbook exists

### Issue: HF Space is Sleeping
**Cause:** Free tier Spaces sleep after inactivity
**Fix:** Click "Wake up" button or upgrade to paid tier

### Issue: "Internal Server Error" (500)
**Cause:** Missing environment variables or code error
**Fix:** 
- Check HF Space logs (click "Logs" in Space)
- Add environment variables in Settings
- Check your RAG code doesn't crash

### Issue: CORS Error in Browser
**Cause:** Backend doesn't allow your frontend origin
**Fix:** Add your Vercel URL to `allow_origins` in CORS config:
```python
allow_origins=[
    "https://foundations-of-physical-ai-and-huma.vercel.app",
    "*"
],
```

### Issue: Frontend Says "Failed to fetch"
**Cause:** `REACT_APP_API_URL` not set in Vercel
**Fix:**
1. Vercel → Your Project → Settings → Environment Variables
2. Add `REACT_APP_API_URL` = `https://mahishahzad-textbook.hf.space`
3. Redeploy frontend

### Issue: Works Locally but Not in Production
**Cause:** Environment variable not set in deployment
**Fix:**
- **Backend:** Add secrets in HF Space Settings
- **Frontend:** Add env var in Vercel Settings
- **Both:** Redeploy after adding variables

## Docusaurus-Specific Notes

Since you're using Docusaurus, the environment variable must be:
- Prefixed with `REACT_APP_`
- Set at build time
- Redeployed after adding

Docusaurus builds to static files, so env vars are baked in at build time.

## Quick Debug Checklist

- [ ] HF Space exists and shows "Running" status
- [ ] HF Space URL works: https://mahishahzad-textbook.hf.space/
- [ ] Backend returns JSON at root endpoint
- [ ] Environment variables set in HF Space Settings
- [ ] `REACT_APP_API_URL` set in Vercel
- [ ] Frontend redeployed after adding env var
- [ ] Browser console (F12) shows no CORS errors
- [ ] Network tab shows request going to HF Space URL

## Need More Help?

Share:
1. Screenshot of HF Space status page
2. Browser console errors (F12 → Console)
3. Network request details (F12 → Network → /chat)
4. HF Space logs (from Space page)

The issue is definitely that your HF Space is returning 404, which means it's not deployed/running correctly.
