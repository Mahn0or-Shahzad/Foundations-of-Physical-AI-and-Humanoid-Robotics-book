# Chatbot Not Responding - Debugging Guide

If your chatbot is deployed but not responding, follow these steps:

## Step 1: Test Backend Directly

First, verify your backend is working:

```bash
# Replace YOUR-BACKEND-URL with your actual Vercel URL
curl https://YOUR-BACKEND-URL.vercel.app/

# Should return:
# {"status":"online","service":"Physical AI Book RAG Chatbot","version":"1.0.0"}
```

If this fails:
- ❌ Backend isn't deployed properly
- Check Vercel dashboard → Your backend project → Deployments
- Check for build errors

## Step 2: Test Chat Endpoint

```bash
curl -X POST https://YOUR-BACKEND-URL.vercel.app/chat \
  -H "Content-Type: application/json" \
  -d '{"user_question":"What is ROS 2?"}'
```

### Possible Responses:

**A) Success Response:**
```json
{
  "answer": "ROS 2 is...",
  "sources": [...],
  "model": "gpt-3.5-turbo"
}
```
✅ Backend works! Problem is in frontend connection.

**B) Error: RAG system not initialized**
```json
{
  "answer": "[Error] RAG system not initialized. Please configure .env..."
}
```
❌ Environment variables not set. Add them in Vercel:
- `OPENAI_API_KEY`
- `QDRANT_API_KEY`
- `QDRANT_HOST`

**C) Timeout (no response after 10+ seconds)**
❌ Vercel's 10-second timeout is too short for RAG queries
→ Solution: Deploy to Railway or Render instead

**D) Connection refused / 404**
❌ Wrong URL or backend not deployed

## Step 3: Check Frontend Configuration

### A) Verify Environment Variable

In Vercel dashboard:
1. Go to your **frontend** project
2. Settings → Environment Variables
3. Check if `REACT_APP_API_URL` exists
4. Value should be: `https://YOUR-BACKEND-URL.vercel.app` (no trailing slash)

**IMPORTANT:** After adding/changing environment variables, you MUST redeploy:
- Go to Deployments tab
- Click "..." on latest deployment
- Click "Redeploy"

### B) Check Browser Console

1. Open your live site
2. Press F12 (or right-click → Inspect)
3. Go to "Console" tab
4. Try sending a message in chat
5. Look for errors

**Common errors and fixes:**

```
Failed to fetch
```
→ Frontend can't reach backend
→ Check REACT_APP_API_URL is set correctly

```
CORS error
```
→ Unusual, backend already allows all origins
→ Check backend logs in Vercel

```
Timeout error
```
→ Backend is too slow (>10s)
→ Deploy to Railway/Render instead

## Step 4: Check Network Requests

In browser DevTools:
1. Go to "Network" tab
2. Send a message in chat
3. Look for `/chat` request
4. Click on it to see:
   - Request URL (should be your backend URL)
   - Status (200 = success, 4xx/5xx = error)
   - Response body (the actual error message)

## Step 5: View Backend Logs

```bash
vercel logs --prod
```

Or in Vercel dashboard:
1. Go to backend project
2. Click latest deployment
3. View "Runtime Logs"

Look for errors like:
- `RAG system not initialized` → Missing env vars
- `OpenAI API error` → Invalid API key
- `Qdrant connection failed` → Wrong Qdrant credentials

## Quick Fixes Checklist

- [ ] Backend deployed to Vercel
- [ ] Backend health check works: `curl https://your-backend.vercel.app/`
- [ ] Environment variables added in Vercel backend:
  - [ ] OPENAI_API_KEY
  - [ ] QDRANT_API_KEY  
  - [ ] QDRANT_HOST
- [ ] Frontend environment variable set:
  - [ ] REACT_APP_API_URL = https://your-backend.vercel.app
- [ ] Frontend redeployed after adding env var
- [ ] Browser console shows no errors
- [ ] Network tab shows request going to correct URL

## Still Not Working?

### Option A: Test Locally First

```bash
# Terminal 1 - Backend
cd backend
python main.py

# Terminal 2 - Frontend
cd frontend
export REACT_APP_API_URL=http://localhost:8000
npm start
```

If it works locally but not in production → deployment issue
If it doesn't work locally → configuration issue

### Option B: Deploy to Railway Instead

Vercel's 10-second timeout can be problematic for RAG systems.

```bash
# Install Railway CLI
npm install -g @railway/cli

# Deploy backend
cd backend
railway login
railway init
railway up

# Add environment variables in Railway dashboard
# Then update REACT_APP_API_URL in frontend to Railway URL
```

Railway has 60-second timeout (much better for AI apps).

## Need More Help?

Share these details:
1. Backend URL
2. What curl https://your-backend.vercel.app/ returns
3. Browser console errors (F12 → Console)
4. Network request details (F12 → Network → /chat request)
5. Backend logs (from Vercel dashboard or `vercel logs`)
