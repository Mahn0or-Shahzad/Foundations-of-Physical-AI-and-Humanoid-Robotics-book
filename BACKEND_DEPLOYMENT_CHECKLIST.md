# Backend Deployment Checklist

## Current Status

✅ Backend deployed: https://ai-book-dusky-seven.vercel.app/  
❌ Environment variables: Not set (RAG system not initialized)

## Issue

The backend returns:
```json
{"answer":"[Error] RAG system not initialized. Please configure .env and ensure Qdrant is accessible."}
```

This means environment variables are missing in Vercel.

## Fix: Add Environment Variables to Vercel

### Step 1: Go to Vercel Dashboard

1. Visit: https://vercel.com/dashboard
2. Find your backend project (probably named `ai-book` or similar)
3. Click on the project

### Step 2: Add Environment Variables

1. Go to **Settings** tab
2. Click **Environment Variables** in sidebar
3. Add these 3 variables:

#### Variable 1: OPENAI_API_KEY
- **Key:** `OPENAI_API_KEY`
- **Value:** `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`
- **Environment:** Production, Preview, Development (select all)
- Click **Save**

#### Variable 2: QDRANT_API_KEY
- **Key:** `QDRANT_API_KEY`
- **Value:** `eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.lhAUUCkQf4sT9Sl5HMOB6n9lC3B4ibL-yQjBUERyzNw`
- **Environment:** Production, Preview, Development (select all)
- Click **Save**

#### Variable 3: QDRANT_HOST
- **Key:** `QDRANT_HOST`
- **Value:** `https://06d74ebd-25df-47f0-8c98-7cd86ecde645.sa-east-1-0.aws.cloud.qdrant.io:6333`
- **Environment:** Production, Preview, Development (select all)
- Click **Save**

### Step 3: Redeploy Backend

**CRITICAL:** Environment variables only apply after redeployment!

1. Go to **Deployments** tab
2. Find the latest deployment
3. Click **"..."** (three dots)
4. Click **Redeploy**
5. Wait for deployment to complete (1-2 minutes)

### Step 4: Test Backend

After redeployment:

```bash
curl -X POST https://ai-book-dusky-seven.vercel.app/chat \
  -H "Content-Type: application/json" \
  -d '{"user_question":"What is ROS 2?"}'
```

Should return an actual AI-generated answer, not an error!

### Expected Success Response:
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is...",
  "sources": ["module1-ros2/ros2-architecture.md"],
  "model": "openai/gpt-4o-mini"
}
```

## Alternative: Use Vercel CLI

If you prefer CLI:

```bash
cd backend
vercel env add OPENAI_API_KEY production
# Paste your OpenRouter key when prompted

vercel env add QDRANT_API_KEY production
# Paste your Qdrant API key

vercel env add QDRANT_HOST production
# Paste your Qdrant host URL

# Redeploy
vercel --prod
```

## Frontend Update

Your frontend code is already updated to use:
```
https://ai-book-dusky-seven.vercel.app
```

Just commit and push:

```bash
git add frontend/src/components/ChatWidget/index.js frontend/docusaurus.config.js
git commit -m "Update frontend to use new backend URL"
git push origin main
```

Vercel will auto-deploy the frontend.

## Complete Testing Flow

Once backend environment variables are set:

### Test 1: Backend Health
```bash
curl https://ai-book-dusky-seven.vercel.app/
# Should return: {"status":"online",...}
```

### Test 2: Backend Chat
```bash
curl -X POST https://ai-book-dusky-seven.vercel.app/chat \
  -H "Content-Type: application/json" \
  -d '{"user_question":"What is physical AI?"}'
# Should return AI answer
```

### Test 3: Frontend Live
1. Visit: https://foundations-of-physical-ai-and-huma.vercel.app/
2. Click chat button (💬)
3. Ask: "What is ROS 2?"
4. Should get AI response!

## Troubleshooting

### Still Getting "RAG system not initialized"?
- Did you add all 3 environment variables?
- Did you redeploy after adding them?
- Check Vercel logs: Deployments → Click deployment → Function Logs

### Timeout errors?
- Vercel has 10-second limit
- Your RAG queries might be too slow
- Check Vercel logs for details

### CORS errors in browser?
- Backend already allows all origins
- Should not be an issue

## Summary

1. ✅ Backend deployed: https://ai-book-dusky-seven.vercel.app/
2. ⏳ Add 3 environment variables in Vercel Dashboard
3. ⏳ Redeploy backend
4. ✅ Frontend updated to new backend URL
5. ⏳ Push frontend changes
6. ✅ Test complete system

After adding environment variables and redeploying, your chatbot will be fully operational! 🚀
