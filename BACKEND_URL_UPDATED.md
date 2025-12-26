# ✅ Backend URL Updated to Vercel

## Changes Made

Your chatbot now uses your **Vercel backend** instead of Hugging Face Spaces!

### Backend URL Changed:
```
OLD: https://mahishahzad-textbook.hf.space
NEW: https://foundations-of-physical-ai-and-huma-one.vercel.app
```

## Files Updated

### 1. ✅ ChatWidget Component
**File:** `frontend/src/components/ChatWidget/index.js`

**Line 55 (API call):**
```javascript
const apiUrl = (typeof process !== 'undefined' && process.env?.REACT_APP_API_URL)
  || 'https://foundations-of-physical-ai-and-huma-one.vercel.app';
```

**Line 88 (Error message):**
```javascript
const apiUrl = (typeof process !== 'undefined' && process.env?.REACT_APP_API_URL)
  || 'https://foundations-of-physical-ai-and-huma-one.vercel.app';
```

### 2. ✅ Docusaurus Config
**File:** `frontend/docusaurus.config.js`

**Line 24:**
```javascript
customFields: {
  REACT_APP_API_URL: process.env.REACT_APP_API_URL || 'https://foundations-of-physical-ai-and-huma-one.vercel.app',
},
```

## How It Works

### Default Behavior (Production)
Your chatbot will connect to:
```
https://foundations-of-physical-ai-and-huma-one.vercel.app/chat
```

### With Environment Variable (Optional)
If you set `REACT_APP_API_URL` in Vercel, it will use that instead.

### Local Development
For local testing, you can override:
```bash
# In frontend directory
echo "REACT_APP_API_URL=http://localhost:8000" > .env
npm start
```

## Deploy Changes

### Option 1: Auto-deploy (GitHub Connected)
```bash
git add .
git commit -m "Update backend URL to Vercel"
git push
```

Vercel will auto-deploy your frontend.

### Option 2: Manual Deploy
```bash
cd frontend
npm run build
vercel --prod
```

## Test Backend First

Before deploying frontend, verify your Vercel backend works:

### Test 1: Health Check
```bash
curl https://foundations-of-physical-ai-and-huma-one.vercel.app/

# Should return:
# {"status":"online","service":"Physical AI Book RAG Chatbot","version":"1.0.0"}
```

### Test 2: Chat Endpoint
```bash
curl -X POST https://foundations-of-physical-ai-and-huma-one.vercel.app/chat \
  -H "Content-Type: application/json" \
  -d '{"user_question":"What is physical AI?"}'

# Should return an answer (not error)
```

### If Tests Fail:
1. **Check backend deployment:** Go to Vercel dashboard → Backend project
2. **Check environment variables:** Settings → Environment Variables
   - `OPENAI_API_KEY` must be set
   - `QDRANT_API_KEY` must be set
   - `QDRANT_HOST` must be set
3. **Check logs:** Deployments → Click latest → View Function Logs
4. **Redeploy backend:** If you just added env vars

## Environment Variables (Optional)

### Frontend Vercel Project
If you want to override the default URL:

1. Go to: https://vercel.com/dashboard
2. Select your **frontend** project
3. Settings → Environment Variables
4. Add:
   - Key: `REACT_APP_API_URL`
   - Value: `https://foundations-of-physical-ai-and-huma-one.vercel.app`
   - Environment: All
5. Redeploy frontend

But this is **optional** since the URL is already hardcoded as fallback.

## Backend Requirements

Your Vercel backend must have these environment variables:

| Variable | Required | Example |
|----------|----------|---------|
| `OPENAI_API_KEY` | ✅ Yes | `sk-proj-...` |
| `QDRANT_API_KEY` | ✅ Yes | `eyJhbGc...` |
| `QDRANT_HOST` | ✅ Yes | `https://xxx.aws.cloud.qdrant.io` |

Check in: Vercel Dashboard → Backend project → Settings → Environment Variables

## Testing After Deploy

### Test 1: Visit Your Site
```
https://foundations-of-physical-ai-and-huma.vercel.app/
```

### Test 2: Open Chatbot
Click the 💬 button in bottom right

### Test 3: Send Message
Type: "What is physical AI?"

### Test 4: Check Response
Should see AI-generated answer (not error)

### Test 5: Check Network (F12)
- Open browser DevTools (F12)
- Go to Network tab
- Send a message
- Look for `/chat` request
- Should show:
  - Request URL: `https://foundations-of-physical-ai-and-huma-one.vercel.app/chat`
  - Status: 200 (success)
  - Response: JSON with answer

## Troubleshooting

### Issue: "Failed to fetch"
**Cause:** Backend is not running or wrong URL  
**Fix:** 
1. Test backend URL manually (curl commands above)
2. Check Vercel backend deployment status
3. Check backend logs for errors

### Issue: "RAG system not initialized"
**Cause:** Missing environment variables in backend  
**Fix:**
1. Go to Vercel → Backend project → Settings → Environment Variables
2. Add: `OPENAI_API_KEY`, `QDRANT_API_KEY`, `QDRANT_HOST`
3. Redeploy backend

### Issue: CORS error
**Cause:** Backend doesn't allow frontend origin  
**Fix:** Check `backend/main.py` line 28:
```python
allow_origins=["*"],  # Should allow all origins
```

### Issue: Timeout (10+ seconds)
**Cause:** Vercel has 10-second limit for serverless functions  
**Fix:** 
- Optimize RAG queries
- Or use Hugging Face Spaces instead (60s limit)

## Summary

✅ Frontend now points to: `https://foundations-of-physical-ai-and-huma-one.vercel.app`  
✅ Fallback URL updated in code  
✅ Docusaurus config updated  
✅ Ready to deploy  

**Next steps:**
1. Test backend (curl commands above)
2. Commit and push changes
3. Vercel auto-deploys frontend
4. Test chatbot on live site

Your chatbot is now configured for Vercel backend! 🚀
