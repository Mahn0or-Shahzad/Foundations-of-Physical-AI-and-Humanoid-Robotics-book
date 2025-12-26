# ✅ FIXED: Vercel Backend Deployment Guide

## Problem Solved
The error `Environment Variable "OPENAI_API_KEY" references Secret "openai_api_key", which does not exist` was caused by `vercel.json` using `@secret_name` syntax.

**I removed the `env` section from `vercel.json`** - now you add environment variables through Vercel Dashboard instead.

## How to Deploy Backend to Vercel

### Step 1: Install Vercel CLI (if not installed)
```bash
npm install -g vercel
```

### Step 2: Login to Vercel
```bash
cd backend
vercel login
```

### Step 3: Deploy
```bash
vercel
```

When prompted:
- **Set up and deploy?** → Yes
- **Which scope?** → Your account
- **Link to existing project?** → No (first time) or Yes (if exists)
- **What's your project's name?** → `ai-book-backend` (or any name)
- **In which directory is your code located?** → `./`
- **Want to override settings?** → No

Wait for deployment to complete. You'll get a URL like:
```
https://ai-book-backend-xxx.vercel.app
```

### Step 4: Add Environment Variables in Vercel Dashboard

**IMPORTANT:** Don't use CLI, use the Dashboard:

1. Go to: https://vercel.com/dashboard
2. Click your backend project (`ai-book-backend`)
3. Click **Settings** tab
4. Scroll to **Environment Variables**
5. Add each variable:

   **Variable 1:**
   - Key: `OPENAI_API_KEY`
   - Value: `your_openai_api_key_here` (get from backend/.env file)
   - Environment: Production, Preview, Development (select all)
   - Click **Save**

   **Variable 2:**
   - Key: `QDRANT_API_KEY`
   - Value: `your_qdrant_api_key_here` (get from backend/.env file)
   - Environment: Production, Preview, Development (select all)
   - Click **Save**

   **Variable 3:**
   - Key: `QDRANT_HOST`
   - Value: `your_qdrant_host_url_here` (get from backend/.env file)
   - Environment: Production, Preview, Development (select all)
   - Click **Save**

   ⚠️ **Note:** Your `QDRANT_HOST` looks like a JWT token. It should be a URL like `https://xxxxx.aws.cloud.qdrant.io`. Check your Qdrant dashboard for the correct host URL.

### Step 5: Redeploy After Adding Variables

**CRITICAL:** Environment variables only apply after redeployment.

1. Go to **Deployments** tab
2. Click **"..."** on the latest deployment
3. Click **Redeploy**
4. Wait for it to complete

### Step 6: Test Your Backend

```bash
# Test health endpoint
curl https://your-backend-url.vercel.app/

# Should return:
# {"status":"online","service":"Physical AI Book RAG Chatbot","version":"1.0.0"}

# Test chat endpoint
curl -X POST https://your-backend-url.vercel.app/chat \
  -H "Content-Type: application/json" \
  -d '{"user_question":"What is physical AI?"}'

# Should return an answer (not error)
```

### Step 7: Update Frontend to Use Vercel Backend

**Option A: In Vercel Dashboard (if frontend on Vercel)**

1. Go to your **frontend** project in Vercel
2. Settings → Environment Variables
3. Add:
   - Key: `REACT_APP_API_URL`
   - Value: `https://your-backend-url.vercel.app` (your actual backend URL)
   - Environment: All
4. Save
5. Redeploy frontend

**Option B: Update Code (fallback URL)**

The frontend already defaults to HF Space URL. If you want to use Vercel backend instead, it will automatically use the `REACT_APP_API_URL` from Vercel environment variables.

## Important Notes

### Vercel Limitations for FastAPI

⚠️ **Vercel has a 10-second timeout** for serverless functions. If your RAG system takes longer than 10 seconds:

- The request will fail with timeout error
- Consider using **Hugging Face Spaces** or **Railway** instead
- Both have longer timeout limits (60+ seconds)

### Why I Recommend Hugging Face Spaces Instead

Your backend is **already deployed** on HF Spaces:
- URL: `https://mahishahzad-textbook.hf.space`
- Better for AI/ML workloads
- Longer timeout (60 seconds)
- Free tier is generous

**Just make sure:**
1. HF Space is running (not sleeping)
2. Environment variables are set in HF Space Settings
3. Frontend uses `REACT_APP_API_URL=https://mahishahzad-textbook.hf.space`

## Files Modified

✅ `backend/vercel.json` - Removed `env` section with secret references

## Troubleshooting

### "Environment Variable references Secret"
**Fixed!** I removed the `env` section. Add variables through Dashboard instead.

### Deployment fails with "Module not found"
- Check `requirements.txt` has all dependencies
- Vercel uses Python 3.9 (specified in `vercel.json`)

### Backend returns 500 error
- Check Vercel logs: `vercel logs` or in Dashboard → Deployments → Logs
- Usually means missing environment variables
- Make sure you redeployed after adding variables

### Frontend can't connect to backend
- Did you add `REACT_APP_API_URL` to frontend?
- Did you redeploy frontend after adding variable?
- Check browser console (F12) for actual error

### Works locally but not on Vercel
- Environment variables: Set in Dashboard, then redeploy
- CORS: Your `main.py` already allows all origins
- Timeout: Vercel limit is 10 seconds

## Recommendation

**Use Hugging Face Spaces for backend** (better for AI workloads):
- Already deployed: `https://mahishahzad-textbook.hf.space`
- Just add environment variables in HF Settings
- Frontend already defaults to this URL

**Use Vercel only if:**
- Your RAG queries complete in < 10 seconds
- You need custom domain
- You prefer Vercel ecosystem

## Quick Commands

```bash
# Deploy to Vercel
cd backend
vercel --prod

# View logs
vercel logs

# Check deployment
curl https://your-backend-url.vercel.app/
```

The error is fixed! You can now deploy without secret issues. 🚀
