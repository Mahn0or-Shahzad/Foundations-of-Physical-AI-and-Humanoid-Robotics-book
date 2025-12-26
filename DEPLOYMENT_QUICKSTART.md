# Quick Deployment Guide

Your chatbot is currently showing "Failed to fetch" because it's trying to connect to localhost (your local machine), which doesn't work when deployed on Vercel.

## Solution: Deploy Your Backend to Vercel

Follow these 3 simple steps:

### 1. Install Vercel CLI

```bash
npm install -g vercel
```

### 2. Deploy Backend

```bash
cd backend
vercel login
vercel
```

Follow the prompts:
- When asked for environment variables, add:
  - `OPENAI_API_KEY`: Your OpenAI API key
  - `QDRANT_API_KEY`: Your Qdrant API key  
  - `QDRANT_HOST`: Your Qdrant host URL

After deployment, Vercel will give you a URL like:
`https://your-app-name.vercel.app`

### 3. Update Frontend Environment Variable

In your Vercel frontend project:
1. Go to Settings → Environment Variables
2. Add: `REACT_APP_API_URL` = `https://your-backend-app.vercel.app`
3. Redeploy the frontend

That's it! Your chatbot should now work on the live site.

## Full Documentation

See `backend/VERCEL_DEPLOYMENT.md` for detailed instructions and troubleshooting.

## Alternative Deployment Options

If Vercel doesn't work for your use case:
- **Railway** (https://railway.app) - Better for longer execution times
- **Render** (https://render.com) - Free tier with generous limits
- **Hugging Face Spaces** - Great for ML/AI applications

All options work with your existing code - just follow their Python deployment guides.

## Testing Locally

Before deploying, test locally:

```bash
# Terminal 1 - Start backend
cd backend
python main.py

# Terminal 2 - Start frontend  
cd frontend
npm start
```

Visit http://localhost:3000 and test the chatbot.
