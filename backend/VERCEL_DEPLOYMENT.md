# Deploy Backend to Vercel

This guide walks you through deploying your RAG chatbot backend to Vercel.

## Prerequisites

1. Install Vercel CLI:
```bash
npm install -g vercel
```

2. Sign up for a free Vercel account at https://vercel.com

3. Have your API keys ready:
   - OpenAI API key (from https://platform.openai.com/api-keys)
   - Qdrant API key and host URL (from https://qdrant.tech/)

## Step 1: Login to Vercel

```bash
vercel login
```

Follow the prompts to authenticate.

## Step 2: Set Environment Variables

You need to add your environment variables as Vercel secrets:

```bash
# Add OpenAI API key
vercel env add OPENAI_API_KEY

# Add Qdrant credentials
vercel env add QDRANT_API_KEY
vercel env add QDRANT_HOST
```

When prompted, paste your actual keys/values and select:
- Environment: Production, Preview, Development (select all)

## Step 3: Deploy

From the `backend` directory, run:

```bash
vercel
```

Follow the prompts:
- Set up and deploy? **Y**
- Which scope? Select your account
- Link to existing project? **N** (first time)
- What's your project's name? `ai-book-chatbot-backend` (or your choice)
- In which directory is your code located? `./`
- Want to override settings? **N**

Vercel will:
1. Build your Python application
2. Deploy it to a serverless function
3. Give you a production URL (e.g., `https://ai-book-chatbot-backend.vercel.app`)

## Step 4: Test Your Deployed Backend

Test the health endpoint:
```bash
curl https://your-app-name.vercel.app/
```

You should see:
```json
{
  "status": "online",
  "service": "Physical AI Book RAG Chatbot",
  "version": "1.0.0"
}
```

Test the chat endpoint:
```bash
curl -X POST https://your-app-name.vercel.app/chat \
  -H "Content-Type: application/json" \
  -d '{"user_question": "What is ROS 2?"}'
```

## Step 5: Update Frontend

Now that your backend is deployed, update your frontend to use the production URL.

### For Vercel-deployed frontend:

Add an environment variable in your Vercel project settings:
1. Go to your frontend project on Vercel dashboard
2. Settings → Environment Variables
3. Add: `REACT_APP_API_URL` = `https://your-backend-app.vercel.app`
4. Redeploy your frontend

### For local development:

Create `.env` in the frontend directory:
```bash
REACT_APP_API_URL=https://your-backend-app.vercel.app
```

Then rebuild:
```bash
npm run build
```

## Troubleshooting

### Build fails
- Check that `requirements.txt` has all dependencies with versions
- Ensure Python version in `vercel.json` matches your local environment

### Timeout errors
- Vercel serverless functions have a 10-second timeout on free tier
- If RAG queries take too long, consider upgrading or optimizing retrieval

### CORS errors
- The backend already has CORS enabled for all origins
- If issues persist, check browser console for specific error messages

### Environment variables not working
- Ensure you added them using `vercel env add` (not just in .env file)
- Redeploy after adding environment variables

## Updating Your Deployment

After making code changes:

```bash
# Deploy to production
vercel --prod

# Or just deploy to preview
vercel
```

## View Logs

To debug issues:
```bash
vercel logs
```

Or view them in the Vercel dashboard.

## Cost Considerations

Vercel's free tier includes:
- 100 GB-hrs serverless function execution
- 100k serverless function invocations
- Unlimited deployments

For a chatbot with moderate usage, this should be sufficient for testing and small-scale production.

## Alternative: Railway or Render

If you need longer execution times or prefer traditional hosting:

### Railway (https://railway.app)
- Better for long-running processes
- Simple Python deployment
- Free tier: $5 credit/month

### Render (https://render.com)
- Free tier for web services
- Longer timeout limits
- Great for Python apps

Both alternatives use standard Python deployments with `requirements.txt` and can run `uvicorn` directly.
