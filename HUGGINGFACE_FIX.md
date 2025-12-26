# Fix Your Hugging Face Spaces Chatbot

## Problem Found ✓
Your backend is deployed and running at: https://mahishahzad-textbook.hf.space

BUT it's returning this error:
```
[Error] RAG system not initialized. Please configure .env and ensure Qdrant is accessible.
```

This means the environment variables (API keys) aren't set in Hugging Face Spaces.

## Solution: Add Environment Variables to HF Space

### Step 1: Add Secrets in Hugging Face

1. Go to https://huggingface.co/spaces/mahishahzad/textbook
2. Click **Settings** tab
3. Scroll to **Repository secrets** section
4. Add these secrets (click "New secret" for each):

   **Secret 1:**
   - Name: `OPENAI_API_KEY`
   - Value: Your OpenAI API key from https://platform.openai.com/api-keys

   **Secret 2:**
   - Name: `QDRANT_API_KEY`
   - Value: Your Qdrant API key

   **Secret 3:**
   - Name: `QDRANT_HOST`
   - Value: Your Qdrant host URL (e.g., https://xxxxx.aws.cloud.qdrant.io)

5. Click **Save** for each secret

### Step 2: Restart Your Space

After adding secrets:
1. The Space should automatically restart
2. Wait 1-2 minutes for it to rebuild
3. Test again: Visit https://mahishahzad-textbook.hf.space/

You should see: `{"status":"online",...}`

### Step 3: Update Frontend Environment Variable

Your frontend needs to know about this backend URL.

**If your frontend is on Vercel:**
1. Go to Vercel dashboard → Your frontend project
2. Settings → Environment Variables
3. Add or update:
   - Name: `REACT_APP_API_URL`
   - Value: `https://mahishahzad-textbook.hf.space`
4. **Important:** Redeploy your frontend after adding this!
   - Go to Deployments tab
   - Click "..." on latest deployment
   - Click "Redeploy"

**If testing locally:**
Create `frontend/.env` file:
```bash
REACT_APP_API_URL=https://mahishahzad-textbook.hf.space
```
Then restart your frontend: `npm start`

## Test After Fix

### Test 1: Backend Health
Visit: https://mahishahzad-textbook.hf.space/
Should show: `{"status":"online",...}`

### Test 2: Backend Chat (using curl)
```bash
curl -X POST https://mahishahzad-textbook.hf.space/chat \
  -H "Content-Type: application/json" \
  -d '{"user_question":"What is physical AI?"}'
```

Should return an actual answer (not an error).

### Test 3: Frontend
1. Open your live website
2. Click the chat button
3. Ask "What is physical AI?"
4. Should get an AI-generated response

## Troubleshooting

### Issue: Still showing RAG system error
- Check that secrets are saved in HF Space settings
- Check the secret names match exactly (case-sensitive)
- Wait 2 minutes for Space to restart after adding secrets
- Check HF Space logs for errors

### Issue: Frontend still says "Failed to fetch"
- Did you add `REACT_APP_API_URL` to frontend?
- Did you redeploy frontend after adding env var?
- Check browser console (F12) for actual error

### Issue: "OpenAI API error"
- Check your OpenAI API key is valid
- Make sure you have API credits available
- Check https://platform.openai.com/usage

### Issue: "Qdrant connection failed"
- Check your Qdrant host URL is correct
- Check your Qdrant API key is valid
- Make sure your Qdrant cluster is running

## Why This Happened

Environment variables (like API keys) need to be set in the deployment platform:
- **Local development**: Uses `.env` file
- **Hugging Face Spaces**: Uses "Repository secrets" in Settings
- **Vercel**: Uses "Environment Variables" in project settings
- **Railway/Render**: Similar secrets management

You can't just copy the `.env` file - you need to set secrets in the platform's UI.

## Quick Checklist

- [ ] Added `OPENAI_API_KEY` to HF Space secrets
- [ ] Added `QDRANT_API_KEY` to HF Space secrets
- [ ] Added `QDRANT_HOST` to HF Space secrets
- [ ] Waited for HF Space to restart (1-2 min)
- [ ] Tested backend: https://mahishahzad-textbook.hf.space/
- [ ] Added `REACT_APP_API_URL` to frontend (Vercel or local .env)
- [ ] Redeployed frontend
- [ ] Tested chatbot on live site

Follow these steps and your chatbot will work! 🚀
