# Fix Qdrant Connection Error

## Current Issue

Your `QDRANT_HOST` is set to a JWT token:
```
eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

This should be a **URL** to your Qdrant cluster!

## How to Get Your Qdrant URL

### Option 1: Qdrant Cloud (Free Tier)

1. **Go to:** https://cloud.qdrant.io/
2. **Login** to your account
3. **Select your cluster**
4. **Find the Connection Details:**
   - Look for "REST URL" or "Cluster URL"
   - Should look like: `https://xxxxxxxx-xxxx-xxxx.us-east-1-0.aws.cloud.qdrant.io:6333`
5. **Copy that URL**

### Option 2: Create New Qdrant Cluster (If Don't Have One)

1. Go to: https://cloud.qdrant.io/
2. Sign up / Login
3. Click "Create Cluster"
4. Choose:
   - Name: `physical-ai-book`
   - Region: Closest to you (e.g., `us-east-1`)
   - Tier: **Free** (1 GB)
5. Wait for cluster to create (~1 minute)
6. Copy the **REST URL**
7. Copy the **API Key** (different from the URL!)

## Update Your .env File

**File:** `backend/.env`

Update line 10 and 11:
```env
# OLD (WRONG):
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.H0VU2QElIJy2MRRMO6F5mUBYslPfEvH_LkayUuOwFNMy
QDRANT_HOST=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.H0VU2QElIJy2MRRMO6F5mUBYslPfEvH_LkayUuOwFNM

# NEW (CORRECT):
QDRANT_API_KEY=your_actual_qdrant_api_key_here
QDRANT_HOST=https://xxxxx-xxxxx-xxxxx.us-east-1-0.aws.cloud.qdrant.io:6333
```

### Example (don't copy, use yours):
```env
QDRANT_API_KEY=qdr_AbCdEfGhIjKlMnOpQrStUvWxYz1234567890
QDRANT_HOST=https://a1b2c3d4-e5f6-7890.us-east-1-0.aws.cloud.qdrant.io:6333
```

## Option 3: Use Local Qdrant (If You Don't Want Cloud)

### Install Qdrant Locally with Docker:
```bash
docker run -p 6333:6333 qdrant/qdrant
```

### Update .env:
```env
QDRANT_API_KEY=  # Leave empty for local
QDRANT_HOST=http://localhost:6333
```

## Test Connection

After updating .env, test:

```bash
cd backend
python -c "from qdrant_client import QdrantClient; client = QdrantClient(url='YOUR_QDRANT_HOST', api_key='YOUR_API_KEY'); print('✓ Connected!'); print(client.get_collections())"
```

Replace `YOUR_QDRANT_HOST` and `YOUR_API_KEY` with your actual values.

## Run Ingestion Again

After fixing `.env`:

```bash
cd backend
python -c "import os; os.environ['PYTHONIOENCODING']='utf-8'; import subprocess; subprocess.run(['uv', 'run', 'ingest.py'])"
```

Should complete successfully!

## What You Should See:

```
Step 3: Generating embeddings...
  ✓ Generated 239 embeddings

Step 4: Storing embeddings in Qdrant...
Creating Qdrant collection: physical_ai_book
  ✓ Collection created successfully
Storing 239 embeddings in Qdrant...
  ✓ Stored 239 embeddings successfully

============================================================
✓ Ingestion complete!
============================================================
Summary:
  - Loaded: 239 documents
  - Generated: 239 embeddings  
  - Stored in: physical_ai_book collection
  - Ready for: RAG retrieval
```

## Summary of What to Fix

1. **Get your Qdrant cluster URL** from https://cloud.qdrant.io/
2. **Update `backend/.env` lines 10-11** with:
   - Correct API key (not the JWT token)
   - Correct host URL (not the JWT token)
3. **Run ingestion again**

The embeddings generation is working perfectly with OpenRouter! Just need to fix the Qdrant connection. 🚀
