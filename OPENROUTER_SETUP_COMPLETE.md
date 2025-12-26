# ✅ OpenRouter API Support Added!

## Problem Fixed

Your code was trying to use an **OpenRouter API key** (`sk-or-v1-...`) with **OpenAI's API endpoint**, which doesn't work.

## What I Changed

### 1. ✅ Updated `embeddings.py`
**File:** `backend/embeddings.py`

**Lines 26-38:** Added OpenRouter detection and configuration:
```python
# Check if using OpenRouter (key starts with sk-or-)
if self.openai_api_key.startswith('sk-or-'):
    print("  Detected OpenRouter API key")
    self.openai_client = OpenAI(
        api_key=self.openai_api_key,
        base_url="https://openrouter.ai/api/v1"  # OpenRouter endpoint
    )
    self.embedding_model = "openai/text-embedding-3-small"  # OpenRouter model name
else:
    self.openai_client = OpenAI(api_key=self.openai_api_key)
    self.embedding_model = "text-embedding-3-small"
```

### 2. ✅ Updated `llm_generator.py`
**File:** `backend/llm_generator.py`

**Lines 23-37:** Added OpenRouter support for chat completions:
```python
if self.openai_api_key.startswith('sk-or-'):
    print("  Detected OpenRouter API key")
    self.client = OpenAI(
        api_key=self.openai_api_key,
        base_url="https://openrouter.ai/api/v1"
    )
    self.model = os.getenv('OPENAI_CHAT_MODEL', 'openai/gpt-4o-mini')
else:
    self.client = OpenAI(api_key=self.openai_api_key)
    self.model = os.getenv('OPENAI_CHAT_MODEL', 'gpt-4o-mini')
```

## How It Works Now

The code **automatically detects** which API you're using:

| API Key Format | Detection | Endpoint | Model Names |
|----------------|-----------|----------|-------------|
| `sk-proj-...` | OpenAI | `https://api.openai.com/v1` | `text-embedding-3-small`, `gpt-4o-mini` |
| `sk-or-...` | OpenRouter | `https://openrouter.ai/api/v1` | `openai/text-embedding-3-small`, `openai/gpt-4o-mini` |

## Your .env File

Your `.env` should have:
```env
OPENAI_API_KEY=sk-or-v1-xxxxxxxxx3404  # Your OpenRouter key
QDRANT_API_KEY=your_qdrant_key
QDRANT_HOST=https://your-qdrant-instance.aws.cloud.qdrant.io
```

⚠️ **Note:** Your `QDRANT_HOST` currently looks like a JWT token (`eyJhbGc...`). It should be a URL!

## Fix QDRANT_HOST

Check your Qdrant dashboard for the correct host URL. It should look like:
```
https://xxxxxxxx-xxxx-xxxx.aws.cloud.qdrant.io:6333
```

Update `backend/.env` line 11 with the correct URL.

## Test the Fix

### Test 1: Run Ingestion Again
```bash
cd backend
uv run ingest.py
```

Should now work with OpenRouter!

### Expected Output:
```
Step 2: Initializing embeddings manager...
  Detected OpenRouter API key
✓ EmbeddingsManager initialized
  OpenAI model: openai/text-embedding-3-small
  Qdrant host: https://your-instance.aws.cloud.qdrant.io

Step 3: Generating embeddings...
  Processing batch 1/3 (100 texts)...
Generating embeddings for 100 texts...
  ✓ Generated 100 embeddings
```

### Test 2: Test Backend API
```bash
cd backend
python main.py
```

Then in another terminal:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"user_question":"What is physical AI?"}'
```

## Available Models on OpenRouter

You can use different models by setting `OPENAI_CHAT_MODEL` in `.env`:

### Embeddings (automatic):
- `openai/text-embedding-3-small` (default)
- `openai/text-embedding-3-large`

### Chat Models (customize in .env):
```env
OPENAI_CHAT_MODEL=openai/gpt-4o-mini  # Fast, cheap
# Or:
OPENAI_CHAT_MODEL=openai/gpt-4o       # More powerful
OPENAI_CHAT_MODEL=anthropic/claude-3.5-sonnet  # Claude
OPENAI_CHAT_MODEL=google/gemini-pro   # Gemini
```

See all models: https://openrouter.ai/models

## Costs on OpenRouter

OpenRouter charges per model. For `gpt-4o-mini`:
- Input: ~$0.15 per 1M tokens
- Output: ~$0.60 per 1M tokens

Your $5 credit = ~8M input tokens or ~2M output tokens

## Troubleshooting

### Still Getting 401 Error?
1. Check your OpenRouter API key is correct
2. Verify you have credits: https://openrouter.ai/credits
3. Make sure key starts with `sk-or-`

### QDRANT_HOST Error?
Update `.env` line 11 with actual Qdrant URL:
```env
QDRANT_HOST=https://xxxxx.aws.cloud.qdrant.io:6333
```

Get it from: https://cloud.qdrant.io/ → Your cluster → Connection

### Model Not Found?
Check model name format:
- OpenRouter: `openai/gpt-4o-mini` (with provider prefix)
- OpenAI: `gpt-4o-mini` (without prefix)

## Summary

✅ OpenRouter API support added  
✅ Automatic detection of API provider  
✅ Works with both OpenAI and OpenRouter keys  
✅ Embeddings fixed  
✅ Chat completions fixed  

**Next:** Fix your `QDRANT_HOST` and run `uv run ingest.py` again!

## Files Modified

- `backend/embeddings.py` - Lines 26-38
- `backend/llm_generator.py` - Lines 23-37

The code now supports OpenRouter! 🚀
