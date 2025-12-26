# ✅ FIXED: "process is not defined" Error in Docusaurus

## Problem
Docusaurus runs in the browser where `process` doesn't exist, causing:
```
ReferenceError: process is not defined
```

## What I Fixed

### 1. Updated ChatWidget Component (frontend/src/components/ChatWidget/index.js)

**Changed line 54 from:**
```javascript
const apiUrl = process.env.REACT_APP_API_URL || 'http://localhost:8000';
```

**To:**
```javascript
const apiUrl = (typeof process !== 'undefined' && process.env?.REACT_APP_API_URL)
  || 'https://mahishahzad-textbook.hf.space';
```

**Also updated line 87** (error message) with the same fix.

This safely checks if `process` exists before accessing it, and defaults to your HF Space URL.

### 2. Added Environment Variable Config (frontend/docusaurus.config.js)

Added after line 20:
```javascript
// Custom fields for environment variables
customFields: {
  REACT_APP_API_URL: process.env.REACT_APP_API_URL || 'https://mahishahzad-textbook.hf.space',
},
```

This makes the environment variable available in Docusaurus build.

## How to Deploy the Fix

### Option 1: Deploy to Vercel (Recommended)

**Step 1: Commit and push changes**
```bash
cd frontend
git add .
git commit -m "Fix process.env error for Docusaurus"
git push
```

**Step 2: Set environment variable in Vercel**
1. Go to https://vercel.com/dashboard
2. Select your frontend project
3. Settings → Environment Variables
4. Add:
   - **Name:** `REACT_APP_API_URL`
   - **Value:** `https://mahishahzad-textbook.hf.space`
   - **Environments:** Production, Preview, Development (all)
5. Save

**Step 3: Redeploy**
- Go to Deployments tab
- Latest deployment → Click "..." → Redeploy
- Wait for build to complete

### Option 2: Test Locally First

```bash
cd frontend

# Set environment variable
export REACT_APP_API_URL=https://mahishahzad-textbook.hf.space

# Install dependencies (if needed)
npm install

# Start dev server
npm start
```

Visit http://localhost:3000 and test the chatbot.

## What This Fix Does

1. **Safely checks for `process`** - Uses `typeof process !== 'undefined'` to avoid errors
2. **Uses optional chaining** - `process.env?.REACT_APP_API_URL` won't crash if undefined
3. **Defaults to HF Space URL** - Your backend URL is hardcoded as fallback
4. **Works in browser** - No more "process is not defined" errors

## Expected Behavior After Fix

✅ No more "process is not defined" error  
✅ Chatbot button appears on page  
✅ Can open chat widget  
✅ Can type messages  
✅ Messages send to: `https://mahishahzad-textbook.hf.space/chat`

## If Chatbot Still Doesn't Respond

After this fix, if you still get errors, check:

### 1. Backend is Running
```bash
curl https://mahishahzad-textbook.hf.space/
```
Should return: `{"status":"online",...}`

If 404, your HF Space isn't deployed. See previous guide: `EXACT_FIX_GUIDE.md`

### 2. CORS Issue
Open browser console (F12 → Console) and look for:
```
Access-Control-Allow-Origin
```

If you see CORS errors, your backend needs to allow your Vercel domain in CORS config.

### 3. Network Request
F12 → Network tab → Send message → Check `/chat` request:
- **Status 200** = Success
- **Status 404** = Backend not found
- **Status 500** = Backend error (check HF Space logs)
- **Status 0 / Failed** = CORS or network issue

## Testing Checklist

After deploying:

- [ ] Visit: https://foundations-of-physical-ai-and-huma.vercel.app/
- [ ] Open browser console (F12)
- [ ] Click chat button (💬)
- [ ] No "process is not defined" error ✓
- [ ] Type a message and send
- [ ] Check Network tab for `/chat` request
- [ ] Verify response received

## Files Modified

```
frontend/
  ├── src/components/ChatWidget/index.js (FIXED - line 54, 87)
  └── docusaurus.config.js (ADDED - customFields)
```

## Next Steps

1. **Commit changes** to git
2. **Push to GitHub** (if using git)
3. **Vercel will auto-deploy** (if connected to GitHub)
4. **Or manually deploy** using Vercel CLI: `vercel --prod`
5. **Test on live site**

The "process is not defined" error is now fixed! 🎉
