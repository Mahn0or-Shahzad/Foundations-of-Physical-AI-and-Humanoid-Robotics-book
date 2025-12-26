#!/bin/bash

echo "=========================================="
echo "  Backend Deployment to Vercel"
echo "=========================================="
echo ""

# Check if vercel CLI is installed
if ! command -v vercel &> /dev/null
then
    echo "❌ Vercel CLI not found."
    echo "Install it with: npm install -g vercel"
    exit 1
fi

echo "✓ Vercel CLI found"
echo ""

# Check if environment variables are set
if [ ! -f .env ]; then
    echo "⚠️  Warning: No .env file found"
    echo "Make sure to add environment variables in Vercel dashboard"
    echo ""
fi

echo "Starting deployment..."
echo ""
echo "When prompted:"
echo "  1. Add environment variables (OPENAI_API_KEY, QDRANT_API_KEY, QDRANT_HOST)"
echo "  2. Select all environments (Production, Preview, Development)"
echo ""
echo "Press Enter to continue..."
read

# Deploy to Vercel
vercel --prod

echo ""
echo "=========================================="
echo "  Deployment Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Copy your deployment URL from above"
echo "2. Add REACT_APP_API_URL to your frontend Vercel project"
echo "3. Redeploy your frontend"
echo ""
echo "Test your deployment:"
echo "  curl https://your-app.vercel.app/"
echo ""
