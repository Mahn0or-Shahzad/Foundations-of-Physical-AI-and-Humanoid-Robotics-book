@echo off
REM Quick start script for Windows

echo Starting Physical AI Book RAG Chatbot Backend...
echo.

REM Check if virtual environment exists
if not exist "venv\Scripts\activate.bat" (
    echo Creating virtual environment...
    python -m venv venv
    echo.
)

REM Activate virtual environment
call venv\Scripts\activate.bat

REM Install dependencies if needed
echo Checking dependencies...
pip install -q -r requirements.txt

REM Start FastAPI server
echo.
echo Starting FastAPI server on http://localhost:8000
echo API docs available at http://localhost:8000/docs
echo.
echo Press Ctrl+C to stop the server
echo.

uvicorn main:app --reload --port 8000
