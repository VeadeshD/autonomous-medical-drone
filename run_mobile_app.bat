@echo off
echo Starting Emergency Dispatch Server (Mobile App)...
echo Keep this window open to keep the server running!
echo.
call .venv\Scripts\activate.bat
python dispatch_server.py
pause
