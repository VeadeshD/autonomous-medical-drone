@echo off
echo Starting Mission Control Dashboard...
echo Keep this window open to keep the dashboard running!
echo.
call .venv\Scripts\activate.bat
python dashboard_server.py
pause
