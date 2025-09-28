@echo off
echo =================================
echo 🔍 IoT System Status Check
echo =================================
echo.

echo 🌐 Testing Web Dashboard (Port 3000)...
powershell -Command "try { Invoke-WebRequest -Uri 'http://localhost:3000/index.html' -Method HEAD -ErrorAction Stop; Write-Host '✅ Web Dashboard: ONLINE' -ForegroundColor Green } catch { Write-Host '❌ Web Dashboard: OFFLINE' -ForegroundColor Red }"

echo.
echo 📱 Testing Flutter App (Port 8080)...
powershell -Command "try { Invoke-WebRequest -Uri 'http://localhost:8080/index.html' -Method HEAD -ErrorAction Stop; Write-Host '✅ Flutter App: ONLINE' -ForegroundColor Green } catch { Write-Host '❌ Flutter App: OFFLINE' -ForegroundColor Red }"

echo.
echo 🤖 Checking Python Processes...
for /f "tokens=2" %%i in ('tasklist /FI "IMAGENAME eq python.exe" /NH 2^>nul ^| find /c "python.exe"') do (
    if %%i GTR 0 (
        echo ✅ Found %%i Python processes running
        tasklist /FI "IMAGENAME eq python.exe" /FO LIST | findstr PID
    ) else (
        echo ❌ No Python processes found
    )
)

echo.
echo 📊 System Status Summary:
echo 🌐 Web Dashboard: http://localhost:3000/index.html
echo 📱 Flutter App: http://localhost:8080/index.html  
echo 🤖 ESP32 Simulator: Background process
echo.
pause