# IoT Demo System PowerShell Launcher
Write-Host "=================================" -ForegroundColor Cyan
Write-Host "🚀 IoT Demo System Launcher" -ForegroundColor Yellow
Write-Host "=================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "🎯 Starting all components..." -ForegroundColor Green
Write-Host ""

# Start ESP32 Simulator
Write-Host "📡 Starting ESP32 Device Simulator..." -ForegroundColor Blue
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd 'D:\SourceCode\chapter4_3_1'; D:\SourceCode\chapter4_3_1\.venv\Scripts\python.exe esp32_simulator.py"

# Wait 3 seconds
Start-Sleep -Seconds 3

# Start Web Server
Write-Host "🌐 Starting Web Dashboard Server..." -ForegroundColor Blue  
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd 'D:\SourceCode\chapter4_3_1\web\src'; python -m http.server 3000"

# Wait 3 seconds
Start-Sleep -Seconds 3

# Start Flutter App
Write-Host "📱 Starting Flutter Mobile App..." -ForegroundColor Blue
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd 'D:\SourceCode\chapter4_3_1\app_flutter\build\web'; python -m http.server 8080"

Write-Host ""
Write-Host "✅ All components started!" -ForegroundColor Green
Write-Host ""
Write-Host "🌐 Web Dashboard: " -NoNewline -ForegroundColor White
Write-Host "http://localhost:3000/index.html" -ForegroundColor Cyan
Write-Host "📱 Flutter Mobile App: " -NoNewline -ForegroundColor White  
Write-Host "http://localhost:8080/index.html" -ForegroundColor Cyan
Write-Host "🤖 ESP32 Simulator: " -NoNewline -ForegroundColor White
Write-Host "Running in background" -ForegroundColor Cyan
Write-Host ""

# Open web dashboard in default browser
Write-Host "🔗 Opening web dashboard..." -ForegroundColor Magenta
Start-Process "http://localhost:3000/index.html"

Write-Host ""
Write-Host "Press any key to exit launcher..." -ForegroundColor Yellow
Read-Host