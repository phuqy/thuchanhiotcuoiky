@echo off
echo =================================
echo 🌐 Opening IoT Demo Interfaces
echo =================================
echo.

echo 🔗 Opening Web Dashboard...
start "" "http://localhost:3000/index.html"

timeout /t 2 /nobreak >nul

echo 🔗 Opening Flutter Mobile App...
start "" "http://localhost:8080/index.html"

echo.
echo ✅ Both interfaces opened in browser!
echo.
echo 📱 Web Dashboard: Modern gradient design
echo 📱 Flutter App: Material Design 3 interface
echo 🤖 ESP32 Data: Real-time sensor updates via MQTT
echo.
pause