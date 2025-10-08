@echo off
echo =================================
echo 🚀 IoT Demo System Launcher (Fixed for Portability)
echo =================================
echo.

echo 🎯 Starting all components...
echo.

REM --- Chạy ESP32 Simulator trong một cửa sổ mới ---
REM Sử dụng đường dẫn tương đối. Lệnh "python" sẽ tự động dùng môi trường .venv đã được kích hoạt.
echo 📡 Starting ESP32 Device Simulator...
start "ESP32 Simulator" cmd /k "python simulators\esp32_simulator.py"

REM Đợi 1 giây để các cửa sổ kịp hiện lên
timeout /t 1 /nobreak >nul

REM --- Chạy Web Dashboard Server trong một cửa sổ mới ---
REM Di chuyển vào thư mục con web/src một cách tương đối và khởi động server.
echo 🌐 Starting Web Dashboard Server...
start "Web Dashboard" cmd /k "cd web\src && python -m http.server 3000"

REM Đợi 1 giây
timeout /t 1 /nobreak >nul

REM --- Chạy Flutter App Server trong một cửa sổ mới ---
REM Di chuyển vào thư mục con app_flutter/build/web và khởi động server.
REM LƯU Ý: Thư mục 'build\web' chỉ tồn tại sau khi bạn đã chạy 'flutter build web'.
REM Nếu lệnh này lỗi, hãy chạy 'flutter build web' trong thư mục 'app_flutter' trước.
echo 📱 Starting Flutter Mobile App...
start "Flutter App" cmd /k "cd app_flutter\build\web && python -m http.server 8080"

echo.
echo ✅ All components started!
echo.
echo 🌐 Web Dashboard: http://localhost:3000
echo 📱 Flutter Mobile App: http://localhost:8080
echo 🤖 ESP32 Simulator: Running in background
echo.
echo Press any key to exit launcher...
pause >nul