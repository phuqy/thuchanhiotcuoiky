@echo off
echo =================================
echo 🔨 Flutter App Builder 
echo =================================
echo.

echo 🏗️  Building Flutter app for web...
cd /d "D:\SourceCode\chapter4_3_1\app_flutter"
D:\Downdload\flutter_windows_3.35.4-stable\flutter\bin\flutter.bat build web

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ✅ Flutter app built successfully!
    echo 📁 Output: app_flutter\build\web
    echo.
    echo 🚀 You can now run: run_all.bat
    echo 🌐 Or access directly: http://localhost:8080
) else (
    echo.
    echo ❌ Flutter build failed!
    echo.
)

echo.
echo Press any key to exit...
pause >nul