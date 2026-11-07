#!/usr/bin/env bash
# Runs the screenshot tests on the booted emulator at two screen sizes and pulls the PNGs into ./screens.
set -x
adb install -r app/build/outputs/apk/debug/app-debug.apk
adb install -r app/build/outputs/apk/androidTest/debug/app-debug-androidTest.apk
mkdir -p screens

run_set() {   # $1 = tag
  adb shell am instrument -w -e screenTag "$1" -e class ir.scan3d.app.ScreenshotTest \
      ir.scan3d.app.test/androidx.test.runner.AndroidJUnitRunner 2>&1 | tail -60
  for f in $(adb shell run-as ir.scan3d.app ls files/screens 2>/dev/null | tr -d '\r'); do
    adb exec-out run-as ir.scan3d.app cat "files/screens/$f" > "screens/$f"
  done
  adb shell run-as ir.scan3d.app rm -rf files/screens files/sessions
}

# pixel_5: 1080x2340 @ 440 dpi (393 x 851 dp)
run_set pixel5
# a small, older phone: 720x1280 @ 320 dpi (360 x 640 dp)
adb shell wm size 720x1280 && adb shell wm density 320 && sleep 3
run_set small
adb shell wm size reset && adb shell wm density reset

adb logcat -d 2>/dev/null | grep -E "Scan3D|AndroidRuntime|FATAL|OpenCV|TestRunner" | tail -120 > screens/logcat.txt || true
ls -la screens
