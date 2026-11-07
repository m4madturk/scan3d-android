# Scan3D Android

Native capture app for the marker-based 3D scanner (pairs with `scan3d_server`).

What it does that a web page cannot: locks focus/exposure/white balance, turns video and
optical stabilisation off, captures at full sensor resolution, detects ArUco stickers and
the ChArUco board live, takes the shot automatically when the phone is still, and has a
guided camera-calibration mode. All photos and measurements go to your own server, which
builds the STL and (for calibration sessions) a per-phone camera profile.

Build: pushed to `main` -> GitHub Actions -> artifact `scan3d-apk` (debug-signed, installable).
