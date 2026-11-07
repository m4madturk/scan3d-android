# Scan3D — marker-based 3D scanner, fully on the phone

Android app (Kotlin, CameraX, OpenCV) that turns photos of an object covered in printed 12 mm
ArUco stickers, standing on a ChArUco board, into an STL. Nothing leaves the phone: detection,
the bundle-adjustment solver, lens calibration, silhouette carving, marching cubes and the STL
writer all run inside the app (`engine/` is pure Kotlin and unit-tested on the JVM).

## Layout
- `app/src/main/java/ir/scan3d/app/engine/` — the maths, no Android dependencies
  - `Geometry.kt` camera model (f, cx, cy, k1, k2), rotations, linear algebra
  - `Pnp.kt` planar PnP (homography init + LM)
  - `Solver.kt` marker-framework bundle adjustment with Huber loss, 3-sigma rejection, covariances
  - `Calibrator.kt` board-only self-calibration with model selection and per-parameter sigma
  - `Silhouette.kt` board-template comparison rendered from the solved pose
  - `Carve.kt` signed-distance visual hull with marker tangent-plane cuts
  - `Mesh.kt` marching cubes + binary STL
- `Detector.kt` OpenCV ArUco/ChArUco detection with sub-pixel refinement (sensor frame)
- `CameraController.kt` CameraX with stabilisation/noise reduction off, AE/AWB/AF lock
- `Pipeline.kt`, `BuildService.kt` the end-to-end build as a foreground service
- `Store.kt` sessions and lens profiles as JSON under the app's private files
- `MainActivity` capture + guidance, `ResultActivity` progress/preview/STL share, `SessionsActivity` history

## Build
GitHub Actions (`.github/workflows/build.yml`):
- `build`: JVM engine tests, release APK (debug-signed, ARM only), committed to the `apk` branch
- `screens`: every screen rendered on an emulator at two phone sizes, PNGs committed to the `screens` branch

Locally: `gradle assembleRelease` (needs Android SDK 35, JDK 17).
