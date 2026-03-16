# 3D Projectile Motion Simulator

A single-file, browser-based 3D ballistic trajectory simulator with realistic physics, multiple projectile support, simulated sensors, and WebAssembly-based guidance code integration for testing airbrake control systems.

## Quick Start

Open `projectile_sim.html` in any modern browser. No build step or server required.

## Features

### Physics Engine
- **US Standard Atmosphere 1976** — 7-layer model computing temperature, pressure, air density, and speed of sound up to ~85 km altitude
- **Mach-dependent drag** — transonic drag rise (cubic to 2.5x at Mach 1.0) and supersonic 1/M² decline
- **Altitude-dependent gravity** — inverse-square falloff from Earth's surface
- **3D wind** — drag computed relative to air mass, not ground
- **RK4 numerical integration** — 4th-order Runge-Kutta with configurable timestep
- **Ground impact interpolation** — linear interpolation within the final timestep for precise impact location

### Multi-Projectile Support
- Run multiple projectiles simultaneously with independent parameters
- 8-color palette for visual distinction
- Tab bar for switching between projectiles
- Each projectile has its own launch parameters, drag stages, and sensors

### Visualization (Plotly.js)
- **3D Trajectory** — speed-colored path (single projectile) or solid colors (multiple), ground track shadow, launch/impact markers
- **Velocity** — Vx, Vy, Vz components vs time
- **Acceleration** — Ax, Ay, Az (finite-difference) vs time
- **Overview** — dual-axis: speed + total acceleration (left) and altitude (right) vs time
- **Orientation** — pitch, yaw, roll angles vs time
- **IMU** — body-frame specific force (forward, right, down) vs time
- **Sensors** — all simulated sensor readings vs time
- **Custom** — user-selectable fields from any category
- **Data Table** — raw numerical data with sampling for performance

### IMU / Body-Frame Acceleration
Simulates accelerometer readings (specific force) in the projectile's body frame:
- Forward (body X), Right (body Y), Down (body Z)
- Specific force = drag acceleration + gravity reaction
- Body frame rotation uses yaw → pitch → roll (Rz Ry Rx) convention

### Drag Stages
Dynamic drag coefficient and frontal area changes during flight:
- **Trigger types**: at apogee (+ optional delay), at a specific time, or at a descent altitude
- Multiple stages per projectile, processed in order
- Used to simulate parachute deployment, staging, etc.

### Simulated Sensors
Post-processed sensor readings with realistic noise models:

| Category | Types |
|---|---|
| Position | GPS X, GPS Y, GPS Altitude |
| Velocity | Speed, Velocity X/Y/Z |
| Orientation | Pitch, Yaw, Roll |
| Angular Rate (Gyro) | Pitch Rate, Yaw Rate, Roll Rate |
| Acceleration (World) | Accel X, Accel Y, Accel Z |
| Acceleration (Body/IMU) | IMU Forward, IMU Right, IMU Down |
| Atmosphere | Air Density, Mach Number, Dynamic Pressure, Temperature, Speed of Sound |
| Other | Barometric Altitude, Range |

Each sensor supports:
- **Noise**: none, Gaussian (white noise), or random walk (bias drift)
- **Noise σ**: configurable standard deviation
- **Bias**: constant offset
- **Update rate**: sample rate in Hz (0 = every timestep), with zero-order hold between samples

### CSV Export
Per-projectile data export with column category picker (time, position, velocity, orientation, atmosphere, forces, IMU, sensors, guidance).

### Flight Statistics
Max altitude, ground range, flight time, impact speed, max speed, max Mach, max dynamic pressure — displayed for the active projectile.

---

## Guidance Code Integration (WebAssembly)

The simulator supports loading C++ guidance code compiled to WebAssembly via Emscripten. This lets you write real, portable C++ (classes, stdlib, 1000+ lines) that also compiles for your actual flight computer. The guidance code runs each simulation timestep, reads sensor data and flight state, and commands airbrake positions (drag coefficient and frontal area).

### Overview

1. Write C++ guidance code implementing `guidance_init()` and `guidance_update()`
2. Compile with Emscripten to a single `.js` file (WASM embedded)
3. Upload the `.js` file to the simulator
4. Enable guidance and run the simulation

When guidance is active, it replaces drag stages — the guidance code has full control over Cd and frontal area.

### Step 1: Install Emscripten

```bash
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh
```

### Step 2: Create the API Files

**sim_api.h** — Include this in your guidance code:

```cpp
// sim_api.h — Include in your C++ guidance code
#pragma once
#include <cmath>
#ifdef __cplusplus
extern "C" {
#endif

// Shared buffer layout (float array indices):
//  [0]  time (s)          [1]  dt (s)
//  [2]  x (m)             [3]  y (m)             [4]  z (m)
//  [5]  vx (m/s)          [6]  vy (m/s)          [7]  vz (m/s)
//  [8]  speed (m/s)       [9]  altitude ASL (m)
//  [10] mach              [11] dynamic_pressure (Pa)
//  [12] air_density        [13] pitch (deg)
//  [14] yaw (deg)         [15] roll (deg)
//  [16] imu_forward        [17] imu_right          [18] imu_down
//  [19] current_cd        [20] current_area (m²)
//  [21] past_apogee (1/0) [22] target_altitude (m)
//  [23] num_sensors       [24..39] sensor values
// Output:
//  [40] new_cd (<=0 = no change)
//  [41] new_area (<=0 = no change)

extern float* sim_buf;

// Convenience accessors
inline float sim_time()       { return sim_buf[0]; }
inline float sim_dt()         { return sim_buf[1]; }
inline float sim_x()          { return sim_buf[2]; }
inline float sim_y()          { return sim_buf[3]; }
inline float sim_z()          { return sim_buf[4]; }
inline float sim_vx()         { return sim_buf[5]; }
inline float sim_vy()         { return sim_buf[6]; }
inline float sim_vz()         { return sim_buf[7]; }
inline float sim_speed()      { return sim_buf[8]; }
inline float sim_altitude()   { return sim_buf[9]; }
inline float sim_mach()       { return sim_buf[10]; }
inline float sim_q()          { return sim_buf[11]; }
inline float sim_rho()        { return sim_buf[12]; }
inline float sim_pitch()      { return sim_buf[13]; }
inline float sim_yaw()        { return sim_buf[14]; }
inline float sim_roll()       { return sim_buf[15]; }
inline float sim_imu_fwd()    { return sim_buf[16]; }
inline float sim_imu_right()  { return sim_buf[17]; }
inline float sim_imu_down()   { return sim_buf[18]; }
inline float sim_cd()         { return sim_buf[19]; }
inline float sim_area()       { return sim_buf[20]; }
inline bool  sim_past_apogee(){ return sim_buf[21] > 0.5f; }
inline float sim_target_alt() { return sim_buf[22]; }
inline int   sim_num_sensors(){ return (int)sim_buf[23]; }
inline float sim_sensor(int i){ return sim_buf[24+i]; }
inline void sim_set_cd(float v)  { sim_buf[40] = v; }
inline void sim_set_area(float v){ sim_buf[41] = v; }

void guidance_init(void);
void guidance_update(void);
void sim_set_buffer(float* buf);

#ifdef __cplusplus
}
#endif
```

**sim_api.cpp** — Link this with your guidance code:

```cpp
#include "sim_api.h"
float* sim_buf = nullptr;
extern "C" void sim_set_buffer(float* buf) {
    sim_buf = buf;
}
```

### Step 3: Write Your Guidance Code

Create `guidance.cpp` implementing the two required functions:

```cpp
#include "sim_api.h"

// Your state variables, classes, filters, etc.
static float max_altitude_seen = 0.0f;

extern "C" {

void guidance_init() {
    // Called once at the start of each simulation run.
    // Reset your internal state here.
    max_altitude_seen = 0.0f;
}

void guidance_update() {
    // Called every timestep during the simulation.
    // Read state via sim_* accessors, write outputs via sim_set_cd/sim_set_area.

    float alt = sim_altitude();
    float target = sim_target_alt();
    float speed = sim_speed();

    if (alt > max_altitude_seen) {
        max_altitude_seen = alt;
    }

    // Example: simple proportional airbrake control after apogee
    if (sim_past_apogee()) {
        // Airbrakes closed during descent
        sim_set_cd(0.3f);
        sim_set_area(0.01f);
    } else {
        // Predict apogee and deploy airbrakes if overshooting
        float error = target - alt;
        if (error < 0) {
            // Over target — full airbrakes
            sim_set_cd(1.2f);
            sim_set_area(0.05f);
        } else if (error < 500.0f && speed > 50.0f) {
            // Approaching target — partial airbrakes
            float brake_fraction = 1.0f - (error / 500.0f);
            sim_set_cd(0.3f + brake_fraction * 0.9f);
            sim_set_area(0.01f + brake_fraction * 0.04f);
        }
        // else: no change (return cd/area <= 0 or don't call sim_set)
    }

    // Access sensor readings (indices match the sensor map shown in the UI)
    // int n = sim_num_sensors();
    // float baro_alt = sim_sensor(0);  // if sensor 0 is barometric altitude
    // float imu_fwd  = sim_sensor(1);  // if sensor 1 is IMU forward
}

} // extern "C"
```

You can use any C++ features — classes, templates, the standard library, Kalman filters, etc. The only requirement is that `guidance_init()` and `guidance_update()` are `extern "C"`.

### Step 4: Compile with Emscripten

```bash
emcc guidance.cpp sim_api.cpp -o guidance.js -O2 \
  -sEXPORTED_FUNCTIONS='["_guidance_init","_guidance_update","_sim_set_buffer","_malloc","_free"]' \
  -sEXPORTED_RUNTIME_METHODS='["ccall","cwrap","HEAPF32"]' \
  -sMODULARIZE=1 -sEXPORT_NAME='createGuidanceModule' \
  -sSINGLE_FILE=1 -sALLOW_MEMORY_GROWTH=1
```

This produces a single `guidance.js` file with the WASM binary embedded as base64.

**Flag explanation:**
| Flag | Purpose |
|---|---|
| `-O2` | Optimization level |
| `-sEXPORTED_FUNCTIONS` | Makes `guidance_init`, `guidance_update`, `sim_set_buffer`, `malloc`, `free` callable from JS |
| `-sEXPORTED_RUNTIME_METHODS` | Exposes `ccall`, `cwrap`, `HEAPF32` for the JS bridge |
| `-sMODULARIZE=1` | Wraps output as a module factory function |
| `-sEXPORT_NAME='createGuidanceModule'` | Names the factory function |
| `-sSINGLE_FILE=1` | Embeds WASM as base64 in the `.js` file (one file to upload) |
| `-sALLOW_MEMORY_GROWTH=1` | Allows dynamic memory allocation to grow |

### Step 5: Upload and Run

1. Open the simulator and go to the **Setup** tab
2. Scroll to the **Guidance Program** section
3. Click **Upload** and select your compiled `guidance.js`
4. Check **Enable Guidance** (auto-enabled on successful load)
5. Set the **Target Altitude** (default: 3048 m / 10,000 ft)
6. Configure sensors if your guidance code reads them (the sensor index mapping is shown in the API Reference section)
7. Click **Run**

### How It Works at Runtime

Each simulation timestep:

1. The RK4 integrator advances the physics state
2. Apogee detection runs (tracks if altitude has started descending)
3. The simulator writes current state to a shared `float[48]` buffer in WASM memory
4. `guidance_update()` is called — your C++ code reads state, computes, and writes new Cd/area
5. The simulator reads back the Cd and area values
6. If `new_cd > 0`, the drag coefficient updates; if `new_area > 0`, the frontal area updates
7. The data point is recorded with `guidance_cd` and `guidance_area` fields

`guidance_init()` is called once at the start of each simulation run, before the integration loop begins.

### Sensor Access

If you configure sensors in the simulator (e.g., a barometric altimeter, IMU channels), the guidance code receives their readings each timestep via `sim_sensor(index)`. The sensor index mapping is displayed in the Guidance Program UI section. Sensors include realistic noise, bias, and update rate effects — your guidance code sees the same imperfect data your real flight computer would.

Up to 16 sensor values can be passed per timestep. Use `sim_num_sensors()` to check how many are available.

### Viewing Guidance Output

After running with guidance enabled:
- The **Custom** view tab has **Guidance Cd** and **Guidance Area** checkboxes to plot how your controller commanded the airbrakes over time
- The **Data Table** view shows `guidance_cd` and `guidance_area` columns
- **CSV export** includes a Guidance category in the column picker

### Notes on Guidance

- When guidance is enabled, drag stages are skipped (guidance has full control)
- Guidance applies to all projectiles when enabled
- Returning `cd <= 0` or `area <= 0` from `guidance_update()` means "no change" — the previous values persist
- The shared buffer uses a flat float array (not structs) to avoid alignment issues across compilers
- Your C++ code can maintain arbitrary internal state between timesteps (static variables, class instances, etc.)

---

## UI Layout

### Setup Tab
Full-width scrollable panel with multi-column grid containing:
- **Projectile selector** — tab bar with color-coded projectile tabs
- **Launch Parameters** — velocity, elevation, azimuth, roll, launch altitude, ground altitude
- **Projectile Properties** — mass, frontal area, drag coefficient
- **Wind** — X, Y, Z components
- **Simulation Settings** — timestep, max time
- **Drag Stages** — dynamic list of Cd/area change triggers
- **Guidance Program** — WASM upload, enable toggle, target altitude, API reference
- **Sensors** — dynamic list of simulated sensor configurations

All sections are collapsible.

### Results Tab
Two-column layout: visualization area (left) + flight statistics panel (right).

The toolbar provides view sub-tabs: 3D, Velocity, Acceleration, Overview, Orientation, IMU, Sensors, Custom, Data Table. A CSV download dropdown allows per-projectile export with column selection.

---

## Default Parameters

| Parameter | Default | Unit |
|---|---|---|
| Velocity | 800 | m/s |
| Elevation Angle | 45 | deg |
| Azimuth Angle | 0 | deg |
| Roll | 0 | deg |
| Launch Altitude | 0 | m ASL |
| Ground Altitude | 0 | m ASL |
| Mass | 10 | kg |
| Frontal Area | 0.01 | m² |
| Drag Coefficient | 0.30 | — |
| Wind (X, Y, Z) | 0, 0, 0 | m/s |
| Time Step | 0.01 | s |
| Max Simulation Time | 600 | s |
| Target Altitude (Guidance) | 3048 | m |

## Dependencies

- [Plotly.js 2.35.2](https://plotly.com/javascript/) (loaded from CDN — requires internet on first load)
- [Emscripten](https://emscripten.org/) (only needed to compile guidance code, not to run the simulator)

## Browser Support

Works in any modern browser with JavaScript and WebGL support (Chrome, Firefox, Safari, Edge).
