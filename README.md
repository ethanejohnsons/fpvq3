# fpvq3

FPV quadcopter flight for Quake III Arena. It's a game-code mod for the stock ioquake3 engine: you fly an acro-mode 5" quad around any Q3 map with an RC transmitter, Liftoff style.

## Requirements

- An ioquake3 engine binary. `fly.sh` looks for `/opt/quake3/ioquake3`; set `IOQ3=` to use another.
- Quake III Arena game data (`baseq3/pak0.pk3` and friends). `fly.sh` looks in `~/Games/quake3`; set `Q3BASE=` to use another folder.
- CMake and a C compiler, to build the game modules.
- An RC transmitter in USB joystick mode. A keyboard works for testing but can't really fly.

## Build and fly

```bash
./install.sh
./fly.sh
```

`install.sh` builds the game, cgame and ui modules (native `.so` and QVM) and installs them with the mod assets into `run/fpvq3`, which `fly.sh` uses as the home folder. `fly.sh` takes an optional map name (default `q3dm17`) and passes any further arguments to the engine:

```bash
./fly.sh q3dm6 +set cg_fov 120
```

## Radio setup

SDL recognises a Jumper T-Pro (OpenTX) as a gamepad, so `fly.sh` passes an `SDL_GAMECONTROLLERCONFIG` mapping for mode 2:

- the left stick carries throttle (CH3) and yaw (CH4)
- the right stick carries pitch (CH2) and roll (CH1)

`fpvq3/fpv.cfg` then binds those sticks to the flight controls. Watch the two stick boxes on the OSD. If an axis is reversed, swap the two commands bound to that stick in `fpv.cfg`.

For another radio, run `tools/calibrate.py`. It walks you through the four stick moves and prints the axis part of an `SDL_GAMECONTROLLERCONFIG` line. Then set your own mapping when you launch, with your radio's SDL GUID (`sdl2-jstest --list` shows it):

```bash
SDL_GAMECONTROLLERCONFIG="<guid>,My Radio,leftx:a3,lefty:a2,rightx:a0,righty:a1,platform:Linux," ./fly.sh
```

## Flying

After a spawn the drone sits with its motors off until it sees stick input ("MOVE THE STICKS TO START"). The first stick movement arms it, so **keep the throttle low when you spawn or reset**. The hold is there because a gamepad reports nothing until a stick moves, and an all-zero command reads as half throttle.

| Control | Binding |
|---|---|
| Reset to spawn | `R` (`dr_reset`), or bind `+button7` to a radio switch |
| Camera uptilt up / down | mouse wheel, or `]` / `[` (`dr_camup` / `dr_camdown`) |
| Chase cam | `F5` |

`fpv.cfg` takes the mouse out of the flight controls. Stock Quake binds the right button to `+strafe`, which would turn mouse movement into roll and pitch.

## Tuning

Client (per pilot):

| Cvar | Default | Meaning |
|---|---|---|
| `dr_rates` | `1.0 0.7 0` ×3 | Betaflight classic rates as `rc super expo` for roll, pitch and yaw |
| `dr_camangle` | 30 | camera uptilt in degrees (0-80), saved between sessions |
| `dr_camstep` | 5 | degrees per `dr_camup` / `dr_camdown` step |
| `cg_fov` | 115 | field of view (set in `fpv.cfg`) |
| `dr_osd`, `dr_sticks` | 1, 1 | show the OSD and the stick overlay |
| `dr_debug` | 0 | 1 prints the drone state to the console twice a second |

Server physics (also sent to clients for prediction):

| Cvar | Default | Meaning |
|---|---|---|
| `dr_twr` | 8 | thrust-to-weight |
| `dr_scale` | 32 | game units per metre |
| `dr_dragside` / `dr_dragtop` | 0.022 / 0.05 | quadratic drag (1/m) |
| `dr_propspeed` | 70 | inflow (m/s) where full-rpm thrust reaches zero |
| `dr_motortau` / `dr_ratetau` | 0.03 / 0.02 | motor spool and rate response (s) |
| `dr_idle` | 0.05 | armed idle |
| `dr_crash` | 0 | impact speed (m/s) that disarms until reset (0 = off) |
| `dr_bounce`, `dr_friction` | 0.3, 0.6 | contact |
| `g_drone` | 1 | 0 = normal Quake movement |

`fpv.cfg` also sets `r_znear 1` so the camera doesn't see through walls it's pressed against. That's a cheat cvar, so it only applies in local `devmap` games, which is what `fly.sh` starts.

## How it works

The physics runs inside `Pmove` as a new `PM_DRONE` movement type, so client prediction and multiplayer work as they do for normal movement. The engine is unmodified, so input and state ride in fields it already networks:

- **Input:**
  - throttle in `upmove`
  - roll in `rightmove`
  - pitch in `forwardmove`
  - yaw as the change in view yaw: the client turns at 1000°/s at full stick, and `Pmove` divides the change back out
- **State:**
  - attitude in `viewangles`
  - body rates in `grapplePoint`
  - motor and flags in spare `stats[]`
  - drone flag and motor level in `generic1`, which reaches other clients through the entity state

The flight model is in `bg_drone.c`:

- rates map to target body rates
- a first-order rate controller
- spooled motors with thrust that falls off with inflow speed
- anisotropic quadratic drag
- corner-impulse collisions, so hits make the drone tumble
- a settle torque so it comes to rest on a face

`tests/run.sh` compiles it on its own and runs headless flight checks: drops, hover throttle, rates, top speed, flips and crashes.

## Layout

| Path | What |
|---|---|
| `ioq3/` | ioquake3 source with the mod's game-code changes; the engine code is untouched |
| `ioq3/code/game/bg_drone.c` | flight model, shared by server and client prediction |
| `ioq3/code/cgame/cg_drone.c` | FPV camera, OSD, procedural drone model |
| `fpvq3/` | mod assets: `fpv.cfg` (controls), `scripts/fpvq3.shader` |
| `install.sh` | builds the modules and installs them into `run/fpvq3` |
| `fly.sh` | launches the engine with the mod |
| `tests/` | headless flight-model checks (`tests/run.sh`) |
| `tools/` | `calibrate.py` (radio axis mapping), `jsread.py` (raw joystick values) |

`ioq3/` is ioquake3 at upstream commit [`52088b9`](https://github.com/ioquake/ioq3/commit/52088b9eae247d8e321c8dc1fb49a879652854f9). Apart from the two new files above, the mod changes these files:

- `cmake/basegame.cmake`
- in `code/game/`: `bg_misc.c`, `bg_pmove.c`, `bg_public.h`, `g_active.c`, `g_client.c`, `g_cmds.c`, `g_combat.c`, `g_local.h`, `g_main.c`
- in `code/cgame/`: `cg_consolecmds.c`, `cg_draw.c`, `cg_local.h`, `cg_main.c`, `cg_players.c`, `cg_predict.c`, `cg_servercmds.c`, `cg_view.c`, `cg_weapons.c`

## Known limitations

- Roll, pitch and throttle have 8-bit resolution; yaw is about 16-bit.
- Yaw can stutter slightly when client and server timing differ.
- No motor sound yet. The mod API has no pitch control, so it needs a few RPM-band loops to crossfade.
- No prop wash or turbulence model.
- Bots still play normal Quake.

## License

GPL v2 or later, like the ioquake3 source it builds on (see `ioq3/COPYING.txt`).
