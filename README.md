# fpvq3

FPV quadcopter flight for Quake III Arena. It's a game-code mod for the stock ioquake3 engine: you fly a 5" quad around any Q3 map, Liftoff style, and you can shoot while you fly. Fly acro with an RC transmitter, or use the self-levelling modes with a gamepad or keyboard and mouse. Bots fly drones too.

## Requirements

- An ioquake3 engine binary. `fly.sh` looks for `/opt/quake3/ioquake3` (and `ioq3ded` for a dedicated server); set `IOQ3=` / `IOQ3DED=` to use others.
- Quake III Arena game data (`baseq3/pak0.pk3` and friends). `fly.sh` looks in `~/Games/quake3`; set `Q3BASE=` to use another folder.
- CMake, a C compiler and `zip`, to build the game modules.
- Something to fly with: an RC transmitter in USB joystick mode, a game controller (PlayStation, Xbox, ...), or keyboard and mouse.

## Build and fly

```bash
./install.sh
./fly.sh
```

`install.sh` builds the game, cgame and ui modules and installs them into `run/fpvq3`, which `fly.sh` uses as the home folder:

- `fpvq3.pk3` holds the QVMs, configs and shader. It's what a server hands to players who join without the mod.
- the native `.so` modules sit next to it. Local games use them.

`fly.sh` takes an optional map name (default `q3dm17`) and passes any further arguments to the engine. Pick your controls with `CONTROLS=` and add bots with `BOTS=`:

```bash
./fly.sh q3dm6 +set cg_fov 120
CONTROLS=gamepad BOTS=3 ./fly.sh q3dm6
```

## Controls

There are three control setups in `fpvq3/controls/`. `fly.sh` runs `fpv.cfg` (shared binds and view settings) and then the one you pick:

| `CONTROLS=` | Flight mode | Layout |
|---|---|---|
| `radio` (default) | acro | RC transmitter, mode 2; mouse 1 or a radio switch fires |
| `gamepad` | easy | left stick moves, right stick looks and aims, R2 fires, R1 / L1 (or Cross / Circle) climb and sink, Triangle / Square change weapon, Share resets |
| `keyboard` | easy | WASD moves, mouse aims, space / C climb and sink, mouse 1 fires, mouse 2 zooms, wheel changes weapon |

Common to all of them: `R` resets (`dr_reset`), `M` cycles the flight mode, `F5` toggles the chase cam, `TAB` shows scores, `1`-`9` pick weapons.

### Flight modes

`dr_mode` picks the mode. It's saved, it's per player, and `M` cycles it in flight:

- **0, acro (rate):** the sticks set rotation rates and the drone holds whatever attitude you leave it in, like a real quad in acro mode. You aim with the drone itself: shots go where the camera looks, so raise the camera uptilt (`dr_camangle`) to shoot while flying fast.
- **1, level (angle):** roll and pitch set a tilt angle, up to 55°. Let go of the sticks and the drone levels itself. Throttle and yaw work as in acro.
- **2, easy:** it flies like a Quake player who can fly. The movement keys or left stick set a speed (up to 14 m/s) along where you look, climb and sink set a vertical speed, and letting go holds position and height. The mouse or right stick aims, and the drone tilts and turns under you to follow. Bots always fly this mode.

All three run the same physics, so a pilot in easy mode is no faster or slower than an acro pilot. Easy mode just does the stick work for you.

### Radio setup

SDL recognises a Jumper T-Pro (OpenTX) as a gamepad, so `fly.sh` passes an `SDL_GAMECONTROLLERCONFIG` mapping for mode 2:

- the left stick carries throttle (CH3) and yaw (CH4)
- the right stick carries pitch (CH2) and roll (CH1)

`controls/radio.cfg` then binds those sticks to the flight controls. Watch the two stick boxes on the OSD. If an axis is reversed, swap the two commands bound to that stick in `radio.cfg`.

For another radio, or to fire and reset from switches on the radio, run `tools/calibrate.py`. It walks you through the four stick moves, then the fire and reset switches (both optional), and prints the mapping part of an `SDL_GAMECONTROLLERCONFIG` line. Set your own mapping when you launch, with your radio's SDL GUID (`sdl2-jstest --list` shows it):

```bash
SDL_GAMECONTROLLERCONFIG="<guid>,My Radio,leftx:a3,lefty:a2,rightx:a0,righty:a1,a:b0,b:b1,platform:Linux," ./fly.sh
```

`radio.cfg` binds `a` to fire and `b` to reset.

### Arming

After a spawn the drone sits with its motors off until it sees input ("MOVE THE STICKS TO START"). In acro and level mode the first stick movement arms it, so **keep the throttle low when you spawn or reset**. The hold is there because a gamepad reports nothing until a stick moves, and an all-zero command reads as half throttle. Easy mode has no throttle, so any movement or look arms it.

## Weapons

With `dr_weapons 1` (the default) drones pick up and fire the normal Quake weapons. The gun hangs under the frame and fires along the aim: the camera direction in acro and level mode, the view in easy mode. Damage, armour, frags and respawns work as in normal deathmatch.

- Shots hit a box `dr_hitbox` units across each way from the centre (default 12, at most 32), bigger than the drone's collision box. A 5" quad would otherwise be very hard to hit.
- Resetting with weapons on self-destructs the drone (it counts as a suicide) and respawns it after the usual delay, so the reset switch is no free escape or heal. With `dr_weapons 0` it puts you straight back on a spawn point.
- `dr_weapons 0` makes it a pure flying game: drones can't fire, and the HUD shows only the OSD.

## Multiplayer

Host a game from your own machine, or run a dedicated server:

```bash
./fly.sh --host q3dm6
./fly.sh --server q3dm6
```

Both run a pure server on the QVMs from `fpvq3.pk3` with downloads on. Friends who have ioquake3 and the Quake III data but not the mod download it when they join. They need to start the engine in the mod's folder with downloads allowed:

```bash
ioquake3 +set fs_game fpvq3 +set cl_allowDownload 1 +connect <address>
```

Without `fs_game fpvq3`, ioquake3 switches folders when it connects. That switch resets `cl_allowDownload` back to 0 unless the player has turned on automatic downloads in the menu, and then the server drops them as an unpure client. With the mod already installed, `./fly.sh --join <address>` does this for you. The server's port is 27960 unless you add `+set net_port <n>`.

Each player picks their own flight mode, rates and camera angle. The server sends its physics cvars to every client, so prediction matches.

## Bots

```bash
BOTS=4 BOT_SKILL=3 ./fly.sh q3dm6
```

`BOTS` sets `bot_minplayers` (so bots leave as people join), and you can still add bots from the menu or with `addbot`. Bots keep the stock Quake brain for goals, routes, aim and weapon choice. An autopilot in `ai_drone.c` turns that into flight: it follows their walking route at standing height, climbs toward ledges and to an enemy's height, and goes over or under walls in the way. They fly easy mode. `dr_botdebug 1` prints what each bot's autopilot is doing.

## Tuning

Client (per pilot):

| Cvar | Default | Meaning |
|---|---|---|
| `dr_mode` | 2 | 0 acro, 1 level, 2 easy (the control configs set it) |
| `dr_rates` | `1.0 0.7 0` ×3 | Betaflight classic rates as `rc super expo` for roll, pitch and yaw |
| `dr_camangle` | 30 | camera uptilt in degrees (0-80), saved between sessions |
| `dr_camstep` | 5 | degrees per `dr_camup` / `dr_camdown` step |
| `cg_fov` | 115 | field of view (set in `fpv.cfg`) |
| `dr_osd`, `dr_sticks` | 1, 1 | show the OSD and the stick overlay |
| `dr_debug` | 0 | 1 prints the drone state to the console twice a second |

Server:

| Cvar | Default | Meaning |
|---|---|---|
| `g_drone` | 1 | 0 = normal Quake movement |
| `dr_weapons` | 1 | drones carry and fire weapons |
| `dr_hitbox` | 12 | half-size of the box shots hit (units, up to 32) |
| `dr_botdebug` | 0 | print the bot autopilot state |

Server physics (also sent to clients for prediction):

| Cvar | Default | Meaning |
|---|---|---|
| `dr_twr` | 8 | thrust-to-weight |
| `dr_scale` | 32 | game units per metre |
| `dr_gravity` | 9.81 | m/s² |
| `dr_dragside` / `dr_dragtop` | 0.022 / 0.05 | quadratic drag (1/m) |
| `dr_propspeed` | 70 | inflow (m/s) where full-rpm thrust reaches zero |
| `dr_motortau` / `dr_ratetau` | 0.03 / 0.02 | motor spool and rate response (s) |
| `dr_idle` | 0.05 | armed idle |
| `dr_crash` | 0 | impact speed (m/s) that disarms until reset (0 = off) |
| `dr_bounce`, `dr_friction` | 0.3, 0.6 | contact |

`fpv.cfg` also sets `r_znear 1` so the camera doesn't see through walls it's pressed against. That's a cheat cvar, so it only applies on servers started with `devmap`. `fly.sh` and `fly.sh --host` use `devmap`, which also means anyone in a hosted game can use cheat commands. `--server` uses `map`.

## How it works

The physics runs inside `Pmove` as a new `PM_DRONE` movement type, so client prediction and multiplayer work as they do for normal movement. The engine is unmodified, so input and state ride in fields it already networks:

- **Input:**
  - throttle (or climb rate) in `upmove`
  - roll in `rightmove`
  - pitch in `forwardmove`
  - yaw as the change in view yaw: the client turns at 1000°/s at full stick, and `Pmove` divides the change back out. In easy mode the view angles are the aim instead.
- **State:**
  - attitude in `viewangles`
  - body rates in `grapplePoint`
  - motor, flags, flight mode and aim in spare `stats[]`
  - drone flag and motor level in `generic1`, and the aim in `angles2`, which reach other clients through the entity state

The flight model is in `bg_drone.c`:

- rates map to target body rates
- level mode turns stick deflection into a target tilt and steers toward it
- easy mode turns the wanted velocity into a tilt and a thrust, with altitude hold
- a first-order rate controller
- spooled motors with thrust that falls off with inflow speed
- anisotropic quadratic drag
- corner-impulse collisions, so hits make the drone tumble
- a settle torque so it comes to rest on a face

Weapons use the stock `PM_Weapon` code, run from the drone branch of `Pmove`, and fire along the aim stored in the player state.

`tests/run.sh` compiles the flight model on its own and runs headless checks for each mode: drops, hover throttle, rates, top speed, flips, crashes, self-levelling, and easy mode's speed, height hold and heading.

## Layout

| Path | What |
|---|---|
| `ioq3/` | ioquake3 source with the mod's game-code changes; the engine code is untouched |
| `ioq3/code/game/bg_drone.c` | flight model and flight modes, shared by server and client prediction |
| `ioq3/code/game/ai_drone.c` | bot autopilot |
| `ioq3/code/cgame/cg_drone.c` | FPV camera, OSD, procedural drone model and gun |
| `fpvq3/` | mod assets: `fpv.cfg` (shared settings), `controls/` (radio, gamepad, keyboard), `scripts/fpvq3.shader` |
| `install.sh` | builds the modules and installs them into `run/fpvq3` |
| `fly.sh` | launches the engine with the mod: solo, host, join or dedicated server |
| `tests/` | headless flight-model checks (`tests/run.sh`) |
| `tools/` | `calibrate.py` (radio axis and switch mapping), `jsread.py` (raw joystick values) |

`ioq3/` is ioquake3 at upstream commit [`52088b9`](https://github.com/ioquake/ioq3/commit/52088b9eae247d8e321c8dc1fb49a879652854f9). Apart from the three new files above (and `ai_drone.h`), the mod changes these files:

- `cmake/basegame.cmake`
- `cmake/utils/qvm_tools.cmake`: QVM objects now rebuild when a header changes. Upstream only tracks the `.c` file, so a changed struct left stale objects that disagreed about its layout, and the QVM server crashed.
- in `code/game/`: `ai_dmnet.c`, `ai_dmq3.c`, `ai_main.c`, `ai_main.h`, `bg_misc.c`, `bg_pmove.c`, `bg_public.h`, `g_active.c`, `g_client.c`, `g_cmds.c`, `g_combat.c`, `g_local.h`, `g_main.c`, `g_weapon.c`
- in `code/cgame/`: `cg_consolecmds.c`, `cg_draw.c`, `cg_local.h`, `cg_main.c`, `cg_players.c`, `cg_predict.c`, `cg_servercmds.c`, `cg_view.c`, `cg_weapons.c`

## Known limitations

- Roll, pitch and throttle have 8-bit resolution; yaw is about 16-bit.
- Yaw can stutter slightly when client and server timing differ.
- No motor sound yet. The mod API has no pitch control, so it needs a few RPM-band loops to crossfade.
- No prop wash or turbulence model.
- Bots fly well enough to fight, but they score a third to a half as many kills as in normal Quake. They steer along their walking routes, so they don't use the third dimension much, and they don't take shortcuts through the air.
- The gun model is the stock weapon model scaled down, so it doesn't look built for a drone.

## License

GPL v2 or later, like the ioquake3 source it builds on (see `ioq3/COPYING.txt`).
