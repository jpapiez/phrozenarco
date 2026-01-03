# Phrozen Arco Bed Layout Reference

## Overview Diagram

```
Y (mm)
  ▲
   │░░░░░░░░░░░░░░░░│                                          │░░░░░░░░│
   │░░░░░░░░░░░░░░░░│     S E R V I C E   A R E A              │░░░░░░░░│
   │░░░░KEEPOUT░░░░░│                                          │KEEPOUT░│
322│░░░░(X<125)░░░░░│ CHUTE   ════WIPER════                    │(X>250)░│
   │░░░░░░░░░░░░░░░░│ X=123  X=140────────200                  │░░░░░░░░│
310│░░░░░░░░░░░░░░░░│─ ─ ─ ─ transit level ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ │░░░░░░░░│
   │░░░░░░░░░░░░░░░░│                                          │░░░░░░░░│
302│░░░░░░░░░░░░░░░░├──────────────────────────────────────────┤░░░░░░░░│
   │                │                                          │        │
300╞════════════════╧══════════════════════════════════════════╧════════╡────┐
   │                                                                    │    │
   │                                                                    │CUT │
260│                                                                    │TER │
   │                         P R I N T   B E D                          │313-│
   │                                                                    │320 │
   │                            300 x 300                               │    │
   │                                                                    │    │
  0└────────────────────────────────────────────────────────────────────┴────┘──▶ X
   0              125                                          250     300  330
                   │◄────────── SAFE  CORRIDOR ───────────────►│
                              (required when Y > 302)
```

## Service Area Detail (Y > 302)

```
   0            123 125 138 140              200             250      300
   │              │   │   │ │                 │               │        │
   │░░░░░░░░░░░░░░│   │   │ │                 │               │░░░░░░░░│
   │░░░░░░░░░░░░░░│   │   │ │                 │               │░░░░░░░░│
   │░░░░░░░░░░░░░░│CHUTE  │ ├─────────────────┤               │░░░░░░░░│ Y=322
   │░░░░░░░░░░░░░░│(purge)│ │  WIPER BRUSH    │               │░░░░░░░░│
   │░░░░KEEPOUT░░░│       │ │                 │               │░░░░░░░░│ Y=310
   │░░░░(X<125)░░░│     entry                 │               │░░░░░░░░│ (transit)
   │░░░░░░░░░░░░░░│                           │               │░░░░░░░░│
───┴░░░░░░░░░░░░░░┴───────────────────────────┴───────────────┴░░░░░░░░┴── Y=302
   │              │                                           │        │
   │   (safe)     │◄─────────  SAFE CORRIDOR  ───────────────►│  KEEP  │
   │              │         (must be here before Y > 302)     │  OUT   │
───┴──────────────┴───────────────────────────────────────────┴────────┴── Y=300
```

## Key Coordinates

**Print Bed**: X = 0-300, Y = 0-300, Z = 0-303

**Service Area** (Y > 300):
- Chute (purge):  X = 123
- Chute entry:    X = 138 (safe approach, left of wiper)
- Wiper brush:    X = 140 to 200
- Service level:  Y = 322

**Cutter** (right of bed at Y = 260):
- Approach: X = 300
- Pre-cut:  X = 313
- Cut:      X = 320

**Keepout Zones** (collision hazard when Y > 302):
- Left rear:  X < 125 (chute housing blocks access)
- Right rear: X > 250

**Safe Corridor**: X = 125-250 (required when Y > 302)

**Waiting Area Y**: Y = 302 (our override; stock = 280)

## GLOBAL_PARAM Variables

| Variable | Value | Description |
|----------|-------|-------------|
| g_bottom_y | 322 | Service level Y |
| g_bottom_print_y | 280 | Waiting area Y (we override to 302) |
| g_spitting_start_x | 123 | Chute/purge X |
| g_wipemouth_start_x | 140 | Wiper left edge |
| g_waittingarea_x | 149 | Waiting area X |
| g_wait_pause_x | 169 | Pause position X |

## Waiting Area Macros

**Waiting Area**: X=138, Y=302 - The safe "home base" for all service operations.
- X=138 is in safe corridor (125-250)
- Y=302 is at threshold (safe to return to print from here)

| Macro | Ends At | Use For |
|-------|---------|---------|
| PRZ_WAITINGAREA | X=138, Y=302 | Safe transit to waiting area |
| PRZ_CUT_WAITINGAREA | X=123, Y=322 | Purging, pre-cut heating (via waiting area) |
| PRZ_PAUSE_WAITINGAREA | X=138, Y=302 | Pause during filament change |
| PRZ_WIPEMOUTH | X=138, Y=302 | Wipe nozzle, returns to waiting area |

**_SAFE_SERVICE_TRANSIT path** (to waiting area X=138, Y=302):
1. If at Y > 302:
   - If X < 125: slide right to X=125 first (exit left keepout)
   - If X > 250: slide left to X=250 first (exit right keepout)
2. Move to X=138, Y=302 (waiting area)

**PRZ_CUT_WAITINGAREA path** (to chute X=123, Y=322):
1. If already at Y~322 (service level): slide direct to X=123
2. Otherwise: go to waiting area → rise to Y=322 → slide to X=123

**PRZ_WIPEMOUTH path**:
1. If already at Y~322 (service level): go direct to wiper
2. Otherwise: go to waiting area → rise to Y=322 → wipe → return to waiting area
