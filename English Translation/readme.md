# Phrozen Arco – English Translation & Logging Patch

Clean English translation and improved logging for the Phrozen Arco AMS (multi-material) Python layer.

---

## Quick Start

### 1) Install (USB Update)

1. Copy the release `.zip` to a USB drive
2. Insert USB into the printer
3. On the Arco screen:

   * Tap the **Info (ⓘ) icon**
   * Go to **Firmware Update**
   * Select **USB Update**
4. Let the update complete

The installer will:

* back up existing files (`.bak`, then timestamped on repeats)
* install `dev.py`, `cmds.py`
* optionally install `custom_prz_logging.cfg`

---

### 2) Enable Logging Config (optional but recommended)

Add this line to your `printer.cfg`:

```ini
[include custom_prz_logging.cfg]
```

Restart Klipper after saving.

---

### 3) Use It

Default startup is **quiet mode (WARN)**.

Toggle verbosity anytime:

```gcode
PRZ_TOGGLE_LOG_LEVEL
```

Or set manually:

```gcode
PRZ_LOG_LEVEL LEVEL=1
PRZ_LOG_FLAGS CATEGORIES=1 TIMESTAMP=0
```

---

## Logging Levels

| Level | Output                     |
| ----- | -------------------------- |
| 0     | ERROR only                 |
| 1     | WARN + ERROR (recommended) |
| 2     | INFO + WARN + ERROR        |
| 3     | DEBUG (full output)        |

---

## What’s Included

* Full English translation of console messages
* Cleaned/consistent logging output
* Debug noise reduction (serial spam, etc.)
* AMS2 (tty2) missing-port messages downgraded to DEBUG
* Optional config + toggle macro

---

## Notes

* `$ P114`, `$ M84`, etc. are **command echoes** from Klipper UI — not affected by logging level
* No changes to motion, printing, or hardware behavior
* Safe to reinstall multiple times (copy-based backups)

---

## Disclaimer

Tested on real hardware, but use at your own risk.
Keep backups of your working config.

---

## Credits

Phrozen firmware base
Translation & cleanup by community
