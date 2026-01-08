Machine Start G-Code:

; ====================================
; machine start g-code with adaptive bed mesh (explicit profile management)
M107 ; turn off part cooling fan
G90 ; absolute positioning
M140 S[first_layer_bed_temperature] ; set bed to final temperature
M104 S140 ; set hotend to probing temperature
M190 S[first_layer_bed_temperature] ; wait for bed to reach final temperature
M109 S140 ; wait for hotend to reach probing temperature
PG28 ; home all axes
PRZ_WIPEMOUTH ; wipe nozzle
M106 S255 ; turn on part cooling fan for probing
BED_MESH_CLEAR ; clear any active mesh profile first
BED_MESH_CALIBRATE mesh_min={adaptive_bed_mesh_min[0]},{adaptive_bed_mesh_min[1]} mesh_max={adaptive_bed_mesh_max[0]},{adaptive_bed_mesh_max[1]} ALGORITHM=[bed_mesh_algo] PROBE_COUNT={bed_mesh_probe_count[0]},{bed_mesh_probe_count[1]} ADAPTIVE=0 ADAPTIVE_MARGIN=0
BED_MESH_PROFILE LOAD=default ; explicitly load the newly created default profile
TP_OUT ; disable PG28/G30/G31 to prevent mid-print homing
PRZ_WAITINGAREA ; move to waiting area
M106 S0 ; turn off part cooling fan
G21 ; set units to millimeters
M83 ; extruder relative mode
M109 S{first_layer_temperature[0]} ; heat to final printing temperature
P0 M1
P28
P2 A1
; ====================================
SET_PRINT_STATS_INFO TOTAL_LAYER=[total_layer_count]


Before Layer Change G-Code:

; BEFORE_LAYER_CHANGE [layer_num] @ [layer_z]mm
;BEFORE_LAYER_CHANGE
;[layer_z]
{if layer_num == 0}
; Clean Purge Line
G0 X{max((min(print_bed_max[0] - 12, first_layer_print_min[0] + 80) - 85), 0)} Y{max((min(print_bed_max[1] - 3, first_layer_print_min[1] + 80) - 85), 0)} Z5 F6000
G0 Z[first_layer_height] F600
G1 E3 F1800
G1 X{(min(print_bed_max[0] - 12, first_layer_print_min[0] + 80))} E{85 * 0.5 * first_layer_height * nozzle_diameter[0]} F3000
G1 Y{max((min(print_bed_max[1] - 3, first_layer_print_min[1] + 80) - 85), 0) + 2} E{2 * 0.5 * first_layer_height * nozzle_diameter[0]} F3000
G1 X{max((min(print_bed_max[0] - 12, first_layer_print_min[0] + 80) - 85), 0)} E{85 * 0.5 * first_layer_height * nozzle_diameter[0]} F3000
G1 Y{max((min(print_bed_max[1] - 3, first_layer_print_min[1] + 80) - 85), 0) + 85} E{83 * 0.5 * first_layer_height * nozzle_diameter[0]} F3000
G1 X{max((min(print_bed_max[0] - 12, first_layer_print_min[0] + 80) - 85), 0) + 2} E{2 * 0.5 * first_layer_height * nozzle_diameter[0]} F3000
G1 Y{max((min(print_bed_max[1] - 3, first_layer_print_min[1] + 80) - 85), 0) + 3} E{82 * 0.5 * first_layer_height * nozzle_diameter[0]} F3000
G1 X{max((min(print_bed_max[0] - 12, first_layer_print_min[0] + 80) - 85), 0) + 3} Z0
G1 X{max((min(print_bed_max[0] - 12, first_layer_print_min[0] + 80) - 85), 0) + 6}
G1 Z1 F600
{endif}


Layer Change G-Code:

;AFTER_LAYER_CHANGE [layer_num] @ [layer_z]mm
SET_PRINT_STATS_INFO CURRENT_LAYER={layer_num + 1}


Change Filament G-Code:

; =================================================================
; Filament Change Sequence with Z-Sandwich
; =================================================================
; IMPORTANT: Requires PG101 and ORCA_PURGE macros from AddOn.cfg
; See README.md for setup instructions.
;
; Z-Sandwich Logic:
;   1. Slicer lifts Z +3mm at start
;   2. PG101 + Firmware do their work (no Z changes in macros)
;   3. Slicer restores Z -3mm at end
; This prevents Z stacking errors from competing Z moves.
; =================================================================

; --- 1. GLOBAL SAFETY LIFT ---
G91
G1 Z3 F12000      ; Lift Z +3mm (maintained throughout process)
G90
M83

; --- 2. EXECUTE CHANGE ---
; Retract OLD filament
G1 E-{retraction_length[current_extruder]} F1800

; Tool Change (Calls PG101 -> Firmware Unload/Load)
T[next_extruder]

; Purge & Wipe
ORCA_PURGE FLUSH={flush_length} RETRACT={retraction_length[next_extruder]}

; --- 3. GLOBAL RESTORE ---
G91
G1 Z-3 F12000     ; Restore Z -3mm (back to print height)
G90
