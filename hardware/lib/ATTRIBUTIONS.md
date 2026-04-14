# Third-party library parts

## XKB TS-1187A tact switch — footprint only

KiCad's stock `Button_Switch_SMD:SW_Push_1P1T_XKB_TS-1187A` references a
3D STEP file that is missing from the KiCad assets. We vendor the
footprint as `tsumikoro:SW_Push_1P1T_XKB_TS-1187A` and repoint the 3D
model at `3dmodels/SW_Push_1P1T_XKB_TS-1187A_approx.step`, which is a
**copy of `SW_Push_1P1T_NO_CK_KMR2.step`** (CK KMR2 tact switch, 4.2×3.2
mm) from the KiCad stock library. The KMR2 body is a visually close
approximation of the TS-1187A (4.5×4.5 mm) — fine for general 3D render
purposes, not dimensionally exact. Footprint pad layout is correct.

Both files are from KiCad's upstream libraries, CC-BY-SA 4.0.

Replace `SW_Push_1P1T_XKB_TS-1187A_approx.step` with a real TS-1187A
STEP (from XKB Connectivity, GrabCAD, or SnapEDA) if pixel-perfect 3D
preview matters for mechanical fit checks.

## Espressif — ESP32 symbols, footprints, 3D models

The following parts in this library are vendored from the upstream
Espressif KiCad library:

- `symbols/tsumikoro.kicad_sym` — `ESP32-S3-MINI-1`, `ESP32-S3-DevKitC`
- `footprints.pretty/ESP32-S3-MINI-1.kicad_mod`
- `footprints.pretty/ESP32-S3-DevKitC.kicad_mod`
- `3dmodels/ESP32-S3-MINI-1.STEP`

Source: <https://github.com/espressif/kicad-libraries>
License: **CC-BY-SA 4.0**

To refresh to a newer upstream version:

```sh
# Re-download upstream library
curl -sL https://github.com/espressif/kicad-libraries/archive/refs/heads/main.tar.gz \
    | tar xz -C /tmp/

# Replace our copies
cp /tmp/kicad-libraries-main/3dmodels/espressif.3dshapes/ESP32-S3-MINI-1.STEP \
   hardware/lib/3dmodels/
cp /tmp/kicad-libraries-main/footprints/Espressif.pretty/ESP32-S3-MINI-1.kicad_mod \
   /tmp/kicad-libraries-main/footprints/Espressif.pretty/ESP32-S3-DevKitC.kicad_mod \
   hardware/lib/footprints.pretty/

# Repoint 3D model path (KIPRJMOD-relative)
sed -i '' 's|\${KICAD.*_3RD_PARTY}/3dmodels/com_github_espressif_kicad-libraries/espressif.3dshapes/|\${KIPRJMOD}/../lib/3dmodels/|' \
    hardware/lib/footprints.pretty/ESP32-S3-MINI-1.kicad_mod
```

Symbols are extracted into `symbols/tsumikoro.kicad_sym` manually (grab the
`(symbol "ESP32-S3-MINI-1" ...)` and `(symbol "ESP32-S3-DevKitC" ...)` blocks
from upstream `symbols/Espressif.kicad_sym` and append them to ours — drop
the `PCM_Espressif:` prefix on embedded Footprint properties so they
resolve against our project-local `tsumikoro:` library).
