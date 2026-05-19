# Third-party library parts

## XKB TS-1187A-B-A-B tact switch

- `footprints.pretty/SW_Push_1P1T_XKB_TS-1187A.kicad_mod` — copied from
  KiCad's stock `Button_Switch_SMD` library. KiCad's upstream footprint
  references a 3D STEP that was never shipped with the installer, so we
  vendor the footprint and repoint it at a working model.
- `3dmodels/SW_Push_1P1T_XKB_TS-1187A.step` — real TS-1187A 3D model
  (body size ~5.1×5.1×3.8 mm).

Source for the STEP:
<https://github.com/MaloufSleep/ME-ESP8266/blob/master/3D_Models/Button_Switch_SMD.3dShapes/TS-1187A-B-A-B.step>

The ME-ESP8266 repo is a public MIT-licensed project that embeds the
switch model alongside its KiCad design. FreeCAD authored the STEP
(`FILE_DESCRIPTION(('FreeCAD Model')`). Treat as practical-to-use for
our in-house design previews; if you need a canonical datasheet-verified
model, pull from XKB Connectivity directly.

The KiCad footprint itself is CC-BY-SA 4.0 (upstream KiCad library).

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
