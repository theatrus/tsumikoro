# Third-party library parts

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
