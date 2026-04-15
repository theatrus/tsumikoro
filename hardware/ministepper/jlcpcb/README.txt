JLCPCB manufacturing package for ministepper
Generated: 2026-04-15T02:54:22Z

Files:
  gerbers/*.gbr           Gerber files (top/inner/bottom copper, mask, silk, paste)
  gerbers/*.drl           Excellon drill files (PTH and NPTH separate)
  gerbers/*-drl_map.gbr   Optional drill map
  ministepper-BOM.csv         JLCPCB-format BOM (upload as Assembly > Add BOM File)
  ministepper-CPL.csv         Pick-and-place (upload as Assembly > Add CPL File)

To order:
  1. Upload the zip to https://cart.jlcpcb.com/quote
  2. Enable "PCB Assembly" and upload the BOM + CPL files
  3. Verify part matching; fix any unmatched LCSC numbers manually

Notes:
  - Gerber layer names use KiCad's native scheme. JLCPCB auto-detects via X2.
  - Drill origin is set to absolute (matches Gerber aux origin).
  - CPL rotations are KiCad's convention. Some footprints (SOT-23, QFN, SOIC)
    may need JLCPCB-specific rotation offsets — verify in JLCPCB's preview
    step before confirming the order.
