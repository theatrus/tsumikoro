# Tsumikoro hardware — Makefile
#
# Targets:
#   make docs            regenerate all docs (images + BOMs) for every PCB
#   make docs-servo      regenerate docs for the servo board only
#   make bom-servo       regenerate the JLCPCB BOM for the servo board only
#   make images-servo    regenerate schematic/PCB images for the servo board
#   make jlc-servo       build the JLCPCB fab/assembly zip for the servo board
#   make clean-docs      remove generated images and BOMs
#   make clean-jlc       remove generated JLCPCB packages
#
# Requires kicad-cli (KiCad 9+/10) on PATH.

KICAD_CLI ?= kicad-cli

PROJECTS    := servo
SERVO_DIR   := servo
SERVO_SCH   := $(SERVO_DIR)/servo.kicad_sch
SERVO_PCB   := $(SERVO_DIR)/servo.kicad_pcb
SERVO_IMG   := $(SERVO_DIR)/docs/images
SERVO_BOM   := $(SERVO_DIR)/jlcpcb_bom.csv

BOM_FIELDS  := $${ITEM_NUMBER},Reference,Value,Footprint,$${QUANTITY},LCSC,MPN,Manufacturer
BOM_LABELS  := Item,Designator,Comment,Footprint,Qty,LCSC Part \#,MPN,Manufacturer

SCRIPTS_DIR := scripts
JLC_SCRIPT  := $(SCRIPTS_DIR)/jlcpcb-package.sh

.PHONY: all docs docs-servo images-servo bom-servo jlc jlc-servo clean-docs clean-jlc help

all: docs

help:
	@echo "Tsumikoro hardware makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  docs          Generate images + BOMs for every PCB project"
	@echo "  docs-servo    Same but only for the servo board"
	@echo "  images-servo  Just the schematic/PCB images for servo"
	@echo "  bom-servo     Just the JLCPCB BOM for servo"
	@echo "  jlc           Build JLCPCB fab+assembly zips for every PCB project"
	@echo "  jlc-servo     JLCPCB zip for servo only"
	@echo "  clean-docs    Remove generated docs"
	@echo "  clean-jlc     Remove generated JLCPCB packages"

docs: docs-servo

docs-servo: images-servo bom-servo

images-servo: $(SERVO_SCH) $(SERVO_PCB)
	@mkdir -p $(SERVO_IMG)
	@echo "[servo] Schematic SVGs"
	$(KICAD_CLI) sch export svg -o $(SERVO_IMG)/ $(SERVO_SCH)
	@echo "[servo] PCB top SVG"
	$(KICAD_CLI) pcb export svg -o $(SERVO_IMG)/pcb-top.svg \
	    --layers "F.Cu,F.Silkscreen,F.Mask,Edge.Cuts" \
	    --page-size-mode 2 $(SERVO_PCB)
	@echo "[servo] PCB bottom SVG (mirrored)"
	$(KICAD_CLI) pcb export svg -o $(SERVO_IMG)/pcb-bottom.svg \
	    --layers "B.Cu,B.Silkscreen,B.Mask,Edge.Cuts" \
	    --mirror --page-size-mode 2 $(SERVO_PCB)
	@echo "[servo] 3D renders"
	$(KICAD_CLI) pcb render --side top    --width 1600 --height 1000 \
	    -o $(SERVO_IMG)/pcb-3d-top.png    $(SERVO_PCB)
	$(KICAD_CLI) pcb render --side bottom --width 1600 --height 1000 \
	    -o $(SERVO_IMG)/pcb-3d-bottom.png $(SERVO_PCB)

bom-servo: $(SERVO_SCH)
	@echo "[servo] JLCPCB BOM"
	$(KICAD_CLI) sch export bom \
	    --output $(SERVO_BOM) \
	    --fields '$(BOM_FIELDS)' \
	    --labels '$(BOM_LABELS)' \
	    --group-by 'Value,Footprint' \
	    --exclude-dnp \
	    $(SERVO_SCH)

clean-docs:
	rm -rf $(SERVO_IMG)
	rm -f  $(SERVO_BOM)

# --- JLCPCB manufacturing packages -------------------------------------------

jlc: jlc-servo

jlc-servo: $(JLC_SCRIPT) $(SERVO_PCB) $(SERVO_SCH)
	@echo "[jlcpcb] building package for servo"
	cd $(SERVO_DIR) && KICAD_CLI=$(KICAD_CLI) ../$(JLC_SCRIPT) \
	    servo.kicad_pcb servo.kicad_sch

clean-jlc:
	rm -rf $(SERVO_DIR)/jlcpcb
	rm -f  $(SERVO_DIR)/*-jlcpcb.zip
