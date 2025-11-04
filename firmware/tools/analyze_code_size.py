#!/usr/bin/env python3
"""
Analyze code size from ARM ELF file.

Usage:
    python3 analyze_code_size.py <elf_file>
"""

import subprocess
import sys
import re
from collections import defaultdict


def get_section_sizes(elf_file):
    """Get section sizes from ELF file."""
    result = subprocess.run(
        ['arm-none-eabi-size', '-A', elf_file],
        capture_output=True, text=True, check=True
    )

    sections = {}
    for line in result.stdout.split('\n'):
        parts = line.split()
        if len(parts) >= 2 and parts[0] not in ['section', 'Total']:
            try:
                section_name = parts[0]
                size = int(parts[1])
                sections[section_name] = size
            except ValueError:
                pass

    return sections


def get_all_symbols(elf_file):
    """Get all symbols with their sizes and types."""
    result = subprocess.run(
        ['arm-none-eabi-nm', '--size-sort', '--print-size', '--radix=d', '--demangle', elf_file],
        capture_output=True, text=True, check=True
    )

    symbols = []
    for line in result.stdout.split('\n'):
        parts = line.split()
        if len(parts) >= 4:
            try:
                addr = int(parts[0])
                size = int(parts[1])
                sym_type = parts[2]
                symbol = ' '.join(parts[3:])

                if size > 0:
                    symbols.append({
                        'name': symbol,
                        'size': size,
                        'type': sym_type,
                        'addr': addr
                    })
            except ValueError:
                pass

    return symbols


def categorize_symbol(symbol_name):
    """Categorize a symbol by its name prefix."""
    name = symbol_name

    # STM32 HAL categories
    if name.startswith('HAL_UART_') or name.startswith('UART_'):
        return 'STM32 HAL: UART'
    elif name.startswith('HAL_DMA_') or name.startswith('DMA_'):
        return 'STM32 HAL: DMA'
    elif name.startswith('HAL_GPIO_') or name.startswith('GPIO_'):
        return 'STM32 HAL: GPIO'
    elif name.startswith('HAL_RCC_') or name.startswith('RCC_'):
        return 'STM32 HAL: RCC'
    elif name.startswith('HAL_TIM_') or name.startswith('TIM_'):
        return 'STM32 HAL: TIM'
    elif name.startswith('HAL_'):
        return 'STM32 HAL: Core'

    # Tsumikoro bus stack
    elif name.startswith('tsumikoro_bus_'):
        return 'Tsumikoro: Bus Handler'
    elif name.startswith('tsumikoro_hal_stm32'):
        return 'Tsumikoro: HAL STM32'
    elif name.startswith('tsumikoro_hal_'):
        return 'Tsumikoro: HAL Core'
    elif name.startswith('tsumikoro_packet_'):
        return 'Tsumikoro: Protocol'
    elif name.startswith('tsumikoro_crc8'):
        return 'Tsumikoro: CRC8'
    elif name.startswith('tsumikoro_'):
        return 'Tsumikoro: Other'

    # System and startup
    elif 'SystemClock' in name or 'SystemInit' in name or name.startswith('System'):
        return 'System: Init'

    # Application
    elif name in ['main', 'Bus_Init', 'GPIO_Init_Custom'] or name.startswith('bus_unsolicited'):
        return 'Application'

    # C runtime and standard library
    elif name.startswith('_') or name.startswith('__'):
        return 'C Runtime/Stdlib'

    # ISR and handlers
    elif 'IRQHandler' in name or 'Handler' in name:
        return 'ISR Handlers'

    else:
        return 'Other/Misc'


def analyze_code_size(elf_file):
    """Main analysis function."""

    # Get section sizes
    sections = get_section_sizes(elf_file)

    # Get all symbols
    symbols = get_all_symbols(elf_file)

    # Categorize symbols
    module_sizes = defaultdict(int)
    module_functions = defaultdict(list)

    for sym in symbols:
        category = categorize_symbol(sym['name'])
        module_sizes[category] += sym['size']
        module_functions[category].append(sym)

    # Calculate totals
    flash_sections = ['.text', '.rodata', '.data']
    ram_sections = ['.data', '.bss', '._user_heap_stack']

    total_flash = sum(sections.get(s, 0) for s in flash_sections) + sections.get('.isr_vector', 0)
    total_ram = sum(sections.get(s, 0) for s in ram_sections)
    total_code = sum(sym['size'] for sym in symbols)

    return {
        'sections': sections,
        'symbols': symbols,
        'module_sizes': module_sizes,
        'module_functions': module_functions,
        'total_flash': total_flash,
        'total_ram': total_ram,
        'total_code': total_code
    }


def print_summary(analysis, flash_size=65536, ram_size=36864):
    """Print formatted summary."""

    sections = analysis['sections']
    module_sizes = analysis['module_sizes']
    module_functions = analysis['module_functions']
    total_flash = analysis['total_flash']
    total_ram = analysis['total_ram']
    total_code = analysis['total_code']

    # Header
    print("\n" + "=" * 80)
    print("CODE SIZE ANALYSIS")
    print("=" * 80)

    # Flash breakdown
    print(f"\nFLASH Memory: {total_flash:,} bytes / {flash_size:,} bytes ({100*total_flash/flash_size:.1f}%)")
    print("-" * 80)
    for section in ['.isr_vector', '.text', '.rodata', '.data']:
        if section in sections:
            size = sections[section]
            pct = 100 * size / total_flash if total_flash > 0 else 0
            print(f"  {section:20s} {size:8,d} bytes ({pct:5.1f}%)")

    # RAM breakdown
    print(f"\nRAM Memory: {total_ram:,} bytes / {ram_size:,} bytes ({100*total_ram/ram_size:.1f}%)")
    print("-" * 80)
    for section in ['.data', '.bss', '._user_heap_stack']:
        if section in sections:
            size = sections[section]
            pct = 100 * size / total_ram if total_ram > 0 else 0
            print(f"  {section:20s} {size:8,d} bytes ({pct:5.1f}%)")

    # Module breakdown
    print(f"\nCode Size by Module: {total_code:,} bytes")
    print("=" * 80)

    # Sort modules by size
    sorted_modules = sorted(module_sizes.items(), key=lambda x: x[1], reverse=True)

    for module, size in sorted_modules:
        pct = 100 * size / total_code if total_code > 0 else 0
        func_count = len(module_functions[module])
        print(f"{module:35s} {size:8,d} bytes ({pct:5.1f}%)  [{func_count:3d} functions]")

    # Top functions overall
    print("\n" + "=" * 80)
    print("Top 20 Largest Functions")
    print("=" * 80)

    sorted_symbols = sorted(analysis['symbols'], key=lambda x: x['size'], reverse=True)
    for i, sym in enumerate(sorted_symbols[:20], 1):
        category = categorize_symbol(sym['name'])
        name_truncated = sym['name'][:50]
        print(f"{i:2d}. {name_truncated:50s} {sym['size']:6,d} bytes  [{category}]")

    # Per-module function breakdown
    print("\n" + "=" * 80)
    print("Per-Module Function Breakdown")
    print("=" * 80)

    for module, size in sorted_modules:
        if size < 100:  # Skip very small modules
            continue

        funcs = module_functions[module]
        sorted_funcs = sorted(funcs, key=lambda x: x['size'], reverse=True)

        print(f"\n{module} - {size:,} bytes total:")
        print("-" * 80)

        # Show top 10 functions in this module
        for func in sorted_funcs[:10]:
            name_truncated = func['name'][:60]
            print(f"  {name_truncated:60s} {func['size']:6,d} bytes")

        if len(sorted_funcs) > 10:
            remaining = len(sorted_funcs) - 10
            remaining_size = sum(f['size'] for f in sorted_funcs[10:])
            print(f"  ... and {remaining} more functions ({remaining_size:,} bytes)")


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 analyze_code_size.py <elf_file>", file=sys.stderr)
        sys.exit(1)

    elf_file = sys.argv[1]

    try:
        analysis = analyze_code_size(elf_file)
        print_summary(analysis)
    except subprocess.CalledProcessError as e:
        print(f"Error running ARM toolchain command: {e}", file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        print("Make sure arm-none-eabi-size and arm-none-eabi-nm are in your PATH", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
