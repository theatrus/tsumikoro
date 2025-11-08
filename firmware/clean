#!/bin/bash
# Clean build artifacts for Tsumikoro firmware projects

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_usage() {
    echo "Usage: $0 [project-name]"
    echo ""
    echo "Clean build artifacts for firmware projects"
    echo ""
    echo "Options:"
    echo "  (no args)       Clean all build directories"
    echo "  project-name    Clean specific project build directory"
    echo ""
    echo "Examples:"
    echo "  $0                    # Clean all projects"
    echo "  $0 ministepper-g071   # Clean ministepper build"
    echo "  $0 servo-g030         # Clean servo build"
    echo ""
    echo "Available projects in build/:"
    if [ -d "$BUILD_DIR" ]; then
        ls -1 "$BUILD_DIR" 2>/dev/null || echo "  (none)"
    else
        echo "  (build directory does not exist)"
    fi
}

clean_project() {
    local project=$1
    local project_path="${BUILD_DIR}/${project}"

    if [ -d "$project_path" ]; then
        echo -e "${YELLOW}Cleaning ${project}...${NC}"
        rm -rf "$project_path"
        echo -e "${GREEN}✓ Cleaned ${project}${NC}"
        return 0
    else
        echo -e "${RED}✗ Project build directory not found: ${project}${NC}"
        return 1
    fi
}

clean_all() {
    if [ ! -d "$BUILD_DIR" ]; then
        echo -e "${YELLOW}No build directory found. Nothing to clean.${NC}"
        return 0
    fi

    local count=0
    echo -e "${YELLOW}Cleaning all build directories...${NC}"

    for project in "$BUILD_DIR"/*; do
        if [ -d "$project" ]; then
            local project_name=$(basename "$project")
            rm -rf "$project"
            echo -e "${GREEN}✓ Cleaned ${project_name}${NC}"
            ((count++))
        fi
    done

    if [ $count -eq 0 ]; then
        echo -e "${YELLOW}No projects found to clean.${NC}"
    else
        echo -e "${GREEN}Cleaned ${count} project(s)${NC}"
    fi
}

# Main script
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    print_usage
    exit 0
fi

cd "$SCRIPT_DIR"

if [ -z "$1" ]; then
    # No arguments - clean all
    clean_all
else
    # Clean specific project
    clean_project "$1"
fi
