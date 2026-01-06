#!/bin/bash

# --- Configuration ---
PROJECT_NAME=$(awk -F'=' '/^PROJECT_REVISION/ {print $2}' *.qpf | cut -d '=' -f2 | tr -d '\'\"\' | tr -d '\r')
# Ensure the path below points to your Quartus installation bin directory if not in PATH
QUARTUS_DIR="$HOME/intelFPGA_lite/17.1/quartus/bin" 
export PATH=$QUARTUS_DIR:$PATH
# ---------------------
echo "Starting Quartus build for $PROJECT_NAME..."

# Optional: Generate system.h if using SoC EDS (requires Qsys generation)
# quartus_hps_generate_header_files system_name

# 1. Run the synthesis, map, and fit (compilation)
# The `quartus_sh --flow` command runs the entire design flow
quartus_sh --flow compile $PROJECT_NAME.qpf

if [ $? -eq 0 ]; then
    echo "Compilation successful."
else
    echo "Compilation failed."
    exit 1
fi

# 2. Generate the final programming file (.sof) if it wasn't done in the flow
# This command generates programming files based on current project settings in the .qsf
quartus_asm --read_settings_files=on --write_settings_files=off $PROJECT_NAME -c $PROJECT_NAME

if [ $? -eq 0 ]; then
    echo "Assembly successful. .sof file generated."
else
    echo "Assembly failed."
    exit 1
fi

echo "Build process complete."
