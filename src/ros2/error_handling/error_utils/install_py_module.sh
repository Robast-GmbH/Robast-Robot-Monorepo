#!/bin/bash

# Get the directory of the current script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Define the filename to check
FILE_NAME="error_definitions_pybind.so"

# Check if the file exists
if [[ -f "${SCRIPT_DIR}/${FILE_NAME}" ]]; then
    echo "File ${FILE_NAME} found in script directory."

    # Get the Python site-packages path dynamically
    SITE_PACKAGES=$(python3 -c "import site; print(site.getusersitepackages())")

    # Check if site-packages path was resolved
    if [[ -n "$SITE_PACKAGES" ]]; then
        echo "Copying ${FILE_NAME} to ${SITE_PACKAGES}..."
        cp "${SCRIPT_DIR}/${FILE_NAME}" "${SITE_PACKAGES}"
        echo "File copied successfully."
    else
        echo "Could not resolve the Python site-packages path."
        exit 1
    fi
else
    echo "File ${FILE_NAME} does not exist in the script directory."
    exit 1
fi
