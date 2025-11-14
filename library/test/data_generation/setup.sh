#!/bin/bash
set -e

echo "Setting up Python environment for test data generation..."

# Check if Python 3 is installed
if python3 --version &> /dev/null; then
    PYTHON=python3
elif python --version &> /dev/null; then
    PYTHON=python
else
    echo "ERROR: Python 3 is not installed!"

    exit 1
fi

echo "Found Python: $($PYTHON --version)"

# Check if PDM is installed
if ! (command -v pdm &> /dev/null) && ! ($PYTHON -m pdm --version &> /dev/null); then
    echo "PDM not found, installing it..."
    $PYTHON -m pip install --user pdm --no-warn-script-location
fi

# Install dependencies
echo "Installing dependencies..."
if (command -v pdm &> /dev/null); then
    pdm install
else
    $PYTHON -m pdm install
fi

echo ""
echo "Setup complete!"
