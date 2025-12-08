#! /bin/bash

CODE=$1
TARGET=$2
PATTERN=$3
N=$4
OUTPUT_PATH=$5

echo "=== Debug Information ==="
echo "Using Python from: $(which python3)"
echo "Python version: $(python3 --version)"
echo "Current directory: $(pwd)"
echo "Script arguments:"
echo "  CODE: $CODE"
echo "  TARGET: $TARGET"
echo "  PATTERN: $PATTERN"
echo "  N: $N"
echo "  OUTPUT_PATH: $OUTPUT_PATH"
echo "  DEBUG: $DEBUG"

# Check if files exist
if [ ! -f "$CODE" ]; then
    echo "ERROR: Code file not found: $CODE"
    exit 1
fi

if [ ! -f "$TARGET" ]; then
    echo "ERROR: Target file not found: $TARGET"
    exit 1
fi

if [ ! -f "$PATTERN" ]; then
    echo "ERROR: Pattern file not found: $PATTERN"
    exit 1
fi

echo "=== Starting Python script ==="

timeout --kill-after=30s 3h python3 $CODE $TARGET $PATTERN $N $OUTPUT_PATH

exit_code=$?
echo "=== Python script finished with exit code: $exit_code ==="