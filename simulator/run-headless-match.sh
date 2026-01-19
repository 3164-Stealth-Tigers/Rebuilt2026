#!/bin/bash
# Run a headless REBUILT 2026 match simulation
# Usage: ./run-headless-match.sh [count]
#   count - number of matches to run (default: 1)

cd "$(dirname "$0")"

COUNT=${1:-1}

echo "Running $COUNT headless match(es)..."
echo ""

# Run the HeadlessMatchRunner directly
./gradlew -q run --args="headless $COUNT" 2>&1

echo ""
echo "Done!"
