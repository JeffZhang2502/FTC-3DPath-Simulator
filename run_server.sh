#!/bin/bash
# Start the headless Java simulation server.
# Usage: ./run_server.sh [port]
# Default port: 9876

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
JAVA_PROJECT="$SCRIPT_DIR/FTC-Path-Simulator"

PORT="${1:-9876}"

echo "=== FTC Auto Simulator — Headless Server ==="
echo "Project: $JAVA_PROJECT"
echo "Port:    $PORT"
echo ""

cd "$JAVA_PROJECT" || exit 1

# Build if needed
if [ ! -f "build.gradle" ]; then
    echo "[ERROR] build.gradle not found in $JAVA_PROJECT"
    exit 1
fi

echo "[BUILD] Compiling..."
javac -d out --release 17 \
    -cp lib/gson-2.10.1.jar \
    $(find src/main/java -name "*.java") 2>&1

if [ $? -ne 0 ]; then
    echo "[ERROR] Java compilation failed. Check that JDK 17+ is installed."
    exit 1
fi

echo "[READY] Starting server on port $PORT..."
echo "        Waiting for Metal renderer connection..."
echo ""
java -cp out:lib/gson-2.10.1.jar simulator.server.SimServer "$PORT"
