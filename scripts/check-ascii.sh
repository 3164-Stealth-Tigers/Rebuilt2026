#!/bin/bash
# Standalone ASCII check - scans .java and .md files for non-ASCII characters.
# Usage: ./scripts/check-ascii.sh

found=0

while IFS= read -r file; do
    matches=$(grep -Pn '[^\x00-\x7F]' "$file" 2>/dev/null)
    if [ -n "$matches" ]; then
        echo "Non-ASCII in $file:"
        echo "$matches"
        echo ""
        found=1
    fi
done < <(find src docs -type f \( -name '*.java' -o -name '*.md' \) 2>/dev/null)

if [ "$found" -eq 0 ]; then
    echo "No non-ASCII characters found."
    exit 0
else
    echo "Non-ASCII characters detected. Please fix the files above."
    exit 1
fi
