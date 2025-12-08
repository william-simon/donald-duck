#!/usr/bin/env bash
set -euo pipefail

# Download RDKit sample SDFs and concatenate into molecules.sdf
urls=(
  "https://raw.githubusercontent.com/rdkit/rdkit/2d25752fe0319309886a595d08c6f379c8c6acfb/Docs/Book/data/actives_5ht3.sdf"
  "https://raw.githubusercontent.com/rdkit/rdkit/2d25752fe0319309886a595d08c6f379c8c6acfb/Docs/Book/data/cdk2.sdf"
  "https://raw.githubusercontent.com/rdkit/rdkit/2d25752fe0319309886a595d08c6f379c8c6acfb/Docs/Book/data/5ht3ligs.sdf"
)

rm -f molecules.sdf
for url in "${urls[@]}"; do
  fname=$(basename "$url")
  echo "Fetching $fname..."
  curl -L -o "$fname" "$url"
  cat "$fname" >> molecules.sdf
done

echo "Cleaning up individual SDFs..."
for url in "${urls[@]}"; do
  rm -f "$(basename "$url")"
done

echo "Combined molecules written to molecules.sdf"
