#!/bin/bash
set -e

# 1. On régénère la documentation fraîchement
echo "=== Building Documentation ==="
./build_docs.sh

# 2. On publie sur la branche 'gh-pages'
echo "=== Pushing to GitHub Pages ==="
# -n : Ajoute un fichier .nojekyll (CRITIQUE pour que Sphinx s'affiche bien sur GitHub)
# -p : Push les changements vers origin
# -f : Force l'écrasement de l'ancienne doc
# build/html : Le dossier source à publier
ghp-import -n -p -f build/html

echo "=== SUCCESS ==="
echo "La documentation sera bientôt accessible en ligne."