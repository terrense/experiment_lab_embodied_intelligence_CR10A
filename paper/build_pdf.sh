#!/usr/bin/env bash
# Build the paper's PDF from the committed Markdown. SIMULATION-ONLY content, no robot involved.
#
# There is no LaTeX toolchain in this environment, so the PDF is built in two steps:
#
#   1. pandoc turns paper/manuscript.md into a self-contained HTML with the figures inlined;
#   2. Edge (headless) prints that HTML to a PDF.
#
# The Markdown in paper/manuscript.md stays the SOURCE OF TRUTH: this script only renders it, and
# a PDF that nobody can rebuild is not an artefact. Re-running it after any edit to the Markdown
# is the whole point.
#
# Usage:
#   bash paper/build_pdf.sh              # -> paper/manuscript.pdf
#   bash paper/build_pdf.sh --html-only  # -> paper/manuscript.html (for inspection)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MD="paper/manuscript.md"
HTML="paper/manuscript.html"
PDF="paper/manuscript.pdf"
CSS="paper/paper.css"

command -v pandoc >/dev/null || { echo "pandoc is missing (apt-get install pandoc)"; exit 1; }

# The figures are referenced from the manuscript as ../assets/..., so pandoc has to run FROM the
# paper directory for them to resolve; the HTML embeds them as data URIs, so the PDF is
# self-contained.
cd "$ROOT/paper"
pandoc "manuscript.md" \
  --from=markdown+pipe_tables+implicit_figures \
  --to=html5 \
  --standalone \
  --self-contained \
  --toc --toc-depth=2 \
  --css="paper.css" \
  --metadata title="Execution-Aware Safety Filtering for Learned Motion Policies on a Position-Controlled Collaborative Manipulator" \
  --metadata author="Xin Shen and collaborators" \
  --output="manuscript.html"
cd "$ROOT"

if [ "${1:-}" = "--html-only" ]; then
  echo "wrote $HTML"
  exit 0
fi

# Edge is the only HTML-to-PDF engine on this machine. --headless --print-to-pdf prints the
# document with the print stylesheet, which the CSS sets to the paper's geometry.
EDGE=""
for c in "/mnt/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe" \
         "/mnt/c/Program Files/Microsoft/Edge/Application/msedge.exe" \
         "/mnt/c/Program Files (x86)/Microsoft/Edge Beta/Application/msedge.exe"; do
  [ -f "$c" ] && EDGE="$c" && break
done
[ -n "$EDGE" ] || { echo "no Edge found; use --html-only and print the HTML yourself"; exit 1; }

WIN_HTML="$(wslpath -w "$ROOT/paper/manuscript.html")"
WIN_PDF="$(wslpath -w "$ROOT/paper/manuscript.pdf")"
rm -f "$PDF"
"$EDGE" --headless --disable-gpu --no-pdf-header-footer \
        --print-to-pdf="$WIN_PDF" "file:///$WIN_HTML" >/dev/null 2>&1 || true

for _ in $(seq 1 20); do [ -s "$PDF" ] && break; sleep 1; done
[ -s "$PDF" ] || { echo "the PDF was not produced"; exit 1; }
echo "wrote $PDF ($(du -h "$PDF" | cut -f1))"
