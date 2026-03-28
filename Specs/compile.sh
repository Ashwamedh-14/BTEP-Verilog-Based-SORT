#!/usr/bin/env bash

set -euo pipefail

if [[ ! -f "./main.tex" ]]; then
    printf "Main tex file not found. Exiting...\n"
fi

for file in *.svg; do
    mkdir -p "./Figures"
    if [[ ! -f "$file" ]]; then
        continue
    fi
    printf "Processing file: %s\n" "$file"
    inkscape "$file" --export-type=pdf --export-latex
    
    pdf="${file/%svg/pdf}"
    tex="${file/%svg/pdf_tex}"
    printf "Generated %s and %s files\n" "$pdf" "$tex"
    mv "$pdf" "$tex" -t "./Figures"
done


latexmk -pdf main.tex
cp main.pdf Specifications.pdf

printf "PDF created and named Specifications.txt\n"
printf "Execute firefox Specifications.pdf & to view the pdf\n"

exit 0

