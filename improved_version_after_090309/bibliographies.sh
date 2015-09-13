# 1. compile the document with LaTeX
pdflatex saponaro_msc_thesis

# 2. run BibTeX on the document
bibtex saponaro_msc_thesis
bibtex web

# 3. recompile with LaTeX three times
pdflatex saponaro_msc_thesis
pdflatex saponaro_msc_thesis
pdflatex saponaro_msc_thesis
