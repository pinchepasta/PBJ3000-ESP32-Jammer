# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sphinx build of the project documentation, as in Zephyr's example
# application: "doxygen" first for the API reference, then "make html".
# The HTML lands in _build_sphinx/html, the API reference in its doxygen/.

SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build_sphinx

help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
