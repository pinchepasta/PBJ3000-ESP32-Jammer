# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sphinx configuration of the project documentation: the guides in this
# directory (Markdown, through MyST), the README and the apps' READMEs.
# The API reference comes from Doxygen (Doxyfile); when it has been built
# first, it is copied into the HTML output as doxygen/.

import re
import shutil
from pathlib import Path

from pygments.lexers.special import TextLexer
from sphinx.highlighting import lexers

project = "esp-sdr-zephyr"
author = "The esp-sdr-zephyr contributors"
copyright = author

extensions = ["myst_parser", "sphinx.ext.intersphinx"]
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
exclude_patterns = ["_build*", "_doxygen", "Thumbs.db", ".DS_Store"]

html_theme = "alabaster"

intersphinx_mapping = {"zephyr": ("https://docs.zephyrproject.org/latest/", None)}

# Links to headings inside the Markdown guides (file.md#section).
myst_heading_anchors = 4

# The diagram sources in the guides (GitHub renders them, Pygments cannot).
lexers["mermaid"] = TextLexer()

DOC = Path(__file__).resolve().parent
REPO = DOC.parent
DOXYGEN_HTML = DOC / "_build_doxygen" / "html"

VERSION = dict(re.findall(r"^(\w+) = (\d+)$", (REPO / "VERSION").read_text(), re.M))
version = release = "{VERSION_MAJOR}.{VERSION_MINOR}.{PATCHLEVEL}".format(**VERSION)
html_title = f"{project} {release}"

# overview.rst shows the README. Its links are relative to the repository root:
# guides and app READMEs become links to their pages here, other files go to
# GitHub.
GITHUB = "https://github.com/cyber-murmel/esp-sdr-zephyr/blob/main/"
LINK = re.compile(r"`([^`<]+?)\s*<([^>\s]+)>`__")


def readme_link(m):
    text, target = m.group(1), m.group(2)
    if re.match(r"[a-z]+:", target):
        return m.group(0)
    app = re.fullmatch(r"apps/(\w+)/README\.rst", target)
    if target.startswith("doc/") and target.endswith(".md") and (REPO / target).is_file():
        return f":doc:`{text} <{target[len('doc/'):-len('.md')]}>`"
    if app and (DOC / "apps" / f"{app.group(1)}.rst").is_file():
        return f":doc:`{text} <apps/{app.group(1)}>`"
    return f"`{text} <{GITHUB}{target}>`__"


def readme_source(app, docname, source):
    if docname == "overview":
        source[0] = LINK.sub(readme_link, (REPO / "README.rst").read_text())


def copy_doxygen(app, exception):
    if exception is None and app.builder.format == "html" and DOXYGEN_HTML.is_dir():
        shutil.copytree(DOXYGEN_HTML, Path(app.outdir) / "doxygen", dirs_exist_ok=True)


def setup(app):
    app.connect("source-read", readme_source)
    app.connect("build-finished", copy_doxygen)
