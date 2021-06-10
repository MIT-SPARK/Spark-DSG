# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = "Kimera-DSG"
copyright = "2021, Toni Rosinol et al."
author = "Toni Rosinol et al."


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ["sphinx.ext.mathjax", "sphinx_rtd_theme", "breathe", "exhale"]

breathe_projects = {"Kimera-DSG": "./_build/doxyoutput/xml"}
breathe_default_project = "Kimera-DSG"

exhale_args = {
    "containmentFolder": "./cpp_api",
    "rootFileName": "kimera_dsg_cpp_api.rst",
    "rootFileTitle": "Kimera-DSG Core C++ API",
    "doxygenStripFromPath": ".",
    "createTreeView": True,
    "treeViewIsBootstrap": True,
    "exhaleExecutesDoxygen": True,
    "exhaleDoxygenStdin": """
        INPUT = ../kimera_dsg/include
        EXTRACT_PRIVATE = NO
        EXTRACT_ALL = NO
        MACRO_EXPANSION = YES
        EXPAND_ONLY_PREDEF = YES
        PREDEFINED += __attribute__((x))=
    """,
}

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
#html_theme = "sphinx_rtd_theme"
html_theme = "alabaster"


# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

primary_domain = "cpp"
highlight_language = "cpp"
