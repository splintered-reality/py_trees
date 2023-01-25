# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

################################################################################
# Setup
################################################################################

import os
import sys

project_dir = os.path.abspath(os.path.join(os.path.abspath(__file__), os.pardir, os.pardir))
version_file = os.path.join(project_dir, 'py_trees', 'version.py')
with open(version_file) as f:
    exec(f.read())  # makes __version__ available

################################################################################
# Autodoc - help it find the project
# https://sphinx-rtd-tutorial.readthedocs.io/en/latest/sphinx-config.html#autodoc-configuration
################################################################################

sys.path.insert(0, project_dir)

################################################################################
# Project Info
################################################################################

project = 'py_trees'
copyright = '2023, Daniel Stonier'
author = 'Daniel Stonier'

version = __version__
release = version

################################################################################
# General configuration
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration
################################################################################

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.coverage',
    'sphinx.ext.doctest',
    'sphinx.ext.ifconfig',
    'sphinx.ext.intersphinx',
    'sphinx.ext.mathjax',
    'sphinx.ext.napoleon',
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx.ext.graphviz',
    'sphinxarg.ext',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'weblinks.rst']

language = 'en'

################################################################################
# Extensions Configuration
################################################################################

# If true, use the :ivar: role for instance variables, else .. attribute::.
napoleon_use_ivar = True

# If you don't add this, todos don't appear
todo_include_todos = True

################################################################################
# HTML
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output
################################################################################

html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# If true, “(C) Copyright …” is shown in the HTML footer. Default is True.
html_show_copyright = False

# If true, "Created using Sphinx" is shown in the HTML footer. Default is True.
html_show_sphinx = False

################################################################################
# Intersphinx
# https://www.sphinx-doc.org/en/master/usage/extensions/intersphinx.html#configuration
################################################################################

# Refer to the Python standard library.
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
}
