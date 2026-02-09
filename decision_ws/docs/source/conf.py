# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html


import os
import sys


sys.path.insert(0, os.path.abspath('../src/fleet_adapter'))
sys.path.insert(0, os.path.abspath('../src/mission_manager'))
sys.path.insert(0, os.path.abspath('../src/fleet_simulation'))
sys.path.insert(0, os.path.abspath('../src/coflot_bringup_fleet'))


# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'coflot_fleet'
copyright = '2026, Victor Forcioli, Alexis Vermeersch, Djino Varrambier Geoffroy'
author = 'Victor Forcioli, Alexis Vermeersch, Djino Varrambier Geoffroy'
release = '1.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'myst_parser',
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx.ext.intersphinx",
    "sphinx_autodoc_typehints",
    'sphinxcontrib.mermaid',
]

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']
exclude_patterns = []

autodoc_mock_imports = [
    "rclpy",
    "numpy",
    "scipy",
    "geometry_msgs",
    "nav_msgs",
    "std_msgs",
    "fleet_interfaces",
]

napoleon_google_docstring = True
napoleon_numpy_docstring = True

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "rclpy": ("https://docs.ros.org/en/humble/p/rclpy/", None),
    "scipy": ("https://docs.scipy.org/doc/scipy/", None),
    "numpy": ("https://numpy.org/doc/stable/", None),
}



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

autodoc_inherit_docstrings = False
nosidebar = True