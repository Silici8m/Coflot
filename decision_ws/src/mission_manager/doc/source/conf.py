import os
import sys

# Ajouter le dossier du package au path pour que Sphinx trouve le code
sys.path.insert(0, os.path.abspath('../../'))

project = 'mission_manager'
copyright = '2024, Fleet Team'
author = 'Victor'

# EXTENSIONS CRITIQUES
extensions = [
    'sphinx.ext.autodoc',      # Génération depuis les docstrings
    'sphinx.ext.napoleon',     # Support du format Google Style
    'sphinx.ext.viewcode',     # Lien vers le code source
    'sphinx.ext.todo',
]

# Config Napoleon (Google Style)
napoleon_google_docstring = True
napoleon_numpy_docstring = False
napoleon_include_init_with_doc = True
napoleon_include_private_with_doc = True

templates_path = ['_templates']
exclude_patterns = []

html_theme = 'sphinx_rtd_theme' # Theme "Read The Docs" (standard ROS)
