import os

# Liste de vos packages Python
packages = [
    "mission_manager",
    "fleet_adapter",
    "fleet_simulation",
    "coflot_bringup_fleet"
]

# Contenu du rosdoc2.yaml
rosdoc2_template = """type: sphinx
source_dir: doc/source
build_dir: doc/_build
"""

# Contenu du conf.py (Active Napoleon pour Google Style)
conf_template = """import os
import sys

# Ajouter le dossier du package au path pour que Sphinx trouve le code
sys.path.insert(0, os.path.abspath('../../'))

project = '{pkg_name}'
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
"""

# Contenu du index.rst
index_template = """Welcome to {pkg_name}'s documentation!
================================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   modules

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
"""

base_path = os.path.join(os.getcwd(), "src")

for pkg in packages:
    # Chemin vers la racine du package (ex: src/mission_manager)
    # Note: Votre arborescence montre src/mission_manager/mission_manager (dossier source)
    # Le rosdoc2.yaml doit être à côté du setup.py et package.xml.
    pkg_root = os.path.join(base_path, pkg)
    
    if not os.path.exists(pkg_root):
        print(f"⚠️  Attention: {pkg_root} introuvable.")
        continue

    print(f"🔧 Configuration de {pkg}...")

    # 1. Création de rosdoc2.yaml
    with open(os.path.join(pkg_root, "rosdoc2.yaml"), "w") as f:
        f.write(rosdoc2_template)

    # 2. Création de l'arborescence doc/source
    doc_dir = os.path.join(pkg_root, "doc", "source")
    os.makedirs(doc_dir, exist_ok=True)

    # 3. Création de conf.py
    with open(os.path.join(doc_dir, "conf.py"), "w") as f:
        f.write(conf_template.format(pkg_name=pkg))

    # 4. Création de index.rst
    with open(os.path.join(doc_dir, "index.rst"), "w") as f:
        f.write(index_template.format(pkg_name=pkg))

print("\n✅ Fichiers de configuration générés. Passez à l'étape 3.")