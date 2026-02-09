#!/bin/bash
set -e

DOCS_DIR=$(pwd)
SRC_DIR="../src"
VENV_PATH="../.venv-doc"

echo "=== Checking Environment ===" 
if [ ! -d "$VENV_PATH" ]; then 
   echo "Error: Virtual environment not found at $VENV_PATH" 
   echo "Please run: python3 -m venv $VENV_PATH && source $VENV_PATH/bin/activate && pip install -r requirements.txt" 
   exit 1 
fi

echo "=== Activate doc venv ==="
source $VENV_PATH/bin/activate

# Vérification des dépendances critiques
python3 -c "import sphinx, myst_parser, sphinxcontrib.mermaid, numpy" 2>/dev/null || { 
   echo "Error: Missing dependencies in venv. Run pip install -r requirements.txt"; 
   exit 1; 
}

echo "=== Source ROS ==="
source /opt/ros/humble/setup.bash
source ../install/setup.bash

echo "=== Clean old rst (except conf.py) ==="
rm -f $DOCS_DIR/source/*.rst
rm -f $DOCS_DIR/source/*.md
rm -rf $DOCS_DIR/source/interfaces
rm -rf $DOCS_DIR/source/launch

# --------------------------------------------------
# READMEs
# --------------------------------------------------

echo "=== Copy package READMEs ==="
cp $SRC_DIR/fleet_interfaces/README.md source/fleet_interfaces_readme.md 2>/dev/null || true
cp $SRC_DIR/fleet_adapter/README.md source/fleet_adapter_readme.md 2>/dev/null || true
cp $SRC_DIR/mission_manager/README.md source/mission_manager_readme.md 2>/dev/null || true
cp $SRC_DIR/fleet_simulation/README.md source/fleet_simulation_readme.md 2>/dev/null || true
cp $SRC_DIR/coflot_bringup_fleet/README.md source/coflot_bringup_fleet_readme.md 2>/dev/null || true

# --------------------------------------------------
# MSG DEFINITIONS
# --------------------------------------------------
echo "=== Copy workspace README ==="
cp ../README.md source/workspace_readme.md 2>/dev/null || true


echo "=== Copy msg definitions ==="
mkdir -p source/interfaces
cp $SRC_DIR/fleet_interfaces/msg/*.msg source/interfaces/

MSG_RST=source/fleet_interfaces_msg.rst

echo "fleet_interfaces — Messages" > $MSG_RST
echo "===========================" >> $MSG_RST
echo "" >> $MSG_RST

for f in source/interfaces/*.msg; do
  [ -e "$f" ] || continue
  name=$(basename $f)
  echo "$name" >> $MSG_RST
  echo "$(echo $name | tr '[:print:]' '-')" >> $MSG_RST
  echo "" >> $MSG_RST
  echo ".. literalinclude:: interfaces/$name" >> $MSG_RST
  echo "   :language: text" >> $MSG_RST
  echo "" >> $MSG_RST
done


# --------------------------------------------------
# LAUNCH FILES
# --------------------------------------------------

echo "=== Copy launch files ==="
mkdir -p source/launch
cp $SRC_DIR/coflot_bringup_fleet/launch/*.py source/launch/ 2>/dev/null || true

LAUNCH_RST=source/coflot_bringup_fleet_launch.rst

echo "coflot_bringup_fleet — Launch files" > $LAUNCH_RST
echo "==================================" >> $LAUNCH_RST
echo "" >> $LAUNCH_RST

for f in source/launch/*.py; do
  name=$(basename $f)
  echo "$name" >> $LAUNCH_RST
  echo "----------------" >> $LAUNCH_RST
  echo "" >> $LAUNCH_RST
  echo ".. literalinclude:: launch/$name" >> $LAUNCH_RST
  echo "   :language: python" >> $LAUNCH_RST
  echo "" >> $LAUNCH_RST
done

# --------------------------------------------------
# PYTHON API
# --------------------------------------------------

echo "=== Generate API rst ==="
sphinx-apidoc -f -o source $SRC_DIR/fleet_adapter/fleet_adapter --no-toc --module-first
sphinx-apidoc -f -o source $SRC_DIR/mission_manager/mission_manager --no-toc --module-first
sphinx-apidoc -f -o source $SRC_DIR/fleet_simulation/fleet_simulation --no-toc --module-first

# --------------------------------------------------
# INDEX
# --------------------------------------------------

echo "=== Create index.rst ==="
cat > source/index.rst <<EOL
Coflot ROS Documentation
========================

.. toctree::
   :maxdepth: 2
   :caption: Workspace

   workspace_readme

.. toctree::
   :maxdepth: 2
   :caption: Packages Overview

   fleet_interfaces_readme
   mission_manager_readme
   fleet_adapter_readme
   coflot_bringup_fleet_readme
   fleet_simulation_readme

.. toctree::
   :maxdepth: 2
   :caption: Python API

   fleet_adapter
   mission_manager
   fleet_simulation

.. toctree::
   :maxdepth: 2
   :caption: ROS Interfaces

   fleet_interfaces_msg
   coflot_bringup_fleet_launch

EOL

# --------------------------------------------------

echo "=== Build HTML ==="
make clean
make html

echo "=== DONE ==="
echo "Open: build/html/index.html"
