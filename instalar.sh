#!/usr/bin/env bash

set -e

echo "=============================================="
echo "   INSTALACIÓN - GRUPO 02 KUKA KR 7 R900-3"
echo "=============================================="

# ------------------------------------------------
# 1. Verificar ROS 2 Jazzy
# ------------------------------------------------

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo
    echo "ERROR: No se encontró ROS 2 Jazzy."
    echo "Instala ROS 2 Jazzy antes de continuar."
    exit 1
fi

echo
echo "[1/5] ROS 2 Jazzy encontrado."

source /opt/ros/jazzy/setup.bash

# ------------------------------------------------
# 2. Instalar dependencias del sistema
# ------------------------------------------------

echo
echo "[2/5] Instalando dependencias..."

sudo apt update

sudo apt install -y \
    git \
    build-essential \
    python3-numpy \
    python3-colcon-common-extensions \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-urdf \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rmw-cyclonedds-cpp

# ------------------------------------------------
# 3. Descargar modelo KUKA
# ------------------------------------------------

echo
echo "[3/5] Preparando modelo KUKA..."

mkdir -p src

if [ -d "src/kuka_robot_descriptions/.git" ]; then

    echo "Repositorio KUKA ya existe."

    git -C src/kuka_robot_descriptions fetch --all

else

    echo "Clonando repositorio KUKA..."

    git clone \
        https://github.com/kroshu/kuka_robot_descriptions.git \
        src/kuka_robot_descriptions

fi

echo "Seleccionando versión f0202b2..."

git -C src/kuka_robot_descriptions checkout f0202b2

# ------------------------------------------------
# 4. Compilar workspace
# ------------------------------------------------

echo
echo "[4/5] Compilando workspace..."

colcon build --symlink-install

# ------------------------------------------------
# 5. Finalizar
# ------------------------------------------------

echo
echo "[5/5] Instalación completada."

echo
echo "=============================================="
echo "             INSTALACIÓN OK"
echo "=============================================="

echo
echo "Para utilizar el proyecto ejecuta:"
echo
echo "source entorno.sh"
echo
echo "Para abrir RViz:"
echo
echo "ros2 launch grupo02_kuka_kr7_bringup display.launch.py"
echo
echo "Para ejecutar FK:"
echo
echo "ros2 run grupo02_robot_kinematics fk_node"
echo
echo "Para ejecutar IK:"
echo
echo "ros2 run grupo02_robot_kinematics ik_node"
echo