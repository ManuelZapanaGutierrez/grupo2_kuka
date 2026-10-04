# Grupo 02 - Cinemática KUKA KR 7 R900-3

Proyecto de Robótica - IMT-342

Implementación en ROS 2 Jazzy de la cinemática directa (FK) y cinemática inversa (IK) para el robot industrial KUKA KR 7 R900-3.

## Integrantes

- Grupo 02

## Descripción

El proyecto permite visualizar el robot KUKA KR 7 R900-3 en RViz2 y realizar:

- Cinemática directa (Forward Kinematics, FK).
- Cinemática inversa (Inverse Kinematics, IK).
- Publicación y recepción de estados articulares mediante `/joint_states`.
- Verificación de las soluciones de cinemática inversa mediante cinemática directa.
- Pruebas con diferentes objetivos cartesianos.

La implementación está desarrollada como paquetes ROS 2 en Python.

## Requisitos

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- Git
- Colcon
- NumPy
- RViz2
- Xacro
- Cyclone DDS

## Estructura del proyecto

```text
grupo2_kuka/
├── .gitignore
├── dependencies.repos
├── entorno.sh
├── README.md
└── src/
    ├── grupo02_kuka_kr7_bringup/
    │   ├── launch/
    │   │   └── display.launch.py
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── grupo02_robot_kinematics/
        ├── grupo02_robot_kinematics/
        │   ├── __init__.py
        │   ├── fk_node.py
        │   └── ik_node.py
        ├── resource/
        ├── package.xml
        ├── setup.cfg
        └── setup.py