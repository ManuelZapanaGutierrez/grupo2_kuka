# Grupo 02 - Cinemática KUKA KR 7 R900-3

Proyecto de Robótica - IMT-342

Implementación en ROS 2 Jazzy de la cinemática directa (FK) y cinemática inversa numérica de posición (IK) para el robot industrial KUKA KR 7 R900-3.

## Integrantes

- Adrian Pacheco Barrios
- Manuel Joel Zapana Gutierrez

## Descripción

El proyecto permite visualizar el robot KUKA KR 7 R900-3 en RViz2 y ejecutar:

- Cinemática directa (Forward Kinematics, FK).
- Cinemática inversa numérica de posición (Inverse Kinematics, IK).
- Cálculo de posición y orientación del efector final `tool0`.
- Jacobiano posicional numérico mediante diferencias finitas.
- Pseudoinversa amortiguada.
- Límites articulares.
- Múltiples configuraciones iniciales o semillas.
- Publicación y recepción de estados articulares mediante `/joint_states`.
- Recepción de objetivos cartesianos mediante `/target`.
- Validación mediante FK, TF y RViz2.

La implementación está desarrollada como paquetes ROS 2 en Python.

La cinemática inversa implementada resuelve únicamente posición cartesiana `(x, y, z)`.

---

# Requisitos

El proyecto fue desarrollado y probado con:

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3
- Git
- Colcon
- NumPy
- RViz2
- Xacro
- ros2_control
- Joint State Publisher GUI
- Robot State Publisher
- Descripción KUKA `kuka_agilus_support`

---

# 1. Comprobación inicial

Verificar la versión de Ubuntu:

```bash
lsb_release -a
```

Debe utilizarse Ubuntu 24.04.

Verificar la distribución de ROS 2:

```bash
echo $ROS_DISTRO
```

Debe aparecer:

```text
jazzy
```

Verificar Git:

```bash
git --version
```

---

# 2. Clonar el proyecto

Desde el directorio personal:

```bash
cd "$HOME"
```

Clonar el repositorio:

```bash
git clone https://github.com/ManuelZapanaGutierrez/grupo2_kuka.git
```

Entrar al workspace:

```bash
cd "$HOME/grupo2_kuka"
```

---

# 3. Comprobar dependencias

Verificar si ROS 2 Control está disponible:

```bash
ros2 pkg list | grep hardware_interface
```

Si no aparece ningún resultado:

```bash
sudo apt update
sudo apt install ros-jazzy-ros2-control
```

Volver a comprobar:

```bash
ros2 pkg list | grep hardware_interface
```

---

# 4. Comprobar la descripción del robot KUKA

Ejecutar:

```bash
ros2 pkg list | grep kuka_agilus_support
```

Si aparece:

```text
kuka_agilus_support
```

no es necesario descargar otra copia.

Si no aparece, descargar la descripción dentro del workspace:

```bash
cd "$HOME/grupo2_kuka"
git clone https://github.com/kroshu/kuka_robot_descriptions.git src/kuka_robot_descriptions
```

Seleccionar exactamente la versión utilizada en este proyecto:

```bash
git -C src/kuka_robot_descriptions checkout f0202b2
```

El mensaje `detached HEAD` no representa un error.

Comprobar el commit:

```bash
git -C src/kuka_robot_descriptions rev-parse --short HEAD
```

Debe aparecer:

```text
f0202b2
```

---

# 5. Compilar el workspace

Entrar al proyecto:

```bash
cd "$HOME/grupo2_kuka"
```

Compilar:

```bash
colcon build --symlink-install
```

La compilación debe terminar sin paquetes marcados como `Failed`.

Cargar el workspace:

```bash
source install/setup.bash
```

Cargar el entorno del proyecto:

```bash
source entorno.sh
```

---

# 6. Comprobar que los paquetes están disponibles

Ejecutar:

```bash
ros2 pkg list | grep grupo02
```

Deben aparecer:

```text
grupo02_kuka_kr7_bringup
grupo02_robot_kinematics
```

Comprobar también:

```bash
ros2 pkg list | grep kuka_agilus_support
```

Debe aparecer:

```text
kuka_agilus_support
```

---

# 7. Carga del entorno en nuevas terminales

Cada terminal nueva utilizada para trabajar con el proyecto debe ejecutar:

```bash
cd "$HOME/grupo2_kuka"
source entorno.sh
```

Si fuera necesario, también puede cargarse:

```bash
source install/setup.bash
```

---

# 8. Abrir el robot en RViz2

Ejecutar:

```bash
cd "$HOME/grupo2_kuka"
source entorno.sh
ros2 launch grupo02_kuka_kr7_bringup display.launch.py
```

Debe aparecer:

- El robot KUKA KR 7 R900-3.
- RViz2.
- Joint State Publisher GUI.
- Los seis joints del robot.
- Los frames correspondientes.

---

# 9. Comprobar la comunicación ROS 2

Con el robot ejecutándose:

```bash
ros2 node list
```

```bash
ros2 topic list
```

Comprobar `/joint_states`:

```bash
ros2 topic echo /joint_states --once
```

Comprobar la descripción del robot:

```bash
ros2 topic echo /robot_description --once
```

---

# 10. Cinemática Directa - FK

Mantener abierto RViz2 y el Joint State Publisher GUI.

En otra terminal:

```bash
cd "$HOME/grupo2_kuka"
source entorno.sh
ros2 run grupo02_robot_kinematics fk_node
```

Debe aparecer un mensaje similar a:

```text
FK Node inicializado. Esperando /joint_states...
```

Mover una o varias articulaciones desde Joint State Publisher GUI.

El nodo mostrará valores similares a:

```text
q: [q1, q2, q3, q4, q5, q6]

Posición [m]:
x = ...
y = ...
z = ...

Orientación (quat):
...
```

El flujo es:

```text
/joint_states
      ↓
   fk_node
      ↓
modelo DH
      ↓
posición + orientación de tool0
```

---

# 11. Validar FK mediante TF

En otra terminal:

```bash
cd "$HOME/grupo2_kuka"
source entorno.sh
timeout 3 ros2 run tf2_ros tf2_echo base_link tool0
```

TF mostrará aproximadamente:

```text
Translation: [x, y, z]
Rotation: ...
```

La posición calculada por FK debe coincidir aproximadamente con la transformación obtenida mediante TF.

La validación utilizada es:

```text
q → FK → p_FK ≈ p_TF
```

---

# 12. Cinemática Inversa - IK

Antes de ejecutar IK debe cerrarse el **Joint State Publisher GUI**.

Esto evita tener dos nodos publicando simultáneamente sobre:

```text
/joint_states
```

No es necesario cerrar RViz2.

En otra terminal:

```bash
cd "$HOME/grupo2_kuka"
source entorno.sh
ros2 run grupo02_robot_kinematics ik_node
```

Debe aparecer algo similar a:

```text
IK Node listo. Enviar objetivo cartesiano a /target...
IK robusta con multiples semillas habilitada.
```

---

# 13. Enviar un objetivo cartesiano

En otra terminal:

```bash
cd "$HOME/grupo2_kuka"
source entorno.sh
```

Enviar:

```bash
ros2 topic pub --once /target geometry_msgs/msg/Point "{x: 0.400, y: 0.200, z: 0.700}"
```

El nodo IK debe recibir aproximadamente:

```text
x = 0.400
y = 0.200
z = 0.700
```

y buscar una configuración articular:

```text
q* = [q1, q2, q3, q4, q5, q6]
```

---

# 14. Criterio de convergencia

La solución de IK se acepta cuando:

```text
||pd - p(q*)|| < 0.001 m
```

Es decir, cuando el error cartesiano es menor a:

```text
1 mm
```

El algoritmo utiliza:

- Jacobiano posicional numérico.
- Diferencias finitas.
- Pseudoinversa amortiguada.
- Límites articulares.
- Límite de incremento.
- Seis configuraciones iniciales.
- Máximo de 300 iteraciones por semilla.

---

# 15. Ejemplo de IK

Objetivo:

```text
pd = [0.400, 0.200, 0.700]
```

Una solución obtenida durante las pruebas fue aproximadamente:

```text
q* =
[2.8035,
 -1.5730,
 -1.5713,
 -1.4685,
 -0.8199,
 -2.7790]
```

Posición alcanzada:

```text
[0.3996, 0.1999, 0.7000]
```

Error:

```text
0.00041 m
```

Por tanto:

```text
0.00041 m < 0.001 m
```

y la solución converge correctamente.

---

# 16. Validación IK → FK

La solución encontrada por IK se publica mediante:

```text
/joint_states
```

El nodo FK puede utilizar nuevamente esa configuración.

La validación es:

```text
posición deseada
      ↓
      IK
      ↓
      q*
      ↓
      FK
      ↓
posición calculada
```

Se verifica:

```text
p(q*) ≈ pd
```

---

# 17. Flujo completo de ROS 2

```text
/target
   ↓
ik_node
   ↓
q*
   ↓
/joint_states
   ↓
robot_state_publisher
   ↓
TF
   ↓
RViz2
```

El nodo FK también recibe:

```text
/joint_states
```

y calcula la posición mediante el modelo DH implementado.

---

# 18. Prueba completa recomendada

## Terminal 1 - RViz2

```bash
cd "$HOME/grupo2_kuka"
source entorno.sh
ros2 launch grupo02_kuka_kr7_bringup display.launch.py
```

## Terminal 2 - FK

```bash
cd "$HOME/grupo2_kuka"
source entorno.sh
ros2 run grupo02_robot_kinematics fk_node
```

Mover los joints desde Joint State Publisher GUI.

## Terminal 3 - TF

```bash
cd "$HOME/grupo2_kuka"
source entorno.sh
timeout 3 ros2 run tf2_ros tf2_echo base_link tool0
```

Comparar FK con TF.

Cerrar Joint State Publisher GUI.

## Terminal 4 - IK

```bash
cd "$HOME/grupo2_kuka"
source entorno.sh
ros2 run grupo02_robot_kinematics ik_node
```

## Terminal 5 - Objetivo

```bash
cd "$HOME/grupo2_kuka"
source entorno.sh
ros2 topic pub --once /target geometry_msgs/msg/Point "{x: 0.400, y: 0.200, z: 0.700}"
```

Comprobar:

- Convergencia.
- Vector `q*`.
- Número de iteraciones.
- Error final.
- Movimiento del robot en RViz2.

Finalmente:

```bash
timeout 3 ros2 run tf2_ros tf2_echo base_link tool0
```

La transformación debe coincidir aproximadamente con el objetivo enviado.

---

# 19. Pruebas realizadas

Se realizaron tres configuraciones de FK y tres objetivos diferentes de IK.

Las pruebas mostraron errores de posición inferiores a 1 mm.

Para la validación de FK se compararon los resultados del modelo DH con las transformaciones TF publicadas por ROS 2.

Para IK, cada solución `q*` se verificó nuevamente mediante FK.

---

# 20. Problemas frecuentes

## `Package 'grupo02_kuka_kr7_bringup' not found`

Ejecutar:

```bash
cd "$HOME/grupo2_kuka"
source install/setup.bash
source entorno.sh
```

Si continúa:

```bash
colcon build --symlink-install
source install/setup.bash
source entorno.sh
```

---

## `Package 'kuka_agilus_support' not found`

Comprobar:

```bash
ros2 pkg list | grep kuka_agilus_support
```

Si no existe:

```bash
git clone https://github.com/kroshu/kuka_robot_descriptions.git src/kuka_robot_descriptions
git -C src/kuka_robot_descriptions checkout f0202b2
colcon build --symlink-install
```

---

## Falta `hardware_interface`

Comprobar:

```bash
ros2 pkg list | grep hardware_interface
```

Si no aparece:

```bash
sudo apt update
sudo apt install ros-jazzy-ros2-control
```

Luego:

```bash
colcon build --symlink-install
```

---

## El robot aparece incorrectamente en RViz2

Comprobar:

```bash
ros2 pkg list | grep kuka_agilus_support
```

y recompilar:

```bash
cd "$HOME/grupo2_kuka"
colcon build --symlink-install
source install/setup.bash
source entorno.sh
```

---

# 21. Estructura principal del proyecto

```text
grupo2_kuka/
├── README.md
├── entorno.sh
├── instalar.sh
├── dependencies.repos
├── docs/
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
```

---

# Repositorio

GitHub:

```text
ManuelZapanaGutierrez/grupo2_kuka
```

Rama principal:

```text
main
```
