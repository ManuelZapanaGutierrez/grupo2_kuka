# Grupo 02 - Cinemática KUKA KR 7 R900-3

Proyecto de Robótica - IMT-342

Implementación en ROS 2 Jazzy de la cinemática directa (FK) y cinemática inversa (IK) para el robot industrial KUKA KR 7 R900-3.

## Integrantes

- Adrian Pacheco Barrios
- Manuel Joel Zapana Gutierrez

## Descripción

El proyecto permite visualizar el robot KUKA KR 7 R900-3 en RViz2 y realizar:

- Cinemática directa (Forward Kinematics, FK).
- Cinemática inversa (Inverse Kinematics, IK).
- Publicación y recepción de estados articulares mediante `/joint_states`.
- Verificación de las soluciones de cinemática inversa mediante cinemática directa.
- Pruebas con diferentes objetivos cartesianos.

La implementación está desarrollada como paquetes ROS 2 en Python.

---

# Estructura del workspace y ejecución en ROS 2

El proyecto se organizó en el workspace:

```text
$HOME/grupo2_kuka
```

El paquete de cinemática se encuentra en:

```text
src/grupo02_robot_kinematics/
```

dentro del módulo:

```text
grupo02_robot_kinematics/
```

Los archivos principales son:

```text
fk_node.py
ik_node.py
__init__.py
```

---

# Obtención e instalación desde cero

Para reproducir el proyecto en una computadora nueva se requiere Ubuntu 24.04, ROS 2 Jazzy y Git.

Si alguno de estos componentes no está instalado, debe instalarse antes de continuar.

El código del proyecto se obtiene directamente desde el repositorio de GitHub.

---

## 1. Comprobación inicial

Primero se verifica la versión de Ubuntu:

```bash
lsb_release -a
```

La computadora debe utilizar Ubuntu 24.04.

A continuación se comprueba la distribución de ROS 2:

```bash
echo $ROS_DISTRO
```

Debe corresponder a ROS 2 Jazzy.

Finalmente se comprueba Git:

```bash
git --version
```

Si ROS 2 Jazzy o Git no están instalados, deben instalarse antes de continuar.

---

## 2. Comprobación de dependencias KUKA

Antes de descargar componentes adicionales se comprueba si el sistema ya dispone de la interfaz de hardware utilizada por la descripción del robot:

```bash
ros2 pkg list | grep hardware_interface
```

Si no aparece `hardware_interface`, se instala ROS 2 Control:

```bash
sudo apt update
```

```bash
sudo apt install ros-jazzy-ros2-control
```

Después se puede comprobar nuevamente:

```bash
ros2 pkg list | grep hardware_interface
```

También se comprueba si la descripción KUKA requerida ya está disponible:

```bash
ros2 pkg list | grep kuka_agilus_support
```

Si aparece `kuka_agilus_support`, no es necesario descargar otra copia de la descripción KUKA.

Si no aparece, se debe descargar la versión utilizada en este proyecto.

---

## 3. Descarga del proyecto

La carpeta del proyecto se crea a partir del repositorio.

No es necesario copiar manualmente los archivos.

Primero:

```bash
cd "$HOME"
```

Después:

```bash
git clone https://github.com/ManuelZapanaGutierrez/grupo2_kuka.git
```

Luego:

```bash
cd "$HOME/grupo2_kuka"
```

La estructura puede comprobarse con:

```bash
ls
```

y los archivos principales de cinemática con:

```bash
ls src/grupo02_robot_kinematics/grupo02_robot_kinematics
```

Deben encontrarse, entre otros:

```text
fk_node.py
ik_node.py
__init__.py
```

---

## 4. Descarga de la descripción KUKA

Si `kuka_agilus_support` no estaba disponible en el sistema, se descarga el repositorio de descripciones KUKA dentro del workspace:

```bash
git clone https://github.com/kroshu/kuka_robot_descriptions.git src/kuka_robot_descriptions
```

A continuación se selecciona exactamente la versión utilizada por el proyecto:

```bash
git -C src/kuka_robot_descriptions checkout f0202b2
```

El mensaje de Git indicando un estado `detached HEAD` no representa un error.

Lo importante es que la terminal indique que el repositorio quedó en el commit:

```text
f0202b2
```

---

## 5. Compilación del workspace

Con las dependencias preparadas se compila todo el workspace:

```bash
cd "$HOME/grupo2_kuka"
```

```bash
colcon build --symlink-install
```

La compilación debe finalizar sin paquetes marcados como:

```text
Failed
```

Una vez terminada la compilación, se carga el workspace:

```bash
source install/setup.bash
```

y se carga el entorno del proyecto:

```bash
source entorno.sh
```

Para comprobar que los paquetes del proyecto están disponibles:

```bash
ros2 pkg list | grep grupo02
```

Deben aparecer:

```text
grupo02_kuka_kr7_bringup
grupo02_robot_kinematics
```

También se comprueba la descripción KUKA:

```bash
ros2 pkg list | grep kuka_agilus_support
```

Si aparecen los paquetes anteriores y `kuka_agilus_support`, el workspace está preparado para ejecutar el proyecto.

---

## 6. Carga del entorno en nuevas terminales

Cada nueva terminal utilizada para trabajar con el proyecto debe cargar nuevamente el entorno:

```bash
cd "$HOME/grupo2_kuka"
```

```bash
source entorno.sh
```

---

# Secuencia completa de arranque

Una vez instalado y compilado el workspace, se puede ejecutar la práctica desde el directorio del proyecto.

Primero se abre el modelo del robot en RViz2:

```bash
cd "$HOME/grupo2_kuka"
```

```bash
source entorno.sh
```

```bash
ros2 launch grupo02_kuka_kr7_bringup display.launch.py
```

El lanzamiento inicia la visualización del robot y permite utilizar el Joint State Publisher GUI para generar configuraciones articulares.

Antes de comenzar una prueba de IK se debe cerrar el GUI, ya que tanto el GUI como `ik_node.py` pueden publicar sobre:

```text
/joint_states
```

---

## Comprobación inicial del sistema ROS 2

Como comprobación inicial se recomienda ejecutar:

```bash
ros2 node list
```

```bash
ros2 topic list
```

```bash
ros2 topic echo /joint_states --once
```

```bash
ros2 topic echo /robot_description --once
```

```bash
ros2 run tf2_ros tf2_echo base_link tool0
```

---

## Compilación únicamente del paquete de cinemática

Para compilar el paquete se utilizó:

```bash
cd "$HOME/grupo2_kuka"
```

```bash
source entorno.sh
```

```bash
colcon build --packages-select \
grupo02_robot_kinematics --symlink-install
```

---

## Ejecución de la cinemática directa

Para ejecutar FK:

```bash
cd "$HOME/grupo2_kuka"
```

```bash
source entorno.sh
```

```bash
ros2 run grupo02_robot_kinematics fk_node
```

---

## Ejecución de la cinemática inversa

Para ejecutar IK:

```bash
cd "$HOME/grupo2_kuka"
```

```bash
source entorno.sh
```

```bash
ros2 run grupo02_robot_kinematics ik_node
```

---

## Envío de un objetivo cartesiano

Un objetivo cartesiano se envía mediante:

```bash
ros2 topic pub /target geometry_msgs/msg/Point \
"{x: 0.400, y: 0.200, z: 0.700}" --once
```

Durante las pruebas de FK se utilizó el Joint State Publisher GUI para generar configuraciones articulares.

Para las pruebas de IK se cerró dicho GUI antes de publicar la solución desde `ik_node.py`, evitando dos publishers simultáneos sobre:

```text
/joint_states
```

El flujo de validación fue:

```text
/target
    ↓
ik_node
    ↓
/joint_states
    ↓
robot_state_publisher
    ↓
RViz2
```

---

# Procedimiento reproducible de las pruebas

## Prueba de FK

Manteniendo abierto RViz2 y el Joint State Publisher GUI, se mueve una o varias articulaciones hasta obtener una configuración de prueba.

En otra terminal se ejecuta:

```bash
cd "$HOME/grupo2_kuka"
```

```bash
source entorno.sh
```

```bash
ros2 run grupo02_robot_kinematics fk_node
```

La posición calculada por FK se compara con la transformación de ROS 2:

```bash
ros2 run tf2_ros tf2_echo base_link tool0
```

---

## Prueba de IK

Se cierra el Joint State Publisher GUI, se ejecuta `ik_node.py` y se publica el objetivo mediante `/target`.

Por ejemplo:

```bash
cd "$HOME/grupo2_kuka"
```

```bash
source entorno.sh
```

```bash
ros2 run grupo02_robot_kinematics ik_node
```

En otra terminal:

```bash
cd "$HOME/grupo2_kuka"
```

```bash
source entorno.sh
```

```bash
ros2 topic pub /target geometry_msgs/msg/Point \
"{x: 0.400, y: 0.200, z: 0.700}" --once
```

La solución se publica en:

```text
/joint_states
```

y se visualiza mediante `robot_state_publisher` y RViz2.

Para cada objetivo se verifica posteriormente la posición alcanzada mediante FK y se comprueba que el error sea inferior a la tolerancia de:

```text
10^-3 m
```

---

# Validación y resultados

La validación del modelo se realizó mediante dos procedimientos complementarios.

Para la cinemática directa:

```text
q → FK → p_FK ≈ p_TF
```

donde `p_FK` corresponde a la posición calculada mediante el modelo DH implementado y `p_TF` a la transformación obtenida a partir del modelo URDF/Xacro y del estado articular publicado en ROS 2.

Para la cinemática inversa:

```text
p_d → IK → q* → FK → p(q*) ≈ p_d
```

La primera comparación permite comprobar la consistencia entre el modelo matemático desarrollado y el modelo utilizado por ROS 2.

La segunda permite verificar que la solución articular calculada por la cinemática inversa realmente reproduce el objetivo cartesiano solicitado.

---

# Integración y repositorio

El repositorio utilizado para el proyecto es:

```text
GitHub: ManuelZapanaGutierrez/grupo2_kuka
```

La rama de trabajo es:

```text
main
```

El estado final del proyecto quedó sincronizado con el repositorio antes de la entrega.

El repositorio contiene el paquete de cinemática, los archivos necesarios del workspace y el entorno utilizado para ejecutar la práctica.

El repositorio incluye además los archivos:

```text
entorno.sh
instalar.sh
dependencies.repos
```

El archivo `README.md` contiene las instrucciones necesarias para clonar, preparar dependencias, compilar y ejecutar el proyecto, así como los comandos para ejecutar FK, IK y las pruebas de validación.

El script `instalar.sh` permite automatizar parte del proceso de preparación inicial del workspace, mientras que el README conserva también el procedimiento manual para facilitar el diagnóstico y la reproducción en otra computadora.
