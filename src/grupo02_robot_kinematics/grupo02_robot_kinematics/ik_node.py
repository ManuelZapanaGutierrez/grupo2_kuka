#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState


class IKNode(Node):

    def __init__(self):

        super().__init__('ik_node')

        # ==========================================================
        # ROS
        # ==========================================================

        self.target_sub = self.create_subscription(
            Point,
            '/target',
            self.target_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # ==========================================================
        # NOMBRES DE LOS JOINTS
        # ==========================================================

        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        # ==========================================================
        # LIMITES REALES DEL KUKA KR 7 R900-3
        #
        # Obtenidos del kr7_r900_3_macro.xacro
        # ==========================================================

        self.limits_lower = np.array([
            -3.2288591125,
            -3.316125575,
            -2.0071286375,
            -3.4906585,
            -2.0943951,
            -6.108652375
        ], dtype=float)

        self.limits_upper = np.array([
             3.2288591125,
             0.6981317,
             2.7925268,
             3.4906585,
             2.0943951,
             6.108652375
        ], dtype=float)

        # ==========================================================
        # CONFIGURACION ACTUAL DEL ROBOT
        # ==========================================================

        self.q_current = np.array([
            0.0,
            -math.pi / 4,
            math.pi / 2,
            0.0,
            math.pi / 4,
            0.0
        ], dtype=float)

        # Indica si ya tenemos una solucion IK publicada
        self.publish_ik_solution = False

        # ==========================================================
        # TIMER
        # ==========================================================

        self.timer = self.create_timer(
            0.05,
            self.timer_publish_joints
        )

        self.get_logger().info(
            'IK Node listo. Enviar objetivo cartesiano a /target...'
        )

        self.get_logger().info(
            'IK robusta con multiples semillas habilitada.'
        )

    # ==============================================================
    # MATRIZ DH ESTANDAR
    # ==============================================================

    @staticmethod
    def dh_matrix(theta, d, a, alpha):

        ct = math.cos(theta)
        st = math.sin(theta)

        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [
                ct,
                -st * ca,
                st * sa,
                a * ct
            ],
            [
                st,
                ct * ca,
                -ct * sa,
                a * st
            ],
            [
                0.0,
                sa,
                ca,
                d
            ],
            [
                0.0,
                0.0,
                0.0,
                1.0
            ]
        ], dtype=float)

    # ==============================================================
    # CINEMATICA DIRECTA POSICIONAL
    #
    # ESTA ES LA MISMA FK VALIDADA EN fk_node.py
    #
    # La IK solamente necesita la posicion del efector final.
    # ==============================================================

    def forward_kinematics_pos(self, q):

        # ----------------------------------------------------------
        # Transformacion fija:
        #
        # base_link -> frame DH inicial
        # ----------------------------------------------------------

        T_base = np.array([
            [1.0,  0.0,  0.0, 0.0],
            [0.0, -1.0,  0.0, 0.0],
            [0.0,  0.0, -1.0, 0.0],
            [0.0,  0.0,  0.0, 1.0]
        ], dtype=float)

        # ----------------------------------------------------------
        # Parametros DH
        #
        #             theta       d          a          alpha
        #
        # A1          q1         -0.34200   0.05000      +pi/2
        # A2          q2          0.09305   0.41000       0
        # A3          q3-pi/2    -0.09305   0.04500      +pi/2
        # A4          q4         -0.44000   0            -pi/2
        # A5          q5          0         0             pi/2
        # A6          q6+pi      -0.07700   0             pi
        # ----------------------------------------------------------

        dh_params = [

            (
                q[0],
                -0.34200,
                0.05000,
                math.pi / 2
            ),

            (
                q[1],
                0.09305,
                0.41000,
                0.0
            ),

            (
                q[2] - math.pi / 2,
                -0.09305,
                0.04500,
                math.pi / 2
            ),

            (
                q[3],
                -0.44000,
                0.00000,
                -math.pi / 2
            ),

            (
                q[4],
                0.00000,
                0.00000,
                math.pi / 2
            ),

            (
                q[5] + math.pi,
                -0.07700,
                0.00000,
                math.pi
            )
        ]

        # ----------------------------------------------------------
        # T = T_base * A1 * A2 * A3 * A4 * A5 * A6
        # ----------------------------------------------------------

        T = T_base.copy()

        for theta, d, a, alpha in dh_params:

            T = T @ self.dh_matrix(
                theta,
                d,
                a,
                alpha
            )

        return T[0:3, 3]

    # ==============================================================
    # JACOBIANO POSICIONAL NUMERICO
    # ==============================================================

    def compute_positional_jacobian(
        self,
        q,
        delta=1e-6
    ):

        J = np.zeros((3, 6), dtype=float)

        # Posicion actual
        p0 = self.forward_kinematics_pos(q)

        # Derivada numerica para cada articulacion
        for i in range(6):

            q_step = q.copy()

            q_step[i] += delta

            p_step = self.forward_kinematics_pos(
                q_step
            )

            J[:, i] = (
                p_step - p0
            ) / delta

        return J

    # ==============================================================
    # SOLUCION IK DESDE UNA SEMILLA
    # ==============================================================

    def solve_ik_from_seed(
        self,
        target_pos,
        q_seed,
        max_iter=300,
        tolerance=1e-3
    ):

        q = q_seed.copy()

        # ----------------------------------------------------------
        # Factor de amortiguamiento.
        #
        # Ayuda cuando el Jacobiano esta cerca de una singularidad.
        # ----------------------------------------------------------

        damping = 0.02

        # ----------------------------------------------------------
        # Tamaño maximo de un paso articular.
        # ----------------------------------------------------------

        max_step = 0.20

        for i in range(max_iter):

            # ------------------------------------------------------
            # FK
            # ------------------------------------------------------

            p_current = self.forward_kinematics_pos(
                q
            )

            # ------------------------------------------------------
            # ERROR CARTESIANO
            # ------------------------------------------------------

            error = (
                target_pos
                - p_current
            )

            err_norm = np.linalg.norm(
                error
            )

            # ------------------------------------------------------
            # CONVERGENCIA
            # ------------------------------------------------------

            if err_norm < tolerance:

                return (
                    q,
                    i + 1,
                    err_norm,
                    True
                )

            # ------------------------------------------------------
            # JACOBIANO
            # ------------------------------------------------------

            J = self.compute_positional_jacobian(
                q
            )

            # ------------------------------------------------------
            # PSEUDOINVERSA AMORTIGUADA
            #
            # dq =
            # J^T (J J^T + lambda^2 I)^-1 e
            # ------------------------------------------------------

            identity = np.eye(3)

            try:

                J_damped = (
                    J.T
                    @ np.linalg.inv(
                        J @ J.T
                        + (damping ** 2)
                        * identity
                    )
                )

            except np.linalg.LinAlgError:

                J_damped = np.linalg.pinv(
                    J
                )

            # ------------------------------------------------------
            # INCREMENTO ARTICULAR
            # ------------------------------------------------------

            dq = 0.6 * (
                J_damped @ error
            )

            # ------------------------------------------------------
            # LIMITAR EL TAMAÑO DEL PASO
            # ------------------------------------------------------

            max_dq = np.max(
                np.abs(dq)
            )

            if max_dq > max_step:

                dq = (
                    dq
                    * max_step
                    / max_dq
                )

            # ------------------------------------------------------
            # ACTUALIZAR CONFIGURACION
            # ------------------------------------------------------

            q = q + dq

            # ------------------------------------------------------
            # RESPETAR LIMITES ARTICULARES
            # ------------------------------------------------------

            q = np.clip(
                q,
                self.limits_lower,
                self.limits_upper
            )

        # ==========================================================
        # RESULTADO FINAL SI NO CONVERGIO
        # ==========================================================

        p_final = self.forward_kinematics_pos(
            q
        )

        err_final = np.linalg.norm(
            target_pos - p_final
        )

        return (
            q,
            max_iter,
            err_final,
            err_final < tolerance
        )

    # ==============================================================
    # CONSTRUIR MULTIPLES SEMILLAS
    # ==============================================================

    def get_seeds(self):

        seeds = []

        # ----------------------------------------------------------
        # 1. POSICION ACTUAL DEL ROBOT
        # ----------------------------------------------------------

        seeds.append(
            self.q_current.copy()
        )

        # ----------------------------------------------------------
        # 2. CONFIGURACION INICIAL DEL PROYECTO
        # ----------------------------------------------------------

        seeds.append(
            np.array([
                0.0,
                -math.pi / 4,
                math.pi / 2,
                0.0,
                math.pi / 4,
                0.0
            ], dtype=float)
        )

        # ----------------------------------------------------------
        # 3. CONFIGURACION ALTERNATIVA
        # ----------------------------------------------------------

        seeds.append(
            np.array([
                math.pi / 2,
                -0.6,
                0.6,
                0.0,
                0.5,
                0.0
            ], dtype=float)
        )

        # ----------------------------------------------------------
        # 4. CONFIGURACION ALTERNATIVA OPUESTA
        # ----------------------------------------------------------

        seeds.append(
            np.array([
                -math.pi / 2,
                -0.6,
                0.6,
                0.0,
                0.5,
                0.0
            ], dtype=float)
        )

        # ----------------------------------------------------------
        # 5. CONFIGURACION ALTERNATIVA
        # ----------------------------------------------------------

        seeds.append(
            np.array([
                math.pi,
                -0.4,
                -0.4,
                -1.2,
                0.3,
                -2.8
            ], dtype=float)
        )

        # ----------------------------------------------------------
        # 6. SEMILLA ADICIONAL
        #
        # Se conserva como una configuracion inicial alternativa.
        # No se considera una solucion exacta del nuevo modelo.
        # ----------------------------------------------------------

        seeds.append(
            np.array([
                3.103,
                -0.425,
                -0.340,
                -1.279,
                0.283,
                -2.779
            ], dtype=float)
        )

        # ----------------------------------------------------------
        # LIMITAR TODAS LAS SEMILLAS
        # ----------------------------------------------------------

        valid_seeds = []

        for seed in seeds:

            seed = np.clip(
                seed,
                self.limits_lower,
                self.limits_upper
            )

            valid_seeds.append(
                seed
            )

        return valid_seeds

    # ==============================================================
    # SOLUCION IK MULTIPLE
    # ==============================================================

    def solve_ik(
        self,
        target_pos
    ):

        seeds = self.get_seeds()

        best_q = None
        best_error = float('inf')
        best_iterations = 0
        best_converged = False

        self.get_logger().info(
            f'Testando {len(seeds)} configuraciones iniciales...'
        )

        # ----------------------------------------------------------
        # PROBAR TODAS LAS SEMILLAS
        # ----------------------------------------------------------

        for index, seed in enumerate(seeds):

            (
                q_sol,
                iterations,
                error,
                converged
            ) = self.solve_ik_from_seed(
                target_pos,
                seed
            )

            self.get_logger().info(
                f'Semilla {index + 1}: '
                f'error={error:.5f} m, '
                f'iteraciones={iterations}, '
                f'convergencia={converged}'
            )

            # ------------------------------------------------------
            # GUARDAR LA MEJOR SOLUCION
            # ------------------------------------------------------

            if error < best_error:

                best_error = error

                best_q = q_sol.copy()

                best_iterations = (
                    iterations
                )

                best_converged = (
                    converged
                )

        return (
            best_q,
            best_iterations,
            best_error,
            best_converged
        )

    # ==============================================================
    # CALLBACK DE JOINT STATES
    # ==============================================================

    def joint_state_callback(self, msg):

        # Mientras no tengamos una solucion IK publicada,
        # usamos la posicion real del robot como semilla.

        if not self.publish_ik_solution:

            name_to_pos = dict(
                zip(
                    msg.name,
                    msg.position
                )
            )

            if all(
                joint in name_to_pos
                for joint in self.joint_names
            ):

                self.q_current = np.array([
                    name_to_pos[joint]
                    for joint in self.joint_names
                ], dtype=float)

    # ==============================================================
    # CALLBACK DEL TARGET
    # ==============================================================

    def target_callback(self, msg):

        target_pos = np.array([
            msg.x,
            msg.y,
            msg.z
        ], dtype=float)

        self.get_logger().info(
            f'\n>>> Objetivo recibido en /target: '
            f'x={msg.x:.3f}, '
            f'y={msg.y:.3f}, '
            f'z={msg.z:.3f}'
        )

        # ==========================================================
        # RESOLVER IK
        # ==========================================================

        (
            q_sol,
            iters,
            err_final,
            converged
        ) = self.solve_ik(
            target_pos
        )

        # ==========================================================
        # POSICION ALCANZADA
        # ==========================================================

        pos_calc = self.forward_kinematics_pos(
            q_sol
        )

        # ==========================================================
        # ESTADO
        # ==========================================================

        if converged:

            status_str = (
                'CONVERGIÓ EXITOSAMENTE'
            )

        else:

            status_str = (
                'NO CONVERGIÓ '
                '(mejor solución encontrada)'
            )

        # ==========================================================
        # RESULTADO
        # ==========================================================

        self.get_logger().info(

            f'\n================ RESULTADO '
            f'CINEMÁTICA INVERSA ================'

            f'\nEstado: {status_str}'

            f'\nIteraciones: {iters}'

            f'\nError final de posición: '
            f'{err_final:.5f} m'

            f'\nPosición alcanzada [x, y, z]: '
            f'{[round(v, 4) for v in pos_calc]}'

            f'\nVector articular solución q*: '
            f'{[round(rad, 4) for rad in q_sol]}'

            f'\n=============================================================='
        )

        # ==========================================================
        # PUBLICAR SOLO SI CONVERGIO
        # ==========================================================

        if converged:

            self.q_current = (
                q_sol.copy()
            )

            self.publish_ik_solution = True

        else:

            self.get_logger().warn(
                'La IK no alcanzó la tolerancia. '
                'No se publicará una nueva pose.'
            )

    # ==============================================================
    # PUBLICACION CONTINUA DE JOINT STATES
    # ==============================================================

    def timer_publish_joints(self):

        if self.publish_ik_solution:

            joint_msg = JointState()

            joint_msg.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            joint_msg.name = (
                self.joint_names
            )

            joint_msg.position = (
                self.q_current.tolist()
            )

            self.joint_pub.publish(
                joint_msg
            )


# ==============================================================
# MAIN
# ==============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = IKNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


# ==============================================================
# EJECUCION
# ==============================================================

if __name__ == '__main__':
    main()