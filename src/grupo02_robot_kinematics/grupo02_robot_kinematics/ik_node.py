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
        ])

        self.limits_upper = np.array([
             3.2288591125,
             0.6981317,
             2.7925268,
             3.4906585,
             2.0943951,
             6.108652375
        ])

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
        ])

        # Indica si ya tenemos una solución publicada
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
    # MATRIZ DH
    # ==============================================================

    @staticmethod
    def dh_matrix(theta, d, a, alpha):

        ct = math.cos(theta)
        st = math.sin(theta)

        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,       sa,       ca,      d],
            [0,        0,        0,      1]
        ], dtype=float)

    # ==============================================================
    # CINEMATICA DIRECTA
    # ==============================================================

    def forward_kinematics_pos(self, q):

        dh_params = [

            (
                q[0],
                0.400,
                0.025,
                -math.pi / 2
            ),

            (
                q[1] - math.pi / 2,
                0.000,
                0.455,
                0.0
            ),

            (
                q[2],
                0.000,
                0.035,
                math.pi / 2
            ),

            (
                q[3],
                0.420,
                0.000,
                -math.pi / 2
            ),

            (
                q[4],
                0.000,
                0.000,
                math.pi / 2
            ),

            (
                q[5],
                0.080,
                0.000,
                0.0
            )
        ]

        T = np.eye(4)

        for theta, d, a, alpha in dh_params:

            T = T @ self.dh_matrix(
                theta,
                d,
                a,
                alpha
            )

        return T[0:3, 3]

    # ==============================================================
    # JACOBIANO POSICIONAL
    # ==============================================================

    def compute_positional_jacobian(
        self,
        q,
        delta=1e-6
    ):

        J = np.zeros((3, 6))

        p0 = self.forward_kinematics_pos(q)

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

        # Factor de amortiguamiento.
        # Ayuda cuando el Jacobiano está cerca de una singularidad.
        damping = 0.02

        # Tamaño máximo de un paso articular.
        max_step = 0.20

        for i in range(max_iter):

            # ------------------------------------------------------
            # FK
            # ------------------------------------------------------

            p_current = self.forward_kinematics_pos(q)

            # ------------------------------------------------------
            # ERROR
            # ------------------------------------------------------

            error = target_pos - p_current

            err_norm = np.linalg.norm(error)

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

            J = self.compute_positional_jacobian(q)

            # ------------------------------------------------------
            # PSEUDOINVERSA AMORTIGUADA
            #
            # dq = J^T (J J^T + λ²I)^-1 e
            # ------------------------------------------------------

            identity = np.eye(3)

            try:

                J_damped = (
                    J.T
                    @ np.linalg.inv(
                        J @ J.T
                        + (damping ** 2) * identity
                    )
                )

            except np.linalg.LinAlgError:

                J_damped = np.linalg.pinv(J)

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
            # ACTUALIZAR
            # ------------------------------------------------------

            q = q + dq

            # ------------------------------------------------------
            # RESPETAR LIMITES
            # ------------------------------------------------------

            q = np.clip(
                q,
                self.limits_lower,
                self.limits_upper
            )

        # ==========================================================
        # RESULTADO FINAL SI NO CONVERGIÓ
        # ==========================================================

        p_final = self.forward_kinematics_pos(q)

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
            ])
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
            ])
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
            ])
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
            ])
        )

        # ----------------------------------------------------------
        # 6. SOLUCION CONOCIDA PARA EL OBJETIVO 0.55,0,0.50
        #
        # Sirve además como una semilla válida para esa región
        # del espacio de trabajo.
        # ----------------------------------------------------------

        seeds.append(
            np.array([
                3.103,
                -0.425,
                -0.340,
                -1.279,
                0.283,
                -2.779
            ])
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

            valid_seeds.append(seed)

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

            q_sol, iterations, error, converged = (
                self.solve_ik_from_seed(
                    target_pos,
                    seed
                )
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
                best_iterations = iterations
                best_converged = converged

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

        # Mientras no tengamos una solución IK publicada,
        # usamos la posición real del robot como semilla.

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
                ])

    # ==============================================================
    # CALLBACK DEL TARGET
    # ==============================================================

    def target_callback(self, msg):

        target_pos = np.array([
            msg.x,
            msg.y,
            msg.z
        ])

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

            f'\n================ RESULTADO CINEMÁTICA INVERSA ================'

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
        # PUBLICAR SOLO SI CONVERGIÓ
        # ==========================================================

        if converged:

            self.q_current = q_sol.copy()

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

            joint_msg.name = self.joint_names

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