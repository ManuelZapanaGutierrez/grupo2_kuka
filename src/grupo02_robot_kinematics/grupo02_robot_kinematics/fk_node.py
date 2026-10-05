#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class FKNode(Node):

    def __init__(self):

        super().__init__('fk_node')

        # ==========================================================
        # SUSCRIPCION A JOINT STATES
        # ==========================================================

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
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
        # ULTIMA CONFIGURACION PROCESADA
        #
        # Evita imprimir continuamente la misma configuracion.
        # ==========================================================

        self.last_q = None

        self.get_logger().info(
            'FK Node inicializado. Esperando /joint_states...'
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
    # CONVERSION DE ROTACION A CUATERNION
    # ==============================================================

    @staticmethod
    def rot_to_quaternion(R):

        tr = (
            R[0, 0]
            + R[1, 1]
            + R[2, 2]
        )

        if tr > 0:

            S = math.sqrt(tr + 1.0) * 2.0

            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S

        elif (
            R[0, 0] > R[1, 1]
            and R[0, 0] > R[2, 2]
        ):

            S = math.sqrt(
                1.0
                + R[0, 0]
                - R[1, 1]
                - R[2, 2]
            ) * 2.0

            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S

        elif R[1, 1] > R[2, 2]:

            S = math.sqrt(
                1.0
                + R[1, 1]
                - R[0, 0]
                - R[2, 2]
            ) * 2.0

            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S

        else:

            S = math.sqrt(
                1.0
                + R[2, 2]
                - R[0, 0]
                - R[1, 1]
            ) * 2.0

            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

        return qx, qy, qz, qw

    # ==============================================================
    # CINEMATICA DIRECTA
    #
    # MODELO DH DEL KUKA KR 7 R900-3
    #
    # La tabla fue obtenida de forma consistente con los frames
    # del URDF/Xacro del robot.
    #
    #                 theta        d          a          alpha
    #
    # A1              q1          -0.342     0.050       +pi/2
    # A2              q2           0.09305   0.410        0
    # A3              q3-pi/2     -0.09305   0.045       +pi/2
    # A4              q4          -0.440     0           -pi/2
    # A5              q5           0         0           +pi/2
    # A6              q6+pi       -0.077     0            pi
    #
    # Existe una transformacion fija entre base_link y el primer
    # frame DH:
    #
    # R = diag(1,-1,-1)
    #
    # ==============================================================

    def compute_fk(self, q):

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
        ])

        # ----------------------------------------------------------
        # Parametros DH
        # ----------------------------------------------------------

        dh_params = [

            # Joint 1
            (
                q[0],
                -0.34200,
                0.05000,
                math.pi / 2
            ),

            # Joint 2
            (
                q[1],
                0.09305,
                0.41000,
                0.0
            ),

            # Joint 3
            (
                q[2] - math.pi / 2,
                -0.09305,
                0.04500,
                math.pi / 2
            ),

            # Joint 4
            (
                q[3],
                -0.44000,
                0.00000,
                -math.pi / 2
            ),

            # Joint 5
            (
                q[4],
                0.00000,
                0.00000,
                math.pi / 2
            ),

            # Joint 6
            (
                q[5] + math.pi,
                -0.07700,
                0.00000,
                math.pi
            )
        ]

        # ----------------------------------------------------------
        # Multiplicacion:
        #
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

        return T

    # ==============================================================
    # CALLBACK DE JOINT STATES
    # ==============================================================

    def joint_state_callback(self, msg):

        # ----------------------------------------------------------
        # Diccionario:
        #
        # nombre del joint -> posicion
        # ----------------------------------------------------------

        name_to_pos = dict(
            zip(
                msg.name,
                msg.position
            )
        )

        # ----------------------------------------------------------
        # Verificar que existan los seis joints
        # ----------------------------------------------------------

        for joint_name in self.joint_names:

            if joint_name not in name_to_pos:

                return

        # ----------------------------------------------------------
        # Obtener q en orden
        # ----------------------------------------------------------

        current_q = np.array([
            name_to_pos[joint_name]
            for joint_name in self.joint_names
        ], dtype=float)

        # ==========================================================
        # EVITAR REPETICION
        # ==========================================================

        if self.last_q is not None:

            difference = np.max(
                np.abs(
                    current_q - self.last_q
                )
            )

            if difference < 1e-5:

                return

        self.last_q = current_q.copy()

        # ==========================================================
        # CALCULAR FK
        # ==========================================================

        T = self.compute_fk(
            current_q
        )

        # ==========================================================
        # POSICION
        # ==========================================================

        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]

        # ==========================================================
        # ORIENTACION
        # ==========================================================

        R = T[0:3, 0:3]

        qx, qy, qz, qw = (
            self.rot_to_quaternion(R)
        )

        # ==========================================================
        # MOSTRAR RESULTADOS
        # ==========================================================

        self.get_logger().info(

            f'\n--- CINEMÁTICA DIRECTA ---'

            f'\nq: '
            f'{[round(v, 4) for v in current_q]}'

            f'\nPosición [m]: '
            f'x={x:.4f}, '
            f'y={y:.4f}, '
            f'z={z:.4f}'

            f'\nOrientación (quat): '
            f'[x={qx:.4f}, '
            f'y={qy:.4f}, '
            f'z={qz:.4f}, '
            f'w={qw:.4f}]'
        )


# ==============================================================
# MAIN
# ==============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = FKNode()

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