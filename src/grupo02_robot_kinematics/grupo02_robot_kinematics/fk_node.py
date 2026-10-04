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

        # Nombres reales de los joints del KUKA
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        self.get_logger().info(
            'FK Node inicializado. Esperando /joint_states...'
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
    # CONVERSION DE ROTACION A CUATERNION
    # ==============================================================

    def rot_to_quaternion(self, R):

        tr = (
            R[0, 0]
            + R[1, 1]
            + R[2, 2]
        )

        if tr > 0:

            S = math.sqrt(tr + 1.0) * 2

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
            ) * 2

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
            ) * 2

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
            ) * 2

            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

        return qx, qy, qz, qw

    # ==============================================================
    # CINEMATICA DIRECTA
    # ==============================================================

    def compute_fk(self, q):

        # Parámetros DH utilizados también por la IK.
        #
        # Distancias en metros
        # Ángulos en radianes

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

        return T

    # ==============================================================
    # CALLBACK DE JOINT STATES
    # ==============================================================

    def joint_state_callback(self, msg):

        # ----------------------------------------------------------
        # Crear diccionario:
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
        # Buscar los seis joints reales
        # ----------------------------------------------------------

        current_q = []

        for joint_name in self.joint_names:

            if joint_name not in name_to_pos:

                return

            current_q.append(
                name_to_pos[joint_name]
            )

        # ----------------------------------------------------------
        # CINEMATICA DIRECTA
        # ----------------------------------------------------------

        T06 = self.compute_fk(
            current_q
        )

        # ----------------------------------------------------------
        # POSICION
        # ----------------------------------------------------------

        pos = T06[0:3, 3]

        # ----------------------------------------------------------
        # ORIENTACION
        # ----------------------------------------------------------

        R = T06[0:3, 0:3]

        qx, qy, qz, qw = (
            self.rot_to_quaternion(R)
        )

        # ----------------------------------------------------------
        # MOSTRAR RESULTADO
        # ----------------------------------------------------------

        self.get_logger().info(

            f'\n--- CINEMÁTICA DIRECTA ---'

            f'\nq: '
            f'{[round(val, 4) for val in current_q]}'

            f'\nPosición [m]: '
            f'x={pos[0]:.4f}, '
            f'y={pos[1]:.4f}, '
            f'z={pos[2]:.4f}'

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

    rclpy.init(args=args)

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