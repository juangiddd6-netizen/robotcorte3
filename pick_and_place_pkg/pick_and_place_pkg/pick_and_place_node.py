#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        
        # Publicador para el brazo
        self.arm_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Publicador para la pinza
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, 
            '/gripper_controller/commands', 
            10
        )
        
        # Salida digital simulada
        self.digital_out_pub = self.create_publisher(Bool, '/digital_output_1', 10)
        
        # Entrada digital simulada
        self.digital_input_state = False
        self.create_subscription(Bool, '/digital_input_1', self.di_callback, 10)

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        self.get_logger().info('>>> NODO PICK AND PLACE INICIADO. ESPERANDO CONEXIONES... <<<')
        time.sleep(2.0)
        self.execute_routine()

    def di_callback(self, msg):
        self.digital_input_state = msg.data

    def move_arm(self, positions, duration_sec=3):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration_sec
        msg.points.append(point)
        self.arm_pub.publish(msg)
        time.sleep(duration_sec + 0.5)

    def set_gripper(self, value):
        msg = Float64MultiArray()
        msg.data = [float(value)]
        self.gripper_pub.publish(msg)
        time.sleep(1.5)

    def set_digital_output(self, state):
        msg = Bool()
        msg.data = state
        self.digital_out_pub.publish(msg)
        self.get_logger().info(f'[SALIDA DIGITAL] /digital_output_1 estado: {state}')

    def wait_for_digital_input(self):
        self.get_logger().info('[ENTRADA DIGITAL] Esperando señal en /digital_input_1 (True) para continuar...')
        timeout = 8.0
        start = time.time()
        while not self.digital_input_state and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.digital_input_state:
            self.get_logger().info('[ENTRADA DIGITAL] Señal recibida correctamente.')
        else:
            self.get_logger().info('[ENTRADA DIGITAL] Tiempo agotado. Continuando rutina por omisión.')

    def execute_routine(self):
        # PUNTO 1: Reposo (Home)
        self.get_logger().info('>>> PUNTO 1: Desplazamiento a Posición Inicial (Home) <<<')
        self.set_gripper(0.0)
        self.move_arm([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        self.set_digital_output(False)

        # PUNTO 2: Agarre (Pick)
        self.get_logger().info('>>> PUNTO 2: Desplazamiento a Posición de Agarre (Pick) <<<')
        self.move_arm([0.5, -1.0, 1.2, -1.77, -1.57, 0.0])
        self.wait_for_digital_input()
        
        self.get_logger().info('Ejecutando sujeción de objeto...')
        self.set_gripper(0.5)
        self.set_digital_output(True)

        # PUNTO 3: Descarga (Place)
        self.get_logger().info('>>> PUNTO 3: Desplazamiento a Posición de Destino (Place) <<<')
        self.move_arm([-0.5, -1.0, 1.2, -1.77, -1.57, 0.0])
        
        self.get_logger().info('Soltando objeto en punto final...')
        self.set_gripper(0.0)
        self.set_digital_output(False)

        # Retorno
        self.get_logger().info('>>> RUTINA COMPLETADA. Regresando a Home <<<')
        self.move_arm([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

