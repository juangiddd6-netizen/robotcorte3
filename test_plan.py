import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState

class MoveItTestClient(Node):
    def __init__(self):
        super().__init__('moveit_test_client')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def send_goal(self):
        self.get_logger().info('Esperando al servidor de acciones move_group...')
        self._action_client.wait_for_server()
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        # 1. Definir un ESTADO INICIAL limpio con is_diff = True
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        start_state = RobotState()
        start_state.is_diff = True  # Permite fusionar con el estado base sin poner en cero la pinza
        start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        start_state.joint_state.name = joint_names
        start_state.joint_state.position = [0.0, -1.0, 1.0, -1.0, 0.0, 0.0]
        goal_msg.request.start_state = start_state
        # 2. Definir la POSICIÓN META
        target_positions = [0.0, -1.57, 1.57, -1.57, 0.0, 0.0]
        
        constraints = Constraints()
        for name, pos in zip(joint_names, target_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = True

        self.get_logger().info('Enviando solicitud con estado inicial y meta limpios...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Petición rechazada por MoveIt.')
            rclpy.shutdown()
            return
            
        self.get_logger().info('Petición aceptada. Calculando ruta...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val
        
        if error_code == 1:
            self.get_logger().info('================================================')
            self.get_logger().info('¡ÉXITO TOTAL! MoveIt planificó la trayectoria correctamente.')
            self.get_logger().info('================================================')
        else:
            self.get_logger().error(f'Falló la planificación. Código de error de MoveIt: {error_code}')
            
        rclpy.shutdown()

def main():
    rclpy.init()
    node = MoveItTestClient()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
