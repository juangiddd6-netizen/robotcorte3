import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
import time

class UR5PickAndPlace(Node):
    def __init__(self):
        super().__init__('ur5_pick_and_place_node')
        self._arm_client = ActionClient(self, MoveGroup, 'move_action')
        
    def wait_for_server(self):
        self.get_logger().info('Esperando al servidor /move_action de MoveIt...')
        self._arm_client.wait_for_server()
        self.get_logger().info('¡Servidor conectado exitosamente!')

    def move_arm_joints(self, target_positions, description=""):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        start_state = RobotState()
        start_state.is_diff = True
        start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        goal_msg.request.start_state = start_state

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
        goal_msg.planning_options.plan_only = False

        self.get_logger().info(f'Ejecutando paso: {description}...')
        future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Paso rechazado: {description}')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        self.get_logger().info(f'Paso completado: {description}')
        return True

    def execute_routine(self):
        self.wait_for_server()

        home_pose       = [0.0, -1.57, 1.57, -1.57, 0.0, 0.0]
        pick_approach   = [0.5, -1.2, 1.4, -1.77, -1.57, 0.0]
        pick_grasp      = [0.5, -1.0, 1.5, -2.07, -1.57, 0.0]
        place_approach  = [-0.5, -1.2, 1.4, -1.77, -1.57, 0.0]
        place_release   = [-0.5, -1.0, 1.5, -2.07, -1.57, 0.0]

        self.get_logger().info('=== INICIANDO RUTINA PICK AND PLACE ===')
        
        self.move_arm_joints(home_pose, "1. Posición Home")
        time.sleep(1.0)

        self.move_arm_joints(pick_approach, "2. Aproximación a Pick")
        self.move_arm_joints(pick_grasp, "3. Posición de Agarre (Pick)")
        self.get_logger().info('>>> ACCIÓN: Cerrando Pinza Robotiq')
        time.sleep(1.0)

        self.move_arm_joints(pick_approach, "4. Elevar objeto")
        self.move_arm_joints(place_approach, "5. Transporte a Zona de Place")
        self.move_arm_joints(place_release, "6. Posición de Liberación (Place)")
        self.get_logger().info('>>> ACCIÓN: Abriendo Pinza Robotiq')
        time.sleep(1.0)

        self.move_arm_joints(home_pose, "7. Retorno a Posición Home")
        self.get_logger().info('=== RUTINA FINALIZADA CON ÉXITO ===')

def main():
    rclpy.init()
    node = UR5PickAndPlace()
    node.execute_routine()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
