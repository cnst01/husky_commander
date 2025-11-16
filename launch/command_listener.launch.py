from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        # 1. Inicia o nó "garçom" que espera pelos comandos
        Node(
            package='husky_commander', # <-- IMPORTANTE: O nome do SEU pacote de robô
            executable='waiter_node.py', # O script que agora é persistente
            name='command_waiter_node',
            output='screen', # Para vermos os logs
        )
        
        # Você pode adicionar outros nós que precisam ser iniciados 
        # imediatamente aqui (ex: drivers do robô, etc.)
    ])