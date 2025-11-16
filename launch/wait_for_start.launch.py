from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- Declaração dos Argumentos ---
    
    # Argumento para o NOME DO PACOTE alvo
    target_pkg_arg = DeclareLaunchArgument(
        'target_pkg',
        default_value='husky_navigation',
        description='O nome do pacote (package) que o waiter_node deve executar.'
    )
    
    # Argumento para o NOME DO NÓ alvo
    target_node_arg = DeclareLaunchArgument(
        'target_node',
        default_value='waypoint_navigation',
        description='O nome do executável (node) que o waiter_node deve executar.'
    )
    # ----------------------------------

    return LaunchDescription([
        
        # Adiciona os argumentos declarados ao launch description
        target_pkg_arg,
        target_node_arg,
        
        # 1. Inicia o nó "garçom" que espera pelo sinal
        Node(
            package='husky_commander', # O pacote onde você salvou o waiter_node.py
            executable='waiter_node.py', 
            name='start_waiter',
            output='screen', # Para vermos os logs
            parameters=[
                # Usa o valor dos argumentos passados no launch
                {'pkg_to_run': LaunchConfiguration('target_pkg')},
                {'node_to_run': LaunchConfiguration('target_node')}
            ]
        )
        
        # Você pode adicionar outros nós que precisam ser iniciados 
        # imediatamente aqui (ex: drivers do robô, etc.)
    ])