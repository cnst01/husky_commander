#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from husky_commander.msg import Command
import subprocess
import sys

class WaiterNode(Node):
    def __init__(self):
        super().__init__('command_waiter_node') # Novo nome para o nó
        
        # Subscrever no novo tópico e com o novo tipo de mensagem
        self.subscription = self.create_subscription(
            Command,
            '/start_command',
            self.listener_callback,
            10  # QoS
        )
        self.get_logger().info(f'Nó "garçom" de comandos iniciado.')
        self.get_logger().info(f'Esperando por comandos em /start_command...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Comando recebido: {msg.type} {msg.package} {msg.command}')
        
        # Valida o tipo de comando
        if msg.type not in ['run', 'launch']:
            self.get_logger().error(f'Tipo de comando desconhecido: "{msg.type}". Use "run" ou "launch".')
            return
            
        # Valida pacote e comando
        if not msg.package or not msg.command:
            self.get_logger().error('Pacote ou comando não podem estar vazios.')
            return

        # Constrói o comando
        cmd_list = ['ros2', msg.type, msg.package, msg.command]
        
        try:
            # Usar subprocess.Popen para iniciar o processo
            # Isso não bloqueia o nó "garçom"
            subprocess.Popen(cmd_list)
            self.get_logger().info(f'Comando "{ " ".join(cmd_list) }" executado com sucesso.')
            
        except Exception as e:
            self.get_logger().error(f'Falha ao executar o comando: {e}')

def main(args=None):
    rclpy.init(args=args)
    waiter_node = WaiterNode()
    
    try:
        rclpy.spin(waiter_node)
    except KeyboardInterrupt:
        waiter_node.get_logger().info('Desligando nó "garçom"...')
    finally:
        # Garante que tudo seja limpo se o nó for encerrado
        if rclpy.ok():
            waiter_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()