#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from husky_commander.msg import Command, Routine
import subprocess
import sys
import time

class WaiterNode(Node):
    def __init__(self):
        super().__init__('command_waiter_node')
        
        self.command_subscription = self.create_subscription(
            Command,
            '/start_command',
            self.command_listener_callback,
            10
        )
        self.routine_subscription = self.create_subscription(
            Routine,
            '/start_routine',
            self.routine_listener_callback,
            10
        )
        
        self.get_logger().info(f'Nó "garçom" de comandos e rotinas iniciado.')
        self.get_logger().info(f'Esperando por comandos em /start_command e /start_routine...')

    def execute_command(self, cmd_msg, log_prefix=""):
        """Função centralizada para executar um único comando."""
        self.get_logger().info(f"{log_prefix}Executando: {cmd_msg.type} {cmd_msg.package} {cmd_msg.command} {' '.join(cmd_msg.args)}")
        
        if cmd_msg.type not in ['run', 'launch']:
            self.get_logger().error(f'{log_prefix}Tipo de comando desconhecido: "{cmd_msg.type}".')
            return False # Falha
            
        if not cmd_msg.package or not cmd_msg.command:
            self.get_logger().error(f'{log_prefix}Pacote ou comando não podem estar vazios.')
            return False # Falha

        # --- LÓGICA DE CONSTRUÇÃO ATUALIZADA ---
        # Começa com o básico
        cmd_list = ['ros2', cmd_msg.type, cmd_msg.package, cmd_msg.command]
        
        # Adiciona os argumentos
        if cmd_msg.args:
            # A mágica está aqui: msg.args é uma lista de strings
            # que é simplesmente concatenada à lista do comando.
            cmd_list.extend(cmd_msg.args)
        # ----------------------------------------
        
        try:
            subprocess.Popen(cmd_list)
            self.get_logger().info(f'{log_prefix}Comando "{ " ".join(cmd_list) }" executado com sucesso.')
            return True # Sucesso
        except Exception as e:
            self.get_logger().error(f'{log_prefix}Falha ao executar o comando: {e}')
            return False # Falha

    def command_listener_callback(self, msg):
        """Processa um único comando."""
        self.get_logger().info('Comando único recebido.')
        self.execute_command(msg, log_prefix="[Comando Único] ")

    def routine_listener_callback(self, msg):
        """Processa uma rotina (lista de comandos) em sequência."""
        self.get_logger().info(f"Rotina recebida com {len(msg.commands)} passo(s). Iniciando...")
        
        for i, cmd in enumerate(msg.commands):
            passo = i + 1
            log_prefix = f"[Rotina Passo {passo}] "
            
            # Chama a função de execução centralizada
            success = self.execute_command(cmd, log_prefix=log_prefix)
            
            if not success:
                self.get_logger().error(f"{log_prefix}Falha ao executar. Rotina abortada.")
                return

            # Se NÃO for o último comando, espera um pouco
            if i < len(msg.commands) - 1:
                wait_time = 5 # Espera 5 segundos (ajuste se precisar)
                self.get_logger().info(f"[Rotina]: Aguardando {wait_time}s para o próximo passo...")
                time.sleep(wait_time)
        
        self.get_logger().info("Rotina concluída com sucesso.")


def main(args=None):
    rclpy.init(args=args)
    waiter_node = WaiterNode()
    
    try:
        rclpy.spin(waiter_node)
    except KeyboardInterrupt:
        waiter_node.get_logger().info('Desligando nó "garçom"...')
    finally:
        if rclpy.ok():
            waiter_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()