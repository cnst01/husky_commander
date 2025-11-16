#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
# ATUALIZADO: Importa ambas as mensagens
from husky_commander.msg import Command, Routine
import subprocess
import sys
import time # NOVO: Importa o 'time' para a pausa

class WaiterNode(Node):
    def __init__(self):
        super().__init__('command_waiter_node')
        
        # Subscrição original para comandos únicos
        self.command_subscription = self.create_subscription(
            Command,
            '/start_command',
            self.command_listener_callback,
            10  # QoS
        )
        
        # NOVA Subscrição para rotinas
        self.routine_subscription = self.create_subscription(
            Routine,
            '/start_routine',
            self.routine_listener_callback,
            10 # QoS
        )
        
        self.get_logger().info(f'Nó "garçom" de comandos e rotinas iniciado.')
        self.get_logger().info(f'Esperando por comandos em /start_command e /start_routine...')

    def command_listener_callback(self, msg):
        """Processa um único comando."""
        self.get_logger().info(f'Comando único recebido: {msg.type} {msg.package} {msg.command}')
        
        # Valida o tipo de comando
        if msg.type not in ['run', 'launch']:
            self.get_logger().error(f'Tipo de comando desconhecido: "{msg.type}". Use "run" ou "launch".')
            return
            
        if not msg.package or not msg.command:
            self.get_logger().error('Pacote ou comando não podem estar vazios.')
            return

        # Constrói o comando
        cmd_list = ['ros2', msg.type, msg.package, msg.command]
        
        try:
            # Popen não bloqueia o nó, o que é bom
            subprocess.Popen(cmd_list)
            self.get_logger().info(f'Comando "{ " ".join(cmd_list) }" executado com sucesso.')
        except Exception as e:
            self.get_logger().error(f'Falha ao executar o comando: {e}')

    def routine_listener_callback(self, msg):
        """Processa uma rotina (lista de comandos) em sequência."""
        self.get_logger().info(f"Rotina recebida com {len(msg.commands)} passo(s). Iniciando...")
        
        for i, cmd in enumerate(msg.commands):
            passo = i + 1
            self.get_logger().info(f"[Rotina Passo {passo}]: {cmd.type} {cmd.package} {cmd.command}")
            
            # Validação (igual ao callback de comando único)
            if cmd.type not in ['run', 'launch']:
                self.get_logger().error(f"[Rotina Passo {passo}]: Tipo desconhecido, pulando.")
                continue # Pula para o próximo comando
            if not cmd.package or not cmd.command:
                self.get_logger().error(f"[Rotina Passo {passo}]: Pacote ou comando vazios, pulando.")
                continue # Pula para o próximo comando

            # Constrói o comando
            cmd_list = ['ros2', cmd.type, cmd.package, cmd.command]
            
            try:
                # Inicia o processo
                subprocess.Popen(cmd_list)
                self.get_logger().info(f"[Rotina Passo {passo}]: Comando executado.")
                
                # Se NÃO for o último comando, espera um pouco para o próximo passo
                if i < len(msg.commands) - 1:
                    wait_time = 5 # Espera 5 segundos (ajuste se precisar)
                    self.get_logger().info(f"[Rotina]: Aguardando {wait_time}s para o próximo passo...")
                    time.sleep(wait_time) # Pausa a execução deste callback
                    
            except Exception as e:
                self.get_logger().error(f'[Rotina Passo {passo}]: Falha ao executar: {e}')
                # Se um passo falhar, é melhor parar a rotina
                self.get_logger().error("[Rotina]: Rotina abortada devido a erro.")
                return # Sai da função callback
        
        self.get_logger().info("Rotina concluída com sucesso.")


def main(args=None):
    rclpy.init(args=args)
    waiter_node = WaiterNode()
    
    try:
        # spin() mantém o nó vivo para receber mensagens
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