import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from husky_commander.msg import Command, Routine 
import threading
import yaml
import os

class GuiPublisherNode(Node):
    def __init__(self):
        super().__init__('gui_publisher_node')
        self.command_publisher_ = self.create_publisher(Command, '/start_command', 10)
        self.routine_publisher_ = self.create_publisher(Routine, '/start_routine', 10)
        self.get_logger().info('Nó publicador da GUI iniciado.')

    def publish_command(self, cmd_type, pkg, cmd, args_list):
        """Publica uma mensagem de comando único."""
        msg = Command()
        msg.type = cmd_type
        msg.package = pkg
        msg.command = cmd
        msg.args = args_list
        
        self.command_publisher_.publish(msg)
        self.get_logger().info(f'Publicando comando: {cmd_type} {pkg} {cmd} {" ".join(args_list)}')

    def publish_routine(self, routine_name, steps_data):
        """
        Publica uma rotina. 
        """
        self.get_logger().info(f"Enviando rotina '{routine_name}'...")
        routine_msg = Routine()
        routine_msg.commands = []

        for step in steps_data:
            cmd_step = Command()
            cmd_step.type = step.get('type', 'launch')
            cmd_step.package = step.get('package', '')
            cmd_step.command = step.get('command', '')
            cmd_step.args = step.get('args', [])
            
            if not cmd_step.package or not cmd_step.command:
                self.get_logger().error(f"Passo da rotina inválido, pulando: {step}")
                continue
                
            routine_msg.commands.append(cmd_step)
            
        if not routine_msg.commands:
            self.get_logger().warn("Rotina estava vazia ou inválida. Nada foi enviado.")
            return

        self.routine_publisher_.publish(routine_msg)
        self.get_logger().info(f"Rotina '{routine_name}' enviada com {len(routine_msg.commands)} passos.")


class AppGUI:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("Controle Remoto do Robô")
        
        manual_frame = tk.LabelFrame(self.root, text="Comando Manual", padx=10, pady=10)
        manual_frame.pack(fill="x", padx=10, pady=10)

        tk.Label(manual_frame, text="Tipo:").grid(row=0, column=0, sticky="e", padx=5, pady=5)
        self.cmd_type_var = tk.StringVar(value="launch")
        cmd_options = ["launch", "run"]
        self.cmd_type_menu = ttk.Combobox(manual_frame, textvariable=self.cmd_type_var, values=cmd_options, state="readonly", width=27)
        self.cmd_type_menu.grid(row=0, column=1, padx=5, pady=5)

        tk.Label(manual_frame, text="Pacote:").grid(row=1, column=0, sticky="e", padx=5, pady=5)
        self.package_entry = tk.Entry(manual_frame, width=30)
        self.package_entry.grid(row=1, column=1, padx=5, pady=5)
        self.package_entry.insert(0, "husky_navigation") 

        tk.Label(manual_frame, text="Comando:").grid(row=2, column=0, sticky="e", padx=5, pady=5)
        self.command_entry = tk.Entry(manual_frame, width=30)
        self.command_entry.grid(row=2, column=1, padx=5, pady=5)
        self.command_entry.insert(0, "nav2_husky.launch.py")

        tk.Label(manual_frame, text="Args:").grid(row=3, column=0, sticky="e", padx=5, pady=5)
        self.args_entry = tk.Entry(manual_frame, width=30)
        self.args_entry.grid(row=3, column=1, padx=5, pady=5)
        self.args_entry.insert(0, "map:='path/to/map.yaml'") # Exemplo

        self.send_button = tk.Button(
            manual_frame, 
            text="ENVIAR COMANDO", 
            command=self.on_send_click, 
            bg="green", 
            fg="white"
        )
        self.send_button.grid(row=4, columnspan=2, pady=10, sticky="ew")
        
        routine_frame = tk.LabelFrame(self.root, text="Rotinas Pré-Definidas", padx=10, pady=10)
        routine_frame.pack(fill="x", padx=10, pady=10)

        self.routines = self.load_routines('routines.yaml')
        
        if not self.routines:
            tk.Label(routine_frame, text="Arquivo 'routines.yaml' não encontrado ou vazio.", fg="red").pack()
        else:
            for routine in self.routines:
                name = routine['name']
                steps = routine['steps']
                
                btn = tk.Button(
                    routine_frame,
                    text=name,
                    command=lambda n=name, s=steps: self.node.publish_routine(n, s),
                    bg="blue",
                    fg="white",
                    font=("Helvetica", 12)
                )
                btn.pack(fill="x", pady=5)
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def load_routines(self, filename):
        """Carrega as definições de rotina do arquivo YAML."""
        script_dir = os.path.dirname(os.path.realpath(__file__))
        filepath = os.path.join(script_dir, filename)
        
        self.node.get_logger().info(f"Carregando rotinas de: {filepath}")
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
                if 'routines' in data:
                    return data['routines']
                else:
                    self.node.get_logger().error(f"Arquivo YAML '{filename}' não contém a chave 'routines'.")
                    return []
        except FileNotFoundError:
            self.node.get_logger().error(f"Arquivo de rotinas '{filename}' não encontrado.")
            return []
        except Exception as e:
            self.node.get_logger().error(f"Erro ao ler o arquivo YAML: {e}")
            return []

    def on_send_click(self):
        """Callback do botão Enviar (Comando Único)."""
        cmd_type = self.cmd_type_var.get()
        pkg = self.package_entry.get()
        cmd = self.command_entry.get()
        
        args_string = self.args_entry.get()
        if args_string:
            args_list = args_string.split()
        else:
            args_list = []
        
        if not pkg or not cmd:
            self.node.get_logger().warn("Pacote e Comando não podem estar vazios!")
            return
            
        self.node.publish_command(cmd_type, pkg, cmd, args_list)

    def on_closing(self):
        self.node.get_logger().info('Janela da GUI fechada. Desligando nó.')
        self.root.destroy()
        rclpy.get_global_executor().shutdown()

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    gui_node = GuiPublisherNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(gui_node)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    app = AppGUI(gui_node)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()