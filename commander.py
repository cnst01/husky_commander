import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
# ATUALIZADO: Importa ambas as mensagens
from husky_commander.msg import Command, Routine 
import threading

class GuiPublisherNode(Node):
    def __init__(self):
        super().__init__('gui_publisher_node')
        # Publicador original para comandos únicos
        self.command_publisher_ = self.create_publisher(Command, '/start_command', 10)
        # NOVO: Publicador para rotinas
        self.routine_publisher_ = self.create_publisher(Routine, '/start_routine', 10)
        
        self.get_logger().info('Nó publicador da GUI iniciado.')

    def publish_command(self, cmd_type, pkg, cmd):
        """Publica uma mensagem de comando único."""
        msg = Command()
        msg.type = cmd_type
        msg.package = pkg
        msg.command = cmd
        self.command_publisher_.publish(msg)
        self.get_logger().info(f'Publicando comando: {cmd_type} {pkg} {cmd}')

    def publish_routine(self, routine_name):
        """Publica uma mensagem de rotina pré-definida."""
        routine_msg = Routine()
        
        if routine_name == "waypoint_nav":
            self.get_logger().info("Enviando rotina de navegação de waypoints...")
            
            # Passo 1 da Rotina
            cmd1 = Command()
            cmd1.type = "launch"
            cmd1.package = "husky_navigation"
            # Assumindo que seu arquivo é 'waypoint_navigation.launch.py'
            cmd1.command = "nav2_husky.launch.py" 
            
            # Passo 2 da Rotina
            cmd2 = Command()
            cmd2.type = "run"
            cmd2.package = "husky_navigation"
            cmd2.command = "waypoint_state_machine"
            
            # Adiciona os passos à rotina
            routine_msg.commands = [cmd1, cmd2]

        elif routine_name == "simulation_bringup":
            self.get_logger().info("Enviando rotina de navegação de waypoints...")
            
            # Passo 1 da Rotina
            cmd1 = Command()
            cmd1.type = "launch"
            cmd1.package = "clearpath_gz"
            cmd1.command = "gz_sim.launch.py" 
            
            # Passo 2 da Rotina
            cmd2 = Command()
            cmd2.type = "launch"
            cmd2.package = "clearpath_gz"
            cmd2.command = "robot_spawn.launch.py"
            
            # Adiciona os passos à rotina
            routine_msg.commands = [cmd1, cmd2]
            
        else:
            self.get_logger().warn(f"Rotina desconhecida: {routine_name}")
            return
            
        # Publica a rotina completa
        self.routine_publisher_.publish(routine_msg)
        self.get_logger().info("Rotina enviada.")


class AppGUI:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("Controle Remoto do Robô")
        
        # --- Frame para Comandos Manuais ---
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
        self.command_entry.insert(0, "mapping_slam.launch.py")

        self.send_button = tk.Button(
            manual_frame, 
            text="ENVIAR COMANDO", 
            command=self.on_send_click, 
            bg="green", 
            fg="white"
        )
        self.send_button.grid(row=3, columnspan=2, pady=10, sticky="ew")
        
        # --- NOVO: Frame para Rotinas ---
        routine_frame = tk.LabelFrame(self.root, text="Rotinas Pré-Definidas", padx=10, pady=10)
        routine_frame.pack(fill="x", padx=10, pady=10)

        self.bringup_button = tk.Button(
            routine_frame,
            text="Iniciar Rotina de Navegação (Waypoint)",
            command=self.on_routine_click_nav,
            bg="blue",
            fg="white",
            font=("Helvetica", 12)
        )
        self.bringup_button.pack(fill="x", pady=5)

        self.bringup_button = tk.Button(
            routine_frame,
            text="Iniciar Rotina de Simulação (Bringup simulation)",
            command=self.on_routine_click_bringup_simu,
            bg="blue",
            fg="white",
            font=("Helvetica", 12)
        )
        self.bringup_button.pack(fill="x", pady=5)
        
        # Tratar o fechamento da janela
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def on_send_click(self):
        """Callback do botão Enviar (Comando Único)."""
        cmd_type = self.cmd_type_var.get()
        pkg = self.package_entry.get()
        cmd = self.command_entry.get()
        
        if not pkg or not cmd:
            self.node.get_logger().warn("Pacote e Comando não podem estar vazios!")
            return
        self.node.publish_command(cmd_type, pkg, cmd)

    def on_routine_click_nav(self):
        """Callback do botão de Rotina."""
        # O nome "waypoint_nav" é a chave para o publisher saber o que fazer
        self.node.publish_routine("waypoint_nav")

    def on_routine_click_bringup_simu(self):
        """Callback do botão de Rotina."""
        # O nome "waypoint_nav" é a chave para o publisher saber o que fazer
        self.node.publish_routine("simulation_bringup")

    def on_closing(self):
        """Callback para quando a janela da GUI é fechada."""
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
