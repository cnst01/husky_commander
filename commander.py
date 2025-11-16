import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
# ATUALIZADO: Importa da sua nova mensagem!
from husky_commander.msg import Command 
import threading

class GuiPublisherNode(Node):
    def __init__(self):
        super().__init__('gui_publisher_node')
        # Tópico e tipo de mensagem atualizados
        self.publisher_ = self.create_publisher(Command, '/start_command', 10)
        self.get_logger().info('Nó publicador da GUI iniciado. Pronto para enviar comandos.')

    def publish_command(self, cmd_type, pkg, cmd):
        """Publica a mensagem de comando."""
        msg = Command()
        msg.type = cmd_type
        msg.package = pkg
        msg.command = cmd
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando comando: {cmd_type} {pkg} {cmd}')

class AppGUI:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("Controle Remoto do Robô")
        
        frame = tk.Frame(self.root, padx=20, pady=20)
        frame.pack()

        # --- Linha 1: Tipo de Comando ---
        tk.Label(frame, text="Tipo:").grid(row=0, column=0, sticky="e", padx=5, pady=5)
        self.cmd_type_var = tk.StringVar(value="launch") # Valor padrão
        cmd_options = ["launch", "run"]
        self.cmd_type_menu = ttk.Combobox(frame, textvariable=self.cmd_type_var, values=cmd_options, state="readonly", width=27)
        self.cmd_type_menu.grid(row=0, column=1, padx=5, pady=5)

        # --- Linha 2: Pacote ---
        tk.Label(frame, text="Pacote:").grid(row=1, column=0, sticky="e", padx=5, pady=5)
        self.package_entry = tk.Entry(frame, width=30)
        self.package_entry.grid(row=1, column=1, padx=5, pady=5)
        # Exemplo para facilitar
        self.package_entry.insert(0, "husky_navigation") 

        # --- Linha 3: Comando ---
        tk.Label(frame, text="Comando:").grid(row=2, column=0, sticky="e", padx=5, pady=5)
        self.command_entry = tk.Entry(frame, width=30)
        self.command_entry.grid(row=2, column=1, padx=5, pady=5)
        # Exemplo para facilitar
        self.command_entry.insert(0, "mapping_slam.launch.py")

        # --- Linha 4: Botão de Envio ---
        self.play_button = tk.Button(
            frame, 
            text="ENVIAR COMANDO", 
            command=self.on_send_click, 
            font=("Helvetica", 14), 
            bg="green", 
            fg="white"
        )
        self.play_button.grid(row=3, columnspan=2, pady=20)

        # Tratar o fechamento da janela
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def on_send_click(self):
        """Callback do botão Enviar."""
        cmd_type = self.cmd_type_var.get()
        pkg = self.package_entry.get()
        cmd = self.command_entry.get()
        
        if not pkg or not cmd:
            self.node.get_logger().warn("Pacote e Comando não podem estar vazios!")
            return
            
        self.node.publish_command(cmd_type, pkg, cmd)

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