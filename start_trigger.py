import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading

class GuiPublisherNode(Node):
    def __init__(self):
        super().__init__('gui_publisher_node')
        self.publisher_ = self.create_publisher(Bool, '/start_signal', 10)
        self.get_logger().info('Nó publicador da GUI iniciado. Pronto para enviar sinal.')

    def publish_start_signal(self):
        """Publica a mensagem de start."""
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info('Sinal "True" publicado em /start_signal.')

class AppGUI:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("Controle Remoto do Robô")
        
        # Criar o botão Play
        self.play_button = tk.Button(
            self.root, 
            text="START ROBÔ", 
            command=self.on_play_click, 
            font=("Helvetica", 24), 
            bg="green", 
            fg="white",
            padx=50,
            pady=30
        )
        self.play_button.pack(pady=60, padx=120)

        # Tratar o fechamento da janela
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def on_play_click(self):
        """Callback do botão Play."""
        self.node.publish_start_signal()
        # Mudar a aparência do botão para dar feedback
        self.play_button.config(text="SINAL ENVIADO", state=tk.DISABLED, bg="gray")

    def on_closing(self):
        """Callback para quando a janela da GUI é fechada."""
        self.node.get_logger().info('Janela da GUI fechada. Desligando nó.')
        self.root.destroy()  # Destruir a janela Tkinter
        # Sinaliza ao rclpy para desligar
        rclpy.get_global_executor().shutdown()

    def run(self):
        """Inicia o loop da GUI."""
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    
    # Inicia o nó ROS em um thread separado
    gui_node = GuiPublisherNode()
    
    # O rclpy.spin precisa rodar em um thread para não bloquear a GUI
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(gui_node)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Inicia a GUI no thread principal
    app = AppGUI(gui_node)
    try:
        app.run() # Isso bloqueia até a janela fechar
    except KeyboardInterrupt:
        pass
    finally:
        # Quando a GUI fecha (app.run() termina), nós paramos o rclpy
        gui_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
