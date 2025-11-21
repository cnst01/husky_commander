import tkinter as tk
from tkinter import ttk, scrolledtext
import rclpy
from rclpy.node import Node
import threading

# Importar os tipos de mensagem que queremos suportar
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool

class SubscriberNode(Node):
    def __init__(self, ui_callback):
        super().__init__('gui_subscriber_node')
        self.ui_callback = ui_callback
        self.subscription = None
        self.current_topic = ""
        self.get_logger().info('Nó monitor iniciado.')

    def start_listening(self, topic_name, msg_type):
        """Destroi a inscrição anterior e cria uma nova."""
        
        # Se já estiver ouvindo algo, cancela
        if self.subscription:
            self.destroy_subscription(self.subscription)
            self.get_logger().info(f'Parando de ouvir: {self.current_topic}')

        try:
            self.current_topic = topic_name
            # Cria a nova inscrição
            self.subscription = self.create_subscription(
                msg_type,
                topic_name,
                self.listener_callback,
                10
            )
            self.get_logger().info(f'Inscrito em: {topic_name} com tipo {msg_type.__name__}')
            return True
        except Exception as e:
            self.get_logger().error(f'Erro ao se inscrever: {e}')
            return False

    def listener_callback(self, msg):
        """Recebe a mensagem do ROS e manda para a GUI."""
        self.ui_callback(msg)


class MonitorGUI:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("Monitor de Tópicos ROS 2 (Auto-Detect)")
        self.root.geometry("600x450") # Um pouco mais largo

        # --- Dicionário de Tipos Suportados (Friendly Name -> Class) ---
        self.msg_types = {
            "GPS (NavSatFix)": NavSatFix,
            "Pose (PoseStamped)": PoseStamped,
            "IMU (Imu)": Imu,
            "Velocidade (Twist)": Twist,
            "Bateria (BatteryState)": BatteryState,
            "Texto (String)": String,
            "Booleano (Bool)": Bool
        }

        # --- Mapeamento Reverso (ROS String Type -> Friendly Name) ---
        # Usado para selecionar automaticamente o tipo no dropdown
        self.ros_type_to_friendly = {
            'sensor_msgs/msg/NavSatFix': "GPS (NavSatFix)",
            'geometry_msgs/msg/PoseStamped': "Pose (PoseStamped)",
            'sensor_msgs/msg/Imu': "IMU (Imu)",
            'geometry_msgs/msg/Twist': "Velocidade (Twist)",
            'sensor_msgs/msg/BatteryState': "Bateria (BatteryState)",
            'std_msgs/msg/String': "Texto (String)",
            'std_msgs/msg/Bool': "Booleano (Bool)"
        }

        # Armazena os tipos reais dos tópicos encontrados: { '/topic': ['type/string'] }
        self.detected_topics = {}

        # --- Frame de Configuração (Topo) ---
        config_frame = tk.LabelFrame(self.root, text="Configuração da Inscrição", padx=10, pady=10)
        config_frame.pack(fill="x", padx=10, pady=5)

        # 1. Linha de Busca de Tópicos
        tk.Label(config_frame, text="Tópico:").grid(row=0, column=0, sticky="e", padx=5)
        
        # Combobox para listar tópicos encontrados
        self.topic_var = tk.StringVar()
        self.topic_combo = ttk.Combobox(config_frame, textvariable=self.topic_var, state="normal", width=35)
        self.topic_combo.grid(row=0, column=1, padx=5, pady=5)
        self.topic_combo.bind("<<ComboboxSelected>>", self.on_topic_selected) # Evento de seleção

        # Botão Atualizar Lista
        self.btn_refresh = tk.Button(config_frame, text="🔄 Atualizar Lista", command=self.refresh_topics)
        self.btn_refresh.grid(row=0, column=2, padx=5, pady=5)

        # 2. Linha de Tipo de Mensagem
        tk.Label(config_frame, text="Tipo de Msg:").grid(row=1, column=0, sticky="e", padx=5)
        self.type_var = tk.StringVar()
        self.type_combo = ttk.Combobox(config_frame, textvariable=self.type_var, values=list(self.msg_types.keys()), state="readonly", width=35)
        self.type_combo.current(0)
        self.type_combo.grid(row=1, column=1, padx=5, pady=5)

        # Botão Conectar
        self.btn_connect = tk.Button(config_frame, text="Monitorar Selecionado", command=self.on_connect, bg="green", fg="white")
        self.btn_connect.grid(row=2, columnspan=3, pady=15, sticky="ew")

        # --- Frame de Display (Baixo) ---
        display_frame = tk.LabelFrame(self.root, text="Dados Recebidos (Tempo Real)", padx=10, pady=10)
        display_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.text_area = scrolledtext.ScrolledText(display_frame, height=10, font=("Consolas", 10), state='disabled')
        self.text_area.pack(fill="both", expand=True)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Atualiza a lista automaticamente ao iniciar
        self.root.after(1000, self.refresh_topics)

    def refresh_topics(self):
        """Consulta o ROS graph para pegar tópicos ativos."""
        try:
            # Retorna lista de tuplas: [('topic_name', ['type']), ...]
            topics_and_types = self.node.get_topic_names_and_types()
            
            # Limpa dicionário anterior
            self.detected_topics = {}
            topic_names = []
            
            for name, types in topics_and_types:
                self.detected_topics[name] = types
                topic_names.append(name)
            
            topic_names.sort()
            
            # Atualiza o Combobox
            self.topic_combo['values'] = topic_names
            self.update_display_text(f"Lista de tópicos atualizada. {len(topic_names)} encontrados.", color="gray")
            
        except Exception as e:
            self.update_display_text(f"Erro ao buscar tópicos: {e}", color="red")

    def on_topic_selected(self, event):
        """Chamado quando o usuário escolhe um tópico da lista."""
        selected_topic = self.topic_var.get()
        
        if selected_topic in self.detected_topics:
            # Pega o primeiro tipo da lista (geralmente só tem um)
            ros_type_str = self.detected_topics[selected_topic][0]
            
            # Tenta mapear para o nosso "Friendly Name"
            if ros_type_str in self.ros_type_to_friendly:
                friendly = self.ros_type_to_friendly[ros_type_str]
                self.type_combo.set(friendly)
                self.update_display_text(f"Tipo detectado automaticamente: {friendly}", color="green")
            else:
                self.update_display_text(f"Tópico selecionado tem tipo '{ros_type_str}' que não está na nossa lista de suporte. Selecione manualmente se compatível.", color="orange")

    def on_connect(self):
        """Lógica ao clicar no botão Monitorar."""
        friendly_name = self.type_var.get()
        topic = self.topic_var.get() # Pega do Combobox (ou o que foi digitado)
        
        if not topic:
            self.update_display_text("Por favor, selecione ou digite um tópico.", color="red")
            return

        if friendly_name not in self.msg_types:
             self.update_display_text("Tipo de mensagem inválido.", color="red")
             return

        msg_class = self.msg_types[friendly_name]
        
        # Solicita ao nó ROS para mudar a inscrição
        success = self.node.start_listening(topic, msg_class)
        
        if success:
            self.update_display_text(f"--- Conectado a {topic} ({friendly_name}) ---\nEsperando dados...", color="blue")
        else:
            self.update_display_text(f"Erro ao conectar em {topic}", color="red")

    def format_message(self, msg):
        """Formata a mensagem para ficar bonita na tela."""
        output = ""
        
        if isinstance(msg, NavSatFix):
            output += f"Status: {msg.status.status}\n"
            output += f"Latitude:  {msg.latitude:.6f}\n"
            output += f"Longitude: {msg.longitude:.6f}\n"
            output += f"Altitude:  {msg.altitude:.2f} m"
            
        elif isinstance(msg, PoseStamped):
            p = msg.pose.position
            o = msg.pose.orientation
            output += f"Frame: {msg.header.frame_id}\n"
            output += f"Pos X: {p.x:.3f} | Y: {p.y:.3f} | Z: {p.z:.3f}\n"
            output += f"Ori X: {o.x:.3f} | Y: {o.y:.3f} | Z: {o.z:.3f} | W: {o.w:.3f}"
            
        elif isinstance(msg, BatteryState):
            output += f"Voltagem: {msg.voltage:.2f} V\n"
            output += f"Carga:    {msg.percentage * 100:.1f} %"
            
        elif isinstance(msg, Twist):
            l = msg.linear
            a = msg.angular
            output += f"Linear  -> X: {l.x:.2f}, Y: {l.y:.2f}\n"
            output += f"Angular -> Z: {a.z:.2f}"

        else:
            # Fallback genérico
            if hasattr(msg, 'data'):
                output = str(msg.data)
            else:
                output = str(msg)
                
        return output

    def msg_callback_handler(self, msg):
        formatted_text = self.format_message(msg)
        self.root.after(0, self.update_display_text, formatted_text)

    def update_display_text(self, text, color="black"):
        self.text_area.config(state='normal')
        self.text_area.delete(1.0, tk.END)
        self.text_area.insert(tk.END, text)
        self.text_area.tag_add("color", "1.0", "end")
        self.text_area.tag_config("color", foreground=color)
        self.text_area.config(state='disabled')

    def on_closing(self):
        self.root.destroy()
        rclpy.get_global_executor().shutdown()

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    
    gui_ref = [None] 
    
    def bridge_callback(msg):
        if gui_ref[0]:
            gui_ref[0].msg_callback_handler(msg)

    subscriber_node = SubscriberNode(bridge_callback)
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(subscriber_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    app = MonitorGUI(subscriber_node)
    gui_ref[0] = app
    
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()