# Husky Commander

## 1. Descrição {#descrição}

husky_commander é um pacote ROS 2 (Jazzy) que fornece uma infraestrutura
completa para operação e monitoramento remoto de robôs.

O objetivo principal deste pacote é permitir que um operador envie
comandos complexos e rotinas de inicialização (bringup) a partir de uma
interface gráfica, sem a necessidade de acesso via terminal (SSH) ao
robô. Além disso, inclui ferramentas para monitoramento de dados em
tempo real.

### Principais Funcionalidades:

- **Comando Remoto:** Execução de ros2 launch e ros2 run via GUI.

- **Suporte a Argumentos:** Envio de argumentos complexos (ex:
  > map:=/path/to/map.yaml).

- **Rotinas Automatizadas:** Definição de sequências de comandos (ex:
  > Iniciar Simulação -\> Spawnar Robô -\> Iniciar Nav2) através de um
  > arquivo routines.yaml.

- **Monitor de Tópicos:** Ferramenta gráfica para visualizar dados de
  > sensores (GPS, IMU, Bateria, Pose) formatados para leitura humana.

## 2. Arquitetura {#arquitetura}

Este sistema é composto por quatro partes principais:

1.  **Este Pacote (husky_commander)** - *Roda no Robô*

    - msg/Command.msg: Define a estrutura de um comando único (incluindo
      > lista de argumentos).

    - msg/Routine.msg: Define uma lista de comandos para execução
      > sequencial.

    - scripts/waiter_node.py: O nó \"cérebro\" que roda no robô, escuta
      > os tópicos /start_command e /start_routine, e gerencia a
      > execução dos processos.

    - launch/command_listener.launch.py: Launch file para iniciar o
      > waiter_node.

2.  **O Aplicativo de Controle (commander.py)** - *Roda no PC de
    > Controle*

    - Interface gráfica (Tkinter) para envio de comandos.

    - Lê o arquivo routines.yaml para gerar botões de rotina
      > dinamicamente.

    - Publica nos tópicos /start_command e /start_routine.

3.  **O Monitor de Tópicos (topic_monitor.py)** - *Roda no PC de
    > Controle*

    - Interface gráfica para assinatura e visualização de tópicos em
      > tempo real.

    - Detecta automaticamente tópicos ativos e seus tipos.

    - Formata mensagens como NavSatFix, PoseStamped e BatteryState de
      > forma legível.

4.  **Arquivo de Configuração (routines.yaml)**

    - Arquivo local (junto ao commander.py) que define as rotinas
      > pré-programadas.

## 3. Instalação e Dependências {#instalação-e-dependências}

Assume-se que este pacote (husky_commander) e seus pacotes-alvo estão no
workspace (ex: \~/clearpath_ws).

1.  Dependências Python (no PC de Controle):  
    > O commander.py precisa ler arquivos YAML.  
    > pip install pyyaml

2.  Dependências ROS (Msg types):  
    > Para o monitor funcionar com tipos comuns:  
    > sudo apt install ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs

3.  Compilação:  
    > Como há mensagens customizadas (.msg), é necessário recompilar
    > sempre que elas mudarem.  
    > cd \~/clearpath_ws  
    > colcon build \--packages-select husky_commander  
    > source install/setup.bash

## 4. Como Usar (Passo a Passo) {#como-usar-passo-a-passo}

### A. No Robô (Lado do \"Ouvinte\") {#a.-no-robô-lado-do-ouvinte}

Este passo \"arma\" o robô para receber comandos.

1.  Acesse o robô (SSH ou terminal direto).

2.  Inicie o ouvinte:  
    > source \~/clearpath_ws/install/setup.bash  
    > ros2 launch husky_commander command_listener.launch.py  
    >   
    > *Log esperado:* \[INFO\]: Esperando por comandos em /start_command
    > e /start_routine\...

### B. No PC de Controle (Commander App) {#b.-no-pc-de-controle-commander-app}

1.  Garanta que o arquivo routines.yaml esteja na mesma pasta que o
    > commander.py.

2.  Execute o App:  
    > source \~/clearpath_ws/install/setup.bash  
    > python3 gui_app.py

**Funcionalidades:**

- **Comando Manual:** Selecione launch ou run, digite o pacote, o
  > arquivo e opcionalmente os argumentos (ex: use_sim_time:=True).
  > Clique em ENVIAR.

- **Rotinas:** Clique nos botões azuis (gerados a partir do YAML) para
  > executar sequências complexas.

### C. No PC de Controle (Topic Monitor) {#c.-no-pc-de-controle-topic-monitor}

Para verificar se o robô está publicando dados corretamente (ex: GPS,
Bateria):

1.  Execute o App de Monitoramento:  
    > source \~/clearpath_ws/install/setup.bash  
    > python3 topic_monitor_app.py

2.  Clique em **\"🔄 Atualizar Lista\"** para buscar tópicos ativos no
    > robô.

3.  Selecione um tópico na lista (o Tipo de Msg será detectado
    > automaticamente).

4.  Clique em **\"Monitorar Selecionado\"**.

## 5. Configurando Rotinas (routines.yaml) {#configurando-rotinas-routines.yaml}

Você pode criar suas próprias sequências de boot editando o arquivo
routines.yaml. Não é necessário alterar o código Python.

**Exemplo de sintaxe:**

routines:  
- name: \"Navegação Completa (Com Mapa)\"  
steps:  
\# Passo 1: Lança o Nav2 com argumentos  
- type: \"launch\"  
package: \"husky_navigation\"  
command: \"nav2_husky.launch.py\"  
args:  
- \"map:=\'/home/robot/maps/office.yaml\'\"  
- \"use_sim_time:=False\"  
  
\# Passo 2: Roda um nó específico  
- type: \"run\"  
package: \"husky_navigation\"  
command: \"waypoint_follower_node\"
