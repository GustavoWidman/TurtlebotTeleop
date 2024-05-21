# PonderadaROS2 - Gustavo Wagon Widman

## Descrição

Esta atividade consiste de um pacote ROS desenvolvido para uma atividade ponderada que deve controlar um TurtleBOT de maneira teleoperada. Escolheu-se utilizar uma CLI para controlar os movimentos do robo. O pacote inclui uma aplicação Python que utiliza o framework ROS (Robot Operating System) para controlar o TurtleBOT utilizando o teclado (WASD ou setas).

## Dependências

Certifique-se de que você possui o `ROS2 Humble` e o `colcon` instalado em sua máquina. Caso não tenha, siga as instruções de instalação amigáveis (nao oficiais) aqui [ROS2 Humble Unofficial Installation](https://rmnicola.github.io/m6-ec-encontros/E01/ros) ou siga as instruções oficiais [ROS2 Installation](https://docs.ros.org/en/humble/Installation.html). Também é necessário ter o simulador `webots` instalado em sua máquina. Para instalar o simulador, rode o seguinte comando:

```bash
sudo apt instal webots
```

O script `run.sh` cuidara de instalar as outras dependências necessárias para o pacote. Note que o `run.sh` necessita do pacote `python3.10-venv` para funcionar. Caso não tenha, instale-o com o seguinte comando:

```bash
sudo apt install python3.10-venv
```

Lista de dependências de Python:

- [inquirerpy](https://pypi.org/project/inquirerpy/) v0.3.4

## Instalação e Execução

Para instalar o pacote, basta clonar o repositório:

```bash
git clone https://github.com/GustavoWidman/PonderadaROS2.git
```

E, em seguida, rodar o script de instalação e execução:

```bash
cd PonderadaROS2
chmod +x run.sh # averiguar se é necessário
./run.sh
```

Bem, já que o pacote foi instalado e esta rodando, para ver o TurtleBOT se movendo, abra um outro terminal e rode o script do simulador `simulator.sh` (ele cuidará de abrir o simulador `webots`):

```bash
chmod +x simulator.sh # averiguar se é necessário
./simulator.sh
```

Agora pronto! O TurtleBOT está pronto para ser controlado. Para controlar o TurtleBOT, basta seguir as instruções que aparecerão no terminal. O TurtleBOT pode ser controlado pelas teclas WASD ou pelas setas do teclado. A barra de espaço serve para parar o TurtleBOT e a tecla "Q" serve como parada de emergência, imediatamente parando o TurtleBOT e encerrando o programa.

## Serviço de Emergência

O pacote inclui um serviço de emergência que pode ser chamado a qualquer momento. Para chamar o serviço de emergência, basta rodar o seguinte comando:

```bash
ros2 service call /emergency_stop_teleop std_srvs/srv/Empty
```

Ou rodar o script `emergency.sh`:

```bash
chmod +x emergency.sh # averiguar se é necessário
./emergency.sh
```

O serviço de emergência imediatamente para o TurtleBOT (caso esteja em movimento) e encerra o programa (caso esteja rodando). O serviço de emergência é uma maneira segura de parar o TurtleBOT em caso de emergência.

## Demonstração

A seguir eu demonstro o pacote em funcionamento, controlando o TurtleBOT no simulador `webots` usando o teclado e chamando o serviço de emergência:

[PonderadaROS2.webm](https://github.com/GustavoWidman/PonderadaROS2/assets/123963822/854a1bcb-815d-46c2-a50e-e65101306cde)
