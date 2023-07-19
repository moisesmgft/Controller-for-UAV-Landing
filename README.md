# Controller-for-UAV-Landing

Repositório para o pacote pid-controller, responsável pela implementação de controlador PID para pouso autônomo de UAV em base móvel.

## Pré-requisitos

- Gazebo Garden
- ROS 2 Humble
- PX4 SITL
- px4_msgs 
- MicroXRCEAgent

[Sugestão de instalação.][1]

## Compilação

Na pasta em que se encontra o pacote `px4_msgs`:

```bash
colcon build
source install/setup.bash
```

Na pasta `Controller-for-UAV-Landing`:

```bash
colcon build
source install/setup.bash
```

## Manual de uso

O pacote `pid-controller` oferece três executáveis:
- `horizontalStepResponse`: para análise do controlador horizontal 
- `verticalStepResponse`: para análise do controlador vertical
- `precisionLanding`: para análise do controlador final na tarefa de pouso autônomo

Como usar:

Inicie o simulador com PX4_SITL.

```bash
cd path/to/PX4-Autopilot
make px4_sitl gz_x500_vision
```

Inicie o agente.

```bash
MicroXRCEAgent udp4 -p 8888
```

Rode o executável desejado.

```bash
ros2 run pid-controller <executable_name>
```

[1]: https://github.com/PX4/PX4-user_guide/blob/main/en/ros/ros2_comm.md#installation-setup