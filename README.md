# UFES-Robotics-Lab4
Laboratório 4 da disciplina de Tópicos Especiais em Robótica V

# Como começar
Para rodar o laboratório é necessário primeiro rodar 2 containers dockers: ros_turtlebot_sim e ros_tools

ros_turtlebot_sim rodará o simulador. ros_tools rodará o nó de controle do robô

# Como controlar o robô
O robô escuta ativamente pelo nó /HDO/goal, este deve o nó no qual deve ser passada a posição alvo.

HDO/goal é do tipo goal2D, logo a mensagem pode ser no formato "{x: <xgoal>, y: <ygoal>, theta: 0.0}", lembrando de substituir <xgoal> e <ygoal> por valores desejados.

# Macros
Existem três macros programadas para wsl windows, windows docker e X-launch:
  
```docker-compose up``` --> para subir ambos containers
  
```make build``` --> para entrar em uma instância do bash de ros_tools e dar colcon no código
  
```make run``` --> para entrar  em uma instância do bash de ros_tools source e run no nó turtle_control_Higor
  
Esses macros devem funcionar somente no windows wsl e talvez precisem de alteração nos parâmetros para rodar.
