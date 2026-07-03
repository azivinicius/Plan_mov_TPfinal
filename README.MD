SIMULAÇÃO DE ENXAME SPH - INSTRUÇÕES DE EXECUÇÃO
================================================

1. COMPILAÇÃO
       colcon build --packages-select SPH
       source install/setup.bash

2. EXECUTE A SIMULAÇÃO PADRÃO
   Para iniciar com 6 robôs em um mundo vazio:
       ros2 launch SPH diff.launch.py

3. PERSONALIZE SUA SIMULAÇÃO
   Argumentos disponíveis no lançamento:
       modo        : "diff_sph" ou "basic"
       mundo       : "empty", "simple", "maze" ou "arena"
       num_robots  : qualquer número inteiro (ex.: 1, 4, 8)

4. EXEMPLOS DE COMANDOS PERSONALIZADOS
   Executar 4 robôs no labirinto:
       ros2 launch SPH sph.launch.py mundo:=maze num_robots:=4

   Executar 8 robôs na arena simples usando o controlador básico:
       ros2 launch SPH sph.launch.py mundo:=simple modo:=basic num_robots:=8

   Executar um único robô na arena:
       ros2 launch SPH sph.launch.py mundo:=arena num_robots:=1


killall -9 gz ruby ros2 python3;cd ~/Plan_mov_TPfinal;colcon build --packages-select SPH;source install/setup.bash;ros2 launch SPH diff.launch.py mundo:=simple num_robots:=3