# VSSS-Simulator
Simulador do VSSS escrito em C++ baseado no MuJoCo

# Dependências:
Para controlar o simulador, é necessário o projeto VSSS-EMC (https://github.com/PEQUI-MEC/VSSS-EMC.git), na branch simulator_gui. O VSSS-EMC se comunica com o VSSS-Simulator para receber as posições dos robôs e da bola e enviar comandos para cada robô simulado.

Execute o script install_dependencies.sh do VSSS-EMC (na branch simulator_gui) para instalar as dependências dos dois programas. Caso já tenha instalado as dependências do VSSS-EMC anteriormente, utilize o install_dependencies.sh deste projeto para instalar apenas as dependências do simulador.


# Licença
O simulador MuJoCo exige uma licença, obtida em https://www.roboti.us/license.html.

Preencha o formulário, incluindo o ID de Computador obtido através do programa getid_linux disponível em https://www.roboti.us/getid/getid_linux.
A licença será enviada por e-mail, que inclui o arquivo mjkey.txt em anexo. Copie o arquivo mjkey.txt para a raiz do projeto VSSS-Simulator.

# Compilação
Código pode ser compilado utilizando os comandos:
```
mkdir build/
cd build/
cmake ..
make
```
# Simular um jogo
Antes de executar o VSSS-Simulator, abra um terminal e execute o comando "roscore" para executar o servidor do ROS, que permite a comunicação com o VSSS-EMC.
Execute então o VSSS-Simulator e o VSSS-EMC através do terminal.

Para jogar uma partida simulada, selecione a opção "Use simulador" da aba Simulator do programa VSSS-EMC e clique no botão de play.

# Atalhos de teclado
- 1: Simular apenas um time
- 2: Simular dois times
- Backspace: Reseta o jogo
