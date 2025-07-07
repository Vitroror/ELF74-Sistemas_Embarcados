# ELF74-Sistemas Embarcados - Projeto Final
Repositório contendo o projeto final da disciplina ELF74 - Sistemas Embarcados - 2025.1

## Projeto: 
Sistema de medição acoplado à rede elétrica embarcado em ESP32 utilizando FreeRTOS

## Tasks:

### vTaskVoltageSampler 
Tarefa responsável por amostrar a rede elétrica a 120Hz, atendendo o critério de Nyquist.

### vTaskVoltageProcessor
Tarefa responsável por receber a fila de amostras recebidas pela task de amostragem e calcular o valor RMS da tensão da rede elétrica, multiplicar este cálculo por um valor de escala de calibração, e inserir este valor convertido em uma fila, com o intuito de ser apresentado no display.

### vTaskVoltageCallibrate
Tarefa responsável por gerar o valor de calibração da leitura de tensão RMS, que multiplicará o valor RMS calculado.

### vTaskDisplay
Tarefa responsável por montar o display da tela, recebendo os valores medidos nos sensores e entregando ao usuário visualmente.
