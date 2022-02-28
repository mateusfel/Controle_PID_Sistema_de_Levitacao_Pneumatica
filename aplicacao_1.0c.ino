//###############################################################
//# Este código implementa a derivadada da saída e não do erro, #
//# evitando assim o estouro da derivada caso o setpoint mude   #
//# bruscamente                                                 #
//###############################################################

#include <Wire.h>           // VL53L0X depende da biblioteca Wire para funcionar e se comunicar com o sensor
#include <VL53L0X.h>        // Biblioteca do sensor utilizado -  A comunicação é via I²C e a taxa de amostragem é próxima de 30 ms
#include <ArduinoRS485.h>   // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>  // Biblioteca oficial de criação de registradores e comunicação MODBUS

#include "modbus_addresses.h" // Nesse cabeçalho encontram-se todas as definições dos endereços de memória MODBUS
#include "variables.h"        // Nesse cabeçalho estão declaradas todas as variáveis utilizadas nesse programa


// Declara o objeto sensor
VL53L0X sensor;

void setup() {
  // Inicia a interface serial com 115200 bauds
  //Serial.begin(baudRate);

  // Incia a interface I²C
  Wire.begin();

  // Configura o Timeout do sensor para 500 ms
  sensor.setTimeout(500);

  // Inicia o sensor e informa se falhou
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}  // Se falhou, entra em um loop infinito e não faz nada
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();

  // start the Modbus RTU server, with (slave) id 1
  if (!ModbusRTUServer.begin(1, baudRate)) {
    Serial.println("Failed to start Modbus RTU Server!");
    while (1);
  }

  // Configura os Registradores Holdings
  ModbusRTUServer.configureHoldingRegisters(0, 30);

  // Configura os pinos dos PWMs como saída
  pinMode(pwmPinL, OUTPUT);
  pinMode(pwmPinR, OUTPUT);
  pinMode(enPinL, OUTPUT);
  pinMode(enPinR, OUTPUT);

  ModbusRTUServer.holdingRegisterWrite(BITS_CONTROLE, 0);
}

void loop() {
  // Poll for Modbus RTU requests
  ModbusRTUServer.poll();

  // Faz a leitura da referência
  referencia.raw.a = ModbusRTUServer.holdingRegisterRead(REFERENCIA_MW1);
  referencia.raw.b = ModbusRTUServer.holdingRegisterRead(REFERENCIA_MW2);

  // Retorna o valor de distância do sensor em mm
  int distancia = sensor.readRangeContinuousMillimeters();

  // Mapreia a distância lida em uma altura que varia de 0 a 1000 mmm
  altura.f = map(distancia, 1014, 65, 0, 1000);

  // Escreve a altura nos Registradores Holding
  ModbusRTUServer.holdingRegisterWrite(ALTURA_MW1, altura.raw.a);
  ModbusRTUServer.holdingRegisterWrite(ALTURA_MW2, altura.raw.b);

  // Erro recebe a diferença entre a referência (valor desejado) e o lido pelo sensor (altura)
  erro.f = referencia.f - altura.f;

  // Escreve o erro nos Registradores Holding
  ModbusRTUServer.holdingRegisterWrite(ERRO_MW1, erro.raw.a);
  ModbusRTUServer.holdingRegisterWrite(ERRO_MW2, erro.raw.b);

  // Verifica se o equipamento foi realmente ligado e só realiza as operações de controle se o bit na posição BIT_LIGA_DESLIGA estiver em 1
  bits_de_controle = ModbusRTUServer.holdingRegisterRead(BITS_CONTROLE);
  if(bitRead(bits_de_controle, BIT_LIGA_DESLIGA)) {

    // Integração pelo método dos trapésios. A integral recebe a área já existente somada ao valor do erro multiplicado pelo intervalo de tempo de amostragem
    integral.f = integral.f + ki.f * ((erro.f + erro_anterior)/2) * dt.f * 10E-6;
    //integral.f = 255;

    // Valor para evitar o reset windup com base no valor de ki
    if(ki.f > 0){
      limite_i = 255 / ki.f; 
    } else {
      limite_i = 0;
    }
    

    // Limita o tamanho da integral. Se ela for maior que um limite estabelecido
    integral.f = constrain(integral.f, -limite_i, limite_i);

    // Escreve a integral nos Registradores Holding
    ModbusRTUServer.holdingRegisterWrite(INTEGRAL_MW1, integral.raw.a);
    ModbusRTUServer.holdingRegisterWrite(INTEGRAL_MW2, integral.raw.b);

    // Verifica o quanto o erro variou e divide pelo intervalo de tempo, obtendo assim a taxa de variação
    derivada.f = kd.f * ((saida.f - saida_anterior_1) / (dt.f * 10E-6));
    saida_anterior_1 = saida.f;
    //derivada.f = 255;

    // Escreve a derivada nos Registradores Holding
    ModbusRTUServer.holdingRegisterWrite(DERIVADA_MW1, derivada.raw.a);
    ModbusRTUServer.holdingRegisterWrite(DERIVADA_MW2, derivada.raw.b);

    // Faz a leitura da compensacao gravada no Registrador Holding
    compensacao = ModbusRTUServer.holdingRegisterRead(COMPENSACAO_MW1);

    // Faz a leitura das constantes gravadas nos Registradores Holding
    kp.raw.a = ModbusRTUServer.holdingRegisterRead(KP_MW1);
    kp.raw.b = ModbusRTUServer.holdingRegisterRead(KP_MW2);
    ki.raw.a = ModbusRTUServer.holdingRegisterRead(KI_MW1);
    ki.raw.b = ModbusRTUServer.holdingRegisterRead(KI_MW2);
    kd.raw.a = ModbusRTUServer.holdingRegisterRead(KD_MW1);
    kd.raw.b = ModbusRTUServer.holdingRegisterRead(KD_MW2);

    // A saída (potência a ser aplicada ao motor entre -255 e 255 do PWM) recebe os cálculo da equação de PID
    // Lembrando que a compensação é um valor de PWM em que o fluxo de ar teoricamente deixaria a bola em uma posição de repouso
    saida.f = kp.f * erro.f + integral.f + derivada.f + compensacao;

    // A variável recebe o valor do limite excedido caso tenha excedido algum deles
    saida.f = constrain(saida.f, limite_inf_saida, limite_sup_saida);

    // Escreve a saida no Registrador Holding
    ModbusRTUServer.holdingRegisterWrite(SAIDA_MW1, saida.raw.a);
    ModbusRTUServer.holdingRegisterWrite(SAIDA_MW2, saida.raw.b);

    // Escreve na saída PWM o valor da equação do controlador
    if (saida.f >= 0) {                   // Caso a saída seja positiva, o motor gira em um sentido empurrando a bola para cima
      analogWrite(pwmPinL, saida.f);
      analogWrite(pwmPinR, 0);
      digitalWrite(enPinL, HIGH);
      digitalWrite(enPinR, HIGH);
    } else if (saida.f < 0){              // Caso a saída seja negativa, o motor gira em um sentido contrário puxando a bola para baixo
      analogWrite(pwmPinL, 0);
      analogWrite(pwmPinR, abs(saida.f));
      digitalWrite(enPinL, HIGH);
      digitalWrite(enPinR, HIGH);
    }

  // Caso o equipamento esteja desligado, desliga a saída e zera todas as variáveis que fazem parte do controle
  } else {
    // Desliga a saída
    saida.f = 0;

    analogWrite(pwmPinL, saida.f);
    analogWrite(pwmPinR, saida.f);
    digitalWrite(enPinL, LOW);
    digitalWrite(enPinR, LOW);

    // Escreve a saida no Registrador Holding
    ModbusRTUServer.holdingRegisterWrite(SAIDA_MW1, saida.raw.a);
    ModbusRTUServer.holdingRegisterWrite(SAIDA_MW2, saida.raw.b);

    // Zera a Integral
    integral.f = 0;

    // Escreve a integral nos Registradores Holding
    ModbusRTUServer.holdingRegisterWrite(INTEGRAL_MW1, integral.raw.a);
    ModbusRTUServer.holdingRegisterWrite(INTEGRAL_MW2, integral.raw.b);

    // Zera a derivada
    derivada.f = 0;

    // Escreve a derivada nos Registradores Holding
    ModbusRTUServer.holdingRegisterWrite(DERIVADA_MW1, derivada.raw.a);
    ModbusRTUServer.holdingRegisterWrite(DERIVADA_MW2, derivada.raw.b);
  }

  // Armazena o erro anterior
  erro_anterior = erro.f;

  // Calcula o tempo de amostragem
  t[0] = micros();    // Pega tempo atual do microcontrolador em microssegundos
  dt.f = t[0] - t[1]; // Tempo atual menos tempo anterior = tempo de amostragem (em microssegundos)
  t[1] = t[0];        // Tempo atual passa a ser o tempo anterior após o cálculo do tempo de amostragem

  // Armazena o tempo de amostragem nos Registradores Holding
  ModbusRTUServer.holdingRegisterWrite(DT_MW1, dt.raw.a);
  ModbusRTUServer.holdingRegisterWrite(DT_MW2, dt.raw.b);

  // Armazena o timestamp atual no registrador
  timestamp.l = t[0];
  ModbusRTUServer.holdingRegisterWrite(MICROS_MW1, timestamp.raw.b);
  ModbusRTUServer.holdingRegisterWrite(MICROS_MW2, timestamp.raw.a);
}
