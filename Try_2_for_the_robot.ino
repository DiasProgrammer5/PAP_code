#include <SoftwareSerial.h>   // Biblioteca para a porta de comunicação série por software


/*****************************************************************************************
 * Definição de constantes                                                               *
 *****************************************************************************************/
#define DEBUG   // Descomentar para depuração através do Monitor Série (CTRL+SHIFT+M)

// Define os pinos de ligação ao Arduino
#define EN12_PIN 5    // Onde liga Enable 1,2 do L293D, ativação do motor esquerdo
#define EN34_PIN 10   // Onde liga Enable 3,4 do L293D, ativação do motor direito
#define BAT_PIN A0    // Onde liga o divisor de tensão para medir a tensão na bateria
#define BUZ_PIN 11    // Onde liga o besouro, ou sinalizador sonoro
#define ECHO_PIN 4    // Onde liga a linha Echo (pulso de resposta) do sensor de distância por ultrasons
#define TRIG_PIN 13   // Onde liga a linha Trig (disparo) do sensor de distância por ultrasons

// Define as constantes do sensor de linha
#define LEFT_SENSOR_PIN A2 // Onde liga o sensor de linha do lado esquerdo   
#define RIGHT_SENSOR_PIN A4 // Onde liga o sensor de linha do lado direito 

// Define as constantes do sensor Sharp
#define SENSOR A1 

// Define outras constantes
#define DIST_MIN 12   // Distância miníma ao obstáculo
#define ADC_MIN 273   // Valor do ADC correspondente ao limite mínimo de alerta da tensão na bateria, 
                      //   calcular este valor tendo em atenção o divisor resistivo 
                      //        ADC_MIN = (R1 / (R1 + R2)) * Vbat_min / (5 / 1024)
                      //        ADC_MIN = (10000 / (10000 + 20000)) * 4,0 / (5 / 1024)
                      //        ADC_MIN = 273


/*****************************************************************************************
 * Definição de variáveis e outras instâncias                                            *
 *****************************************************************************************/
// Declaração de variáveis globais
long distancia;   // Para armazenar o valor da distância ao obstáculo
long tempo;       // Para armazenar o valor do tempo de resposta
float vbat;       // Para armazenar o valor da tensão da bateria

// Vetores com os pinos onde ligam os motores. O motor esquerdo liga aos pinos 6 e 7 do 
//   Arduino e o motor direito liga aos pinos 2 e 3. Ter em atenção as polaridades, ou  
//   sentido de rotação, dos motores
int motor_left[] = {6, 7};
int motor_right[] = {8, 9}; 

#define LINE_MIN 0
#define LINE_MAX 1023

// Definir a velocidade dos motores
#define MOTOR_SPEED 100

// Instância para a porta de comunicação série por software com o módulo Bluetooth (BT)
SoftwareSerial portBT(2, 3);   // RX = pino digital 2, TX = pino digital 3


/*****************************************************************************************
 *  Função de inicialização do microcontrolador, executada apenas uma vez                *
 *****************************************************************************************/
void setup() {
  #ifdef DEBUG
  // Inicia a porta de comunicação série através do cabo de programação e 
  //   aguarda que se estabeleça a ligação.
  Serial.begin(9600);
  while (!Serial) {
    ; // Aguarda pela ligação série (Necessário caso seja utilizada a placa o Arduino Leonardo)
  }
  #endif

  // Define os pinos digitais ligados aos ENABLEs do L293D como saídas
  pinMode(EN12_PIN, OUTPUT);
  pinMode(EN34_PIN, OUTPUT);
  // e coloca-as no estado lógico alto (HIGH) - motores ativos
  digitalWrite(EN12_PIN, HIGH);
  digitalWrite(EN34_PIN, HIGH);

  pinMode(TRIG_PIN, OUTPUT);   // Define o pino como saída para o disparo ultrasónico
  pinMode(ECHO_PIN, INPUT);    // Define o pino como entrada para receber o pulso de resposta
  pinMode(BAT_PIN, INPUT);     // Define o pino como entrada para leitura do nível de tensão da bateria
  pinMode(BUZ_PIN, OUTPUT);    // Define o pino como saída para ativar o besouro

  digitalWrite(BUZ_PIN, HIGH);   // Ativa o besouro
  
  pinMode(LEFT_SENSOR_PIN, INPUT);  // Define o pino como entrada para receber a leitura do sensor de linha do lado esquerdo 
  pinMode(RIGHT_SENSOR_PIN, INPUT);  // Define o pino como entrada para receber a leitura do sensor de linha do lado direito 
  
  // Define os pinos do Arduino como saídas para comando da ponte em H L293D onde ligam os motores
  int i;
  for(i = 0; i < 2; i++) {
    pinMode(motor_left[i], OUTPUT);
    pinMode(motor_right[i], OUTPUT);
  } 

  // Inicializa a porta de comunicação série com o módulo Bluetooth
  portBT.begin(9600);

  // Faz a primeira pulsação ultrasónica
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);

  delay(1000);   // Aguarda um segundo
  digitalWrite(BUZ_PIN, LOW);   // Silencia o besouro

  #ifdef DEBUG
  Serial.println("\nPronto a andar...");   // Envia mensagem para depuração
  #endif
}


/*****************************************************************************************
 *  Função principal do programa, executada continuamente                                *
 *****************************************************************************************/
void loop() {
  
  // Calcula o valor da tensão aos terminais da bateria e envia mensagem para depuração
  /*#ifdef DEBUG
  float vbat = 3 * analogRead(BAT_PIN) * (5.0 / 1024);
  Serial.print(vbat, 2); Serial.println("V");
  #endif*/
  //delay(500);
  // Verifica a tensão na bateria, se for inferior ao limite mínimo faz o besouro ressoar e para o robô
  if (analogRead(BAT_PIN) < ADC_MIN) {
    //digitalWrite(BUZ_PIN, HIGH);
    motors_stop();
    #ifdef DEBUG
    Serial.println("Bateria muito fraca! Recarregue-a.");   // Envia mensagem para depuração
    #endif
    while (1) {
      //digitalWrite(BUZ_PIN, HIGH);
      delay(500);
      digitalWrite(BUZ_PIN, LOW);
      delay(500);
    }
  }
  digitalWrite(TRIG_PIN, LOW);       // Prepara a pulsação ultrasónica
  delayMicroseconds(5);              // Pequena pausa de estabilização
  digitalWrite(TRIG_PIN, HIGH);      // Dispara a pulsação ultrasónica
  delayMicroseconds(10);             // Mais uma pequena pausa de estabilização
  tempo = pulseIn(ECHO_PIN, HIGH);   // Mede o tempo que decorreu entre o envio da pulsação ultrasónico e a receção do eco de resposta
  distancia = int(0.017f * tempo);   // Cálculo da distância utilizando a velocidade do som como constante
  /*#ifdef DEBUG
  Serial.print(distancia); Serial.println("cm"); // Envia mensagem para depuração
  #endif*/
  if (distancia < DIST_MIN) {   // Se a distância for inferior a estipolado desativa os motores, mantendo-os parados
    //digitalWrite(BUZ_PIN, HIGH);
     digitalWrite(EN12_PIN, LOW);
     digitalWrite(EN34_PIN, LOW);
  } else {   // Caso contrário, ativa os motores
    digitalWrite(BUZ_PIN, LOW);
     digitalWrite(EN12_PIN, HIGH);
     digitalWrite(EN34_PIN, HIGH);
  }

  float volts = analogRead(SENSOR)*0.0048828125;  // value from sensor * (5/1024)
  int distance = 13*pow(volts, -1); // worked out from datasheet graph
  delay(1000); // slow down serial port 
  
  if (distance <= 30){
    Serial.println(distance);   // print the distance
  }


  int leftSensor = analogRead(LEFT_SENSOR_PIN);
  int rightSensor = analogRead(RIGHT_SENSOR_PIN);

  if (leftSensor > LINE_MAX / 2 && rightSensor > LINE_MAX / 2) {
    drive_forward();
  } else {
    if (leftSensor > LINE_MAX / 2 && rightSensor < LINE_MAX / 2) {
      spin_left();
    } else {
      if (leftSensor < LINE_MAX / 2 && rightSensor > LINE_MAX / 2) {
        spin_right();
       } else {
         motors_stop();    
      }
    }
  }

  analogWrite(motor_left[1], MOTOR_SPEED);
  analogWrite(motor_right[1], MOTOR_SPEED);


  /*#ifdef DEBUG
  Serial.print(digitalRead(LEFT_SENSOR_PIN)); Serial.println(" left"); 
  Serial.print(digitalRead(RIGHT_SENSOR_PIN)); Serial.println(" right");
   #endif*/
  // Verifica se foram recebidos caráteres de comando válidos através do 
  //   módulo bluetooth e processa-os
  /*if (portBT.available() > 0) {
    char inByte = portBT.read();
    #ifdef DEBUG
    Serial.write(inByte); Serial.println();
    #endif
    if (inByte == 'w') {                 // Caráter 'w', ASCII 119 
      drive_forward();
    } else {
      if (inByte == 'x') {               // Caráter 'x', ASCII 120
        drive_backward();
      } else {
        if (inByte == 'q') {             // Caráter 'q', ASCII 113
          turn_left();
        } else {
          if (inByte == 'e') {           // Caráter 'e', ASCII 101
            turn_right();
          } else {
            if (inByte == 'a') {         // Caráter 'a', ASCII 97
              spin_left();
            } else {
              if (inByte == 'd') {       // Caráter 'd', ASCII 100
                spin_right();
              } else {
                if (inByte == 'z') {     // Caráter 'z', ASCII 122
                  back_left();
                } else {
                  if (inByte == 'c') {   // Caráter 'c', ASCII 99
                    back_right();
                  } else {               // Caso do caráter 's' (ASCII 115) ou caráter não válido
                    motors_stop();       // Pára os motores
                  }
                }
              }
            }
          }
        }   
      }
    }
  }*/
  
}


/*****************************************************************************************
 *  Implementação de outras funções                                                      *
 *****************************************************************************************/

// Pára os motores do robô
void motors_stop(){
  // Motor esquerdo parado
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], LOW);
  // Motor direito parado
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], LOW);
}

// Faz o robô seguir em frente
void drive_forward(){
  // Motor esquerdo no sentido antihorário
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], HIGH);
  // Motor direito no sentido horário
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], HIGH);
}

// Faz o robô recuar
void drive_backward(){
  // Motor esquerdo no sentido horário
  digitalWrite(motor_left[0], HIGH);
  digitalWrite(motor_left[1], LOW);
  // Motor direito no sentido antihorário
  digitalWrite(motor_right[0], HIGH);
  digitalWrite(motor_right[1], LOW);
}

// Faz o robô girar à esquerda
void spin_left(){
  // Motor esquerdo no sentido horário
  digitalWrite(motor_left[0], HIGH);
  digitalWrite(motor_left[1], LOW);
  // Motor direito no sentido horário
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], HIGH);
}

// Faz o robô girar à direita
void spin_right(){
  // Motor esquerdo no sentido antihorário
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], HIGH);
  // Motor direito no sentido antihorário
  digitalWrite(motor_right[0], HIGH);
  digitalWrite(motor_right[1], LOW);
}

// Faz o robô virar à esquerda
void turn_left(){
  // Motor esquerdo parado
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], LOW);
  // Motor direito no sentido horário
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], HIGH);
}

// Faz o robô girar à direita
void turn_right(){
  // Motor esquerdo no sentido antihorário
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], HIGH);
  // Motor direito parado
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], LOW);
} 

// Faz o robô recuar à esquerda
void back_left(){
  // Motor esquerdo parado
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], LOW);
  // Motor direito no sentido antihorário
  digitalWrite(motor_right[0], HIGH);
  digitalWrite(motor_right[1], LOW);
}

// Faz o robô recuar à direita
void back_right(){
  // Motor esquerdo no sentido horário
  digitalWrite(motor_left[0], HIGH);
  digitalWrite(motor_left[1], LOW);
  // Motor direito parado
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], LOW);
}
