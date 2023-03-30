#include "src/fuzzy_lib/fuzzy_lib.hpp"
#include "src/referencia_lib/referencia_lib.hpp"

//Pinos
#define PIN_MOTOR1 25
#define PIN_MOTOR2 33
#define PIN_ENCODER 22

//PWM
const int CANAL_PWM = 0;
const int FREQUENCIA_PWM = 1000;
const int RESOLUCAO_PWM = 10; 

//Parametros Sensor
const float MIN_PWM = 0;
const float MAX_PWM = pow(2, RESOLUCAO_PWM) - 1;
const int PULSOS_POR_VOLTA = 3;
const float COEFICIENTE_VELOCIDADE = ((1 / TA) * 60) / PULSOS_POR_VOLTA;  //(Ta[ms] * coeficienteRPM) / PulsosPorVolta
const int VELOCIDADE_MAX = 9750;

//Variaveis Globais
unsigned long PULSOS_ENCODER = 0;

//Controlador FUZZY PID

//Referencia
Referencia REFERENCIA(0, VELOCIDADE_MAX, onda_senoidal);

//Funcoes
void IRAM_ATTR interrupcao_encoder() {PULSOS_ENCODER++;}
float captura_velocidade();
void imprimi_informacoes(float velocidadeAtual, float sinalControle, float referencia, bool monitorSerial);

void setup() {
  //SERIAL
  Serial.begin(115200);

  //PINOS
  pinMode(PIN_ENCODER, INPUT_PULLUP);
  pinMode(PIN_MOTOR2, OUTPUT);
  digitalWrite(PIN_MOTOR2, LOW);

  //PWM
  ledcAttachPin(PIN_MOTOR1, CANAL_PWM);
  ledcSetup(CANAL_PWM, FREQUENCIA_PWM, RESOLUCAO_PWM);
  ledcWrite(CANAL_PWM, MIN_PWM);

  // //INTERRUPCAO
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), interrupcao_encoder, FALLING);

  //CONTROLADOR

  //REFERENCIA
  REFERENCIA.velocidade = 0.05;
  
  delay(1000);
}

void loop() {
  float velocidadeAtual = captura_velocidade();

  float sinalControle = CC.Calcula_Acao_Controle(REFERENCIA.modulo, velocidadeAtual);

  ledcWrite(CANAL_PWM, MAX_PWM - sinalControle);

  imprimi_informacoes(velocidadeAtual, sinalControle, REFERENCIA.modulo, false);

  REFERENCIA.atualiza_referencia();
}

float captura_velocidade() {
  PULSOS_ENCODER = 0; //reseta contador
  delay(TA_S);  //conta TA_S de pulsos

  return (COEFICIENTE_VELOCIDADE * PULSOS_ENCODER);
}

void imprimi_informacoes(float velocidadeAtual, float sinalControle, float referencia, bool monitorSerial) {
  if (monitorSerial)
      Serial.println("Referencia: " + String(referencia) + " RPM | Atual: " + String(velocidadeAtual) + " RPM | Erro: " + String(referencia - velocidadeAtual) + " RPM | Sinal de controle: " + String(sinalControle) + "bit");
  else {
      Serial.print("Max ");
      Serial.print(VELOCIDADE_MAX);
      Serial.print(" Min ");
      Serial.print(0);
      Serial.print(" Referenncia ");
      Serial.print(referencia);
      Serial.print(" Atual ");
      Serial.println(velocidadeAtual);
    }
}