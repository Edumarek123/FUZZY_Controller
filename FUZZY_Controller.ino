#include "src/fuzzy_lib/fuzzy_lib.hpp"
#include "src/referencia_lib/referencia_lib.hpp"

// Pinos
#define PIN_MOTOR1 25
#define PIN_MOTOR2 33
#define PIN_ENCODER 22

// PWM
const int CANAL_PWM = 0;
const int FREQUENCIA_PWM = 1000;
const int RESOLUCAO_PWM = 10;

// Parametros Sensor
const float TA = 0.08;
const int TA_MS = 80;
const float MIN_PWM = 0;
const float MAX_PWM = pow(2, RESOLUCAO_PWM) - 1;
const int PULSOS_POR_VOLTA = 3;
const float COEFICIENTE_VELOCIDADE = ((1 / TA) * 60) / PULSOS_POR_VOLTA; //(Ta[ms] * coeficienteRPM) / PulsosPorVolta
const int VELOCIDADE_MAX = 9750;

// Variaveis Globais
unsigned long PULSOS_ENCODER = 0;

// Controlador FUZZY
ControladorFuzzy CC(MIN_PWM, MAX_PWM);

// Referencia
Referencia REFERENCIA(0, VELOCIDADE_MAX, onda_senoidal);

// Funcoes
void IRAM_ATTR interrupcao_encoder() {PULSOS_ENCODER++;}
float captura_velocidade();
void imprimi_informacoes(float velocidadeAtual, float sinalControle, float referencia, bool monitorSerial);

void setup()
{
  // SERIAL
  Serial.begin(115200);

  // PINOS
  pinMode(PIN_ENCODER, INPUT_PULLUP);
  pinMode(PIN_MOTOR2, OUTPUT);
  digitalWrite(PIN_MOTOR2, LOW);

  // PWM
  ledcAttachPin(PIN_MOTOR1, CANAL_PWM);
  ledcSetup(CANAL_PWM, FREQUENCIA_PWM, RESOLUCAO_PWM);
  ledcWrite(CANAL_PWM, MAX_PWM - MIN_PWM);

  // //INTERRUPCAO
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), interrupcao_encoder, FALLING);

  // CONTROLADOR
  CC.centrosEk = new _ponto[N_Regras]{-4000, -2000, 0, 2000, 4000};
  CC.centrosDEk = new _ponto[N_Regras]{-400, -200, 0, 200, 400};
  CC.centrosDU = new _ponto[N_Regras]{-40, -20, 0, 20, 40};
  CC.baseEk = new _ponto[N_Regras]{3000, 3000, 3000, 3000, 3000};
  CC.baseDEk = new _ponto[N_Regras]{300, 300, 300, 300, 300};

  // REFERENCIA
  REFERENCIA.velocidade = 0.05;

  delay(1000);
}

void loop()
{
  float velocidadeAtual = captura_velocidade();

  float sinalControle = CC.Calcula_Acao_Controle(REFERENCIA.modulo, velocidadeAtual);
  ledcWrite(CANAL_PWM, MAX_PWM - sinalControle);

  imprimi_informacoes(velocidadeAtual, sinalControle, REFERENCIA.modulo, false);
  
  REFERENCIA.atualiza_referencia();
}

float captura_velocidade()
{
  PULSOS_ENCODER = 0; // reseta contador
  delay(80);        // conta TA_S de pulsos
  return (COEFICIENTE_VELOCIDADE * PULSOS_ENCODER);
}

void imprimi_informacoes(float velocidadeAtual, float sinalControle, float referencia, bool monitorSerial)
{
  if (monitorSerial)
    Serial.println("Referencia: " + String(referencia) + " RPM | Atual: " + String(velocidadeAtual) + " RPM | Erro: " + String(referencia - velocidadeAtual) + " RPM | Sinal de controle: " + String(sinalControle) + "bit");
  else
  {
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