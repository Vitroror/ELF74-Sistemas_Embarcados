#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <TFT_eSPI.h>

// ====== PARÂMETROS TENSÃO ======
const int pinVoltageSensor = 34; // Pino usado pra fazer a leitura do sensor de tensão
const float V_OFFSET = 1.6f; // offset do sensor medido no scope
const float NOM1    = 0.28f; // valor esperado no sensor quando alimentado com 127V
const float NOM2    = 0.37f; // valor esperado no sensor quando alimentado com 220V
float gMAX = 0.0; // tolerância máxima global
float gMIN = 0.0; // tolerância mínima global

const int CURRENT_SENSE_PIN = 35;
const float VOLTAGE_REFERENCE = 3.3;
const float SENSITIVITY = 0.04;
const float OFFSET_VOLT = VOLTAGE_REFERENCE / 2;

// TAXA DE AMOSTRAGEM
const int   SAMPLES = 120;                                   // 120 amostras -> critério de Nyquist
TickType_t SAMPLE_PERIOD = pdMS_TO_TICKS(1000UL / SAMPLES);  // periodo definido de leituras

// ====== FILA DE AMOSTRAS, LEITURAS DE TENSÃO RMS E FATOR DE CALIBRAÇÃO======
QueueHandle_t xQueueSamples;
QueueHandle_t xQueueRMSVoltage;
QueueHandle_t xQueueCurrent;
float gScaleFactor = 1.0f;  // começa em 1.0, vai ser calibrado
int gZeroFlag = 0;
float grmsSensor = 0;

// ====== OBJETO DO DISPLAY ======
TFT_eSPI tft = TFT_eSPI();

// ====== VARIAVEIS DE HANDLER DAS TASKS ======
TaskHandle_t taskCalibHandle = NULL;
TaskHandle_t taskProcessorHandle = NULL;
TaskHandle_t taskSamplerHandle = NULL;
TaskHandle_t taskCurrentSamplerHandle = NULL;
TaskHandle_t taskDisplayHandle = NULL;

// ====== PROTÓTIPOS DAS TASKS ======
void vTaskVoltageCallibrate(void *pvParameters);
void vTaskVoltageSampler(void *pvParameters);
void vTaskVoltageProcessor(void *pvParameters);
void vTaskCurrentSampler(void *pvParameters);
void vTaskDisplay(void *pvParameters);

// ====== SETUP ======
void setup() {
  Serial.begin(115200); // (debug) abre a porta serial
  analogReadResolution(12); // configura toda a resolução do ADC da ESP
  analogSetAttenuation(ADC_11db); // configura atenuação para ADC da ESP

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  // filas para amostras e leituras RMS
  xQueueSamples = xQueueCreate(SAMPLES, sizeof(float));
  xQueueRMSVoltage = xQueueCreate(1, sizeof(float));
  xQueueCurrent = xQueueCreate(10, sizeof(float));

  // por enquanto só criamos a task de calibração
  xTaskCreate(vTaskVoltageCallibrate,"Calibration", 4096, nullptr, configMAX_PRIORITIES-1, &taskCalibHandle);
  xTaskCreate(vTaskDisplay, "Display", 2048, nullptr, 1, &taskDisplayHandle);

  xTaskCreate(vTaskCurrentSampler, "CurrentSampler", 2048, nullptr, 2, &taskCurrentSamplerHandle);
}

void loop() {
  vTaskDelay(1000); /* libera a CPU por 1000 ticks */
}

void vTaskVoltageCallibrate(void* pv) {
  TickType_t xLastWake = xTaskGetTickCount(); // define quantos ticks se passaram desde que o scheduler foi chamado

  Serial.println("=== Iniciando calibração do sensor de tensão ===");

  double sumSq = 0; // a partir daqui faz-se uma amostragem e calculo RMS inicial
  for (int i = 0; i < SAMPLES; i++) {
    int raw = analogRead(pinVoltageSensor);
    float v = raw * (3.3f/4095.0f) - V_OFFSET;
    sumSq += (double)v * v;
    vTaskDelayUntil(&xLastWake, SAMPLE_PERIOD);
  }
  float grmsSensor = sqrt(sumSq / (double)SAMPLES);

  // Teste automático contra 127 V e 220 V
  if (grmsSensor >= 0.7*NOM1 && grmsSensor <= 1.3*NOM1) {
    gScaleFactor = 127 / grmsSensor;
    gMIN = 0.9*127;
    gMAX = 1.1*127;
    gZeroFlag = 0;
    Serial.println("Detectado: 127 V"); // se os valores baterem, estamos medindo 127V
  }
  else if (grmsSensor >= 0.7*NOM2 && grmsSensor <= 1.3*NOM2) {
    gScaleFactor = 220 / grmsSensor;
    gMIN = 0.9*220;
    gMAX = 1.1*220;
    gZeroFlag = 0;
    Serial.println("Detectado: 220 V"); // se os valores baterem, estamos medindo 220V
  }
  else {
    gScaleFactor = 0 / grmsSensor;
    gMIN = 0.9*0;
    gMAX = 1.1*0;
    gZeroFlag = 1;
    Serial.println("Falha na detecção; assumindo tensão nula"); // contingência - fallback para 0V
  }
  Serial.println("=== Fim da calibração ===");

  // 5) Recria as tasks de leitura e processamento
  xTaskCreate(vTaskVoltageSampler,   "Sampler",   2048, nullptr, 2, &taskSamplerHandle);
  xTaskCreate(vTaskVoltageProcessor, "Processor", 2048, nullptr, 1, &taskProcessorHandle);

  // 6) Mata a própria task de calibração
  vTaskDelete(nullptr);
}

void vTaskVoltageSampler(void *pv) {
  TickType_t xLastWake = xTaskGetTickCount();
  for (;;) {
    // lê e centraliza
    int raw = analogRead(pinVoltageSensor);
    float v = raw * (3.3f / 4095.0f) - V_OFFSET;

    // empurra na fila (descarta o mais antigo se cheia)
    if (xQueueSendToBack(xQueueSamples, &v, 0) != pdTRUE) {
      float dummy;
      xQueueReceive(xQueueSamples, &dummy, 0);
      xQueueSendToBack(xQueueSamples, &v, 0);
    }

    // se agora a fila atingiu SAMPLE_HZ, notifica o Processor
    if (uxQueueMessagesWaiting(xQueueSamples) >= SAMPLES) {
      xTaskNotifyGive(taskProcessorHandle);
    }

    // libera a CPU até a próxima amostra (~8 ms)
    vTaskDelayUntil(&xLastWake, SAMPLE_PERIOD);
  }
}

void vTaskVoltageProcessor(void *pv) {
  float buffer[SAMPLES];
  for (;;) {
    // bloqueia até o Sampler notificar
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // drena exatamente SAMPLES amostras
    int n = 0;
    while (n < SAMPLES && xQueueReceive(xQueueSamples, &buffer[n], 0) == pdTRUE) {
      n++;
    }

    // soma dos quadrados
    double sumSq = 0;
    for (int i = 0; i < n; i++) {
      sumSq += (double)buffer[i] * buffer[i];
    }

    // finaliza calculo RMS e aplica fator de calibração
    grmsSensor = sqrt(sumSq / (double)n);
    float rmsRede   = grmsSensor * gScaleFactor;

    // envia a leitura sem bloqueio para a fila de interação com display
    xQueueOverwrite(xQueueRMSVoltage, &rmsRede);

    // (debug) imprime valor de leitura
    Serial.print("RMS rede: ");
    Serial.println(rmsRede, 4);
    Serial.println(grmsSensor, 4);

    // handler de calibração caso leitura se torne inconsistente
    if(gZeroFlag == 0)
    {
      if (rmsRede < gMIN || rmsRede > gMAX)
      {
        Serial.println("=== Necessário Calibração ===");
        xTaskCreate(vTaskVoltageCallibrate,"Calibration", 4096, nullptr, configMAX_PRIORITIES-1, &taskCalibHandle);
        vTaskDelete(taskSamplerHandle);
        vTaskDelete(NULL);
      }
    } else{
      if (grmsSensor > 0.15)
      {
        Serial.println("=== Necessário Calibração ===");
        xTaskCreate(vTaskVoltageCallibrate,"Calibration", 4096, nullptr, configMAX_PRIORITIES-1, &taskCalibHandle);
        vTaskDelete(taskSamplerHandle);
        vTaskDelete(NULL);
      }
    }
  }
}

// ====== TASK DE CORRENTE ======
void vTaskCurrentSampler(void *pv) {
  TickType_t xLastWake = xTaskGetTickCount();
  for (;;) {
    int raw_adc = analogRead(CURRENT_SENSE_PIN);

    float voltage = raw_adc * (VOLTAGE_REFERENCE / 4095.0);

    float current = (voltage - OFFSET_VOLT) / SENSITIVITY;

    if (xQueueSendToBack(xQueueCurrent, &current, 0) != pdTRUE) {
      float dummy;
      xQueueReceive(xQueueCurrent, &dummy, 0);
      xQueueSendToBack(xQueueCurrent, &current, 0);
    }

    Serial.print("Corrente: ");
    Serial.println(current, 4);

    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(100));
  }
}

// Função para desenhar seta de tendência com cor customizada
void setaTendencia(int16_t x, int16_t y, float current_val, float previous_val, float threshold, uint16_t cor) {
  tft.fillRect(x - 6, y - 1, 12, 12, TFT_BLACK);
  if (current_val > previous_val + threshold) {
    // Seta para cima (usando cor fornecida)
    tft.fillTriangle(x, y, x-5, y+10, x+5, y+10, cor);
  } else if (current_val < previous_val - threshold) {
    // Seta para baixo (usando cor fornecida)
    tft.fillTriangle(x, y+10, x-5, y, x+5, y, cor);
  } else {
    // Valor estável (círculo amarelo - mantém amarelo para indicar estabilidade)
    tft.fillCircle(x, y+5, 3, cor);
  }
}

void vTaskDisplay(void *pv){
  float rmsRecebido = 0.0, correnteRecebido = 0.0;
  float rmsPrev = 0.0, correntePrev = 0.0, potenciaPrev = 0.0, potenciaCalc = 0.0;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_MAROON);
  tft.setCursor(70, 0);
  tft.print("Corrente RMS:");
  tft.drawLine(10,20,20,20,TFT_MAROON);
  tft.drawCircle(30,20,10,TFT_MAROON);
  tft.drawLine(40,20,50,20,TFT_MAROON);
  tft.setCursor(26,13);
  tft.print("A");

  tft.setTextColor(TFT_BLUE);
  tft.setCursor(70, 45);
  tft.print("Tensao RMS:");
  tft.drawLine(10,65,20,65,TFT_BLUE);
  tft.drawCircle(30,65,10,TFT_BLUE);
  tft.drawLine(40,65,50,65,TFT_BLUE);
  tft.setCursor(26,58);
  tft.print("V");

  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(70, 90);
  tft.print("Potencia:");
  tft.drawLine(10,110,20,110,TFT_YELLOW);
  tft.drawCircle(30,110,10,TFT_YELLOW);
  tft.drawLine(40,110,50,110,TFT_YELLOW);
  tft.setCursor(26,103);
  tft.print("W");

  while (1)
  {
    potenciaPrev = potenciaCalc;
    rmsPrev = rmsRecebido;
    correntePrev = correnteRecebido;

    if (xQueueReceive(xQueueRMSVoltage, &rmsRecebido, pdMS_TO_TICKS(100))){
      potenciaCalc = rmsRecebido * correnteRecebido;

      // Escrever o valor da tensao
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
      tft.setCursor(90, 65);
      tft.print(rmsRecebido, 2);
      tft.print(" V ");
      setaTendencia(215, 67, rmsRecebido, rmsPrev, 1.0, TFT_BLUE);

      // Escrever o valor da potencia
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.setCursor(90, 110);
      tft.print(potenciaCalc, 2);
      tft.print(" W ");
      setaTendencia(215, 112, potenciaCalc, potenciaPrev, 1.0, TFT_YELLOW);
    }

    if (xQueueReceive(xQueueCurrent, &correnteRecebido, pdMS_TO_TICKS(100))){
      potenciaCalc = rmsRecebido * correnteRecebido;

      // Escrever o valor da corrente
      tft.setTextColor(TFT_MAROON, TFT_BLACK);
      tft.setCursor(90, 20);
      tft.print(correnteRecebido, 2);
      tft.print(" A ");
      setaTendencia(215, 22, correnteRecebido, correntePrev, 0.1, TFT_MAROON);

      // Escrever o valor da potencia
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.setCursor(90, 110);
      tft.print(potenciaCalc, 2);
      tft.print(" W ");
      setaTendencia(215, 112, potenciaCalc, potenciaPrev, 1.0, TFT_YELLOW);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
