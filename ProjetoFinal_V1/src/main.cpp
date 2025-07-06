#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <TFT_eSPI.h>

// ====== PARÂMETROS ======
const int pinVoltageSensor = 34;       // Pino usado pra fazer a leitura do sensor de tensão
const float V_OFFSET = 1.6f;    // offset do sensor medido no scope 
const float V_INPUT  = 129.4f;  // referência, tensãod de rede definida no multimetro

// ====== TOLERÂNCIA DE MEDIDA ======
const float V_INPUT_MAX_ERROR = V_INPUT + 0.1*V_INPUT;
const float V_INPUT_MIN_ERROR = V_INPUT - 0.1*V_INPUT;

// TAXA DE AMOSTRAGEM
const int   SAMPLES = 120;                                   // 120 amostras -> critério de Nyquist
TickType_t SAMPLE_PERIOD = pdMS_TO_TICKS(1000UL / SAMPLES);  // periodo definido de leituras

// ====== FILA DE AMOSTRAS, LEITURAS DE TENSÃO RMS E FATOR DE CALIBRAÇÃO======
QueueHandle_t xQueueSamples;
QueueHandle_t xQueueRMSVoltage;
float gScaleFactor = 1.0f;  // começa em 1.0, vai ser calibrado

// ====== OBJETO DO DISPLAY ======
TFT_eSPI tft = TFT_eSPI();

// ====== VARIAVEIS DE HANDLER DAS TASKS ======
TaskHandle_t taskCalibHandle = NULL;
TaskHandle_t taskProcessorHandle = NULL;
TaskHandle_t taskSamplerHandle = NULL;
TaskHandle_t taskDisplayHandle = NULL;

// ====== PROTÓTIPOS DAS TASKS ======
void vTaskVoltageCallibrate(void *pvParameters);
void vTaskVoltageSampler(void *pvParameters);
void vTaskVoltageProcessor(void *pvParameters);
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

  // por enquanto só criamos a task de calibração
  xTaskCreate(vTaskVoltageCallibrate,"Calibration", 4096, nullptr, configMAX_PRIORITIES-1, &taskCalibHandle);
  xTaskCreate(vTaskDisplay, "Display", 2048, nullptr, 1, &taskDisplayHandle);
}

void loop() {
  vTaskDelay(1000); /* libera a CPU por 1000 ticks */
}

void vTaskVoltageCallibrate(void* pv) {
  TickType_t xLastWake = xTaskGetTickCount(); // define quantos ticks se passaram desde que o scheduler foi chamado
  double sumSq = 0;

  Serial.println("=== Iniciando Calibração do Sensor de Tensão ===");

  // faça 120 leituras espaçadas pelo período de amostragem
  for (int i = 0; i < SAMPLES; i++) {
    int raw = analogRead(pinVoltageSensor);
    float v = raw * (3.3f/4095.0f) - V_OFFSET;
    sumSq += (double)v * v;

    // libera a CPU até o próximo “tick” de amostragem
    vTaskDelayUntil(&xLastWake, SAMPLE_PERIOD);
  }

  // após 120 amostras, já terão se passado cerca de 1000 ms
  float rmsSensor   = sqrt(sumSq / (double)SAMPLES);
  gScaleFactor      = V_INPUT / rmsSensor;
  Serial.println("=== Calibração concluída ===");

  // agora que gScaleFactor está ajustado, pode-se criar as outras tasks
  xTaskCreate(vTaskVoltageSampler, "Sampler", 2048, nullptr, 2, &taskSamplerHandle);
  xTaskCreate(vTaskVoltageProcessor, "Processor", 2048, nullptr, 1, &taskProcessorHandle);

  // fim da calibração — mata a si própria
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

    // libera a CPU até a próxima amostra (~8 ms)
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
    float rmsSensor = sqrt(sumSq / (double)n);
    float rmsRede   = rmsSensor * gScaleFactor;

    // envia a leitura sem bloqueio para a fila de interação com display
    xQueueOverwrite(xQueueRMSVoltage, &rmsRede);

    //(ou, se preferir bloquear até 10 ms:)
    // xQueueSend(xQueueRMS, &rmsRede, pdMS_TO_TICKS(10));

    // (debug) imprime valor de leitura
    Serial.print("RMS rede: ");
    Serial.println(rmsRede, 4);
    
    // handler de calibração caso leitura se torne inconsistente 
    if (rmsRede < V_INPUT_MIN_ERROR || rmsRede > V_INPUT_MAX_ERROR)
    {
      Serial.println("=== Necessário Calibração ===");
      xTaskCreate(vTaskVoltageCallibrate,"Calibration", 4096, nullptr, configMAX_PRIORITIES-1, &taskCalibHandle);
      vTaskDelete(taskSamplerHandle);
      vTaskDelete(NULL);
    }
  }
}

void vTaskDisplay(void *pv){
  float rmsRecebido, correnteRecebido = 10.0;
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
    if (xQueueReceive(xQueueRMSVoltage, &rmsRecebido, pdMS_TO_TICKS(100))){
      float potencia = rmsRecebido * correnteRecebido;

      // Escrever o valor da tensao
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
      tft.setCursor(90, 65);
      tft.print(rmsRecebido, 2);
      tft.print(" V ");

      // Escrever o valor da potencia
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.setCursor(90, 110);
      tft.print(potencia, 2);
      tft.print(" W ");

      vTaskDelay(pdMS_TO_TICKS(500));
    }

    // if (xQueueReceive(xQueueRMSCurrent, &correnteRecebido, pdMS_TO_TICKS(100))){
    //   float potencia = rmsRecebido * correnteRecebido;

    //   // Escrever o valor da corrente
    //   tft.setTextColor(TFT_MAROON, TFT_BLACK);
    //   tft.setCursor(90, 20);
    //   tft.print(correnteRecebido, 2);
    //   tft.print(" A");

    //   // Escrever o valor da potencia
    //   tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    //   tft.setCursor(90, 110);
    //   tft.print(potencia, 2);
    //   tft.print(" W ");

    //   vTaskDelay(pdMS_TO_TICKS(500));
  }
  
}