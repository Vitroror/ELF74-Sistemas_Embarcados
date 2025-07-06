#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

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

// ====== VARIAVEIS DE HANDLER DAS TASKS ======
TaskHandle_t taskCalibHandle = NULL;
TaskHandle_t taskProcessorHandle = NULL;
TaskHandle_t taskSamplerHandle = NULL;

// ====== PROTÓTIPOS DAS TASKS ======
void vTaskVoltageCallibrate(void *pvParameters);
void vTaskVoltageSampler(void *pvParameters);
void vTaskVoltageProcessor(void *pvParameters);

// ====== SETUP ======
void setup() {
  Serial.begin(115200); // (debug) abre a porta serial
  analogReadResolution(12); // configura toda a resolução do ADC da ESP
  analogSetAttenuation(ADC_11db); // configura atenuação para ADC da ESP

  // filas para amostras e leituras RMS
  xQueueSamples = xQueueCreate(SAMPLES, sizeof(float));
  xQueueRMSVoltage = xQueueCreate(1, sizeof(float));

  // por enquanto só criamos a task de calibração
  xTaskCreate(vTaskVoltageCallibrate,"Calibration", 4096, nullptr, configMAX_PRIORITIES-1, &taskCalibHandle);
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