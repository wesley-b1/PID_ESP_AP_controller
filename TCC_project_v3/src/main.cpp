//*********************************************************************************************************************** */
//****************************************************INCLUSÃO DAS BIBLIOTECAS****************************************************** */
//*********************************************************************************************************************** */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include "SPIFFS.h"       // Gerenciamento de arquivos
#include <Arduino_JSON.h> // Para envio ao websocket (atualizar gráfico)
// INCLUINDO BIBLIOTECA DO DISPLAY OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// --- incluir a fonte que deseja usar ---
#include <Fonts/FreeSerif12pt7b.h> // <--- necessário para FreeSerif12pt7b

//*********************************************************************************************************************** */
// ============================
// CONFIGURAÇÕES DO DISPLAY
// ============================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C // Endereço I2C padrão
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Configurando o Wi-Fi e WebSocket
WebSocketsServer websocketserver = WebSocketsServer(8080);
WebServer webserver(80);

// Fila para receber a PV lida
QueueHandle_t pvFila;
// Semáforo para proteger acesso a variável y
SemaphoreHandle_t pvSemaforo;

// MACRO para LOG's
#define LOGI(tag, msg) Serial.printf("[INFO][%s] %s\n", tag, msg)
#define LOGW(tag, msg) Serial.printf("[WARN][%s] %s\n", tag, msg)
#define LOGE(tag, msg) Serial.printf("[ERROR][%s] %s\n", tag, msg)
//**************************************************************
// ******************VARIÁVEIS GLOBAIS***************************
//*********************************************************** */
// DEFINIÇÕES P/ AMOSTRAGEM E INÍCIO DO PID
static const TickType_t samplingInterval = 10 / portTICK_PERIOD_MS;      // 50ms ou 0.05s
static const TickType_t timeToStartInterval = 5000 / portTICK_PERIOD_MS; // 5s
static const uint16_t RESOLUTION = 4096;
// TASKS
TaskHandle_t serialTaskHandle = NULL;
TaskHandle_t iniciarTimersTaskHandle = NULL;
// TIMERS HANDLE
static TimerHandle_t muxConfigTimer = NULL;        // Seleciona planta via multiplexador
static TimerHandle_t getSensorReadingTimer = NULL; // Executa computeU() em malha fechada
static TimerHandle_t stepInputStartTimer = NULL;   // Aplica degrau em malha aberta
// True malha fechada e false para malha aberta
// Controle da definição do tipo de controle
volatile bool usarPID = true; // variável normal, controlada em tempo de execução // true para (PID) ou false (Degrau)
// Pinos do MUX (CD4052)
static const uint8_t A_PIN = 14;
static const uint8_t B_PIN = 27;
volatile float VCC = 3.3;
volatile uint16_t sensorReadingInt;
volatile float sensorReadingVoltage;
// Entradas analógicas
static const uint8_t SENSOR_X = 35; // plantas 1 a 4
static const uint8_t SENSOR_Y = 34; // plantas 5 e 6
// Saídas para atuar a planta
static const uint8_t MV_PIN_1 = 25;
static const uint8_t MV_PIN_2 = 26;
// variáveis de controle
bool iniciarExperimento = false;
bool experimentoIniciado = false;
// VARIÁVEIS PID
unsigned long lastTime = 0;
double u = 0, y = 0;
double sp = 0.75 * RESOLUTION; //  0.75 * 4096 = 3072
double iTerm = 0, lastY = 0;
double kp = 0.0, ki = 0.0, kd = 0.0; // inicialização
double uMin = 0, uMax = 255;
unsigned long sampleTime = samplingInterval * portTICK_PERIOD_MS;
// Seletor de planta (1 a 7)
// 1° ordem [1 ao 4]
// 2° ordem [5 ao 7]
int plantaSelecionada = 4;
// Para saber se os ganhos vieram do usuário.
bool ganhosFornecidosViaWeb = false;
//********************************************************************************************************

//=========================================================================================================
// FUNÇÕES PID
void computeU();
void setTunings(double kP, double kI, double kD);
void setControlLimits(double min, double max);
// CALLBACKS
void getSensorReadingCallback(TimerHandle_t xTimer);
void setStepInputReadingCallback(TimerHandle_t xTimer);
void setMUXCallback(TimerHandle_t xTimer);
// CONFIGURAÇÕES
void configurarPinos();
void configurarMUX();
void configurarMalhaFechada();
void configurarMalhaAberta();
// Parar todos os timers e resetar sistema:
void pararTimers();
// Task para ler a serial:
void serialTask(void *pvParameters);
// Função para iniciar os timers quando autorizado:
void iniciarTimersSeNecessario(void *pvParameters);
//=========================================================================================================

// *********************************CONEXÃO WEB E ADICIONANDO ARQUIVOS******************************************************
void initWebSocket();
// Criando o callback do WebSocket
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
// Inicialização do WIFI
void initWiFi();
// Inicialização do SPIFFS
void initSPIFFS();
// Configurando rotas do WebServer
void initWebServer();

//************INICIALIZA O DISPLAY **************/
// ============================
// DECLARAÇÕES DE FUNÇÕES
// ============================

void inicializaDisplay();
void limpaDisplay();
void exibeMensagemInicial(IPAddress ip);

void setup()
{
  Serial.begin(115200);
  inicializaDisplay();
  initWiFi();
  initSPIFFS();
  initWebServer();
  initWebSocket();
  configurarPinos();

  Serial.println("Digite 'start' para iniciar o experimento.");
  // Criação das Handles de proteção
  pvFila = xQueueCreate(10, sizeof(float)); // Até 10 leituras de PV
  pvSemaforo = xSemaphoreCreateMutex();     // Proteção para a variável y
  // Criação das tasks
  xTaskCreate(serialTask, "serialTask", 4096, NULL, 1, &serialTaskHandle);

  // iniciarTimerSeNecessario precisa ser NULL na serialTask
  // xTaskCreate(iniciarTimersSeNecessario, "iniciarTimersTask", 4096, NULL, 1, &iniciarTimersTaskHandle);
}

void loop()
{
  websocketserver.loop();
  webserver.handleClient();
  // vTaskSuspend(NULL); // Tudo controlado por timers
}

// Inicializa o display com endereço I2C 0x3C (padrão da maioria dos módulos)
void inicializaDisplay()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS))
  {
    Serial.println(F("Falha ao iniciar display SSD1306!"));
    for (;;)
      ; // trava o programa
  }
  limpaDisplay(); // Limpa o display
  Serial.println("Display inicializado com sucesso!");
}

// Limpa o display completamente
void limpaDisplay()
{
  display.clearDisplay();
  display.display();
}

// Mostra mensagem de boas-vindas
void exibeMensagemInicial(IPAddress ip)
{

  limpaDisplay();
  display.setFont(&FreeSerif12pt7b); // Muda a fonte
  display.setTextSize(0.1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(1, 15); //(coluna, linha): da linha 0 a 15 é amarelo
  display.print("IP:");
  display.setCursor(1, 45);
  display.setTextSize(0.05);
  display.print(ip);
  // display.setFont(); // Retorna para a fonte normal
  display.display();
  // delay(2000);
  // limpaDisplay();
}

void initWiFi()
{
  const char *ssid = "ESP32_AP";
  const char *password = "12345678"; // mínimo 8 caracteres
  // const char *ip;
  Serial.println("Iniciando Access Point...");

  // Cria a rede Wi-Fi
  WiFi.softAP(ssid, password);

  // Obtém o IP local do ESP32 na rede criada
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point iniciado! IP: ");
  Serial.println(IP);
  exibeMensagemInicial(IP);
}

void initWebSocket()
{
  websocketserver.begin();
  websocketserver.onEvent(onWebSocketEvent);
}
// Inicialização do SPIFFS:
void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("Erro ao montar SPIFFS");
  }
  else
  {
    Serial.println("SPIFFS montado com sucesso");
  }
}
// Callback do Websocket
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  if (type == WStype_CONNECTED)
  {
    IPAddress ip = websocketserver.remoteIP(num);
    Serial.printf("[WS] Cliente %u conectado: %s\n", num, ip.toString().c_str());
    websocketserver.sendTXT(num, "{\"evt\":\"connected\"}");
    // opcional: enviar status inicial
    // websocketserver.sendTXT(num, "{\"evt\":\"status\",\"pin\":0,\"state\":0}");
    return;
  }

  if (type == WStype_DISCONNECTED)
  {
    Serial.printf("[WS] Cliente %u desconectado\n", num);
    return;
  }

  if (type == WStype_TEXT)
  {
    String msg = String((char *)payload, length);
    Serial.printf("[WS] Mensagem de %u: %s\n", num, msg.c_str());
    // seu parsing existente:
    msg.trim();
    if (msg == "start")
    {
      iniciarExperimento = true;
      if (!experimentoIniciado && iniciarTimersTaskHandle == NULL)
      {
        xTaskCreate(iniciarTimersSeNecessario, "iniciarTimersTask", 4096, NULL, 1, &iniciarTimersTaskHandle);
      }
    }
    else if (msg == "stop")
    {
      pararTimers();
    }
    else if (msg.startsWith("setSp:"))
    {
      float spVolts = msg.substring(6).toFloat(); // valor entre 0 e 3.3V. Ex: 1.6V * 1000 = 1600 milivolts
      sp = spVolts * RESOLUTION / VCC;            // conversão para bits (0–4096). Ex.: sp=1600*4096/3.3
    }
    else if (msg.startsWith("setKp:"))
    {
      kp = msg.substring(6).toFloat();
      ganhosFornecidosViaWeb = true;
      setTunings(kp, ki, kd);
    }
    else if (msg.startsWith("setKi:"))
    {
      ki = msg.substring(6).toFloat();
      ganhosFornecidosViaWeb = true;
      setTunings(kp, ki, kd);
    }
    else if (msg.startsWith("setKd:"))
    {
      kd = msg.substring(6).toFloat();
      ganhosFornecidosViaWeb = true;
      setTunings(kp, ki, kd);
    }
    else if (msg.startsWith("setPlanta:"))
    {
      plantaSelecionada = msg.substring(10).toInt();
    }
    else if (msg.startsWith("setMalha:"))
    {
      String tipo = msg.substring(9);
      if (tipo == "aberta")
      {
        usarPID = false;
        Serial.println("Modo: Malha ABERTA (degrau)");
        // modoMalhaFechada = false;
      }
      else
      {
        // modoMalhaFechada = true;
        usarPID = true;
        Serial.println("Modo: Malha FECHADA (PID)");
      }
    }
    return;
  }

  // Outros tipos (binário etc.)
  Serial.printf("[WS] Evento tipo %d recebido (num=%u len=%u)\n", (int)type, num, (unsigned)length);
}

// void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
//{
//   if (type == WStype_TEXT)
//   {
//     String cmd = String((char *)payload);
//     cmd.trim();
//
//     if (cmd == "start")
//     {
//       iniciarExperimento = true;
//       if (!experimentoIniciado && iniciarTimersTaskHandle == NULL)
//       {
//         xTaskCreate(iniciarTimersSeNecessario, "iniciarTimersTask", 4096, NULL, 1, &iniciarTimersTaskHandle);
//       }
//     }
//
//     else if (cmd == "stop")
//     {
//       pararTimers(); // isso também zera o flag (deve conter: ganhosFornecidosViaWeb = false;)
//     }
//
//     else if (cmd.startsWith("setSp:"))
//     {
//       sp = cmd.substring(6).toFloat();
//     }
//
//     else if (cmd.startsWith("setKp:"))
//     {
//       kp = cmd.substring(6).toFloat();
//       ganhosFornecidosViaWeb = true;
//       setTunings(kp, ki, kd);
//     }
//
//     else if (cmd.startsWith("setKi:"))
//     {
//       ki = cmd.substring(6).toFloat();
//       ganhosFornecidosViaWeb = true;
//       setTunings(kp, ki, kd);
//     }
//
//     else if (cmd.startsWith("setKd:"))
//     {
//       kd = cmd.substring(6).toFloat();
//       ganhosFornecidosViaWeb = true;
//       setTunings(kp, ki, kd);
//     }
//
//     else if (cmd.startsWith("setPlanta:"))
//     {
//       plantaSelecionada = cmd.substring(10).toInt();
//     }
//   }
// }
//  Configurando rotas do WebServer
void initWebServer()
{
  webserver.serveStatic("/", SPIFFS, "/index.html");
  webserver.serveStatic("/chart.js", SPIFFS, "/chart.js");
  webserver.serveStatic("/style.css", SPIFFS, "/style.css");
  webserver.serveStatic("/script.js", SPIFFS, "/script.js");
  webserver.begin();
}
// FUNÇÕES PID
// Limita iTerm e u entre uMin e uMax
// Atualiza lastY e lastTime após cada cálculo
void computeU()
{
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  float receberPV; // Recebe valor da Fila

  if (xQueueReceive(pvFila, &receberPV, 0) == pdTRUE) // Descarregando a Fila
  {
    if (xSemaphoreTake(pvSemaforo, portMAX_DELAY)) // Ocupando semáforo
    {
      y = receberPV;              // Protegendo variável de processo Y (PV)
      xSemaphoreGive(pvSemaforo); // Liberando recurso
    }
  }

  if (timeChange >= sampleTime)
  {
    double e = 0, dY = 0;

    if (xSemaphoreTake(pvSemaforo, portMAX_DELAY))
    {
      e = sp - y;     // erro
      dY = y - lastY; // derivada da PV (não do erro)
      lastY = y;
      xSemaphoreGive(pvSemaforo);
    }
    iTerm += (ki * e); // termo integral
    if (iTerm > uMax)
      iTerm = uMax;
    else if (iTerm < uMin)
      iTerm = uMin;
    u = kp * e + iTerm - kd * dY; // PID completo
    if (u > uMax)
      u = uMax;
    else if (u < uMin)
      u = uMin;

    lastTime = now;
  }
}
// Define os ganhos do PID levando em conta o tempo de amostragem
void setTunings(double kP, double kI, double kD)
{
  double sampleTimeInSec = ((double)sampleTime / 1000);
  kp = kP;
  ki = kI * sampleTimeInSec;
  kd = kD / sampleTimeInSec;
}
// Define os limites da saída u e do termo integral iTerm.
void setControlLimits(double min, double max)
{
  if (min > max)
    return;
  uMin = min;
  uMax = max;
  if (u > uMax)
    u = uMax;
  else if (u < uMin)
    u = uMin;
  if (iTerm > uMax)
    iTerm = uMax;
  else if (iTerm < uMin)
    iTerm = uMin;
}

//***********************************EXECUÇÃO DO PID**************************************************** */
// CALLBACKS
void getSensorReadingCallback(TimerHandle_t xTimer)
{
  float pv;

  if (!experimentoIniciado)
    return;
  // LOGI("PID", "Executando callback");

  // LOGI("ControlePID", "Entrou no callback do sensor");
  // valor digital máximo aceito pelo DAC é de 2^8 – 1 = 255
  // significa que haverá 255 níveis entre o 0V e a tensão de alimentação na saída do DAC
  // resolução de 8 bits
  if (plantaSelecionada >= 1 && plantaSelecionada <= 4)
    sensorReadingInt = analogRead(SENSOR_X);
  else if (plantaSelecionada >= 5 && plantaSelecionada <= 7)
    sensorReadingInt = analogRead(SENSOR_Y);

  pv = sensorReadingInt;      // Variável que vai carregar a Fila
  xQueueSend(pvFila, &pv, 0); // será usado por computeU() para atualizar y
  // Atualiza o valor em volts com base na PV tratada (y)
  // Isso garante que o valor exibido na Serial esteja sincronizado com o PID
  sensorReadingVoltage = (VCC * y) / RESOLUTION;

  if (usarPID)
  {
    computeU();
    if (plantaSelecionada >= 1 && plantaSelecionada <= 4)
    {
      // valor digital máximo aceito pelo DAC é de 2^8 – 1 = 255
      // significa que haverá 255 níveis entre o 0V e a tensão de alimentação na saída do DAC
      // resolução de 8 bits
      dacWrite(MV_PIN_1, (int)u); // enviando U pelo DAC1 GPIO25
    }
    else if (plantaSelecionada >= 5 && plantaSelecionada <= 7)
    {
      dacWrite(MV_PIN_2, (int)u); // enviando U pelo DAC2 GPIO26
    }
  }
  // PLOTANDO VIA SERIAL
  Serial.print(xTaskGetTickCount() / 1000.0, 1);
  Serial.print(",");
  Serial.print("P");
  Serial.print(plantaSelecionada);
  Serial.print(",");
  Serial.print(sp * VCC / RESOLUTION);
  Serial.print(",");
  Serial.println(sensorReadingVoltage, 3);

  // ENVIANDO PARA GRÁFICO VIA WEBSOCKET
  JSONVar readings;
  readings["time"] = String(xTaskGetTickCount() / 1000.0, 1);
  readings["MV"] = String((int)u * VCC / 255.0, 2); // tensão de saída real
  readings["PV"] = String(sensorReadingVoltage, 2);
  String jsonString = JSON.stringify(readings);
  websocketserver.broadcastTXT(jsonString);
}
//**************************************************************************************************** */

void setStepInputReadingCallback(TimerHandle_t xTimer)
{
  if (!usarPID)
  {
    if (plantaSelecionada >= 1 && plantaSelecionada <= 4)
      dacWrite(MV_PIN_1, 255);
    else if (plantaSelecionada >= 5 && plantaSelecionada <= 7)
      dacWrite(MV_PIN_2, 255);
  }
}

void setMUXCallback(TimerHandle_t xTimer)
{
  switch (plantaSelecionada)
  {
  case 1: // Planta 1 -> X0
    // A=0 B=0
    digitalWrite(A_PIN, LOW);
    digitalWrite(B_PIN, LOW);
    break;
  case 2: // Planta 2 -> X1
    // A=1 B=0
    digitalWrite(A_PIN, HIGH);
    digitalWrite(B_PIN, LOW);
    break;
  case 3: // Planta 3 -> X2
    // A=0 B=1
    digitalWrite(A_PIN, LOW);
    digitalWrite(B_PIN, HIGH);
    break;
  case 4: // Planta 4 -> X3
    // A=1 B=1
    digitalWrite(A_PIN, HIGH);
    digitalWrite(B_PIN, HIGH);
    break;
  case 5: // Planta 5 -> Y0
    // A=0 B=0
    digitalWrite(A_PIN, LOW);
    digitalWrite(B_PIN, LOW);
    break;
  case 6: // Planta 6 -> Y1
    // A=1 B=0
    digitalWrite(A_PIN, HIGH);
    digitalWrite(B_PIN, LOW);
    break;
  case 7: // Planta 7 -> Y2
    // A=0 B=1
    digitalWrite(A_PIN, LOW);
    digitalWrite(B_PIN, HIGH);
    break;
  default:
    // Serial.println("Planta inválida.");
    LOGE("ControlePID", "Planta inválida.");
    return;
  }
  // LOGI("ControlePID", "MUX configurado para Planta");
  Serial.print("MUX configurado para Planta ");
  Serial.println(plantaSelecionada);
}

void configurarPinos()
{
  Serial.begin(115200);
  pinMode(MV_PIN_1, OUTPUT);
  pinMode(MV_PIN_2, OUTPUT);
  pinMode(SENSOR_X, INPUT);
  pinMode(SENSOR_Y, INPUT);
  pinMode(A_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
}

void configurarMUX()
{
  // Timer do MUX (necessário em MA e MF)
  muxConfigTimer = xTimerCreate("muxConfigTimer", pdMS_TO_TICKS(50), pdFALSE, NULL, setMUXCallback);
  xTimerStart(muxConfigTimer, 0);
}

void configurarMalhaFechada()
{
  setControlLimits(0, 255);
  dacWrite(MV_PIN_1, 0);
  dacWrite(MV_PIN_2, 0);
  // Para usar os ganhos padrões apenas se necessário
  if (!ganhosFornecidosViaWeb)
  {
    switch (plantaSelecionada)
    {
    case 1:
      // Planta 1 1° ordem
      setTunings(1.922, 0.602, 0.0); // KP, KI, KD
      break;
    case 2:
      // Planta 2 1° ordem
      setTunings(2.033, 0.525, 0.0); // KP, KI, KD
      break;
    case 3:
      // Planta 3 1° ordem
      setTunings(2.054, 0.299, 0.0); // KP, KI, KD
      break;
    case 4:
      // Planta 4 1° ordem
      setTunings(1.961, 0.241, 0.0); // KP, KI, KD
      break;
    case 5:
      // Planta 1 2° ordem
      // CH-0: 4.7uF (SOBREAMORTECIDO)
      setTunings(1.961, 0.241, 0.0); // KP, KI, KD
      break;
    case 6:
      // Planta 2 2° ordem
      ////CH-2: 680nF (SUBAMORTECIDO)
      setTunings(1.961, 0.241, 0.0); // KP, KI, KD
      break;
    case 7:
      // Planta 3 2° ordem
      // CH-3: 2.2uF (CRITICAMENTE-AMORTECIDO)
      setTunings(1.961, 0.241, 0.0); // KP, KI, KD
      break;
    }
  }

  vTaskDelay(pdMS_TO_TICKS(8000)); // Esperar descarga do capacitor por 8s
  // criando timer de malha fechada (com PID ativo)
  getSensorReadingTimer = xTimerCreate("getSensorReadingTimer", samplingInterval, pdTRUE, NULL, getSensorReadingCallback);
  // Confirmando se o getSensorReadingTimer foi criado corretamente
  if (getSensorReadingTimer != NULL)
  {
    LOGI("ControlePID", "Iniciando getSensorReadingTimer!.");
    xTimerStart(getSensorReadingTimer, 0); // inicia o timer
  }
  else
  {
    // Serial.println("Erro ao criar getSensorReadingTimer!");
    LOGE("ControlePID", "Erro ao criar getSensorReadingTimer!.");
  }
}

void configurarMalhaAberta()
{
  // ativando time de  malha aberta (sem PID)
  stepInputStartTimer = xTimerCreate("stepInputStartTimer", timeToStartInterval, pdFALSE, NULL, setStepInputReadingCallback);
  xTimerStart(stepInputStartTimer, 0);
}

// Função para iniciar os timers quando autorizado:
void iniciarTimersSeNecessario(void *pvParameters) // Task de execução única
{

  if (iniciarExperimento && !experimentoIniciado)
  {
    configurarMUX();
    if (usarPID)
      configurarMalhaFechada();
    else if (!usarPID)
      configurarMalhaAberta();
    experimentoIniciado = true;
    // Serial.println("Timers iniciados...");
    LOGI("ControlePID", "Timers iniciados....");

    // experimentoIniciado = true;
    /*--------------------------------------------------------------------------------*/
    TickType_t tempoInicio = xTaskGetTickCount();
    Serial.print("Timers iniciados aos ");
    Serial.print(tempoInicio / 1000.0, 2);
    Serial.println(" segundos desde o boot.");
    /*--------------------------------------------------------------------------------- */
  }
  iniciarTimersTaskHandle = NULL;
  vTaskDelete(NULL); // autodelete da própria task
  // vTaskDelay(pdMS_TO_TICKS(100));
}

// parar todos os timers e resetar sistema:
void pararTimers()
{
  // Parando (xTimerStop) e deletando (xTimerDelete) os timers após o STOP
  LOGI("ControlePID", "Parando timers...");
  if (getSensorReadingTimer != NULL)
  {
    Serial.println("Parando getSensorReadingTimer...");
    xTimerStop(getSensorReadingTimer, 0);   // pausa a contagem
    xTimerDelete(getSensorReadingTimer, 0); // remove o timer da memória
    getSensorReadingTimer = NULL;
  }

  if (stepInputStartTimer != NULL)
  {
    Serial.println("Parando stepInputStartTimer...");
    xTimerStop(stepInputStartTimer, 0);   // pausa a contagem
    xTimerDelete(stepInputStartTimer, 0); // remove o timer da memória
    stepInputStartTimer = NULL;
  }

  if (muxConfigTimer != NULL)
  {
    Serial.println("Parando muxConfigTimer...");
    xTimerStop(muxConfigTimer, 0);   // pausa a contagem
    xTimerDelete(muxConfigTimer, 0); // remove o timer da memória
    muxConfigTimer = NULL;
  }
  dacWrite(MV_PIN_1, 0);
  dacWrite(MV_PIN_2, 0);

  iniciarExperimento = false;
  experimentoIniciado = false;

  if (iniciarTimersTaskHandle != NULL)
  {
    vTaskDelete(iniciarTimersTaskHandle); // remove o timer da memória
    iniciarTimersTaskHandle = NULL;
  }
  // Garante que novos testes futuros voltarão a usar os ganhos padrões
  // Caso o usuário não envie novos valores pela interface.
  ganhosFornecidosViaWeb = false;
  Serial.println("Experimento parado. Aguardando novo 'start'.");
}

void serialTask(void *pvParameters)
{
  while (true)
  {
    if (Serial.available())
    {
      String comando = Serial.readStringUntil('\n');
      comando.trim();
      comando.toLowerCase();

      if (comando == "start" && !experimentoIniciado && iniciarTimersTaskHandle == NULL)
      {
        Serial.println("Iniciando experimento...");
        iniciarExperimento = true;
        xTaskCreate(iniciarTimersSeNecessario, "iniciarTimersTask", 4096, NULL, 1, &iniciarTimersTaskHandle);
      }
      else if (comando == "stop")
      {
        Serial.println("Parando experimento...");
        pararTimers(); // isso agora também precisa deletar a task
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
