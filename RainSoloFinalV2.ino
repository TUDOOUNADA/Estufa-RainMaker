//bibliotecas usadas
#include <OneWire.h>
#include <DallasTemperature.h>
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <esp_task_wdt.h>
#include <SimpleTimer.h>

#define DEFAULT_TERMO 30
#define DEFAULT_ET1 5
#define DEFAULT_ET2 2


//Define Task
TaskHandle_t Task1;

//Nome e senha do bluetooth
const char *service_name = "Estufa";
const char *pop = "Tomatinho";

// Definir pinos
uint8_t gpio_B1 = 13;
uint8_t gpio_V1 = 12;
uint8_t gpio_E1 = 14;
uint8_t gpio_L1 = 16;
uint8_t gpio_M1 = 17;

uint8_t gpio_U1 = 35;
uint8_t gpio_N1 = 34;
uint8_t gpio_T1 = 27;

uint8_t gpio_reset = 0;

//Estado das cargas
bool B_State = LOW;
bool V_State = LOW;
bool E_State = LOW;
bool M_State = LOW;
bool L_State = LOW;
bool B_State2 = LOW;
bool V_State2 = LOW;
bool E_State2 = LOW;
bool M_State2 = LOW;
bool L_State2 = LOW;
bool Flag = LOW;
bool OffSet = LOW;

//Variaveis para valores de sensores
float TemperaturaV = 0.0;
float UmidadeV = 0.0;
float NivelV = 0.0;
float TemperaturaV2 = 0.0;
float UmidadeV2 = 0.0;
float NivelV2 = 0.0;

uint8_t NivMin = 30;
uint8_t UmiMin = 15;
uint8_t UmiMax = 70;
uint8_t TempMin = DEFAULT_TERMO;
uint8_t TempMax = DEFAULT_TERMO;
uint16_t VTempo1 = DEFAULT_ET1;
uint16_t VTempo2 = DEFAULT_ET2;
uint16_t VT1 = 0;
uint16_t ET1 = 0;
uint16_t ET2 = 0;

//Inicia sensor de temperatura
OneWire oneWire(gpio_T1);
DallasTemperature sensor(&oneWire);
//Inicia simple timer
SimpleTimer Timer;

//Cria variaveis de Switch para cada botão
static Switch *BT1 = NULL;
static LightBulb *BT2 = NULL;
static Switch *BT3 = NULL;
static Switch *BT4 = NULL;
static Switch *BT5 = NULL;
static Device *BT6 = NULL;

//Cria variaveis de medidas para sensores
static TemperatureSensor Temperatura("Temperatura");
static TemperatureSensor Umidade("Umidade");
static TemperatureSensor Nivel("Nível");

//Função para conectar app ao ESP32
void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
      printQR(service_name, pop, "ble");
#else
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
      printQR(service_name, pop, "softap");
#endif
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      break;
  }
}

//Função para receber informação dos botões
void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();

  if (strcmp(device_name, "Bomba") == 0) {
    if (strcmp(param_name, "Power") == 0) {
      B_State = val.val.b;
      (B_State == false) ? digitalWrite(gpio_B1, 0) : digitalWrite(gpio_B1, 1);
      param->updateAndReport(val);
    }
  } else if (strcmp(device_name, "Ventilador") == 0) {
    if (strcmp(param_name, "Power") == 0) {
      V_State = val.val.b;
      (V_State == false) ? digitalWrite(gpio_V1, 0) : digitalWrite(gpio_V1, 1);
      param->updateAndReport(val);
    }
  } else if (strcmp(device_name, "Exaustor") == 0) {
    if (strcmp(param_name, "Power") == 0) {
      E_State = val.val.b;
      (E_State == false) ? digitalWrite(gpio_E1, 0) : digitalWrite(gpio_E1, 1);
      param->updateAndReport(val);
    }
  } else if (strcmp(device_name, "Lampada") == 0) {
    if (strcmp(param_name, "Power") == 0) {
      L_State = val.val.b;
      (L_State == false) ? digitalWrite(gpio_L1, 0) : digitalWrite(gpio_L1, 1);
      param->updateAndReport(val);
    }
  } else if (strcmp(device_name, "Manual") == 0) {
    if (strcmp(param_name, "Power") == 0) {
      M_State = val.val.b;
      (M_State == false) ? digitalWrite(gpio_M1, 0) : digitalWrite(gpio_M1, 1);
      param->updateAndReport(val);
    }
  }
  else if (strcmp(device_name, "Termostato") == 0) {
    if (strcmp(param_name, "Temperatura Max") == 0) {
      int TMax = val.val.i;
      TempMax = TMax;
      param->updateAndReport(val);
      delay(50);
    } else if (strcmp(param_name, "Temperatura Min") == 0) {
      int TMin = val.val.i;
      TempMin = TMin;
      param->updateAndReport(val);
      delay(50);
    }
        if (strcmp(param_name, "Tempo para Ventilar") == 0) {
      int VTi1 = val.val.i;
      VTempo1 = VTi1;
      param->updateAndReport(val);
      delay(50);
    } else if (strcmp(param_name, "Ciclos de Ventilação") == 0) {
      int VTi2 = val.val.i;
      VTempo2 = VTi2;
      param->updateAndReport(val);
      delay(50);
    }
  }
}


//Função de Controle automatico
void Auto() {

  if ((TemperaturaV > TempMax) || (OffSet)) {
    if (VT1 <= 600) {
      digitalWrite(gpio_E1, 0);
      E_State = false;
      digitalWrite(gpio_V1, 1);
      V_State = true;
    }

    else if (VT1 > 600) {
      digitalWrite(gpio_V1, 0);
      V_State = false;
      digitalWrite(gpio_E1, 1);
      E_State = true;
    }

    else if (VT1 > 1200) {
      VT1 = 0;
    }
      OffSet = 1;
      VT1++;
      ET1 = 0;
      ET2 = 0;
  }

  if ((TemperaturaV < TempMin) && (ET2<(VTempo2*600))) {
    OffSet = 0;
    digitalWrite(gpio_V1, 0);
    V_State = false;
    digitalWrite(gpio_E1, 0);
    E_State = false;
    VT1 = 0;
  }

      if ((ET1 > (VTempo1*600)) && !OffSet ) {
        if (VT1 <= 600) {
          digitalWrite(gpio_E1, 0);
          E_State = false;
          digitalWrite(gpio_V1, 1);
          V_State = true;
      }
        else if (VT1 > 600) {
          digitalWrite(gpio_V1, 0);
          V_State = false;
          digitalWrite(gpio_E1, 1);
          E_State = true; } 
          
        else if (VT1 > 1200) { 
          VT1 = 0;}
        
      if (ET2 > (VTempo2*600*2)) {
        ET1 = 0;
        ET2 = 0;
        VT1 = 0;
      }
      ET2++;
      VT1++;
    }
    if (ET1<=VTempo1){
    ET1++;}

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(gpio_L1, 1);
    L_State = true;

    if ((UmidadeV < UmiMin) && (NivelV > NivMin)) {
      digitalWrite(gpio_B1, 1);
      B_State = true; }
  }
}

void setup() {

  // Definir pinos, serial e inicio do sensor
  Serial.begin(115200);
  sensor.begin();
  sensor.setResolution(10);

  pinMode(gpio_B1, OUTPUT);
  pinMode(gpio_E1, OUTPUT);
  pinMode(gpio_V1, OUTPUT);
  pinMode(gpio_M1, OUTPUT);
  pinMode(gpio_L1, OUTPUT);

  pinMode(gpio_N1, INPUT);
  pinMode(gpio_T1, INPUT);
  pinMode(gpio_U1, INPUT);
  pinMode(gpio_reset, INPUT);

  //Desliga todas as saidas
  digitalWrite(gpio_B1, 0);
  digitalWrite(gpio_E1, 0);
  digitalWrite(gpio_V1, 0);
  digitalWrite(gpio_M1, 0);
  digitalWrite(gpio_L1, 0);

  //Cria um node para o aplicativo reconhecer
  Node my_node;
  my_node = RMaker.initNode("Estufa");
  
  //Cria novos Switchs para cada variavel de botão
  BT1 = new Switch("Bomba", &gpio_B1);
  BT2 = new LightBulb("Lampada", &gpio_L1);
  BT3 = new Switch("Ventilador", &gpio_V1);
  BT4 = new Switch("Exaustor", &gpio_E1);
  BT5 = new Switch("Manual", &gpio_M1);
  BT6 = new Device("Termostato", "esp.device.thermostat	", NULL);

  Param Temperature1("Temperatura Max", ESP_RMAKER_PARAM_RANGE, value(DEFAULT_TERMO), PROP_FLAG_READ | PROP_FLAG_WRITE);
  Temperature1.addUIType(ESP_RMAKER_UI_SLIDER);
  Temperature1.addBounds(value(0), value(40), value(1));
  BT6->addParam(Temperature1);
  Param Temperature2("Temperatura Min", ESP_RMAKER_PARAM_RANGE, value(DEFAULT_TERMO), PROP_FLAG_READ | PROP_FLAG_WRITE);
  Temperature2.addUIType(ESP_RMAKER_UI_SLIDER);
  Temperature2.addBounds(value(0), value(40), value(1));
  BT6->addParam(Temperature2);

  Param Tempo1("Tempo para Ventilar", ESP_RMAKER_PARAM_RANGE, value(DEFAULT_ET1), PROP_FLAG_READ | PROP_FLAG_WRITE);
  Tempo1.addUIType(ESP_RMAKER_UI_SLIDER);
  Tempo1.addBounds(value(1), value(45), value(1));
  BT6->addParam(Tempo1);
  Param Tempo2("Ciclos de Ventilação", ESP_RMAKER_PARAM_RANGE, value(DEFAULT_ET2), PROP_FLAG_READ | PROP_FLAG_WRITE);
  Tempo2.addUIType(ESP_RMAKER_UI_SLIDER);
  Tempo2.addBounds(value(1), value(10), value(1));
  BT6->addParam(Tempo2);

  //Utiliza uma função de write_callback para cada botão ao precionar
  BT1->addCb(write_callback);
  BT2->addCb(write_callback);
  BT3->addCb(write_callback);
  BT4->addCb(write_callback);
  BT5->addCb(write_callback);
  BT6->addCb(write_callback);

  //Adiciona os dispositivos no App
  my_node.addDevice(*BT1);
  my_node.addDevice(*BT2);
  my_node.addDevice(*BT3);
  my_node.addDevice(*BT4);
  my_node.addDevice(*BT5);
  my_node.addDevice(Temperatura);
  my_node.addDevice(Umidade);
  my_node.addDevice(Nivel);
  my_node.addDevice(*BT6);
  //Intervalo de 2000
  Timer.setInterval(2000);

  //Utilizar parametros para configurar fuso horario e inicia o Rain Maker
  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();
  RMaker.enableScenes();
  RMaker.start();

  //WifiProv para conexão com a internet
  WiFi.onEvent(sysProvEvent);
#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif

  //Desliga todas as cargas no App
  BT1->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  BT2->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  BT3->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  BT4->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  BT5->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
  //habilita segundo nucleo
  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
}

void loop() {

  // Botão de boot para reiniciar configuraçoes do wifi ou ESP32
  if (digitalRead(gpio_reset) == LOW) {
    Serial.printf("Reset Button Pressed!\n");
    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW) delay(50);
    int endTime = millis();
    if ((endTime - startTime) > 10000) {
      Serial.printf("Reset to factory.\n");
      RMakerFactoryReset(2);
    } else if ((endTime - startTime) > 3000) {
      Serial.printf("Reset Wi-Fi.\n");
      RMakerWiFiReset(2);
    }
  }

  //Condição quando conectado a internet
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(gpio_M1, 0);
    M_State = false;
  } else {
    if (Timer.isReady()) {
      Timer.reset();
    }
  }


  if (!M_State) {
    Auto();
  }
  delay(100);
  if (M_State) {
  ET1 = 0;
  ET1 = 0;
  ET2 = 0;}

  //loop de leitura de sensores e modo automatico
  sensor.requestTemperatures();
  TemperaturaV = sensor.getTempCByIndex(0);  //Valor Temperatura

  if (UmidadeV < 100&&UmidadeV>0) {
  UmidadeV = map(analogRead(gpio_U1), 2900, 800, 0, 100);  //Valor Umidade
  }

  NivelV = map(analogRead(gpio_N1), 0, 4095, 0, 100);  //Valor Nivel

  //Desliga bomba quando nivel baixo ou umidade alta
  if ((NivelV > NivMin) && (Flag)) {
    Flag = 0;
  }

  if ((NivelV < NivMin) || (UmidadeV > UmiMax)) {
    digitalWrite(gpio_B1, 0);
    B_State = false;

    if ((!Flag) && (NivelV < NivMin)) {
      esp_rmaker_raise_alert("Nível d'agua baixo !\nNecessário reabastecer");
      Flag = 1;
      delay(50);
    }
  }
}

void loop_2() {
  //Leitura de sensores e cargas
  if (WiFi.status() == WL_CONNECTED) {

    if(((TemperaturaV-TemperaturaV2)>1.5)||((TemperaturaV-TemperaturaV2)<-1.5)){
    Temperatura.updateAndReportParam(ESP_RMAKER_DEF_TEMPERATURE_NAME, TemperaturaV);
    TemperaturaV2=TemperaturaV;
    delay(50);
    }

    if(((UmidadeV-UmidadeV2)>10)||((UmidadeV-UmidadeV2)<-10)){
    Umidade.updateAndReportParam(ESP_RMAKER_DEF_TEMPERATURE_NAME, UmidadeV);
    UmidadeV2=UmidadeV;
    delay(50);
    }

    if(((NivelV-NivelV2)>10)||((NivelV-NivelV2)<-10)){
    Nivel.updateAndReportParam(ESP_RMAKER_DEF_TEMPERATURE_NAME, NivelV);
    NivelV2=NivelV;
    delay(50);
    } 


    if ((B_State) != (B_State2)) {
      BT1->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, B_State);
      B_State2 = B_State;
      delay(50);
    }

    if ((L_State) != (L_State2)) {
      BT2->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, L_State);
      L_State2 = L_State;
      delay(50);
    }
    if ((V_State) != (V_State2)) {
      BT3->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, V_State);
      V_State2 = V_State;
      delay(50);
    }

    if ((E_State) != (E_State2)) {
      BT4->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, E_State);
      E_State2 = E_State;
      delay(50);
    }

    if ((M_State) != (M_State2)) {
      BT5->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, M_State);
      M_State2 = M_State;
      delay(50);
    }
  }
  delay(500);
}
