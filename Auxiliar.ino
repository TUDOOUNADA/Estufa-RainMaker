// Desabilita watchdog e cria void para segundo nucleo
void Task1code( void * pvParameters )
{
    esp_task_wdt_init(30, false);
    while(1)
    {
      loop_2 ();
    }
}

//Função de Controle automatico
void Auto() {

  //Inicio de ventilação por temperatura
  if (((TemperaturaV > TempMax) || (OffSet)) && (ET2!=0)) {
      Ciclo();
      OffSet = 1;
      if (ET1!=0){ET1 = 0;}
  }

  //Desliga de ventilação por tempo
  else if ((TemperaturaV < TempMin) && (OffSet)) {
    OffSet = 0;
    digitalWrite(gpio_V1, 0);
    V_State = false;
    digitalWrite(gpio_E1, 0);
    E_State = false;
    VT1 = 0;
  }

  //Inicio de ventilação por tempo
  if ((ET1<=(VTempo1*600)) && (!OffSet)){
  ET1++;
}
  else if ((ET1 > (VTempo1*600)) && (!OffSet)) {
      Ciclo();
      ET2++;
      if (ET2 > (VTempo2*600*2)) {
        ET1 = 0;
        ET2 = 0;
        VT1 = 0;
          digitalWrite(gpio_V1, 0);
          V_State = false;
          digitalWrite(gpio_E1, 0);
          E_State = false;
      }
    }

  //Caso sem conexão
  if (WiFi.status() != WL_CONNECTED) {

    digitalWrite(gpio_L1, 1);
    L_State = true;

    if ((UmidadeV < UmiMin) && (NivelV > NivMin)) {
      digitalWrite(gpio_B1, 1);
      B_State = true; 
    }
  }
}

//Ciclo de Ventilação
void Ciclo(){
if (VT1 < 600) {
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
    VT1++;
  }
