// Desabilita watchdog e cria void para segundo nucleo
void Task1code( void * pvParameters )
{
    esp_task_wdt_init(30, false);
    while(1)
    {
      loop_2 ();
    }
}

//Ciclo de Ventilação
void Ciclo(){
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
    VT1++;
  }
