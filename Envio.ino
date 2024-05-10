// Desabilita watchdog e cria void para segundo nucleo
void Task1code( void * pvParameters )
{
    esp_task_wdt_init(30, false);
    while(1)
    {
      loop_2 ();
    }
}