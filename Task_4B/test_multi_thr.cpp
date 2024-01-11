#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2 // Specify the on which is your LED
#endif

const int led = 2;
TaskHandle_t chan_task;
int count[3] = {1, 2, 3};
QueueHandle_t queue;

void toggle_led_1(void *param){
  for(;;){
    bl();
  }
}

void toggle_led_2(void *param){
  for(;;){
    digitalWrite(led, HIGH);
    vTaskDelay(333 / portTICK_PERIOD_MS);
    Serial.println("blah blah");
    digitalWrite(led, LOW);
    vTaskDelay(333 / portTICK_PERIOD_MS);    
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(led,OUTPUT);
  Serial.begin(300);
  //task 1

  queue = xQueueCreate(5, sizeof(int));

  xTaskCreatePinnedToCore(
      toggle_led_1 //function to all
      , "blink led at 0.5s" // name for the task
      , 1024 // stack size in the cpu
      , NULL //parameters being passed
      , 1 // priority out of 25
      , NULL // task handler name
      , ARDUINO_RUNNING_CORE // core to run task
  );

  xTaskCreate(
      toggle_led_2 //function to all
      , "blink led at 1s" // name for the task
      , 1024 // stack size in the cpu
      , NULL //parameters being passed
      , 2 // priority out of 25
      , &chan_task // task handler name
      //, ARDUINO_RUNNING_CORE // core to run task
  );
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("pritham is rizz");
  // delay(1000);
  xQueueSendToBack(queue, &count, 0);
  delay(1000);
}

void bl(){
    int cnt[3];
    
    digitalWrite(led, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    if (xQueueReceive(queue, &cnt, portMAX_DELAY) == pdPASS){
      Serial.println(cnt[2]);
      if(cnt==5){
        vTaskSuspend( chan_task );
      }
      else if(cnt == 10){
        vTaskResume(chan_task);
      }
    }

    digitalWrite(led, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);  
}