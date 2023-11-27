
#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_interface.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_err.h"
#include "music.h"
#include "ssd1306.h"
#include "sprites.h"



////// DEFINITIONS ///////////////////////////////////////
#define BLINK_GPIO 10

#define LEFT_BUTTON 0
#define CENTER_BUTTON 4
#define RIGHT_BUTTON 1

uint8_t isOn = 0;

#define I2C_MASTER_SCL_IO 9       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 8        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */

#define I2C_ADDRESS 0x68 // I2C address of MPU6050
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_GYRO_SENS 0X1B
#define MPU6050_ACCE_SENS 0X1C

/////////////////////////////////////////////////////

uint8_t data[50];

short raw_accel_x;
short raw_accel_y;
short raw_accel_z;

short raw_temp;

short raw_gyro_x;
short raw_gyro_y;
short raw_gyro_z;


/// MPU - Values ///////////////////////////////////////
float accel_x;
float accel_y;
float accel_z;

float accel_N;
float accel_N_prev = 0;
float accel_N_diff;
int pasos = -2;

float temp;

float gyro_x;
float gyro_y;
float gyro_z;

float gyro_N;
float gyro_N_prev = -100;
float gyro_N_diff;


char x_acce_str[20];
char y_acce_str[20];
char z_acce_str[20];

char N_acce_str[20];

char x_gyro_str[20];
char y_gyro_str[20];
char z_gyro_str[20];

///// Tiempos ////////////////////////////////////////////////
TickType_t tick_count_init = 0;
TickType_t actual_ticks = 0;
uint32_t actual_ms = 0;
char time_str[20];

int hour = 0;
int minute = 0;
int second = 0;
int mili_second = 0;

////// VARIABLES ////////////////////////////////////////////////////


float health = 100;  //Indica la salud de la mascota del 1 al 100
float happiness = 100;  //Indica la felicidad de la mascota del 1 al 100

float health_decay = 0.1; //Indica cuanto baja la salud (de un total de 100) cada 50ms
float happiness_decay = 0.25; //Indica cuanto baja la felicidad (de un total de 100) cada 50ms


///// SEMAPHORES /////////////////////////////////////////////////////

SemaphoreHandle_t xHatchSemaphore;
SemaphoreHandle_t xTimeSemaphore;
SemaphoreHandle_t xDeadSemaphore;
SemaphoreHandle_t xSadnessSemaphore;

///// QUEUES /////////////////////////////////////////////////////
QueueHandle_t xInputQueue;
QueueHandle_t xFeedQueue;
QueueHandle_t xPlayQueue;

//OBJETOS
static ssd1306_handle_t ssd1306_dev = NULL;
i2c_cmd_handle_t cmd;
esp_adc_cal_characteristics_t adc1_chars;
esp_timer_handle_t cal_timer = NULL;


//Config I2C
i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, // I2C LCD is a master node
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };


static esp_err_t setUpI2C(){
    //Configuración Bus I2C    
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    return ESP_OK;
}

static esp_err_t setUpButtons(){
    gpio_set_direction(LEFT_BUTTON, GPIO_MODE_INPUT);
    gpio_set_direction(CENTER_BUTTON, GPIO_MODE_INPUT);
    gpio_set_direction(RIGHT_BUTTON, GPIO_MODE_INPUT);

    gpio_set_pull_mode(LEFT_BUTTON, GPIO_PULLUP_ONLY);   //set pullup
    gpio_set_pull_mode(CENTER_BUTTON, GPIO_PULLUP_ONLY);   //set pullup
    gpio_set_pull_mode(RIGHT_BUTTON, GPIO_PULLUP_ONLY);   //set pullup

    return ESP_OK;
}

static void setUpScreen(){
    ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);
} 

static void setUpMPU(){
	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1);
	i2c_master_write_byte(cmd, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}


static void setSensitivity(){

    //Gyro - Nos ubicamos en el registro 0x1B y ponemos +/- 250 deg/s
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_GYRO_SENS, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0b00000000, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    //Acce - Nos ubicamos en el registro 0x1C y ponemos +/- 2g
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCE_SENS, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0b00000000, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
}

static void update(void *pvParameters){
    for(;;){
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data,   0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+1, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+2, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+3, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+4, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+5, 0));

    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+6, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+7, 0));

    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+8,   0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+9, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+10, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+11, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+12, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+13, 1));

    //i2c_master_read(cmd, data, sizeof(data), 1);
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    raw_accel_x = (data[0] << 8) | data[1];
    raw_accel_y = (data[2] << 8) | data[3];
    raw_accel_z = (data[4] << 8) | data[5];

    raw_temp = (data[6] << 8) | data[7];

    raw_gyro_x = (data[8] << 8) | data[9];
    raw_gyro_y = (data[10] << 8) | data[11];
    raw_gyro_z = (data[11] << 8) | data[13];


    accel_x = raw_accel_x / (16384.0f);
    accel_y = raw_accel_y / (16384.0f);
    accel_z = raw_accel_z / (16384.0f);

    accel_N = sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));

    gyro_x = raw_gyro_x / (131.0f);
    gyro_y = raw_gyro_y / (131.0f);
    gyro_z = raw_gyro_z / (131.0f);

    gyro_N =  sqrt((gyro_x * gyro_x) + (gyro_y * gyro_y) + (gyro_z * gyro_z));

    //Actualización deL giro/aceleración
    accel_N_diff = accel_N - accel_N_prev;
    gyro_N_diff = gyro_N - gyro_N_prev;

    //Actualizar aceleración previa con la actual
    accel_N_prev = accel_N;

    // Revisión HATCH
    if(accel_N_diff > 1.5){
      xSemaphoreGiveFromISR(xHatchSemaphore, pdFALSE);  // Desbloqueamos el semáforo
    }

    ssd1306_refresh_gram(ssd1306_dev);
    }
}



static void setUpBlink(){
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}


// STATE MACHINE
#define DEFAULT 0
#define FEED 1
#define PLAY 2
#define CURE 3
int state = DEFAULT;

void setState(int newState){
    state = newState;
}


// TASK FUNCTIONS
static void blink(){
    isOn = !isOn;
    gpio_set_level(BLINK_GPIO, isOn);
}


static void hatch(void *pvParameters){

    while(1){
        xSemaphoreTake(xHatchSemaphore, portMAX_DELAY); //Bloqueamos la tarea mientras no se agite
        tick_count_init = xTaskGetTickCount(); //Iniciamos a contar el tiempo

        ssd1306_fill_rectangle(ssd1306_dev, 58,29, 73, 52, 0); //Se borra el huevo
        ssd1306_draw_bitmap(ssd1306_dev, 36,21, Stego_56_32, 56, 32); // Se dibuja el stego

        xSemaphoreGive(xTimeSemaphore); //Permitimos que el tiempo empiece a correr..
    }

}


static void dead(void *pvParameters){

    char* game_over = "Game Over";

    while(1){
        xSemaphoreTake(xDeadSemaphore, portMAX_DELAY); //Bloqueamos la tarea mientras tenga salud

        ssd1306_fill_rectangle(ssd1306_dev, 36,21, 92, 53, 0); //Se borra el Stego
        ssd1306_draw_bitmap(ssd1306_dev, 36,21, Dead_56_32, 56, 32); // Se dibuja el Stego muerto
        ssd1306_draw_bitmap(ssd1306_dev, 0,0, Background_128_64, 128, 64); // Se dibuja el background
        ssd1306_fill_rectangle(ssd1306_dev, 0,0,96, 8, 0); // Se borran las barras 
        ssd1306_fill_rectangle(ssd1306_dev, 94,0, 94 + 40, 32, 0); // Se borra el cuadrado de actividades
        ssd1306_draw_string(ssd1306_dev, 32,0, (const uint8_t*)game_over, 14, 1); // GAME OVER
    }

}

static void sad(void *pvParameters){
    while(1){
        xSemaphoreTake(xSadnessSemaphore, portMAX_DELAY);
        health -= health_decay;
    } 
}

static void time(void *pvParameters){

    float buffer = 0;
    int bufferF = 0;
    int bufferP = 0;

    while(1){

        vTaskDelay(50/portTICK_PERIOD_MS);

        if(xSemaphoreTake(xTimeSemaphore, portMAX_DELAY)  == pdTRUE){; //Bloqueamos la tarea mientras no se haya abierto el huevo
        


        //Actualizar el tiempo
        actual_ticks = xTaskGetTickCount() - tick_count_init;
        actual_ms = actual_ticks * portTICK_PERIOD_MS;

        hour = actual_ms / (3.6e+6);
        minute = (actual_ms - (hour * (3.6e+6))) / 60000;
        second = (actual_ms - (hour * (3.6e+6)) - (minute*60000))/1000;
        mili_second = (actual_ms - (hour * (3.6e+6)) - (minute*60000) - (second*1000));

        //Se baja el valor de health y hapiness bajo un factor
        health -= health_decay;
        happiness -= happiness_decay;

        //Se revisan las colas de juego y comida
        if(xQueueReceive(xFeedQueue, &bufferF, 0) == pdTRUE){
            health += bufferF;
            if(health > 100){
                health = 100;
            }
            ssd1306_fill_rectangle(ssd1306_dev, 1, 3, 31 * (health/100), 4, 1); // Se aumenta si recibe vida
        }

        if(xQueueReceive(xPlayQueue, &bufferP, 0) == pdTRUE){    
            happiness += bufferP;
            if(happiness > 100){
                happiness = 100;
            }
            ssd1306_fill_rectangle(ssd1306_dev, 45, 3, 75 - (32 - 31*(happiness/100)), 4, 1); // Se disminuye la barra de felicidad
        }

        xSemaphoreGive(xTimeSemaphore); //Desbloqueamos el semáforo de tiempo para que el tiempo siga corriendo...

        //Si el Stego se queda sin vida...
        if(health/100 <= 0){
            xSemaphoreGiveFromISR(xDeadSemaphore, pdFALSE);  // Desbloqueamos el semáforo de la muerte...
            xSemaphoreTake(xTimeSemaphore, portMAX_DELAY); // Bloqueamos el semáforo actual.
        }

        //Si el Stego se queda sin felicidad...
        if(happiness/100 <= 0){
            xSemaphoreGiveFromISR(xSadnessSemaphore, pdFALSE);  // Desbloqueamos el semáforo de la tristeza...
        }

        ssd1306_fill_rectangle(ssd1306_dev, 32 - 31*(1 - (health/100) ), 3, 31, 4, 0); // Se disminuye la barra de salud

        buffer = 76 - 31*(1 - (happiness/100) );
        if(buffer > 45){
            ssd1306_fill_rectangle(ssd1306_dev, buffer,3, 75, 4, 0); // Se disminuye la barra de felicidad
        }
   
        }
    }

}

static void input(void *pvParameters){

    char command = 0; // 1: LEFT, 2: CENTER, 3: RIGHT

    while(1){
        vTaskDelay(100/portTICK_PERIOD_MS);

        // LEFT
        if(gpio_get_level(LEFT_BUTTON) == 0){
            command = 1;
            xQueueSend(xInputQueue, &command,  ( TickType_t ) 0);
        }

        // CENTER
        else if(gpio_get_level(CENTER_BUTTON) == 0){
            command = 2;
            xQueueSend(xInputQueue, &command,  ( TickType_t ) 0);
        }

        // RIGHT
        else if(gpio_get_level(RIGHT_BUTTON) == 0){
            command = 3;
            xQueueSend(xInputQueue, &command,  ( TickType_t ) 0);
        }

    }
}

int mod(int a, int b)
{
    int r = a % b;
    return r < 0 ? r + b : r;
}

static void stateUpdate(void *pvParameters){

    int command = 0;

    int currentActivity = 0;

    int buff = 0;


    while(1){

        xQueueReceive(xInputQueue, &command, portMAX_DELAY); // Bloqueamos la tarea hasta que se reciba un comando


        // SELECCIÓN DE ACTIVIDAD
        if(state == DEFAULT){

            if(command == 1){ // Si se presiona el botón izquierdo y se está sobre la actividad default
                currentActivity = mod(currentActivity - 1 , 3);
            }
            else if(command == 3){
                    currentActivity = mod(currentActivity + 1, 3);
            }

            if(currentActivity == 0){
                ssd1306_fill_rectangle(ssd1306_dev, 94,0, 94 + 40, 32, 0); // Se borra el cuadrado de actividades
                ssd1306_draw_bitmap(ssd1306_dev, 94,0, Cuadro_40_32, 40, 32); // Se dibujan el cuadrado de actividades
                ssd1306_draw_bitmap(ssd1306_dev, 100,0, Comida_24_24, 24, 24); // Se dibuja la comida (default)
            }
            else if(currentActivity == 1){
                ssd1306_fill_rectangle(ssd1306_dev, 94,0, 94 + 40, 32, 0); // Se borra el cuadrado de actividades
                ssd1306_draw_bitmap(ssd1306_dev, 94,0, Cuadro_40_32, 40, 32); // Se dibujan el cuadrado de actividades
                ssd1306_draw_bitmap(ssd1306_dev, 99,0, Jugar_24_24, 24, 24); // Se dibuja jugar (default)
            }
            else if(currentActivity == 2){
                ssd1306_fill_rectangle(ssd1306_dev, 94,0, 94 + 40, 32, 0); // Se borra el cuadrado de actividades
                ssd1306_draw_bitmap(ssd1306_dev, 94,0, Cuadro_40_32, 40, 32); // Se dibujan el cuadrado de actividades
                ssd1306_draw_bitmap(ssd1306_dev, 104,3, Curar_24_24, 24, 24); // Se dibuja curar (default)
            }

            if(command == 2){
                if(currentActivity == 0){
                    buff = 10;
                    xQueueSend(xFeedQueue, &buff,  ( TickType_t ) 0);
                }
                else if(currentActivity == 1){
                    buff = 10;
                    xQueueSend(xPlayQueue, &buff,  ( TickType_t ) 0);
                }
                else if(currentActivity == 2){
                    //setState(CURE);
                }
            }  
        }
    }
}


// ALL TASK SETUP
void setUpTasks( void )
{

    // SEMAPHORE init
    xHatchSemaphore = xSemaphoreCreateBinary();
    xTimeSemaphore = xSemaphoreCreateBinary();
    xDeadSemaphore = xSemaphoreCreateBinary();
    xSadnessSemaphore = xSemaphoreCreateBinary();

    // QUEUE init
    xInputQueue = xQueueCreate( 10, sizeof( unsigned char ));
    xFeedQueue = xQueueCreate( 10, sizeof( unsigned char ));
    xPlayQueue = xQueueCreate( 10, sizeof( unsigned char ));



    // TIMER 01: Blink LED //
    TimerHandle_t xBlink;
    xBlink = xTimerCreate
                    ("BlinkTimer",
                    2000/portTICK_PERIOD_MS,
                    pdTRUE,
                    ( void * ) 0,
                    blink);

    // TASK 01: MPU & Screen Update//
    TaskHandle_t xUpdate;
    xUpdate = xTaskCreate(
                    update,       /* Function that implements the task. */
                    "Update",          /* Text name for the task. */
                    2048 ,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */

    // TASK 02: Hatch//
    TaskHandle_t xHatch;
    xHatch = xTaskCreate(
                    hatch,       /* Function that implements the task. */
                    "Hatch",          /* Text name for the task. */
                    2048 ,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */

    // TASK 03: Time Update//
    TaskHandle_t xTime;
    xTime = xTaskCreate(
                    time,       /* Function that implements the task. */
                    "Time",          /* Text name for the task. */
                    2048 ,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    3,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */


    // TASK 04: RIP//
    TaskHandle_t xDead;
    xTime = xTaskCreate(
                    dead,       /* Function that implements the task. */
                    "Dead",          /* Text name for the task. */
                    2048 ,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    3,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */

    // TASK 05: :( //
    TaskHandle_t xSad;
    xSad = xTaskCreate(
                    sad,       /* Function that implements the task. */
                    "Sad",          /* Text name for the task. */
                    2048 ,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    3,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */

    // TASK 05: Input //
    TaskHandle_t xInput;
    xInput = xTaskCreate(
                    input,       /* Function that implements the task. */
                    "Input",          /* Text name for the task. */
                    2048 ,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */


    // TASK 06: SM //
    TaskHandle_t xState;
    xState = xTaskCreate(
                    stateUpdate,       /* Function that implements the task. */
                    "State",          /* Text name for the task. */
                    2048 ,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */


    // Inicio de los timers
    xTimerStart(xBlink, 0);

}

/***Función encargada de inciar el Tamagotchi en su estado bebe***/
void initTamagothchi(){

    // Invert Background
    for (int i = 0; i < 16*64; i++) {
        Background_128_64[i] = ~Background_128_64[i];
    }

    // Invert Stego Egg
    for (int i = 0; i < 48; i++) {
        StegoEgg_16_24[i] = ~StegoEgg_16_24[i];
    }

    // Invert Stego!
        for (int i = 0; i < 224; i++) {
        Stego_56_32[i] = ~Stego_56_32[i];
    }

    // Invert Dead Stego!
        for (int i = 0; i < 224; i++) {
        Dead_56_32[i] = ~Dead_56_32[i];
    }

    // Invert Bars
    for (int i = 0; i < 96; i++) {
        Bars_96_8[i] = ~Bars_96_8[i];
    }

    //Invert Cuadrado Activities
    for (int i = 0; i < 160; i++) {
        Cuadro_40_32[i] = ~Cuadro_40_32[i];
    }

    //Invetr comida
    for (int i = 0; i < 72; i++) {
        Comida_24_24[i] = ~Comida_24_24[i];
    }

    //Invetr jugar
    for (int i = 0; i < 72; i++) {
        Jugar_24_24[i] = ~Jugar_24_24[i];
    }

    //Invertir curar
    for (int i = 0; i < 72; i++) {
        Curar_24_24[i] = ~Curar_24_24[i];
    }



    ssd1306_draw_bitmap(ssd1306_dev, 58,29, StegoEgg_16_24, 16, 24); // Se dibuja el huevo
    ssd1306_draw_bitmap(ssd1306_dev, 0,0, Background_128_64, 128, 64); // Se dibuja el background
    ssd1306_draw_bitmap(ssd1306_dev, 0,0, Bars_96_8, 96, 8); // Se dibujan las barras de salud y felicidad
    ssd1306_fill_rectangle(ssd1306_dev, 1,3, 32, 4, 1); // Se rellenan las barra de salud al máximo
    ssd1306_fill_rectangle(ssd1306_dev, 45,3, 76, 4, 1); // Se rellenan las barra de felicidad al máximo

    ssd1306_draw_bitmap(ssd1306_dev, 94,0, Cuadro_40_32, 40, 32); // Se dibujan el cuadrado de actividades
    ssd1306_draw_bitmap(ssd1306_dev, 100,0, Comida_24_24, 24, 24); // Se dibuja la comida (default)

}

void app_main() {

    setUpI2C();
    setUpButtons();
    setUpBlink();
    setUpMPU();
    setSensitivity();
    setUpScreen();


    initTamagothchi();

    setUpTasks();
    setState(DEFAULT);
    
}
