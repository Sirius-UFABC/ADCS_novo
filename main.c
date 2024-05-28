#include <stdio.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Bibliotecas incorporadas por ADCs
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

#include <math.h>
#define pi M_PI

static esp_adc_cal_characteristics_t adc1_chars;

#define pino_s0  18 //pino que s0, s1 e s2 estão conectados
#define pino_s1  19
#define pino_s2  23
#define pino_s3  

#define pin_SIG  34 

#define NUM_CHANNELS 16 //nº total de canais no multiplexador

float vetor[NUM_CHANNELS];

void init_multiplexador(){
    gpio_config_t pin_conf = {
         .pin_bit_mask = (1ULL << pino_s0) | (1ULL << pino_s1) | (1ULL << pino_s2) | (1ULL << pino_s3), //o bit correspondente ao pino específico é definido como 1 e os outros são 0
         .mode = GPIO_MODE_OUTPUT,
    };

    gpio_config(&pin_conf);

    gpio_set_level(pino_s0,0); 
    gpio_set_level(pino_s1,0);
    gpio_set_level(pino_s2,0);
    gpio_set_level(pino_s3,0);

    pin_conf.pin_bit_mask = (1ULL << pin_SIG); 
    pin_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&pin_conf);

}

uint16_t read_fotodiodo(uint8_t fotodiodo) {
    for (int i = 0; i < 4; i++){
        gpio_set_level(pino_s0 + i, (fotodiodo & (1 << i)) ? 1:0);
    }

    int adc_reading = adc1_get_raw(ADC1_CHANNEL_6);
    

    return adc_reading;

}

void mainTask(void){
    init_multiplexador();

    //Faz a leitura dos fotodiodos e armazena os valores da voltagem em um vetor
    while (1) {
        for (int fotodiodo = 0; fotodiodo < NUM_CHANNELS; fotodiodo++) {
            uint16_t adc_value = read_fotodiodo(fotodiodo);
            uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_value, &adc1_chars); // Converte o valor raw para mV depois da calibração
            vetor[fotodiodo] = voltage;

            printf("Fotodiodo %d: %ld mV\n", fotodiodo, voltage);
            vTaskDelay(pdMS_TO_TICKS(1000));

        }


        //Imprime todos os valores das leituras em ordem
        printf("\nVetor\n");
        for (int i = 0; i < (sizeof(vetor)/ sizeof(vetor[0])); ++i) {
        printf("%.2f ", vetor[i]);
        }

        //Determina o maior valor de voltagem no vetor
        int maior = vetor[0];
        for (int i = 1; i < sizeof(vetor) / sizeof(vetor[0]); ++i) {
            if (vetor[i] > maior) {
            maior = vetor[i];
            }
        }
        printf("\nO maior valor no vetor é: %d\n", maior);
        
        //Normalização do vetor pelo maior valor
        for (int i = 0; i < (sizeof(vetor)/ sizeof(vetor[0])); ++i) {
            vetor[i] /= maior;
        }

        //Imprime os valores normalizados do vetor
        printf("\nVetor após divisão por %d:\n", maior);
        for (int i = 0; i < (sizeof(vetor)/ sizeof(vetor[0])); ++i) {
        printf("%.2f ", vetor[i]);
        }

        float d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15, d16;
        //Lado Norte 
        d1 = vetor[0], d2 = vetor[1]; d3 = vetor[2]; d4 = vetor[3];
        //Lado Oeste
        d5 = vetor[4], d6 = vetor[5]; d7 = vetor[6], d8 = vetor[7];
        //Lado Sul 
        d9 = vetor[8], d10 = vetor[9]; d11 = vetor[10], d12 = vetor[11];
        //Lado Leste 
        d13 = vetor[12], d14 = vetor[13]; d15 = vetor[14], d16 = vetor[15];

        
        double Nx, Ny, Lx, Ly, Sx, Sy, Ox, Oy; 
        Nx = (d1*cos(1*(pi/4)))+(d2*cos(1*(pi/4)))+(d3*cos(3*(pi/4)))+(d4*cos(3*(pi/4)));
        Ny = (d1*sin(1*(pi/4)))+(d2*sin(1*(pi/4)))+(d3*sin(3*(pi/4)))+(d4*sin(3*(pi/4)));
        Ox = (d5*cos(3*(pi/4)))+(d6*cos(3*(pi/4)))+(d7*cos(5*(pi/4)))+(d8*cos(5*(pi/4)));
        Oy = (d5*sin(3*(pi/4)))+(d6*sin(3*(pi/4)))+(d7*sin(5*(pi/4)))+(d8*sin(5*(pi/4)));
        Sx = (d9*cos(5*(pi/4)))+(d10*cos(5*(pi/4)))+(d11*cos(7*(pi/4)))+(d12*cos(7*(pi/4)));
        Sy = (d9*sin(5*(pi/4)))+(d10*sin(5*(pi/4)))+(d11*sin(7*(pi/4)))+(d12*sin(7*(pi/4)));
        Lx = (d13*cos(7*(pi/4)))+(d14*cos(7*(pi/4)))+(d15*cos(1*(pi/4)))+(d16*cos(1*(pi/4)));
        Ly = (d13*sin(7*(pi/4)))+(d14*sin(7*(pi/4)))+(d15*sin(1*(pi/4)))+(d16*sin(1*(pi/4)));

        //soma das componentes
        double Rx, Ry;
        Rx = Nx + Lx + Sx + Ox;
        Ry = Ny + Ly + Sy + Oy;

        //módulo do vetor
        double R = sqrt(pow(Rx,2) + pow(Ry,2));
        //double x[] = {Nx, Lx, Sx, Ox};
        //double y[] = {Ny, Ly, Sy, Oy};

        char* quadrante;
        double azimute, elevacao;

        if (Rx > 0 && Ry > 0) {
        quadrante = "1°";
        azimute = 90 - atan2(Ry, Rx) * 180 / pi;
        } else if (Rx < 0 && Ry > 0) {
        quadrante = "2°";
        azimute = 90 - atan2(Ry, Rx) * 180 / pi + 180;
        } else if (Rx < 0 && Ry < 0) {
        quadrante = "3°";
        azimute = 90 - atan2(Ry, Rx) * 180 / pi + 180;
        } else if (Rx > 0 && Ry < 0) {
        quadrante = "4°";
        azimute = 90 - atan2(Ry, Rx) * 180 / pi;
        }

        // Calcular a elevação (ângulo em graus)
        elevacao = acos(R / (sqrt(pow(Nx, 2) + pow(Ny, 2)))) * (180 / pi);

        // Imprimir os resultados
        printf("Nx: %.2f, Ny: %.2f\n", Nx, Ny);
        printf("Lx: %.2f, Ly: %.2f\n", Lx, Ly);
        printf("Sx: %.2f, Sy: %.2f\n", Sx, Sy);
        printf("Ox: %.2f, Oy: %.2f\n", Ox, Oy);
        printf("Rx: %.2f, Ry: %.2f\n", Rx, Ry);

        printf("Módulo do vetor R: %.2f\n", R);
        //printf("Quadrante: %s\n", quadrante);
        //printf("Azimute: %.2f\n", azimute);
        //printf("Elevacao: %.2f\n", elevacao);
         
    }



}


void app_main(){

    // Configure ADC 
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_DEFAULT, 100, &adc1_chars); // Calibração
    //ADC_ATTEN_DB_0 (100 mV ~ 950 mV) define a atenuação, ver qual a faixa de tensão de entrada mensurável do Fotodiodo para escolher a atenuação adequada
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);

    xTaskCreate(mainTask, "main_task", 4096, NULL, 5, NULL);

}
