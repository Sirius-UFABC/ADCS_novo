#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
// definição dos pinos de seleção do multiplexador
#define S0_PIN GPIO_NUM_16
#define S1_PIN GPIO_NUM_17
#define S2_PIN GPIO_NUM_18
#define S3_PIN GPIO_NUM_19

// definição do canal ADC
#define ADC_CHANNEL ADC1_CHANNEL_0  // GPIO 36
#define NUM_CHANNELS 16

#define DEFAULT_VREF    1100  // tensão de referência padrão em milivolts (1.1V)

// função para configurar os pinos GPIO
void init_gpio() {
    esp_rom_gpio_pad_select_gpio(S0_PIN);
    esp_rom_gpio_pad_select_gpio(S1_PIN);
    esp_rom_gpio_pad_select_gpio(S2_PIN);
    esp_rom_gpio_pad_select_gpio(S3_PIN);

    gpio_set_direction(S0_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(S1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(S2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(S3_PIN, GPIO_MODE_OUTPUT);
}

// função para selecionar o canal do multiplexador
void select_mux_channel(uint8_t channel) {
    gpio_set_level(S0_PIN, (channel >> 0) & 0x01);
    gpio_set_level(S1_PIN, (channel >> 1) & 0x01);
    gpio_set_level(S2_PIN, (channel >> 2) & 0x01);
    gpio_set_level(S3_PIN, (channel >> 3) & 0x01);
}


static esp_adc_cal_characteristics_t *adc_chars;

//calibração do ADC
void init_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_6); //atenuação de 11db é obsoleta
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}


float voltagem[NUM_CHANNELS]; //número de canais
float azimute, elevacao;
char* quadrante; //define uma string


void app_main(void) {
    init_gpio();
    init_adc();

    while (1) {
        for (uint8_t channel = 0; channel < 16; channel++) {
            select_mux_channel(channel); //seleção do canal atual
            vTaskDelay(pdMS_TO_TICKS(10)); // atraso para estabilização da leitura
            int adc_reading = adc1_get_raw(ADC_CHANNEL); //valor analógico é lido
            uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
            voltagem[channel] = voltage / 1000.0; // converte para volts e armazena no vetor
            printf("Channel %d: %d\n", channel, adc_reading);
        }

        // impressão dos valores armazenados no array
        for (int i = 0; i < 16; i++) {
            printf("vetor[%d] = %.3f V\n", i, voltagem[i]);
        }
        
        float d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15, d16;
        //Lado Norte 
        d1 = voltagem[0], d2 = voltagem[1], d9 = voltagem[8], d10 = voltagem[9];
        //Lado Leste 
        d3 = voltagem[2], d4 = voltagem[3], d11 = voltagem[10], d12 = voltagem[11];
        //Lado Sul 
        d5 = voltagem[4], d6 = voltagem[5], d13 = voltagem[12], d14 = voltagem[13];
        //Lado Oeste 
        d7 = voltagem[6], d8 = voltagem[7], d15 = voltagem[14], d16 = voltagem[15];

        float Nx, Ny, Lx, Ly, Sx, Sy, Ox, Oy; 
        
        Nx = (d1*cos(3*(M_PI/4)))+(d2*cos(1*(M_PI/4)))+(d9*cos(3*(M_PI/4)))+(d10*cos(1*(M_PI/4)));
        Ny = (d1*sin(1*(M_PI/4)))+(d2*sin(1*(M_PI/4)))+(d9*sin(3*(M_PI/4)))+(d10*sin(1*(M_PI/4)));
        Lx = (d3*cos(1*(M_PI/4)))+(d4*cos(7*(M_PI/4)))+(d11*cos(1*(M_PI/4)))+(d12*cos(7*(M_PI/4)));
        Ly = (d3*sin(1*(M_PI/4)))+(d4*sin(7*(M_PI/4)))+(d11*sin(1*(M_PI/4)))+(d12*sin(7*(M_PI/4)));
        Sx = (d5*cos(7*(M_PI/4)))+(d6*cos(5*(M_PI/4)))+(d13*cos(7*(M_PI/4)))+(d14*cos(5*(M_PI/4)));
        Sy = (d5*sin(7*(M_PI/4)))+(d6*sin(5*(M_PI/4)))+(d13*sin(7*(M_PI/4)))+(d14*sin(5*(M_PI/4)));
        Ox = (d7*cos(5*(M_PI/4)))+(d8*cos(3*(M_PI/4)))+(d15*cos(7*(M_PI/4)))+(d16*cos(3*(M_PI/4)));
        Oy = (d7*sin(5*(M_PI/4)))+(d8*sin(3*(M_PI/4)))+(d15*sin(7*(M_PI/4)))+(d16*sin(3*(M_PI/4)));

        //soma das componentes
        float Rx, Ry;

        Rx = Nx + Lx + Sx + Ox;
        Ry = Ny + Ly + Sy + Oy;

        //módulo do vetor
        float R = sqrt(pow(Rx,2) + pow(Ry,2));

        if (Rx > 0 && Ry > 0) {
        quadrante = "1°";
        azimute = 90 - atan2(Ry, Rx) * 180 / M_PI;
        } else if (Rx < 0 && Ry > 0) {
        quadrante = "2°";
        azimute = 90 - atan2(Ry, Rx) * 180 / M_PI + 180;
        } else if (Rx < 0 && Ry < 0) {
        quadrante = "3°";
        azimute = 90 - atan2(Ry, Rx) * 180 / M_PI + 180;
        } else if (Rx > 0 && Ry < 0) {
        quadrante = "4°";
        azimute = 90 - atan2(Ry, Rx) * 180 / M_PI;
        }

        // calcular a elevação (ângulo em graus)
        elevacao = acos(R / (sqrt(pow(Nx, 2) + pow(Ny, 2)))) * (180 / M_PI);

        // imprimir os resultados
        printf("Nx: %.3f, Ny: %.3f\n", Nx, Ny);
        printf("Lx: %.3f, Ly: %.3f\n", Lx, Ly);
        printf("Sx: %.3f, Sy: %.3f\n", Sx, Sy);
        printf("Ox: %.3f, Oy: %.3f\n", Ox, Oy);
        printf("Rx: %.3f, Ry: %.3f\n", Rx, Ry);

        printf("Módulo do vetor R: %.3f\n", R);
        printf("Quadrante: %s\n", quadrante);
        printf("Azimute: %.3f\n", azimute);
        printf("Elevacao: %.3f\n", elevacao);
         
    }




        vTaskDelay(pdMS_TO_TICKS(1000)); // aguarda 1 segundo antes da próxima leitura
    }
