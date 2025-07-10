    #include <string.h>
    #include <stdio.h>
    #include <esp_now.h>
    #include <esp_wifi.h>
    #include <driver/adc.h>
    #include <esp_log.h>
    #include <nvs_flash.h>
    #include "driver/gpio.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"

    #define POTENTIOMETER1_ADC_CHANNEL ADC1_CHANNEL_6  // GPIO34 
    #define POTENTIOMETER2_ADC_CHANNEL ADC1_CHANNEL_7  // GPIO35 
    #define ADC_WIDTH ADC_WIDTH_BIT_12
    #define ADC_ATTEN ADC_ATTEN_DB_12
    #define PWM_MIN 1000  // Valor mínimo do PWM para o ESC
    #define PWM_MAX 2000  // Valor máximo do PWM para o ESC
    #define BUTTON_GPIO GPIO_NUM_14  // Substitua pelo pino GPIO onde o botão está conectado
    #define LED_GPIO GPIO_NUM_2     // Substitua pelo pino GPIO onde o LED está conectado
    int i = 0;

    // Endereço MAC do receptor
    uint8_t receiver_mac[] = {0x2c, 0xbc, 0xbb, 0x4b, 0xdb, 0x1c};  // Substitua pelo MAC do receptor 2c:bc:bb:4b:db:1c

    typedef struct {
        uint16_t pwm_value1;
        uint16_t pwm_value2;
    } esp_now_message_t;

    // Função de callback para enviar dados via ESP-NOW
    void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
        if (status == ESP_NOW_SEND_SUCCESS) {
            ESP_LOGI("ESP_NOW", "Mensagem enviada com sucesso!");
        } else {
            ESP_LOGE("ESP_NOW", "Falha ao enviar mensagem");
        }
    }

    void init_gpio() {
        // Configura o pino do botão como entrada
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Usar pull-up interno
        gpio_config(&io_conf);

        // Configura o pino do LED como saída
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << LED_GPIO);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
    }

    void button_task(void *pvParameter) {
        bool turnOn = false;
        bool last_button_state = gpio_get_level(BUTTON_GPIO);
        while (true) {
            bool current_button_state = gpio_get_level(BUTTON_GPIO);
            if (current_button_state != last_button_state) {
                ESP_LOGI("Controller", "ON");
                for(i=0;i<4;i++){
                    gpio_set_level(LED_GPIO, true);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    gpio_set_level(LED_GPIO, false);
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                gpio_set_level(LED_GPIO, true);
                vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(LED_GPIO, false);
                vTaskDelay(pdMS_TO_TICKS(700));
                turnOn = true;
                while (turnOn) {
                    
                    gpio_set_level(LED_GPIO, true); 
                    //// while padrao comeca aqui.
                    // Lê o valor do potenciômetro
                    int pot_value1 = adc1_get_raw(POTENTIOMETER1_ADC_CHANNEL);
                    int pot_value2 = adc1_get_raw(POTENTIOMETER2_ADC_CHANNEL);
                    // Mapeia o valor para o intervalo do PWM
                    uint16_t pwm_value1 = (pot_value1 * (PWM_MAX - PWM_MIN) / 4095) + PWM_MIN;
                    uint16_t pwm_value2 = (pot_value2 * (PWM_MAX - PWM_MIN) / 4095) + PWM_MIN;
                    
                    // Prepara a mensagem
                    esp_now_message_t message;
                    message.pwm_value1 = pwm_value1;
                    message.pwm_value2 = pwm_value2;
                    // Envia a mensagem via ESP-NOW
                    esp_err_t ret = esp_now_send(receiver_mac, (uint8_t *)&message, sizeof(message));
                    if (ret != ESP_OK) {
                        ESP_LOGE("ESP_NOW", "Falha ao enviar mensagem: %s", esp_err_to_name(ret));
                    }
                    vTaskDelay(pdMS_TO_TICKS(20));
                    //// while padrao termina aqui.
                    
                    bool current_button_state = gpio_get_level(BUTTON_GPIO);
                    if (current_button_state != last_button_state) {
                        turnOn = false;
                        ESP_LOGI("Controller", "OFF");
                        // Prepara a mensagem
                        pwm_value1 = 1500;
                        pwm_value2 = 1500;
                        esp_now_message_t message;
                        message.pwm_value1 = pwm_value1;
                        message.pwm_value2 = pwm_value2;
                        // Envia a mensagem via ESP-NOW
                        esp_err_t ret = esp_now_send(receiver_mac, (uint8_t *)&message, sizeof(message));
                        if (ret != ESP_OK) {
                            ESP_LOGE("ESP_NOW", "Falha ao enviar mensagem: %s", esp_err_to_name(ret));
                        }
                        //Led
                        for(i=0;i<30;i++){
                            gpio_set_level(LED_GPIO, true);
                            vTaskDelay(pdMS_TO_TICKS(40));
                            gpio_set_level(LED_GPIO, false);
                            vTaskDelay(pdMS_TO_TICKS(40));
                        }
                    }
                }
            }
        }
    }

    void app_main() {
        // Inicializa o NVS
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // Se o NVS estiver corrompido ou desatualizado, apague e reinicialize
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        // Inicializa o ADC
        adc1_config_width(ADC_WIDTH);
        adc1_config_channel_atten(POTENTIOMETER1_ADC_CHANNEL, ADC_ATTEN);

        // Inicializa o Wi-Fi
        wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
        ret = esp_wifi_init(&wifi_config);
        if (ret != ESP_OK) {
            ESP_LOGE("Wi-Fi", "Falha ao inicializar Wi-Fi: %s", esp_err_to_name(ret));
            return;
        }

        ret = esp_wifi_set_mode(WIFI_MODE_STA);
        if (ret != ESP_OK) {
            ESP_LOGE("Wi-Fi", "Falha ao definir modo Wi-Fi: %s", esp_err_to_name(ret));
            return;
        }

        ret = esp_wifi_start();
        if (ret != ESP_OK) {
            ESP_LOGE("Wi-Fi", "Falha ao iniciar Wi-Fi: %s", esp_err_to_name(ret));
            return;
        }

        // Inicializa o ESP-NOW
        ret = esp_now_init();
        if (ret != ESP_OK) {
            ESP_LOGE("ESP_NOW", "Falha ao inicializar ESP-NOW: %s", esp_err_to_name(ret));
            return;
        }

        ret = esp_now_register_send_cb(esp_now_send_cb);
        if (ret != ESP_OK) {
            ESP_LOGE("ESP_NOW", "Falha ao registrar callback do ESP-NOW: %s", esp_err_to_name(ret));
            return;
        }

        // Adiciona o peer (receptor)
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, receiver_mac, 6);
        peer_info.channel = 0;
        peer_info.encrypt = false;
        ret = esp_now_add_peer(&peer_info);
        if (ret != ESP_OK) {
            ESP_LOGE("ESP_NOW", "Falha ao adicionar peer: %s", esp_err_to_name(ret));
            return;
        }

        // Inicializa os GPIOs
        init_gpio();
        // Cria a tarefa para tratar o botão
        xTaskCreate(&button_task, "button_task", 2048, NULL, 10, NULL);
        
        ESP_LOGI("Main", "Transmissor ESP-NOW inicializado com sucesso!");

        /*while (true) {


            // Lê o valor do potenciômetro
            int pot_value = adc1_get_raw(POTENTIOMETER_ADC_CHANNEL);

            // Mapeia o valor para o intervalo do PWM
            uint16_t pwm_value = (pot_value * (PWM_MAX - PWM_MIN) / 4095) + PWM_MIN;

            // Prepara a mensagem
            esp_now_message_t message;
            message.pwm_value = pwm_value;

            // Envia a mensagem via ESP-NOW
            ret = esp_now_send(receiver_mac, (uint8_t *)&message, sizeof(message));
            if (ret != ESP_OK) {
                ESP_LOGE("ESP_NOW", "Falha ao enviar mensagem: %s", esp_err_to_name(ret));
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);  // Aguarda 1ms
        }*/
    }

