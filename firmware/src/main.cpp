#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>
#include <fcntl.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/i2c_slave.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

namespace {

constexpr gpio_num_t PIN_LEITURA = GPIO_NUM_0;
constexpr gpio_num_t PIN_CHUTE = GPIO_NUM_1;
constexpr gpio_num_t PIN_FAULT = GPIO_NUM_2;
constexpr gpio_num_t PIN_DONE = GPIO_NUM_3;
constexpr gpio_num_t PIN_CARREGAMENTO = GPIO_NUM_4;
constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_6;
constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_7;
constexpr adc_channel_t ADC_CANAL = ADC_CHANNEL_0;  // GPIO0 no ESP32-C3
constexpr uint16_t ENDERECO_I2C = 0x42;
constexpr size_t TAMANHO_STATUS_I2C = 14;

constexpr float ADC_EM_200V = 3700.0f;
constexpr float TENSAO_CALIBRACAO = 200.0f;
constexpr float FATOR_CALIBRACAO = 1.0f;
constexpr uint8_t AMOSTRAS_ADC = 16;
constexpr uint32_t INTERVALO_LOG_MS = 100;
constexpr uint32_t TIMEOUT_CARGA_MS = 15000;
constexpr uint32_t TEMPO_IGNORAR_SINAIS_MS = 5;
constexpr uint32_t FILTRO_DONE_MS = 2;
constexpr uint32_t FILTRO_FAULT_MS = 2;
constexpr float SETPOINT_MINIMO = 20.0f;
constexpr float SETPOINT_MAXIMO = 200.0f;
constexpr float TENSAO_MAXIMA_SEGURANCA = 205.0f;

constexpr char WIFI_SSID[] = "TauraBots-Kicker";
constexpr char WIFI_SENHA[] = "taurabots2026";
constexpr int8_t WIFI_POTENCIA_QUARTO_DBM = 34;  // 8,5 dBm
const char *TAG = "kicker";

adc_oneshot_unit_handle_t adc_handle;
i2c_slave_dev_handle_t i2c_handle;
httpd_handle_t servidor;
SemaphoreHandle_t mutex_estado;
QueueHandle_t fila_comandos;
QueueHandle_t fila_eventos_i2c;

bool carregando;
bool auto_chute_ativo;
float tensao_auto_chute;
uint64_t inicio_carga;
uint64_t ultimo_log;
uint64_t inicio_done_low;
uint64_t inicio_fault_low;
float valor_setado = 200.0f;
float valor_atual;
uint16_t leitura_adc;
char buffer_serial[32];
size_t indice_serial;

enum class TipoComando : uint8_t { Iniciar, Parar, Chute, Setpoint, Percentual };
struct ComandoRemoto { TipoComando tipo; float valor; };
enum class TipoEventoI2c : uint8_t { Recebido, LeituraSolicitada };
struct EventoI2c {
    TipoEventoI2c tipo;
    uint8_t tamanho;
    uint8_t dados[4];
};
struct EstadoWeb {
    float tensao;
    float alvo;
    float setpoint;
    uint16_t adc;
    bool carregando;
    bool auto_chute;
    bool done;
    bool fault;
} estado_web;

uint64_t agora_ms() { return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL; }
void esperar_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
bool done_ativo() { return gpio_get_level(PIN_DONE) == 0; }
bool fault_ativo() { return gpio_get_level(PIN_FAULT) == 0; }

void atualizar_leitura()
{
    uint32_t soma = 0;
    for (uint8_t i = 0; i < AMOSTRAS_ADC; ++i) {
        int bruto = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CANAL, &bruto));
        soma += static_cast<uint32_t>(bruto);
        esp_rom_delay_us(50);
    }
    const uint16_t media = static_cast<uint16_t>(soma / AMOSTRAS_ADC);
    float tensao = media * TENSAO_CALIBRACAO / ADC_EM_200V * FATOR_CALIBRACAO;
    if (tensao < 0.5f) tensao = 0.0f;

    xSemaphoreTake(mutex_estado, portMAX_DELAY);
    leitura_adc = media;
    valor_atual = tensao;
    xSemaphoreGive(mutex_estado);
}

void mostrar_status()
{
    atualizar_leitura();
    printf("\n========== STATUS ==========\n");
    printf("Carregamento: %s\nSetpoint: %.1f V\nADC: %u\nTensão medida: %.1f V\n",
           carregando ? "ON" : "OFF", valor_setado, leitura_adc, valor_atual);
    if (auto_chute_ativo) printf("Auto-Kick: ARMADO para %.1f V\n", tensao_auto_chute);
    else printf("Auto-Kick: DESARMADO\n");
    printf("DONE#: %d - %s\n", gpio_get_level(PIN_DONE), done_ativo() ? "ATIVO" : "INATIVO");
    printf("FAULT#: %d - %s\n", gpio_get_level(PIN_FAULT), fault_ativo() ? "ATIVO" : "INATIVO");
    printf("CHARGE: %d\nKICK: %d\n============================\n",
           gpio_get_level(PIN_CARREGAMENTO), gpio_get_level(PIN_CHUTE));
}

void parar_carregamento(const char *motivo)
{
    auto_chute_ativo = false;
    const float tensao = valor_atual;
    const bool estava_carregando = carregando;
    const uint64_t tempo = estava_carregando ? agora_ms() - inicio_carga : 0;
    gpio_set_level(PIN_CARREGAMENTO, 0);
    carregando = false;
    inicio_done_low = inicio_fault_low = 0;
    printf("\n============================\nCarga encerrada: %s\n", motivo);
    if (estava_carregando) printf("Tempo de carga: %llu ms\n", static_cast<unsigned long long>(tempo));
    printf("Tensão no desligamento: %.1f V\n============================\n", tensao);
}

void iniciar_carregamento()
{
    atualizar_leitura();
    if (carregando) { printf("O carregamento já está ligado.\n"); return; }
    if (fault_ativo()) { printf("Carga não iniciada: FAULT# está ativo.\n"); return; }
    gpio_set_level(PIN_CARREGAMENTO, 0);
    gpio_set_level(PIN_CHUTE, 0);
    esperar_ms(5);
    inicio_carga = ultimo_log = agora_ms();
    inicio_done_low = inicio_fault_low = 0;
    carregando = true;
    gpio_set_level(PIN_CARREGAMENTO, 1);
    printf("\n============================\nCarregamento ON\n============================\n");
}

void solicitar_chute()
{
    if (carregando) parar_carregamento("COMANDO KICK / AUTO-KICK");
    gpio_set_level(PIN_CARREGAMENTO, 0);
    gpio_set_level(PIN_CHUTE, 1);
    esperar_ms(50);
    gpio_set_level(PIN_CHUTE, 0);
    printf("KICK disparado!\n");
}

void controlar_carregamento()
{
    if (!carregando) { inicio_done_low = inicio_fault_low = 0; return; }
    const uint64_t agora = agora_ms();
    const uint64_t tempo = agora - inicio_carga;
    if (valor_atual >= TENSAO_MAXIMA_SEGURANCA) parar_carregamento("LIMITE DE SEGURANÇA");
    else if (auto_chute_ativo && valor_atual >= tensao_auto_chute) {
        printf("\n>> ALVO DE PORCENTAGEM ATINGIDO! Disparando... <<\n");
        solicitar_chute();
    } else if (valor_atual >= valor_setado) parar_carregamento("SETPOINT DO ADC");
    else if (tempo >= TIMEOUT_CARGA_MS) parar_carregamento("TIMEOUT");
    else if (tempo >= TEMPO_IGNORAR_SINAIS_MS) {
        if (fault_ativo()) {
            if (!inicio_fault_low) inicio_fault_low = agora;
            if (agora - inicio_fault_low >= FILTRO_FAULT_MS) {
                parar_carregamento("FAULT DO LT3751");
                return;
            }
        } else inicio_fault_low = 0;
        if (done_ativo()) {
            if (!inicio_done_low) inicio_done_low = agora;
            if (agora - inicio_done_low >= FILTRO_DONE_MS) parar_carregamento("DONE DO LT3751");
        } else inicio_done_low = 0;
    }
}

void imprimir_log()
{
    const uint64_t agora = agora_ms();
    if (!carregando || agora - ultimo_log < INTERVALO_LOG_MS) return;
    ultimo_log = agora;
    printf("Tensão: %.1f V | Tempo: %llu ms\n", valor_atual,
           static_cast<unsigned long long>(agora - inicio_carga));
}

void publicar_estado_web()
{
    xSemaphoreTake(mutex_estado, portMAX_DELAY);
    estado_web.tensao = valor_atual;
    estado_web.alvo = tensao_auto_chute;
    estado_web.setpoint = valor_setado;
    estado_web.adc = leitura_adc;
    estado_web.carregando = carregando;
    estado_web.auto_chute = auto_chute_ativo;
    estado_web.done = done_ativo();
    estado_web.fault = fault_ativo();
    xSemaphoreGive(mutex_estado);
}

void processar_comando(char *comando)
{
    if (!comando || !comando[0]) return;
    for (size_t i = 0; comando[i]; ++i) if (comando[i] == ',') comando[i] = '.';
    char *sinal = strchr(comando, '%');
    if (sinal) {
        *sinal = '\0';
        char *fim;
        const float percentual = strtof(comando, &fim);
        if (fim != comando && !*fim && percentual > 0.0f && percentual <= 100.0f) {
            tensao_auto_chute = percentual / 100.0f * SETPOINT_MAXIMO;
            auto_chute_ativo = true;
            printf(">>> Disparo automático engatilhado para %.1f%% (%.1f V) <<<\n",
                   percentual, tensao_auto_chute);
            iniciar_carregamento();
        } else printf("Porcentagem inválida. Envie de 1%% a 100%%.\n");
        return;
    }
    if (!strcmp(comando, "1")) { iniciar_carregamento(); return; }
    if (!strcmp(comando, "2")) { parar_carregamento("COMANDO MANUAL"); return; }
    if (!strcmp(comando, "3")) { solicitar_chute(); return; }
    if (!strcasecmp(comando, "status") || !strcasecmp(comando, "s")) { mostrar_status(); return; }
    char *fim;
    const float novo = strtof(comando, &fim);
    if (fim != comando && !*fim) {
        if (carregando) printf("Pare o carregamento antes de alterar o setpoint.\n");
        else if (novo >= SETPOINT_MINIMO && novo <= SETPOINT_MAXIMO) {
            valor_setado = novo;
            printf("Novo setpoint: %.1f V\n", valor_setado);
        } else printf("Setpoint inválido. Use de %.0f a %.0f V.\n", SETPOINT_MINIMO, SETPOINT_MAXIMO);
        return;
    }
    printf("Comando inválido: %s\n", comando);
}

void ler_serial()
{
    char c;
    while (read(STDIN_FILENO, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (indice_serial) {
                buffer_serial[indice_serial] = '\0';
                processar_comando(buffer_serial);
                indice_serial = 0;
            }
        } else if (c == '\b' || c == 127) {
            if (indice_serial) --indice_serial;
        } else if (indice_serial < sizeof(buffer_serial) - 1) buffer_serial[indice_serial++] = c;
        else { indice_serial = 0; printf("Comando muito longo.\n"); }
    }
}

esp_err_t pagina_handler(httpd_req_t *req)
{
    float tensao, alvo;
    bool carga, auto_chute;
    xSemaphoreTake(mutex_estado, portMAX_DELAY);
    tensao = estado_web.tensao; alvo = estado_web.alvo;
    carga = estado_web.carregando; auto_chute = estado_web.auto_chute;
    xSemaphoreGive(mutex_estado);

    char estado_auto[48];
    if (auto_chute) snprintf(estado_auto, sizeof(estado_auto), "ARMADO para %.1f V", alvo);
    else strcpy(estado_auto, "DESARMADO");
    char pagina[2300];
    const int n = snprintf(pagina, sizeof(pagina),
        "<!DOCTYPE html><html lang='pt-BR'><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'><title>TauraBots Kicker</title><style>"
        "body{font-family:Arial,sans-serif;text-align:center;background:#111;color:#eee;margin:0;padding:35px 15px}"
        ".box{max-width:420px;margin:auto;background:#1d1d1d;padding:25px;border-radius:15px}"
        ".v{font-size:32px;font-weight:bold;margin:20px 0}.estado{margin-bottom:20px;color:#bbb}"
        "button{width:100%%;padding:22px;font-size:25px;font-weight:bold;border:0;border-radius:12px;background:#d62828;color:white;cursor:pointer}"
        "button:active{transform:scale(.98)}</style></head><body><div class='box'><h1>TauraBots Kicker</h1>"
        "<div style='color:#f59e0b;font-weight:bold;margin-bottom:15px'>INTERFACE ESP-IDF - PORCENTAGEM</div>"
        "<div class='v'>%.1f V</div><div class='estado'>Carregamento: %s</div>"
        "<div class='estado'>Auto-Kick: %s</div>"
        "<form method='POST' action='/percent'><label for='p'>Porcentagem do disparo:</label><br><br>"
        "<input id='p' name='p' type='number' min='1' max='100' step='1' value='80' required "
        "style='width:100%%;box-sizing:border-box;padding:15px;font-size:22px;text-align:center;border-radius:10px;border:0;margin-bottom:12px'>"
        "<button type='submit' style='background:#f59e0b;margin-bottom:18px'>CARREGAR E DISPARAR (%%)</button></form>"
        "<form method='POST' action='/kick' onsubmit=\"return confirm('Confirmar disparo imediato?')\">"
        "<button type='submit'>DISPARAR AGORA</button></form><p><a href='/' style='color:#aaa'>Atualizar status</a></p>"
        "</div></body></html>", tensao, carga ? "ON" : "OFF", estado_auto);
    if (n < 0 || n >= static_cast<int>(sizeof(pagina)))
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Página excedeu o buffer");
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    return httpd_resp_send(req, pagina, n);
}

esp_err_t percentual_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len >= 32)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Porcentagem não informada");
    char corpo[32] = {};
    int total = 0;
    while (total < req->content_len) {
        const int n = httpd_req_recv(req, corpo + total, req->content_len - total);
        if (n <= 0) return ESP_FAIL;
        total += n;
    }
    if (strncmp(corpo, "p=", 2)) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Campo inválido");
    char *fim;
    const float percentual = strtof(corpo + 2, &fim);
    if (fim == corpo + 2 || *fim || percentual <= 0.0f || percentual > 100.0f)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Use uma porcentagem de 1 a 100");
    const ComandoRemoto comando{TipoComando::Percentual, percentual};
    if (xQueueSend(fila_comandos, &comando, 0) != pdTRUE) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_send(req, "Fila de comandos ocupada", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t chute_handler(httpd_req_t *req)
{
    const ComandoRemoto comando{TipoComando::Chute, 0.0f};
    if (xQueueSend(fila_comandos, &comando, 0) != pdTRUE) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_send(req, "Fila de comandos ocupada", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, nullptr, 0);
}

void iniciar_wifi_e_servidor()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    wifi_config_t cfg = {};
    memcpy(cfg.ap.ssid, WIFI_SSID, sizeof(WIFI_SSID) - 1);
    cfg.ap.ssid_len = sizeof(WIFI_SSID) - 1;
    memcpy(cfg.ap.password, WIFI_SENHA, sizeof(WIFI_SENHA) - 1);
    cfg.ap.channel = 1;
    cfg.ap.max_connection = 1;
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(WIFI_POTENCIA_QUARTO_DBM));

    httpd_config_t config_http = HTTPD_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(httpd_start(&servidor, &config_http));
    httpd_uri_t pagina = {}; pagina.uri = "/"; pagina.method = HTTP_GET; pagina.handler = pagina_handler;
    httpd_uri_t percentual = {}; percentual.uri = "/percent"; percentual.method = HTTP_POST; percentual.handler = percentual_handler;
    httpd_uri_t chute = {}; chute.uri = "/kick"; chute.method = HTTP_POST; chute.handler = chute_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(servidor, &pagina));
    ESP_ERROR_CHECK(httpd_register_uri_handler(servidor, &percentual));
    ESP_ERROR_CHECK(httpd_register_uri_handler(servidor, &chute));
    printf("Wi-Fi criado: %s\nServidor Web iniciado em http://192.168.4.1\n", WIFI_SSID);
}

void inicializar_hardware()
{
    gpio_config_t saidas = {};
    saidas.pin_bit_mask = (1ULL << PIN_CARREGAMENTO) | (1ULL << PIN_CHUTE);
    saidas.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&saidas));
    gpio_set_level(PIN_CARREGAMENTO, 0);
    gpio_set_level(PIN_CHUTE, 0);
    gpio_config_t entradas = {};
    entradas.pin_bit_mask = (1ULL << PIN_DONE) | (1ULL << PIN_FAULT);
    entradas.mode = GPIO_MODE_INPUT;
    entradas.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&entradas));

    adc_oneshot_unit_init_cfg_t unidade = {};
    unidade.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unidade, &adc_handle));
    adc_oneshot_chan_cfg_t canal = {};
    canal.atten = ADC_ATTEN_DB_12;
    canal.bitwidth = ADC_BITWIDTH_12;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CANAL, &canal));
}

uint8_t crc8(const uint8_t *dados, size_t tamanho)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < tamanho; ++i) {
        crc ^= dados[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

uint16_t para_decivolts(float tensao)
{
    if (tensao <= 0.0f) return 0;
    if (tensao >= 6553.5f) return UINT16_MAX;
    return static_cast<uint16_t>(tensao * 10.0f + 0.5f);
}

void escrever_u16_le(uint8_t *destino, uint16_t valor)
{
    destino[0] = static_cast<uint8_t>(valor & 0xff);
    destino[1] = static_cast<uint8_t>(valor >> 8);
}

bool i2c_requisicao_cb(i2c_slave_dev_handle_t, const i2c_slave_request_event_data_t *, void *)
{
    const EventoI2c evento{TipoEventoI2c::LeituraSolicitada, 0, {}};
    BaseType_t tarefa_acordada = pdFALSE;
    xQueueSendFromISR(fila_eventos_i2c, &evento, &tarefa_acordada);
    return tarefa_acordada == pdTRUE;
}

bool i2c_recebimento_cb(i2c_slave_dev_handle_t, const i2c_slave_rx_done_event_data_t *dados, void *)
{
    EventoI2c evento{TipoEventoI2c::Recebido, 0, {}};
    if (dados->length <= sizeof(evento.dados)) {
        evento.tamanho = static_cast<uint8_t>(dados->length);
        for (uint8_t i = 0; i < evento.tamanho; ++i) evento.dados[i] = dados->buffer[i];
    } else {
        evento.tamanho = 0xff;
    }
    BaseType_t tarefa_acordada = pdFALSE;
    xQueueSendFromISR(fila_eventos_i2c, &evento, &tarefa_acordada);
    return tarefa_acordada == pdTRUE;
}

bool enfileirar_comando_i2c(const EventoI2c &evento)
{
    if (evento.tamanho == 1 && evento.dados[0] == 0x00) return true;  // Seleciona o registrador de status.

    ComandoRemoto comando{};
    switch (evento.dados[0]) {
        case 0x01:
            if (evento.tamanho != 1) return false;
            comando.tipo = TipoComando::Iniciar;
            break;
        case 0x02:
            if (evento.tamanho != 1) return false;
            comando.tipo = TipoComando::Parar;
            break;
        case 0x03:
            if (evento.tamanho != 1) return false;
            comando.tipo = TipoComando::Chute;
            break;
        case 0x04: {
            if (evento.tamanho != 3) return false;
            const uint16_t decivolts = static_cast<uint16_t>(evento.dados[1]) |
                                      (static_cast<uint16_t>(evento.dados[2]) << 8);
            comando.valor = decivolts / 10.0f;
            if (comando.valor < SETPOINT_MINIMO || comando.valor > SETPOINT_MAXIMO) return false;
            comando.tipo = TipoComando::Setpoint;
            break;
        }
        case 0x05:
            if (evento.tamanho != 2 || evento.dados[1] < 1 || evento.dados[1] > 100) return false;
            comando.tipo = TipoComando::Percentual;
            comando.valor = evento.dados[1];
            break;
        default:
            return false;
    }
    return xQueueSend(fila_comandos, &comando, 0) == pdTRUE;
}

void tarefa_i2c(void *)
{
    uint8_t sequencia = 0;
    uint8_t ultimo_resultado = 0;
    while (true) {
        EventoI2c evento;
        if (xQueueReceive(fila_eventos_i2c, &evento, portMAX_DELAY) != pdTRUE) continue;
        if (evento.tipo == TipoEventoI2c::Recebido) {
            ultimo_resultado = (evento.tamanho > 0 && evento.tamanho != 0xff && enfileirar_comando_i2c(evento)) ? 0 : 1;
            if (ultimo_resultado) ESP_LOGW(TAG, "Comando I2C inválido ou fila cheia");
            continue;
        }

        EstadoWeb estado;
        xSemaphoreTake(mutex_estado, portMAX_DELAY);
        estado = estado_web;
        xSemaphoreGive(mutex_estado);

        uint8_t resposta[TAMANHO_STATUS_I2C] = {};
        resposta[0] = 0xa5;
        resposta[1] = 0x01;
        resposta[2] = (estado.carregando ? 1U : 0U) |
                      (estado.auto_chute ? 2U : 0U) |
                      (estado.done ? 4U : 0U) |
                      (estado.fault ? 8U : 0U);
        resposta[3] = sequencia++;
        escrever_u16_le(&resposta[4], para_decivolts(estado.tensao));
        escrever_u16_le(&resposta[6], para_decivolts(estado.setpoint));
        escrever_u16_le(&resposta[8], para_decivolts(estado.alvo));
        escrever_u16_le(&resposta[10], estado.adc);
        resposta[12] = ultimo_resultado;
        resposta[13] = crc8(resposta, TAMANHO_STATUS_I2C - 1);

        uint32_t escritos = 0;
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_slave_reset_tx_fifo(i2c_handle));
        const esp_err_t erro = i2c_slave_write(i2c_handle, resposta, sizeof(resposta), &escritos, 100);
        if (erro != ESP_OK || escritos != sizeof(resposta))
            ESP_LOGW(TAG, "Resposta I2C incompleta: %lu/%u (%s)", static_cast<unsigned long>(escritos),
                     static_cast<unsigned>(sizeof(resposta)), esp_err_to_name(erro));
    }
}

void iniciar_i2c()
{
    i2c_slave_config_t config = {};
    config.i2c_port = I2C_NUM_0;
    config.sda_io_num = PIN_I2C_SDA;
    config.scl_io_num = PIN_I2C_SCL;
    config.clk_source = I2C_CLK_SRC_DEFAULT;
    config.send_buf_depth = 64;
    config.receive_buf_depth = 64;
    config.slave_addr = ENDERECO_I2C;
    config.addr_bit_len = I2C_ADDR_BIT_LEN_7;
    config.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_slave_device(&config, &i2c_handle));

    i2c_slave_event_callbacks_t callbacks = {};
    callbacks.on_request = i2c_requisicao_cb;
    callbacks.on_receive = i2c_recebimento_cb;
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(i2c_handle, &callbacks, nullptr));
    if (xTaskCreate(tarefa_i2c, "kicker_i2c", 4096, nullptr, 12, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar tarefa I2C");
        abort();
    }
    printf("I2C escravo: endereço 0x%02X, SDA=GPIO%d, SCL=GPIO%d\n",
           ENDERECO_I2C, PIN_I2C_SDA, PIN_I2C_SCL);
}

void processar_fila_remota()
{
    ComandoRemoto comando;
    while (xQueueReceive(fila_comandos, &comando, 0) == pdTRUE) {
        switch (comando.tipo) {
            case TipoComando::Iniciar:
                iniciar_carregamento();
                break;
            case TipoComando::Parar:
                parar_carregamento("COMANDO REMOTO");
                break;
            case TipoComando::Chute:
                solicitar_chute();
                break;
            case TipoComando::Setpoint:
                if (carregando) printf("I2C: pare o carregamento antes de alterar o setpoint.\n");
                else {
                    valor_setado = comando.valor;
                    printf("Novo setpoint remoto: %.1f V\n", valor_setado);
                }
                break;
            case TipoComando::Percentual:
                tensao_auto_chute = comando.valor / 100.0f * SETPOINT_MAXIMO;
                auto_chute_ativo = true;
                printf("Disparo remoto armado para %.0f%% (%.1f V)\n", comando.valor, tensao_auto_chute);
                iniciar_carregamento();
                break;
        }
    }
}

}  // namespace

extern "C" void app_main()
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    mutex_estado = xSemaphoreCreateMutex();
    fila_comandos = xQueueCreate(8, sizeof(ComandoRemoto));
    fila_eventos_i2c = xQueueCreate(8, sizeof(EventoI2c));
    if (!mutex_estado || !fila_comandos || !fila_eventos_i2c) {
        ESP_LOGE(TAG, "Falha ao criar objetos do FreeRTOS");
        abort();
    }

    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs);
    inicializar_hardware();
    atualizar_leitura();
    publicar_estado_web();
    printf("\n================================\nESP32-C3 iniciado com ESP-IDF\n================================\n");
    printf("Comandos: 1=carga, 2=parar, 3=chute, status=estado, 20-200=setpoint, 1%%-100%%=auto-kick\n");
    iniciar_i2c();
    iniciar_wifi_e_servidor();

    while (true) {
        atualizar_leitura();
        controlar_carregamento();
        ler_serial();
        processar_fila_remota();
        imprimir_log();
        publicar_estado_web();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
