#include "vl6180x.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace vl6180x {
namespace {

// Registradores públicos (datasheet VL6180X, seção 6).
constexpr uint16_t IDENTIFICATION__MODEL_ID = 0x000;
constexpr uint16_t IDENTIFICATION__MODEL_REV_MAJOR = 0x001;
constexpr uint16_t IDENTIFICATION__MODEL_REV_MINOR = 0x002;
constexpr uint16_t IDENTIFICATION__MODULE_REV_MAJOR = 0x003;
constexpr uint16_t IDENTIFICATION__MODULE_REV_MINOR = 0x004;
constexpr uint16_t SYSTEM__MODE_GPIO1 = 0x011;
constexpr uint16_t SYSTEM__INTERRUPT_CONFIG_GPIO = 0x014;
constexpr uint16_t SYSTEM__INTERRUPT_CLEAR = 0x015;
constexpr uint16_t SYSTEM__FRESH_OUT_OF_RESET = 0x016;
constexpr uint16_t SYSRANGE__START = 0x018;
constexpr uint16_t SYSRANGE__INTERMEASUREMENT_PERIOD = 0x01B;  // só para o modo contínuo
constexpr uint16_t SYSRANGE__MAX_CONVERGENCE_TIME = 0x01C;
constexpr uint16_t SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x01E;
constexpr uint16_t SYSRANGE__PART_TO_PART_RANGE_OFFSET = 0x024;
constexpr uint16_t SYSRANGE__VHV_RECALIBRATE = 0x02E;
constexpr uint16_t SYSRANGE__VHV_REPEAT_RATE = 0x031;
constexpr uint16_t SYSALS__INTERMEASUREMENT_PERIOD = 0x03E;
constexpr uint16_t SYSALS__ANALOGUE_GAIN = 0x03F;
constexpr uint16_t SYSALS__INTEGRATION_PERIOD = 0x040;
constexpr uint16_t RESULT__RANGE_STATUS = 0x04D;
constexpr uint16_t RESULT__INTERRUPT_STATUS_GPIO = 0x04F;
constexpr uint16_t RESULT__RANGE_VAL = 0x062;
constexpr uint16_t RESULT__RANGE_RETURN_RATE = 0x066;
constexpr uint16_t READOUT__AVERAGING_SAMPLE_PERIOD = 0x10A;

constexpr uint8_t RANGE_NOVA_AMOSTRA = 0x04;  // RESULT__INTERRUPT_STATUS_GPIO[2:0]
constexpr uint8_t LIMPAR_TODAS_INTERRUPCOES = 0x07;

struct Par {
    uint16_t registrador;
    uint8_t valor;
};

// Configuração obrigatória "private registers" (AN4545 / datasheet seção 9).
// Deve ser carregada uma vez após cada energização (SYSTEM__FRESH_OUT_OF_RESET = 1).
constexpr Par CONFIG_OBRIGATORIA[] = {
    {0x0207, 0x01}, {0x0208, 0x01}, {0x0096, 0x00}, {0x0097, 0xfd}, {0x00e3, 0x00},
    {0x00e4, 0x04}, {0x00e5, 0x02}, {0x00e6, 0x01}, {0x00e7, 0x03}, {0x00f5, 0x02},
    {0x00d9, 0x05}, {0x00db, 0xce}, {0x00dc, 0x03}, {0x00dd, 0xf8}, {0x009f, 0x00},
    {0x00a3, 0x3c}, {0x00b7, 0x00}, {0x00bb, 0x3c}, {0x00b2, 0x09}, {0x00ca, 0x09},
    {0x0198, 0x01}, {0x01b0, 0x17}, {0x01ad, 0x00}, {0x00ff, 0x05}, {0x0100, 0x05},
    {0x0199, 0x05}, {0x01a6, 0x1b}, {0x01ac, 0x3e}, {0x01a7, 0x1f}, {0x0030, 0x00},
};

uint64_t agora_ms() { return static_cast<uint64_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS; }

}  // namespace

Sensor::Sensor(I2cSoft &barramento, uint8_t endereco) : barramento_(barramento), endereco_(endereco) {}

esp_err_t Sensor::escrever8(uint16_t registrador, uint8_t valor)
{
    const uint8_t dados[3] = {static_cast<uint8_t>(registrador >> 8),
                              static_cast<uint8_t>(registrador & 0xff), valor};
    return barramento_.escrever(endereco_, dados, sizeof(dados));
}

esp_err_t Sensor::ler8(uint16_t registrador, uint8_t *valor)
{
    const uint8_t reg[2] = {static_cast<uint8_t>(registrador >> 8),
                            static_cast<uint8_t>(registrador & 0xff)};
    return barramento_.escrever_ler(endereco_, reg, sizeof(reg), valor, 1);
}

esp_err_t Sensor::ler16(uint16_t registrador, uint16_t *valor)
{
    const uint8_t reg[2] = {static_cast<uint8_t>(registrador >> 8),
                            static_cast<uint8_t>(registrador & 0xff)};
    uint8_t dados[2] = {};
    const esp_err_t erro = barramento_.escrever_ler(endereco_, reg, sizeof(reg), dados, sizeof(dados));
    if (erro == ESP_OK) *valor = static_cast<uint16_t>((dados[0] << 8) | dados[1]);  // big-endian
    return erro;
}

esp_err_t Sensor::ler_bloco(uint16_t registrador, uint8_t *dados, size_t tamanho)
{
    const uint8_t reg[2] = {static_cast<uint8_t>(registrador >> 8),
                            static_cast<uint8_t>(registrador & 0xff)};
    return barramento_.escrever_ler(endereco_, reg, sizeof(reg), dados, tamanho);
}

esp_err_t Sensor::aguardar_pronto(uint32_t timeout_ms)
{
    // RESULT__RANGE_STATUS bit 0 = range_device_ready.
    const uint64_t inicio = agora_ms();
    while (true) {
        uint8_t status = 0;
        const esp_err_t erro = ler8(RESULT__RANGE_STATUS, &status);
        if (erro != ESP_OK) return erro;
        if (status & 0x01) return ESP_OK;
        if (agora_ms() - inicio >= timeout_ms) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

esp_err_t Sensor::carregar_configuracao_obrigatoria()
{
    for (const Par &par : CONFIG_OBRIGATORIA) {
        const esp_err_t erro = escrever8(par.registrador, par.valor);
        if (erro != ESP_OK) return erro;
    }
    return ESP_OK;
}

esp_err_t Sensor::iniciar(Identificacao *id)
{
    Identificacao info = {};
    uint8_t maior = 0;
    uint8_t menor = 0;
    uint8_t fresh = 0;
    uint8_t offset = 0;

    esp_err_t erro = ler8(IDENTIFICATION__MODEL_ID, &info.model_id);
    if (erro != ESP_OK) return erro;
    if (id) *id = info;
    if (info.model_id != MODEL_ID_ESPERADO) return ESP_ERR_INVALID_RESPONSE;

    if ((erro = ler8(IDENTIFICATION__MODEL_REV_MAJOR, &maior)) != ESP_OK) return erro;
    if ((erro = ler8(IDENTIFICATION__MODEL_REV_MINOR, &menor)) != ESP_OK) return erro;
    info.revisao_modelo = static_cast<uint8_t>(((maior & 0x07) << 4) | (menor & 0x07));
    if ((erro = ler8(IDENTIFICATION__MODULE_REV_MAJOR, &maior)) != ESP_OK) return erro;
    if ((erro = ler8(IDENTIFICATION__MODULE_REV_MINOR, &menor)) != ESP_OK) return erro;
    info.revisao_modulo = static_cast<uint8_t>(((maior & 0x07) << 4) | (menor & 0x07));

    if ((erro = ler8(SYSTEM__FRESH_OUT_OF_RESET, &fresh)) != ESP_OK) return erro;
    info.fresh_out_of_reset = (fresh & 0x01) != 0;

    if (info.fresh_out_of_reset) {
        if ((erro = carregar_configuracao_obrigatoria()) != ESP_OK) return erro;
        if ((erro = escrever8(SYSTEM__FRESH_OUT_OF_RESET, 0x00)) != ESP_OK) return erro;
    } else {
        // O ESP reiniciou, mas o sensor continuou energizado e pode ter ficado
        // em modo contínuo (firmware anterior). Detecta e interrompe.
        if ((erro = garantir_parado()) != ESP_OK) return erro;
    }

    // Configurações públicas recomendadas (AN4545).
    const Par publicas[] = {
        {SYSTEM__MODE_GPIO1, 0x10},                     // GPIO1 sinaliza "nova amostra" (não usado aqui)
        {READOUT__AVERAGING_SAMPLE_PERIOD, AVERAGING_AMOSTRAS},
        {SYSALS__ANALOGUE_GAIN, 0x46},                  // ganho ALS (ALS não é usado)
        {SYSRANGE__VHV_REPEAT_RATE, 0xFF},              // recalibração automática a cada 255 medições
        {SYSALS__INTEGRATION_PERIOD, 0x63},
        {SYSALS__INTERMEASUREMENT_PERIOD, 0x31},
        {SYSTEM__INTERRUPT_CONFIG_GPIO, RANGE_NOVA_AMOSTRA},
        {SYSRANGE__MAX_CONVERGENCE_TIME, MAX_CONVERGENCIA_MS},
        {SYSRANGE__INTERMEASUREMENT_PERIOD, 0x06},      // só vale no modo contínuo (não usado)
    };
    for (const Par &par : publicas)
        if ((erro = escrever8(par.registrador, par.valor)) != ESP_OK) return erro;

    if ((erro = aguardar_pronto(300)) != ESP_OK) return erro;

    // Calibração de temperatura (VHV) única; o bit volta a 0 ao terminar.
    if ((erro = escrever8(SYSRANGE__VHV_RECALIBRATE, 0x01)) != ESP_OK) return erro;
    const uint64_t inicio = agora_ms();
    while (true) {
        uint8_t vhv = 0;
        if ((erro = ler8(SYSRANGE__VHV_RECALIBRATE, &vhv)) != ESP_OK) return erro;
        if (!(vhv & 0x01)) break;
        if (agora_ms() - inicio >= 100) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // Valores de calibração de fábrica (somente leitura para diagnóstico).
    if ((erro = ler8(SYSRANGE__PART_TO_PART_RANGE_OFFSET, &offset)) != ESP_OK) return erro;
    info.offset_mm = static_cast<int8_t>(offset);
    if ((erro = ler16(SYSRANGE__CROSSTALK_COMPENSATION_RATE, &info.crosstalk)) != ESP_OK) return erro;

    if ((erro = escrever8(SYSTEM__INTERRUPT_CLEAR, LIMPAR_TODAS_INTERRUPCOES)) != ESP_OK) return erro;
    if (id) *id = info;
    return ESP_OK;
}

esp_err_t Sensor::garantir_parado()
{
    // O sensor só é considerado parado quando, após uma janela de observação,
    // (a) não gerou amostras sozinho e (b) está pronto (device_ready = 1).
    // Caso contrário está em modo contínuo ou ocupado: um START (0x01) o
    // interrompe. Repete a verificação sem depender do estado anterior.
    constexpr uint32_t JANELA_MS = 250;  // > 3x o maior período contínuo usado
    for (int tentativa = 0; tentativa < 4; ++tentativa) {
        esp_err_t erro = escrever8(SYSTEM__INTERRUPT_CLEAR, LIMPAR_TODAS_INTERRUPCOES);
        if (erro != ESP_OK) return erro;
        vTaskDelay(pdMS_TO_TICKS(JANELA_MS));
        bool pronta = false;
        uint8_t status = 0;
        if ((erro = amostra_pronta(&pronta)) != ESP_OK) return erro;
        if ((erro = ler8(RESULT__RANGE_STATUS, &status)) != ESP_OK) return erro;
        if (!pronta && (status & 0x01)) return escrever8(SYSTEM__INTERRUPT_CLEAR, LIMPAR_TODAS_INTERRUPCOES);
        if ((erro = escrever8(SYSRANGE__START, 0x01)) != ESP_OK) return erro;
        vTaskDelay(pdMS_TO_TICKS(TEMPO_MAXIMO_MEDICAO_MS + 20));
    }
    return ESP_ERR_INVALID_STATE;  // continua ocupado ou produzindo amostras sozinho
}

esp_err_t Sensor::ler_registros(Registros *r)
{
    esp_err_t erro;
    if ((erro = ler8(RESULT__RANGE_STATUS, &r->range_status)) != ESP_OK) return erro;
    if ((erro = ler8(RESULT__INTERRUPT_STATUS_GPIO, &r->interrupt_status)) != ESP_OK) return erro;
    if ((erro = ler8(SYSRANGE__START, &r->sysrange_start)) != ESP_OK) return erro;
    if ((erro = ler8(SYSTEM__INTERRUPT_CONFIG_GPIO, &r->interrupt_config)) != ESP_OK) return erro;
    return ler8(SYSTEM__FRESH_OUT_OF_RESET, &r->fresh_out_of_reset);
}

esp_err_t Sensor::destravar() { return garantir_parado(); }

esp_err_t Sensor::disparar_medicao(bool verificar)
{
    if (verificar) {
        uint8_t status = 0;
        esp_err_t erro = ler8(RESULT__RANGE_STATUS, &status);
        if (erro != ESP_OK) return erro;
        if (!(status & 0x01)) return ESP_ERR_INVALID_STATE;  // ainda ocupado
        // Necessário: testado em hardware, sem esta limpeza o VL6180X passa a
        // ignorar os STARTs e trava (só volta desligando a alimentação).
        if ((erro = escrever8(SYSTEM__INTERRUPT_CLEAR, LIMPAR_TODAS_INTERRUPCOES)) != ESP_OK) return erro;
    }
    return escrever8(SYSRANGE__START, 0x01);  // startstop = 1, modo single-shot
}

esp_err_t Sensor::amostra_pronta(bool *pronta)
{
    uint8_t status = 0;
    const esp_err_t erro = ler8(RESULT__INTERRUPT_STATUS_GPIO, &status);
    if (erro == ESP_OK) *pronta = (status & 0x07) == RANGE_NOVA_AMOSTRA;
    return erro;
}

esp_err_t Sensor::ler_amostra(Amostra *amostra)
{
    // Duas leituras em bloco (auto-incremento) em vez de quatro transações:
    //   0x04D RANGE_STATUS, 0x04E ALS_STATUS, 0x04F INTERRUPT_STATUS_GPIO
    //   0x062 RANGE_VAL ... 0x066-0x067 RANGE_RETURN_RATE (big-endian)
    static_assert(RESULT__INTERRUPT_STATUS_GPIO == RESULT__RANGE_STATUS + 2, "mapa de registradores");
    static_assert(RESULT__RANGE_RETURN_RATE == RESULT__RANGE_VAL + 4, "mapa de registradores");
    Amostra a = {};
    uint8_t status[3] = {};
    uint8_t resultado[6] = {};
    esp_err_t erro = ler_bloco(RESULT__RANGE_STATUS, status, sizeof(status));
    if (erro != ESP_OK) return erro;
    a.codigo_erro = static_cast<uint8_t>(status[0] >> 4);
    a.erro_sistema = static_cast<uint8_t>(status[2] >> 6);
    if ((erro = ler_bloco(RESULT__RANGE_VAL, resultado, sizeof(resultado))) != ESP_OK) return erro;
    a.distancia_bruta_mm = resultado[0];
    a.taxa_retorno = static_cast<uint16_t>((resultado[4] << 8) | resultado[5]);
    if ((erro = escrever8(SYSTEM__INTERRUPT_CLEAR, LIMPAR_TODAS_INTERRUPCOES)) != ESP_OK) return erro;
    *amostra = a;
    return ESP_OK;
}

bool codigo_valido(uint8_t codigo_erro) { return codigo_erro == 0; }

const char *descrever_codigo(uint8_t codigo_erro)
{
    // Tabela RESULT__RANGE_STATUS[7:4] do datasheet.
    switch (codigo_erro) {
        case 0: return "sem erro";
        case 1: return "VCSEL continuity test";
        case 2: return "VCSEL watchdog test";
        case 3: return "VCSEL watchdog";
        case 4: return "PLL1 lock";
        case 5: return "PLL2 lock";
        case 6: return "early convergence estimate (sinal baixo)";
        case 7: return "max convergence (sinal insuficiente)";
        case 8: return "no target ignore";
        case 11: return "max signal-to-noise (luz ambiente)";
        case 12: return "raw ranging underflow";
        case 13: return "raw ranging overflow";
        case 14: return "ranging underflow (distancia < 0)";
        case 15: return "ranging overflow (fora do alcance)";
        default: return "reservado";
    }
}

}  // namespace vl6180x
