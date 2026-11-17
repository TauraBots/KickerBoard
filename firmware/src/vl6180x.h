#pragma once

// Driver mínimo do VL6180X (ST) para medição de distância, sobre I2cSoft.
// Referências: datasheet VL6180X (DocID026171) e nota de aplicação AN4545.
// Atenção: não é compatível com VL53L0X/VL53L1X (mapa de registradores diferente).

#include <cstdint>

#include "esp_err.h"
#include "i2c_soft.h"

namespace vl6180x {

constexpr uint8_t ENDERECO_PADRAO = 0x29;
constexpr uint8_t MODEL_ID_ESPERADO = 0xB4;

// Medições únicas disparadas uma após a outra: a taxa depende do tempo real de
// convergência (curto com alvo próximo e sinal forte, até o máximo sem alvo).
//
// Usamos medições únicas em vez do modo contínuo porque, no modo contínuo, o bit
// SYSRANGE__START alterna o estado (liga/desliga). Se o ESP reinicia com o
// sensor energizado, o firmware perde o estado e pode deixá-lo sempre parado.
// Com medição única, cada disparo tem resultado bem definido.
//
// Tempo máximo de convergência: o padrão do datasheet é 49 ms. 30 ms limita o
// pior caso (sem alvo) a ~25 Hz, com pequena perda de alcance no limite (~150 mm).
constexpr uint8_t MAX_CONVERGENCIA_MS = 30;
constexpr uint32_t TEMPO_MAXIMO_MEDICAO_MS = MAX_CONVERGENCIA_MS + 10;  // + averaging e overhead

// READOUT__AVERAGING_SAMPLE_PERIOD: tempo = 1,3 ms + N x 64,5 us.
// AN4545 recomenda 48 (4,4 ms). 16 (2,3 ms) permite ~120 Hz com alvo próximo,
// ao custo de um pouco mais de ruído na distância.
constexpr uint8_t AVERAGING_AMOSTRAS = 16;

// Duração mínima de uma medição com alvo próximo: pré-calibração (3,2 ms) +
// convergência curta + averaging. Antes disso não adianta consultar o sensor.
constexpr uint32_t ESPERA_MINIMA_MEDICAO_MS = 5;

struct Identificacao {
    uint8_t model_id;
    uint8_t revisao_modelo;   // major.minor
    uint8_t revisao_modulo;   // major.minor
    bool fresh_out_of_reset;  // true se o sensor acabou de ser energizado
    int8_t offset_mm;         // SYSRANGE__PART_TO_PART_RANGE_OFFSET (calibração de fábrica)
    uint16_t crosstalk;       // SYSRANGE__CROSSTALK_COMPENSATION_RATE (9.7 Mcps)
};

struct Amostra {
    uint8_t distancia_bruta_mm;  // Só tem significado quando codigo_erro == 0.
    uint8_t codigo_erro;         // RESULT__RANGE_STATUS[7:4]
    uint8_t erro_sistema;        // RESULT__INTERRUPT_STATUS_GPIO[7:6]: 1 = laser safety, 2 = PLL
    uint16_t taxa_retorno;       // RESULT__RANGE_RETURN_RATE, formato 9.7 Mcps
};

class Sensor {
public:
    explicit Sensor(I2cSoft &barramento, uint8_t endereco = ENDERECO_PADRAO);

    // Verifica o ID, aplica as configurações obrigatórias (se recém-ligado),
    // as configurações públicas recomendadas e a calibração de temperatura (VHV).
    // Garante que o sensor termina parado (mesmo que tenha ficado em modo
    // contínuo após um reset do ESP).
    esp_err_t iniciar(Identificacao *id);

    // Dispara uma medição única. Com verificar = true, confere device_ready e
    // limpa interrupções antes (usar na 1ª medição e após erros). Logo depois de
    // ler_amostra() bem-sucedido isso já está garantido e basta o START.
    esp_err_t disparar_medicao(bool verificar = true);

    // Consulta se há uma nova amostra de distância (sem bloquear).
    esp_err_t amostra_pronta(bool *pronta);

    // Lê a amostra atual e limpa a interrupção para liberar a próxima.
    esp_err_t ler_amostra(Amostra *amostra);

    // Diagnóstico: registradores de estado do ranging.
    struct Registros {
        uint8_t range_status;      // 0x04D: [7:4] erro, [0] device_ready
        uint8_t interrupt_status;  // 0x04F: [7:6] erro sistema, [2:0] evento de range
        uint8_t sysrange_start;    // 0x018
        uint8_t interrupt_config;  // 0x014
        uint8_t fresh_out_of_reset;// 0x016
    };
    esp_err_t ler_registros(Registros *r);

    // Tenta tirar o sensor de um estado ocupado/travado (sem reset por hardware).
    esp_err_t destravar();

    uint8_t endereco() const { return endereco_; }

private:
    esp_err_t escrever8(uint16_t registrador, uint8_t valor);
    esp_err_t ler8(uint16_t registrador, uint8_t *valor);
    esp_err_t ler16(uint16_t registrador, uint16_t *valor);
    esp_err_t ler_bloco(uint16_t registrador, uint8_t *dados, size_t tamanho);  // auto-incremento
    esp_err_t aguardar_pronto(uint32_t timeout_ms);
    esp_err_t carregar_configuracao_obrigatoria();
    esp_err_t garantir_parado();

    I2cSoft &barramento_;
    uint8_t endereco_;
};

// Bit 0 (= VALIDA) é sem erro; os demais seguem a tabela do datasheet.
bool codigo_valido(uint8_t codigo_erro);
const char *descrever_codigo(uint8_t codigo_erro);
inline float taxa_mcps(uint16_t taxa_9_7) { return taxa_9_7 / 128.0f; }

}  // namespace vl6180x
