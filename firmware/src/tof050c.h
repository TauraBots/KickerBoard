#pragma once

// Monitor de distância TOF050C (VL6180X) — somente diagnóstico.
//
// Este módulo NÃO atua no kicker: não carrega, não dispara e não altera o
// protocolo I2C escravo (0x42). Ele apenas lê o sensor em um barramento I2C
// por software próprio e publica/imprime as leituras.

#include <cstdint>

namespace tof050c {

// Barramento I2C por software dedicado ao sensor (independente do escravo em GPIO6/GPIO7).
// GPIO5/GPIO10: não são pinos de strapping do ESP32-C3 (2, 8 e 9), não são o
// UART0 da ROM (20/21) e não aparecem no esquemático Schematic1_3.
constexpr int PIN_SDA = 5;
constexpr int PIN_SCL = 10;
// XSHUT do módulo: reset por hardware do VL6180X. GPIO20 (RX do UART0, sem uso:
// o console é USB) não é strapping e fica só como entrada durante o boot.
// Open-drain: só puxa para 0; o nível alto vem do pull-up do próprio módulo.
constexpr int PIN_XSHUT = 20;

struct Leitura {
    bool sensor_ok;           // sensor inicializado e respondendo
    bool tem_amostra;         // já houve ao menos uma amostra desde a inicialização
    bool valida;              // última amostra sem erro (codigo_erro == 0)
    uint8_t distancia_mm;     // só tem significado se valida == true
    uint8_t distancia_bruta;  // RESULT__RANGE_VAL cru, mesmo quando inválido
    uint8_t codigo_erro;      // RESULT__RANGE_STATUS[7:4]
    uint8_t erro_sistema;     // 1 = laser safety, 2 = PLL
    float sinal_mcps;         // taxa de retorno do sinal
    uint64_t instante_ms;     // momento real da última amostra (esp_timer, ms)
    bool bola_presente;       // estado do detector (BOLA_PRESENTE / BOLA_AUSENTE)
    uint8_t bola_confirmacoes;// confirmações acumuladas em BOLA_AUSENTE
    uint32_t bola_deteccoes;  // transições AUSENTE -> PRESENTE desde o boot
    uint32_t amostras;
    uint32_t invalidas;
    uint32_t erros_i2c;
    uint32_t timeouts;
    uint32_t falhas_inicializacao;
};

// Cria a tarefa do sensor. Uma falha aqui não interrompe o firmware do kicker.
void iniciar();

// Cópia consistente da última leitura (protegida por mutex). Retorna false
// se o módulo não foi iniciado.
bool obter_leitura(Leitura *destino);

// Presença da bola utilizável agora: sensor OK, estado BOLA_PRESENTE e última
// leitura com no máximo bola::MAX_IDADE_LEITURA_MS. Nesta etapa NÃO é usada
// para acionar o kicker; existe para validar a detecção.
bool bola_presente_autorizada();

// Comando serial "tof [arg]": sem argumento liga/desliga as linhas "[TOF] ...";
// "r" alterna entre 5 linhas/s e todas as amostras.
void processar_comando(const char *argumento);

}  // namespace tof050c
