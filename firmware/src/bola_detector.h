#pragma once

// Detector de presença da bola a partir das amostras do TOF050C (VL6180X).
//
// Lógica pura (sem ESP-IDF/FreeRTOS): recebe amostras com timestamp e eventos
// de falha, e mantém o estado BOLA_AUSENTE / BOLA_PRESENTE. Não aciona nada;
// nesta etapa serve apenas para validar a confiabilidade da detecção.
//
// Regras:
//  - AUSENTE -> PRESENTE: N leituras VÁLIDAS e CONSECUTIVAS com distância
//    <= limite_entrada_mm. Leitura inválida, acima do limite ou com intervalo
//    maior que max_idade_ms desde a anterior zera a contagem.
//  - PRESENTE -> AUSENTE (histerese): leitura válida > limite_saida_mm.
//  - Falha => AUSENTE imediatamente (fail-safe): leitura inválida, timeout,
//    erro I2C/sensor ou última leitura mais velha que max_idade_ms.

#include <cstdint>

namespace bola {

// Parâmetros iniciais (ajustar após os testes físicos).
constexpr uint8_t LIMITE_ENTRADA_MM = 49;    // leitura < 50 mm = posse da bola (definido pela equipe)
constexpr uint8_t LIMITE_SAIDA_MM = 55;      // histerese de 6 mm
constexpr uint8_t CONFIRMACOES_NECESSARIAS = 3;
constexpr uint32_t MAX_IDADE_LEITURA_MS = 100;

static_assert(LIMITE_SAIDA_MM >= LIMITE_ENTRADA_MM, "a histerese exige saida >= entrada");
static_assert(CONFIRMACOES_NECESSARIAS >= 1, "pelo menos uma confirmacao");

enum class Estado : uint8_t { Ausente, Presente };

enum class Causa : uint8_t {
    Nenhuma,
    Confirmada,          // N leituras válidas <= limite de entrada
    AcimaDoLimiteSaida,  // leitura válida > limite de saída
    LeituraInvalida,     // status de range != 0 ou erro de sistema
    LeituraDesatualizada,// sem leitura nova dentro de max_idade_ms
    Timeout,             // medição sem resultado
    ErroSensor,          // erro I2C, reinicialização, sensor ausente
};

const char *nome_estado(Estado estado);
const char *descrever_causa(Causa causa);

// Resultado de cada chamada: informa se houve transição e por quê.
struct Evento {
    bool transicao;
    Estado estado;      // estado após a chamada
    Causa causa;        // causa da transição (Nenhuma se não houve)
    uint8_t confirmacoes; // contagem atual em AUSENTE (0..N)
    uint32_t idade_ms;  // para LeituraDesatualizada
};

class Detector {
public:
    // Amostra do sensor. t_ms deve ser o instante real da leitura (monotônico).
    Evento processar_amostra(bool valida, uint8_t distancia_mm, uint64_t t_ms);

    // Chamar periodicamente: invalida a presença se a última leitura envelheceu.
    Evento verificar_idade(uint64_t agora_ms);

    // Falhas externas à amostra (timeout, erro I2C, reinicialização).
    Evento invalidar(Causa causa);

    // Presença utilizável agora: PRESENTE e última leitura dentro de max_idade_ms.
    bool presenca_autorizada(uint64_t agora_ms) const;

    Estado estado() const { return estado_; }
    uint8_t confirmacoes() const { return confirmacoes_; }
    uint32_t transicoes_para_presente() const { return transicoes_presente_; }

private:
    Evento sem_transicao() const;
    Evento ir_para(Estado novo, Causa causa, uint32_t idade_ms = 0);

    Estado estado_ = Estado::Ausente;
    uint8_t confirmacoes_ = 0;
    bool tem_leitura_ = false;
    uint64_t t_ultima_leitura_ms_ = 0;  // última amostra recebida (válida ou não)
    uint32_t transicoes_presente_ = 0;
};

}  // namespace bola
