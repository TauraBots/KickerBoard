#pragma once

// Autorização do chute REAL (lógica pura, sem ESP-IDF).
//
// Todo pedido de chute (Serial "3", I2C 0x03, auto-kick) vira um pedido com
// timestamp. O chute físico só ocorre se, dentro da validade do pedido:
//   - o TOF050C estiver operante;
//   - a última leitura for válida e recente (<= max_idade_tof_ms);
//   - o detector indicar BOLA_PRESENTE (3 confirmações, histerese) OU o
//     preditor indicar que a bola em movimento chega dentro da latência do êmbolo;
//   - o FAULT# do LT3751 estiver inativo.
// Cada pedido gera no máximo UMA autorização. Não há fila: um novo pedido
// substitui o pendente. FAULT# ou cancelamento descartam o pedido.

#include <cstdint>

namespace autorizacao {

enum class Origem : uint8_t { Serial, I2c, AutoKick };

enum class Motivo : uint8_t {
    Nenhum,
    SemPedido,
    PedidoExpirado,
    TofIndisponivel,
    LeituraTofInvalida,
    LeituraTofDesatualizada,
    SemBola,
    FaultLt3751,
    Cancelado,
};

const char *nome_origem(Origem origem);
const char *descrever_motivo(Motivo motivo);

struct Config {
    uint32_t validade_pedido_ms = 200;
    uint32_t max_idade_tof_ms = 100;
};

// Estado do TOF já lido pela tarefa do sensor (cópia; aqui não há I2C) + FAULT#.
struct Entrada {
    bool sensor_ok;
    bool tem_amostra;
    bool leitura_valida;
    bool bola_presente;
    uint64_t instante_ms;
    bool fault;
    // Preditor de chegada: bola em movimento deve tocar o kicker dentro da
    // latência do êmbolo (agora >= t_disparo). Substitui bola_presente.
    bool chegada_prevista = false;
};

struct Decisao {
    enum class Tipo : uint8_t {
        Nada,        // sem pedido, ou pedido aguardando sem mudança
        Bloqueio,    // pedido pendente bloqueado (motivo mudou desde a última decisão)
        Autorizado,  // CHUTAR AGORA
        Expirado,    // pedido descartado por prazo; motivo = último bloqueio
        Descartado,  // pedido descartado por FAULT#
    };
    Tipo tipo;
    Motivo motivo;
    Origem origem;
    uint32_t idade_pedido_ms;
};

class Autorizador {
public:
    explicit Autorizador(const Config &config = Config()) : config_(config) {}

    // Registra um pedido. Retorna true se substituiu um pedido pendente.
    bool solicitar(uint64_t agora_ms, Origem origem);
    // Descarta o pedido pendente (ex.: comando "parar"). Retorna true se havia um.
    bool cancelar();

    Decisao atualizar(uint64_t agora_ms, const Entrada &entrada);
    Motivo avaliar(uint64_t agora_ms, const Entrada &entrada) const;

    bool pendente() const { return pendente_; }
    uint32_t autorizacoes() const { return autorizacoes_; }
    uint32_t bloqueios() const { return bloqueios_; }

private:
    Config config_;
    bool pendente_ = false;
    uint64_t t_pedido_ms_ = 0;
    Origem origem_ = Origem::Serial;
    Motivo ultimo_motivo_ = Motivo::Nenhum;
    uint32_t autorizacoes_ = 0;
    uint32_t bloqueios_ = 0;  // pedidos expirados ou descartados sem chute
};

}  // namespace autorizacao
