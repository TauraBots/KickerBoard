#pragma once

// Máquina de estados do kicker em MODO SIMULAÇÃO (Etapa 3).
//
// Lógica pura (sem ESP-IDF/FreeRTOS/GPIO): integra a tensão SIMULADA do banco
// de capacitores, o estado de presença da bola do TOF050C e o comando de chute,
// e decide se um chute SERIA autorizado.
//
// Separação explícita:
//  - Estado lógico de autorização: Estado / Motivo (o que a lógica decidiu).
//  - Estado físico das saídas: SaidasFisicas (o que os pinos devem fazer).
//    Nesta etapa, PERMITE_ACIONAMENTO_FISICO = false e as saídas físicas são
//    SEMPRE desligadas, qualquer que seja a decisão lógica.
//
// A tensão simulada nunca é usada para acionar nada: ela só alimenta a lógica.

#include <cstdint>

namespace kicker_sim {

constexpr bool PERMITE_ACIONAMENTO_FISICO = false;

enum class Estado : uint8_t {
    Desabilitado,
    PreCarga,
    AguardandoComando,
    CargaSolicitada,
    AguardandoBola,
    ChuteAutorizado,
    ChuteBloqueado,
    Falha,
    Emergencia,
};

enum class Motivo : uint8_t {
    Nenhum,
    SemComando,
    ComandoExpirado,
    TensaoInsuficiente,
    TensaoForaDaFaixa,
    TofIndisponivel,
    LeituraTofInvalida,
    LeituraTofDesatualizada,
    SemBola,
    Emergencia,
    Falha,
    FalhaExterna,
    Desabilitado,
    CargaNaoSolicitada,
    CancelamentoExplicito,
    SetpointForaDosLimites,
    SetpointAbaixoDaTensao,
    EmergenciaAindaAtiva,
    FalhaAindaAtiva,
    EstadoNaoPermite,
    SubstituidoPorNovoComando,
};

const char *nome_estado(Estado estado);
const char *descrever_motivo(Motivo motivo);

struct Config {
    float tensao_pre_carga_v = 50.0f;
    float setpoint_min_v = 20.0f;        // limites já usados pelo firmware
    float setpoint_max_v = 200.0f;
    float tensao_max_seguranca_v = 205.0f;
    float tolerancia_v = 2.0f;           // faixa de autorização: setpoint ± tolerância
    float taxa_carga_v_por_s = 100.0f;   // subida da tensão SIMULADA
    uint32_t validade_comando_ms = 200;  // comando de chute expira após este prazo
    uint32_t max_idade_tof_ms = 100;     // idade máxima da leitura do TOF
};

// Cópia do estado do TOF050C (já lido pela tarefa do sensor; aqui não há I2C).
struct EntradaTof {
    bool sensor_ok;
    bool tem_amostra;
    bool leitura_valida;
    bool bola_presente;     // estado do detector de presença (3 confirmações, histerese)
    uint64_t instante_ms;   // timestamp real da última amostra
    uint8_t distancia_mm;   // só para log
};

struct SaidasFisicas {
    bool carregamento;  // GPIO4
    bool chute;         // GPIO1
};

struct Notificacao {
    enum class Tipo : uint8_t {
        Transicao,          // de -> para, com motivo
        TensaoAtingida,     // valor = tensão simulada
        SetpointAceito,     // valor = setpoint
        SetpointRejeitado,  // valor = setpoint pedido, motivo
        ChuteRecebido,
        ChuteSubstituido,   // havia um comando pendente; o novo o substitui
        ChuteRejeitado,     // motivo
        ChuteExpirado,      // motivo = último bloqueio antes de expirar
        ChuteCancelado,     // motivo
        Bloqueio,           // comando pendente bloqueado (motivo mudou)
        Autorizado,         // CHUTE_AUTORIZADO (simulação)
        RearmeNegado,       // motivo
    };
    Tipo tipo;
    Estado de;
    Estado para;
    Motivo motivo;
    float valor;
};

class Ouvinte {
public:
    virtual ~Ouvinte() = default;
    virtual void notificar(const Notificacao &n) = 0;
};

class Maquina {
public:
    explicit Maquina(const Config &config = Config(), Ouvinte *ouvinte = nullptr);

    // Chamado periodicamente. falha_externa: FAULT# real ou falha simulada.
    void atualizar(uint64_t agora_ms, const EntradaTof &tof, bool falha_externa);

    // Comandos (Serial/I2C são traduzidos para estes eventos).
    bool habilitar(uint64_t agora_ms);
    void desabilitar(uint64_t agora_ms);
    bool solicitar_carga(uint64_t agora_ms, float setpoint_v);
    void parar(uint64_t agora_ms);
    bool solicitar_chute(uint64_t agora_ms);
    void cancelar_chute(uint64_t agora_ms);
    void sinalizar_emergencia(uint64_t agora_ms, bool ativa);
    void sinalizar_falha(uint64_t agora_ms);
    bool rearmar(uint64_t agora_ms);

    // Saídas físicas: SEMPRE desligadas nesta etapa.
    SaidasFisicas saidas_fisicas() const;

    Estado estado() const { return estado_; }
    float tensao_simulada() const { return tensao_v_; }
    float setpoint() const { return setpoint_v_; }
    bool chute_pendente() const { return chute_pendente_; }
    Motivo ultimo_bloqueio() const { return ultimo_bloqueio_; }
    uint32_t autorizacoes() const { return autorizacoes_; }
    const Config &config() const { return config_; }

    // Motivo pelo qual um chute NÃO seria autorizado agora (Nenhum = autorizaria).
    Motivo avaliar(uint64_t agora_ms, const EntradaTof &tof) const;

private:
    void ir_para(Estado novo, Motivo motivo);
    void notificar(Notificacao::Tipo tipo, Motivo motivo = Motivo::Nenhum, float valor = 0.0f);
    void descartar_chute(Motivo motivo, bool expirado);
    void entrar_em_falha(Motivo motivo);
    void integrar_tensao(uint64_t agora_ms);

    Config config_;
    Ouvinte *ouvinte_;
    Estado estado_ = Estado::Desabilitado;
    float tensao_v_ = 0.0f;
    float setpoint_v_ = 0.0f;
    bool carga_solicitada_ = false;
    bool chute_pendente_ = false;
    uint64_t t_chute_ms_ = 0;
    bool emergencia_ativa_ = false;
    bool falha_ativa_ = false;   // última condição de falha observada
    bool tem_t_anterior_ = false;
    uint64_t t_anterior_ms_ = 0;
    Motivo ultimo_bloqueio_ = Motivo::Nenhum;
    uint32_t autorizacoes_ = 0;
};

}  // namespace kicker_sim
