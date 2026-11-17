#include "kicker_sim.h"

namespace kicker_sim {

const char *nome_estado(Estado estado)
{
    switch (estado) {
        case Estado::Desabilitado: return "DESABILITADO";
        case Estado::PreCarga: return "PRE_CARGA";
        case Estado::AguardandoComando: return "AGUARDANDO_COMANDO";
        case Estado::CargaSolicitada: return "CARGA_SOLICITADA";
        case Estado::AguardandoBola: return "AGUARDANDO_BOLA";
        case Estado::ChuteAutorizado: return "CHUTE_AUTORIZADO";
        case Estado::ChuteBloqueado: return "CHUTE_BLOQUEADO";
        case Estado::Falha: return "FALHA";
        case Estado::Emergencia: return "EMERGENCIA";
    }
    return "?";
}

const char *descrever_motivo(Motivo motivo)
{
    switch (motivo) {
        case Motivo::Nenhum: return "-";
        case Motivo::SemComando: return "sem comando de chute";
        case Motivo::ComandoExpirado: return "comando de chute expirado";
        case Motivo::TensaoInsuficiente: return "tensao insuficiente";
        case Motivo::TensaoForaDaFaixa: return "tensao fora da faixa";
        case Motivo::TofIndisponivel: return "TOF indisponivel";
        case Motivo::LeituraTofInvalida: return "leitura do TOF invalida";
        case Motivo::LeituraTofDesatualizada: return "leitura do TOF desatualizada";
        case Motivo::SemBola: return "sem bola";
        case Motivo::Emergencia: return "emergencia";
        case Motivo::Falha: return "falha";
        case Motivo::FalhaExterna: return "falha externa (FAULT#/simulada)";
        case Motivo::Desabilitado: return "sistema desabilitado";
        case Motivo::CargaNaoSolicitada: return "carga nao solicitada";
        case Motivo::CancelamentoExplicito: return "cancelamento explicito";
        case Motivo::SetpointForaDosLimites: return "setpoint fora dos limites";
        case Motivo::SetpointAbaixoDaTensao: return "setpoint abaixo da tensao atual (sem descarga controlada)";
        case Motivo::EmergenciaAindaAtiva: return "emergencia ainda ativa";
        case Motivo::FalhaAindaAtiva: return "falha ainda ativa";
        case Motivo::EstadoNaoPermite: return "estado atual nao permite";
        case Motivo::SubstituidoPorNovoComando: return "substituido por novo comando";
    }
    return "?";
}

Maquina::Maquina(const Config &config, Ouvinte *ouvinte) : config_(config), ouvinte_(ouvinte) {}

void Maquina::notificar(Notificacao::Tipo tipo, Motivo motivo, float valor)
{
    if (ouvinte_) ouvinte_->notificar(Notificacao{tipo, estado_, estado_, motivo, valor});
}

void Maquina::ir_para(Estado novo, Motivo motivo)
{
    if (novo == estado_) return;
    const Estado anterior = estado_;
    estado_ = novo;
    if (ouvinte_) ouvinte_->notificar(Notificacao{Notificacao::Tipo::Transicao, anterior, novo, motivo, tensao_v_});
}

void Maquina::descartar_chute(Motivo motivo, bool expirado)
{
    if (!chute_pendente_) return;
    chute_pendente_ = false;
    notificar(expirado ? Notificacao::Tipo::ChuteExpirado : Notificacao::Tipo::ChuteCancelado, motivo);
}

void Maquina::entrar_em_falha(Motivo motivo)
{
    if (estado_ == Estado::Emergencia) return;
    descartar_chute(motivo, false);
    carga_solicitada_ = false;
    ir_para(Estado::Falha, motivo);
}

void Maquina::integrar_tensao(uint64_t agora_ms)
{
    const uint64_t dt_ms = (tem_t_anterior_ && agora_ms > t_anterior_ms_) ? agora_ms - t_anterior_ms_ : 0;
    tem_t_anterior_ = true;
    t_anterior_ms_ = agora_ms;

    float alvo = -1.0f;  // < 0: tensão simulada fica parada
    if (estado_ == Estado::PreCarga) alvo = config_.tensao_pre_carga_v;
    else if (carga_solicitada_ && (estado_ == Estado::CargaSolicitada || estado_ == Estado::ChuteBloqueado ||
                                   estado_ == Estado::AguardandoBola))
        alvo = setpoint_v_;
    if (alvo > config_.tensao_max_seguranca_v) alvo = config_.tensao_max_seguranca_v;
    if (alvo < 0.0f || tensao_v_ >= alvo) return;

    tensao_v_ += config_.taxa_carga_v_por_s * static_cast<float>(dt_ms) / 1000.0f;
    if (tensao_v_ > alvo) tensao_v_ = alvo;
}

Motivo Maquina::avaliar(uint64_t agora_ms, const EntradaTof &tof) const
{
    if (estado_ == Estado::Emergencia || emergencia_ativa_) return Motivo::Emergencia;
    if (estado_ == Estado::Falha || falha_ativa_) return Motivo::Falha;
    if (estado_ == Estado::Desabilitado) return Motivo::Desabilitado;
    if (!chute_pendente_) return Motivo::SemComando;
    // Timestamp do comando no futuro também é tratado como expirado (conservador).
    if (t_chute_ms_ > agora_ms || agora_ms - t_chute_ms_ > config_.validade_comando_ms)
        return Motivo::ComandoExpirado;
    if (!carga_solicitada_) return Motivo::CargaNaoSolicitada;
    if (tensao_v_ < setpoint_v_ - config_.tolerancia_v) return Motivo::TensaoInsuficiente;
    if (tensao_v_ > setpoint_v_ + config_.tolerancia_v || tensao_v_ > config_.tensao_max_seguranca_v)
        return Motivo::TensaoForaDaFaixa;
    if (!tof.sensor_ok) return Motivo::TofIndisponivel;
    if (!tof.tem_amostra || !tof.leitura_valida || tof.instante_ms > agora_ms) return Motivo::LeituraTofInvalida;
    if (agora_ms - tof.instante_ms > config_.max_idade_tof_ms) return Motivo::LeituraTofDesatualizada;
    if (!tof.bola_presente) return Motivo::SemBola;
    return Motivo::Nenhum;
}

void Maquina::atualizar(uint64_t agora_ms, const EntradaTof &tof, bool falha_externa)
{
    integrar_tensao(agora_ms);
    falha_ativa_ = falha_externa;

    if (estado_ == Estado::Emergencia) return;
    if (falha_externa && estado_ != Estado::Falha) {
        entrar_em_falha(Motivo::FalhaExterna);
        return;
    }
    if (estado_ == Estado::Falha || estado_ == Estado::Desabilitado) return;

    // Perda do sensor enquanto há carga/chute em andamento => FALHA (exige rearme).
    const bool em_espera = estado_ == Estado::CargaSolicitada || estado_ == Estado::AguardandoBola ||
                           estado_ == Estado::ChuteBloqueado;
    if (em_espera && !tof.sensor_ok) {
        entrar_em_falha(Motivo::TofIndisponivel);
        return;
    }

    switch (estado_) {
        case Estado::PreCarga:
            if (tensao_v_ >= config_.tensao_pre_carga_v) {
                notificar(Notificacao::Tipo::TensaoAtingida, Motivo::Nenhum, tensao_v_);
                ir_para(Estado::AguardandoComando, Motivo::Nenhum);
            }
            break;
        case Estado::CargaSolicitada:
            // A rampa satura exatamente no setpoint.
            if (tensao_v_ >= setpoint_v_) {
                notificar(Notificacao::Tipo::TensaoAtingida, Motivo::Nenhum, tensao_v_);
                ir_para(Estado::AguardandoBola, Motivo::Nenhum);
            }
            break;
        case Estado::ChuteBloqueado:  // estado transitório: volta a esperar
            ir_para(tensao_v_ >= setpoint_v_ - config_.tolerancia_v ? Estado::AguardandoBola : Estado::CargaSolicitada,
                    Motivo::Nenhum);
            break;
        case Estado::ChuteAutorizado:
            // Simulação do disparo: o banco simulado descarrega e volta à pré-carga.
            tensao_v_ = 0.0f;
            carga_solicitada_ = false;
            ir_para(Estado::PreCarga, Motivo::Nenhum);
            break;
        default:
            break;
    }

    if (!chute_pendente_) return;
    const Motivo motivo = avaliar(agora_ms, tof);
    if (motivo == Motivo::Nenhum && estado_ == Estado::AguardandoBola) {
        chute_pendente_ = false;  // consumido: um comando gera no máximo uma autorização
        ultimo_bloqueio_ = Motivo::Nenhum;
        ++autorizacoes_;
        ir_para(Estado::ChuteAutorizado, Motivo::Nenhum);
        notificar(Notificacao::Tipo::Autorizado);
        return;
    }
    if (motivo == Motivo::ComandoExpirado) {
        // Informa o último bloqueio observado antes de expirar (ex.: sem bola).
        descartar_chute(ultimo_bloqueio_ == Motivo::Nenhum ? Motivo::ComandoExpirado : ultimo_bloqueio_, true);
        ultimo_bloqueio_ = Motivo::ComandoExpirado;
        ir_para(Estado::ChuteBloqueado, Motivo::ComandoExpirado);
        return;
    }
    if (motivo != ultimo_bloqueio_) {
        ultimo_bloqueio_ = motivo;
        notificar(Notificacao::Tipo::Bloqueio, motivo);
    }
}

bool Maquina::habilitar(uint64_t)
{
    if (estado_ != Estado::Desabilitado || emergencia_ativa_ || falha_ativa_) return false;
    carga_solicitada_ = false;
    ir_para(Estado::PreCarga, Motivo::Nenhum);
    return true;
}

void Maquina::desabilitar(uint64_t)
{
    if (estado_ == Estado::Emergencia) return;  // só sai da emergência com rearme
    descartar_chute(Motivo::Desabilitado, false);
    carga_solicitada_ = false;
    if (estado_ != Estado::Falha) ir_para(Estado::Desabilitado, Motivo::Desabilitado);
}

bool Maquina::solicitar_carga(uint64_t, float setpoint_v)
{
    const bool estado_permite = estado_ == Estado::PreCarga || estado_ == Estado::AguardandoComando ||
                                estado_ == Estado::CargaSolicitada || estado_ == Estado::AguardandoBola ||
                                estado_ == Estado::ChuteBloqueado;
    Motivo recusa = Motivo::Nenhum;
    if (!estado_permite) recusa = Motivo::EstadoNaoPermite;
    else if (!(setpoint_v >= config_.setpoint_min_v && setpoint_v <= config_.setpoint_max_v))
        recusa = Motivo::SetpointForaDosLimites;  // também rejeita NaN
    else if (setpoint_v < tensao_v_ - config_.tolerancia_v)
        recusa = Motivo::SetpointAbaixoDaTensao;
    if (recusa != Motivo::Nenhum) {
        notificar(Notificacao::Tipo::SetpointRejeitado, recusa, setpoint_v);
        return false;
    }

    setpoint_v_ = setpoint_v;
    carga_solicitada_ = true;
    notificar(Notificacao::Tipo::SetpointAceito, Motivo::Nenhum, setpoint_v);
    ir_para(tensao_v_ >= setpoint_v_ - config_.tolerancia_v ? Estado::AguardandoBola : Estado::CargaSolicitada,
            Motivo::Nenhum);
    return true;
}

void Maquina::parar(uint64_t)
{
    descartar_chute(Motivo::CancelamentoExplicito, false);
    carga_solicitada_ = false;
    if (estado_ == Estado::PreCarga) ir_para(Estado::Desabilitado, Motivo::CancelamentoExplicito);
    else if (estado_ == Estado::CargaSolicitada || estado_ == Estado::AguardandoBola ||
             estado_ == Estado::ChuteBloqueado)
        ir_para(Estado::AguardandoComando, Motivo::CancelamentoExplicito);
}

bool Maquina::solicitar_chute(uint64_t agora_ms)
{
    Motivo recusa = Motivo::Nenhum;
    switch (estado_) {
        case Estado::CargaSolicitada:
        case Estado::AguardandoBola:
        case Estado::ChuteBloqueado:
            break;
        case Estado::Emergencia: recusa = Motivo::Emergencia; break;
        case Estado::Falha: recusa = Motivo::Falha; break;
        case Estado::Desabilitado: recusa = Motivo::Desabilitado; break;
        case Estado::PreCarga:
        case Estado::AguardandoComando: recusa = Motivo::CargaNaoSolicitada; break;
        default: recusa = Motivo::EstadoNaoPermite; break;
    }
    if (recusa != Motivo::Nenhum) {
        notificar(Notificacao::Tipo::ChuteRejeitado, recusa);
        return false;
    }

    // Nunca há fila: um novo comando substitui o pendente (com novo prazo).
    if (chute_pendente_) notificar(Notificacao::Tipo::ChuteSubstituido, Motivo::SubstituidoPorNovoComando);
    chute_pendente_ = true;
    t_chute_ms_ = agora_ms;
    ultimo_bloqueio_ = Motivo::Nenhum;
    notificar(Notificacao::Tipo::ChuteRecebido);
    return true;
}

void Maquina::cancelar_chute(uint64_t)
{
    descartar_chute(Motivo::CancelamentoExplicito, false);
}

void Maquina::sinalizar_emergencia(uint64_t, bool ativa)
{
    emergencia_ativa_ = ativa;
    if (!ativa) return;  // liberar o botão não sai da emergência: exige rearme
    descartar_chute(Motivo::Emergencia, false);
    carga_solicitada_ = false;
    tensao_v_ = 0.0f;  // o banco simulado é considerado descarregado
    ir_para(Estado::Emergencia, Motivo::Emergencia);
}

void Maquina::sinalizar_falha(uint64_t)
{
    entrar_em_falha(Motivo::FalhaExterna);
}

bool Maquina::rearmar(uint64_t)
{
    Motivo recusa = Motivo::Nenhum;
    if (estado_ == Estado::Emergencia) {
        if (emergencia_ativa_) recusa = Motivo::EmergenciaAindaAtiva;
    } else if (estado_ == Estado::Falha) {
        if (falha_ativa_) recusa = Motivo::FalhaAindaAtiva;
    } else {
        recusa = Motivo::EstadoNaoPermite;
    }
    if (recusa != Motivo::Nenhum) {
        notificar(Notificacao::Tipo::RearmeNegado, recusa);
        return false;
    }
    // Rearme leva a DESABILITADO: voltar a operar exige "habilitar" explícito.
    ir_para(Estado::Desabilitado, Motivo::Nenhum);
    return true;
}

SaidasFisicas Maquina::saidas_fisicas() const
{
    // Estado físico separado do lógico: nesta etapa NENHUM estado lógico
    // (inclusive CHUTE_AUTORIZADO) liga carregamento ou chute.
    static_assert(!PERMITE_ACIONAMENTO_FISICO, "acionamento fisico nao implementado nesta etapa");
    return SaidasFisicas{false, false};
}

}  // namespace kicker_sim
