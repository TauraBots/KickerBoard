#include "autorizacao_chute.h"

namespace autorizacao {

const char *nome_origem(Origem origem)
{
    switch (origem) {
        case Origem::Serial: return "Serial";
        case Origem::I2c: return "I2C";
        case Origem::AutoKick: return "auto-kick";
    }
    return "?";
}

const char *descrever_motivo(Motivo motivo)
{
    switch (motivo) {
        case Motivo::Nenhum: return "-";
        case Motivo::SemPedido: return "sem pedido de chute";
        case Motivo::PedidoExpirado: return "pedido expirado";
        case Motivo::TofIndisponivel: return "TOF indisponivel";
        case Motivo::LeituraTofInvalida: return "leitura do TOF invalida";
        case Motivo::LeituraTofDesatualizada: return "leitura do TOF desatualizada";
        case Motivo::SemBola: return "sem bola";
        case Motivo::FaultLt3751: return "FAULT# do LT3751 ativo";
        case Motivo::Cancelado: return "cancelado";
    }
    return "?";
}

bool Autorizador::solicitar(uint64_t agora_ms, Origem origem)
{
    const bool substituiu = pendente_;
    pendente_ = true;
    t_pedido_ms_ = agora_ms;
    origem_ = origem;
    ultimo_motivo_ = Motivo::Nenhum;
    return substituiu;
}

bool Autorizador::cancelar()
{
    if (!pendente_) return false;
    pendente_ = false;
    ++bloqueios_;
    return true;
}

Motivo Autorizador::avaliar(uint64_t agora_ms, const Entrada &e) const
{
    if (!pendente_) return Motivo::SemPedido;
    // Timestamp do pedido no futuro também conta como expirado (conservador).
    if (t_pedido_ms_ > agora_ms || agora_ms - t_pedido_ms_ > config_.validade_pedido_ms)
        return Motivo::PedidoExpirado;
    if (e.fault) return Motivo::FaultLt3751;
    if (!e.sensor_ok) return Motivo::TofIndisponivel;
    if (!e.tem_amostra || !e.leitura_valida || e.instante_ms > agora_ms) return Motivo::LeituraTofInvalida;
    if (agora_ms - e.instante_ms > config_.max_idade_tof_ms) return Motivo::LeituraTofDesatualizada;
    if (!e.bola_presente) return Motivo::SemBola;
    return Motivo::Nenhum;
}

Decisao Autorizador::atualizar(uint64_t agora_ms, const Entrada &e)
{
    if (!pendente_) return Decisao{Decisao::Tipo::Nada, Motivo::SemPedido, origem_, 0};
    const uint32_t idade = t_pedido_ms_ <= agora_ms ? static_cast<uint32_t>(agora_ms - t_pedido_ms_) : 0;
    const Motivo motivo = avaliar(agora_ms, e);

    if (motivo == Motivo::Nenhum) {
        pendente_ = false;  // consumido: um pedido gera no máximo uma autorização
        ultimo_motivo_ = Motivo::Nenhum;
        ++autorizacoes_;
        return Decisao{Decisao::Tipo::Autorizado, Motivo::Nenhum, origem_, idade};
    }
    if (motivo == Motivo::PedidoExpirado) {
        pendente_ = false;
        ++bloqueios_;
        const Motivo anterior = ultimo_motivo_ == Motivo::Nenhum ? Motivo::PedidoExpirado : ultimo_motivo_;
        ultimo_motivo_ = Motivo::Nenhum;
        return Decisao{Decisao::Tipo::Expirado, anterior, origem_, idade};
    }
    if (motivo == Motivo::FaultLt3751) {
        // Falha do carregador: o pedido é descartado na hora (não espera a validade).
        pendente_ = false;
        ++bloqueios_;
        ultimo_motivo_ = Motivo::Nenhum;
        return Decisao{Decisao::Tipo::Descartado, motivo, origem_, idade};
    }
    if (motivo != ultimo_motivo_) {
        ultimo_motivo_ = motivo;
        return Decisao{Decisao::Tipo::Bloqueio, motivo, origem_, idade};
    }
    return Decisao{Decisao::Tipo::Nada, motivo, origem_, idade};
}

}  // namespace autorizacao
