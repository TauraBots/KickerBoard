#include "bola_detector.h"

namespace bola {

const char *nome_estado(Estado estado)
{
    return estado == Estado::Presente ? "BOLA_PRESENTE" : "BOLA_AUSENTE";
}

const char *descrever_causa(Causa causa)
{
    switch (causa) {
        case Causa::Nenhuma: return "-";
        case Causa::Confirmada: return "leituras validas consecutivas dentro do limite";
        case Causa::AcimaDoLimiteSaida: return "distancia acima do limite de saida";
        case Causa::LeituraInvalida: return "leitura invalida do sensor";
        case Causa::LeituraDesatualizada: return "leitura desatualizada";
        case Causa::Timeout: return "timeout de medicao";
        case Causa::ErroSensor: return "erro de comunicacao/sensor";
    }
    return "?";
}

Evento Detector::sem_transicao() const
{
    return Evento{false, estado_, Causa::Nenhuma, confirmacoes_, 0};
}

Evento Detector::ir_para(Estado novo, Causa causa, uint32_t idade_ms)
{
    confirmacoes_ = 0;
    if (novo == estado_) return sem_transicao();
    estado_ = novo;
    if (novo == Estado::Presente) ++transicoes_presente_;
    return Evento{true, estado_, causa, confirmacoes_, idade_ms};
}

Evento Detector::processar_amostra(bool valida, uint8_t distancia_mm, uint64_t t_ms)
{
    // Intervalo grande desde a amostra anterior quebra a sequência "consecutiva".
    const bool intervalo_longo = tem_leitura_ && t_ms - t_ultima_leitura_ms_ > MAX_IDADE_LEITURA_MS;
    tem_leitura_ = true;
    t_ultima_leitura_ms_ = t_ms;

    if (!valida) return ir_para(Estado::Ausente, Causa::LeituraInvalida);

    if (estado_ == Estado::Presente) {
        if (distancia_mm > LIMITE_SAIDA_MM) return ir_para(Estado::Ausente, Causa::AcimaDoLimiteSaida);
        return sem_transicao();  // dentro da histerese: continua presente
    }

    // Estado AUSENTE: conta confirmações.
    if (intervalo_longo) confirmacoes_ = 0;
    if (distancia_mm > LIMITE_ENTRADA_MM) {
        confirmacoes_ = 0;
        return sem_transicao();
    }
    if (++confirmacoes_ >= CONFIRMACOES_NECESSARIAS) {
        const Evento evento = ir_para(Estado::Presente, Causa::Confirmada);
        // Mantém a contagem visível no evento de confirmação (ex.: "3/3").
        return Evento{evento.transicao, evento.estado, evento.causa, CONFIRMACOES_NECESSARIAS, 0};
    }
    return sem_transicao();
}

Evento Detector::verificar_idade(uint64_t agora_ms)
{
    if (!tem_leitura_ || agora_ms <= t_ultima_leitura_ms_) return sem_transicao();
    const uint64_t idade = agora_ms - t_ultima_leitura_ms_;
    if (idade <= MAX_IDADE_LEITURA_MS) return sem_transicao();
    if (estado_ == Estado::Ausente) {
        confirmacoes_ = 0;  // a sequência de confirmação também perde validade
        return sem_transicao();
    }
    return ir_para(Estado::Ausente, Causa::LeituraDesatualizada, static_cast<uint32_t>(idade));
}

Evento Detector::invalidar(Causa causa)
{
    return ir_para(Estado::Ausente, causa);
}

bool Detector::presenca_autorizada(uint64_t agora_ms) const
{
    if (estado_ != Estado::Presente || !tem_leitura_) return false;
    if (agora_ms < t_ultima_leitura_ms_) return true;  // leitura mais nova que "agora"
    return agora_ms - t_ultima_leitura_ms_ <= MAX_IDADE_LEITURA_MS;
}

}  // namespace bola
