#include "preditor_chegada.h"

namespace chegada {

void Preditor::configurar(const Config &config)
{
    config_ = config;
    if (config_.amostras_regressao < 2) config_.amostras_regressao = 2;
    if (config_.amostras_regressao > MAX_AMOSTRAS) config_.amostras_regressao = MAX_AMOSTRAS;
    resetar();
}

void Preditor::resetar()
{
    n_ = 0;
    tem_d_ultima_ = false;
    previsao_ = Previsao{};
}

Previsao Preditor::processar_amostra(bool valida, uint8_t distancia_mm, int64_t t_us)
{
    // Intervalo longo (medição perdida, sensor reiniciado): a sequência não é confiável.
    if (tem_ultima_ && t_us - t_ultima_us_ > static_cast<int64_t>(config_.max_intervalo_us)) resetar();
    tem_ultima_ = true;
    t_ultima_us_ = t_us;

    if (!valida || distancia_mm > config_.d_max_mm) {
        resetar();
        return previsao_;
    }

    // Bola se afastando (quique, desvio): descarta a aproximação; esta leitura recomeça.
    if (tem_d_ultima_ && distancia_mm > d_ultima_ + config_.tolerancia_recuo_mm) resetar();
    tem_d_ultima_ = true;
    d_ultima_ = distancia_mm;

    // Perto do contato a leitura não acrescenta velocidade confiável: mantém a previsão,
    // mas conta como leitura recente (a bola segue chegando).
    if (distancia_mm <= config_.d_contato_mm + config_.margem_contato_mm) {
        if (previsao_.valida) previsao_.t_ultima_leitura_us = t_us;
        return previsao_;
    }

    if (n_ == MAX_AMOSTRAS) {
        for (uint8_t i = 1; i < MAX_AMOSTRAS; ++i) {
            t_[i - 1] = t_[i];
            d_[i - 1] = d_[i];
        }
        --n_;
    }
    t_[n_] = t_us;
    d_[n_] = distancia_mm;
    ++n_;
    recalcular();
    return previsao_;
}

void Preditor::recalcular()
{
    previsao_ = Previsao{};
    // Bola rápida (> ~2,3 m/s) dá só 3 leituras válidas antes do contato: exigir as
    // N=3 atrasava a primeira previsão para depois do instante de disparo. Começa com
    // AMOSTRAS_MINIMAS e passa a usar N assim que houver leituras suficientes.
    if (n_ < AMOSTRAS_MINIMAS) return;
    const uint8_t n = n_ < config_.amostras_regressao ? n_ : config_.amostras_regressao;

    // Regressão linear d(t) nas últimas n leituras (t em ms, relativo à última).
    const int64_t t_ref = t_[n_ - 1];
    float soma_t = 0.0f, soma_d = 0.0f;
    for (uint8_t i = n_ - n; i < n_; ++i) {
        soma_t += static_cast<float>(t_[i] - t_ref) / 1000.0f;
        soma_d += d_[i];
    }
    const float media_t = soma_t / n, media_d = soma_d / n;
    float stt = 0.0f, std_ = 0.0f;
    for (uint8_t i = n_ - n; i < n_; ++i) {
        const float dt = static_cast<float>(t_[i] - t_ref) / 1000.0f - media_t;
        stt += dt * dt;
        std_ += dt * (d_[i] - media_d);
    }
    if (stt <= 0.0f) return;
    const float v = -std_ / stt;  // mm/ms, positivo = aproximando
    if (v < config_.v_min_mm_ms || v > config_.v_max_mm_ms) return;

    // Distância ajustada pela reta no instante da última leitura (menos ruído que a crua).
    const float d_agora = media_d + (std_ / stt) * (0.0f - media_t);
    const float falta_mm = d_agora - config_.d_contato_mm;
    const float falta_ms = falta_mm > 0.0f ? falta_mm / v : 0.0f;

    previsao_.valida = true;
    previsao_.t_ultima_leitura_us = t_ref;
    previsao_.v_mm_ms = v;
    previsao_.amostras = n;
    previsao_.t_contato_us = t_ref + static_cast<int64_t>(falta_ms * 1000.0f);
    previsao_.t_disparo_us = previsao_.t_contato_us - static_cast<int64_t>(config_.latencia_kicker_us);
}

bool hora_de_disparar(const Previsao &previsao, int64_t agora_us, const Config &config)
{
    if (!previsao.valida) return false;
    if (agora_us - previsao.t_ultima_leitura_us > static_cast<int64_t>(config.max_idade_leitura_us)) return false;
    return agora_us >= previsao.t_disparo_us &&
           agora_us <= previsao.t_contato_us + static_cast<int64_t>(config.janela_pos_contato_us);
}

bool Preditor::disparar_agora(int64_t agora_us) const
{
    return hora_de_disparar(previsao_, agora_us, config_);
}

}  // namespace chegada
