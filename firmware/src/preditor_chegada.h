#pragma once

// Previsão da chegada da bola ao kicker (lógica pura, sem ESP-IDF).
//
// O detector de presença (bola_detector) exige 3 leituras <= 49 mm. Com o
// sensor a ~50 Hz e a bola a ~1 m/s, a faixa 49 mm -> contato (~33 mm) é
// atravessada em menos de uma leitura: a presença só é confirmada DEPOIS do
// contato, com a bola já quicando para trás (medido: chute ~24 ms após o contato).
//
// Este preditor usa as leituras da aproximação, ainda longe, para estimar a
// velocidade (regressão linear das últimas N leituras) e o instante do contato:
//     t_contato = t_ultima + (d_ultima - d_contato) / v
//     t_disparo = t_contato - latencia_kicker
// O firmware autoriza o chute quando agora >= t_disparo (com um pedido ativo).
//
// Regras:
//  - Só leituras válidas, com d > d_contato + margem, entram na regressão.
//  - Leitura inválida, intervalo longo ou bola se afastando zeram o histórico
//    (e a previsão).
//  - Velocidade fora de [v_min, v_max] => sem previsão.
//  - A previsão vale até t_contato + janela_pos_contato_us; depois expira.

#include <cstdint>

namespace chegada {

struct Config {
    uint8_t d_contato_mm = 33;          // leitura com a bola encostada (medido: 31-35 mm)
    uint8_t margem_contato_mm = 5;      // leituras <= d_contato + margem não entram na regressão
    uint8_t d_max_mm = 200;             // acima disso a leitura é ignorada
    uint8_t amostras_regressao = 3;     // N leituras para estimar a velocidade (2..MAX_AMOSTRAS)
    // Antecipação do disparo em relação ao contato previsto ("pred lat"). Não é a
    // latência da bola parada (~9 ms a 150 V): com a bola chegando, disparar 9-15 ms
    // antes faz o êmbolo terminar o curso e "matar" a bola; 0 ms chuta fraco.
    // Varredura a ~140-150 V (jogadas): 6 ms => 5/5 chutes fortes.
    uint32_t latencia_kicker_us = 6000;
    uint32_t max_intervalo_us = 45000;  // intervalo maior entre leituras quebra a sequência
    uint8_t tolerancia_recuo_mm = 3;    // leitura que AUMENTA mais que isso = bola se afastando
    float v_min_mm_ms = 0.15f;          // 0,15 m/s: abaixo disso o detector de presença resolve
    float v_max_mm_ms = 6.0f;           // 6 m/s: acima disso é ruído
    uint32_t janela_pos_contato_us = 20000;
    // Só dispara pela previsão se a leitura mais recente da sequência tiver no máximo
    // isto. Leituras perdidas na aproximação deixavam a previsão extrapolar ~60 ms a
    // partir de 2 leituras distantes e o chute saía cedo (bola "morta").
    uint32_t max_idade_leitura_us = 30000;
};

constexpr uint8_t MAX_AMOSTRAS = 6;
constexpr uint8_t AMOSTRAS_MINIMAS = 2;  // primeira previsão já com 2 leituras

struct Previsao {
    bool valida;
    int64_t t_contato_us;
    int64_t t_disparo_us;
    float v_mm_ms;          // velocidade de aproximação (mm/ms = m/s)
    uint8_t amostras;       // leituras usadas
    int64_t t_ultima_leitura_us;  // leitura mais recente da sequência (inclusive perto do contato)
};

// true se a previsão é válida, agora está em [t_disparo, t_contato + janela] e a
// leitura mais recente tem no máximo max_idade_leitura_us.
bool hora_de_disparar(const Previsao &previsao, int64_t agora_us, const Config &config);

class Preditor {
public:
    explicit Preditor(const Config &config = Config()) : config_(config) {}

    void configurar(const Config &config);
    const Config &config() const { return config_; }

    // Processa uma amostra do sensor (t_us: instante real da leitura).
    // Retorna a previsão atualizada (valida = false se não houver).
    Previsao processar_amostra(bool valida, uint8_t distancia_mm, int64_t t_us);

    // Falha do sensor (timeout, erro I2C): descarta tudo.
    void resetar();

    // Previsão atual (pode estar expirada; use disparar_agora()).
    const Previsao &previsao() const { return previsao_; }

    // hora_de_disparar() com a configuração deste preditor.
    bool disparar_agora(int64_t agora_us) const;

private:
    void recalcular();

    Config config_;
    int64_t t_[MAX_AMOSTRAS] = {};
    uint8_t d_[MAX_AMOSTRAS] = {};
    uint8_t n_ = 0;
    bool tem_ultima_ = false;
    int64_t t_ultima_us_ = 0;    // última amostra recebida (qualquer distância)
    bool tem_d_ultima_ = false;
    uint8_t d_ultima_ = 0;       // última distância válida da sequência atual
    Previsao previsao_ = {};
};

}  // namespace chegada
