#include "preditor_chegada_testes.h"

#include <cstdio>

#include "preditor_chegada.h"

namespace chegada {
namespace {

struct Cenario {
    Preditor p;
    int64_t t_us = 1000000;
    int falhas = 0;
    const char *nome = "";
    autorizacao::Escrita escrever = nullptr;

    // Amostra válida após dt_ms (instante da leitura = t_us).
    Previsao amostra(uint8_t mm, uint32_t dt_ms = 20, bool valida = true)
    {
        t_us += static_cast<int64_t>(dt_ms) * 1000;
        return p.processar_amostra(valida, mm, t_us);
    }
    void verificar(bool c, const char *expr, int linha)
    {
        if (c) return;
        ++falhas;
        char msg[200];
        snprintf(msg, sizeof(msg), "    FALHOU [%s] linha %d: %s", nome, linha, expr);
        escrever(msg);
    }
};

#define VERIFICAR(c, cond) (c).verificar((cond), #cond, __LINE__)

int64_t abs64(int64_t v) { return v < 0 ? -v : v; }

// Bola a 1 m/s (1 mm/ms), leituras a cada 20 ms: 113 -> 93 -> 73 -> 53 mm. Contato
// (33 mm) 20 ms após a última leitura; disparo = contato - 6 ms (antecipação padrão).
void caso_aproximacao_constante(Cenario &c)
{
    VERIFICAR(c, !c.amostra(113).valida);
    VERIFICAR(c, c.amostra(93).valida);  // 2 leituras já bastam
    c.amostra(73);
    const Previsao p = c.amostra(53);
    VERIFICAR(c, p.valida);
    VERIFICAR(c, p.amostras == 3);
    VERIFICAR(c, p.v_mm_ms > 0.99f && p.v_mm_ms < 1.01f);
    VERIFICAR(c, abs64(p.t_contato_us - (c.t_us + 20000)) <= 500);
    VERIFICAR(c, abs64(p.t_disparo_us - (c.t_us + 14000)) <= 500);
    VERIFICAR(c, !c.p.disparar_agora(c.t_us + 13000));
    VERIFICAR(c, c.p.disparar_agora(c.t_us + 14500));
}

// Bola rápida (2,5 m/s) com só 2 leituras antes do contato: precisa prever na 2ª.
void caso_bola_rapida_duas_leituras(Cenario &c)
{
    VERIFICAR(c, !c.amostra(148, 20).valida);
    const Previsao p = c.amostra(98, 20);
    VERIFICAR(c, p.valida);
    VERIFICAR(c, p.v_mm_ms > 2.4f && p.v_mm_ms < 2.6f);
    VERIFICAR(c, p.t_disparo_us < p.t_contato_us);
}

void caso_bola_se_afastando(Cenario &c)
{
    c.amostra(100);
    c.amostra(80);
    VERIFICAR(c, c.amostra(60).valida);
    // Quique: a leitura aumenta além da tolerância => descarta a previsão.
    VERIFICAR(c, !c.amostra(70).valida);
    VERIFICAR(c, !c.p.disparar_agora(c.t_us));
}

void caso_leitura_invalida_descarta(Cenario &c)
{
    c.amostra(120);
    VERIFICAR(c, c.amostra(100).valida);
    VERIFICAR(c, !c.amostra(0, 20, false).valida);
    // Depois da falha, precisa de 2 leituras novas.
    VERIFICAR(c, !c.amostra(80).valida);
    VERIFICAR(c, c.amostra(60).valida);
}

void caso_intervalo_longo_descarta(Cenario &c)
{
    c.amostra(120);
    VERIFICAR(c, c.amostra(100).valida);
    VERIFICAR(c, !c.amostra(80, 60).valida);  // 60 ms > max_intervalo (45 ms)
}

void caso_bola_parada_longe(Cenario &c)
{
    // Bola parada a 60 mm (ou robô parado diante de um obstáculo): v ~ 0 => sem previsão.
    for (int i = 0; i < 6; ++i) VERIFICAR(c, !c.amostra(60).valida);
}

void caso_bola_para_antes(Cenario &c)
{
    c.amostra(100);
    VERIFICAR(c, c.amostra(80).valida);
    // Desacelera e para a 60 mm: a regressão das últimas 3 leituras perde a velocidade.
    c.amostra(70);
    c.amostra(66);
    c.amostra(65);
    VERIFICAR(c, !c.amostra(65).valida);
}

void caso_perto_do_contato_mantem(Cenario &c)
{
    c.amostra(113);
    const Previsao p = c.amostra(93);
    VERIFICAR(c, p.valida);
    // Leitura dentro de d_contato + margem: não entra na regressão, previsão mantida.
    const Previsao q = c.amostra(36);
    VERIFICAR(c, q.valida);
    VERIFICAR(c, q.t_contato_us == p.t_contato_us);
}

void caso_janela_expira(Cenario &c)
{
    c.amostra(113);
    c.amostra(93);
    c.amostra(73);
    const Previsao p = c.amostra(53);
    c.amostra(36, 15);  // perto do contato: mantém a previsão e renova a leitura
    VERIFICAR(c, c.p.disparar_agora(p.t_contato_us));
    VERIFICAR(c, !c.p.disparar_agora(p.t_contato_us + 20001));
}

// Leituras perdidas: previsão feita com a bola longe não dispara sem leitura recente.
void caso_previsao_velha(Cenario &c)
{
    c.amostra(189, 20);
    const Previsao p = c.amostra(144, 20);  // 2,25 m/s: contato em ~49 ms
    VERIFICAR(c, p.valida);
    VERIFICAR(c, p.t_disparo_us - c.t_us > 30000);
    VERIFICAR(c, !c.p.disparar_agora(p.t_disparo_us));  // 43 ms sem leitura: não dispara
    // Chega leitura nova: volta a valer.
    const Previsao q = c.amostra(99, 20);
    VERIFICAR(c, q.valida);
    VERIFICAR(c, c.p.disparar_agora(q.t_disparo_us));
}

void caso_velocidade_absurda(Cenario &c)
{
    c.amostra(190, 20);
    VERIFICAR(c, !c.amostra(40, 20).valida);  // 7,5 m/s > v_max: ruído
}

void caso_resetar(Cenario &c)
{
    c.amostra(113);
    VERIFICAR(c, c.amostra(93).valida);
    c.p.resetar();
    VERIFICAR(c, !c.p.previsao().valida);
    VERIFICAR(c, !c.p.disparar_agora(c.t_us + 50000));
}

struct Caso {
    const char *nome;
    void (*funcao)(Cenario &);
};

const Caso CASOS[] = {
    {"P1. Aproximacao a 1 m/s: contato e disparo", caso_aproximacao_constante},
    {"P2. Bola rapida: preve com 2 leituras", caso_bola_rapida_duas_leituras},
    {"P3. Bola se afastando: descarta", caso_bola_se_afastando},
    {"P4. Leitura invalida: descarta", caso_leitura_invalida_descarta},
    {"P5. Intervalo longo: descarta", caso_intervalo_longo_descarta},
    {"P6. Bola parada longe: sem previsao", caso_bola_parada_longe},
    {"P7. Bola para antes do kicker: sem previsao", caso_bola_para_antes},
    {"P8. Perto do contato: mantem previsao", caso_perto_do_contato_mantem},
    {"P9. Janela pos-contato expira", caso_janela_expira},
    {"P10. Velocidade absurda: ignora", caso_velocidade_absurda},
    {"P11. Reset (falha do sensor)", caso_resetar},
    {"P12. Previsao sem leitura recente nao dispara", caso_previsao_velha},
};

}  // namespace

int executar_testes(autorizacao::Escrita escrever)
{
    int reprovados = 0;
    char linha[160];
    escrever("======= TESTES preditor_chegada =======");
    for (const Caso &caso : CASOS) {
        Cenario c;
        c.nome = caso.nome;
        c.escrever = escrever;
        caso.funcao(c);
        if (c.falhas) ++reprovados;
        snprintf(linha, sizeof(linha), "[%s] %s", c.falhas ? "REPROVADO" : "APROVADO ", caso.nome);
        escrever(linha);
    }
    const int total = static_cast<int>(sizeof(CASOS) / sizeof(CASOS[0]));
    snprintf(linha, sizeof(linha), "Resultado: %d/%d aprovados", total - reprovados, total);
    escrever(linha);
    escrever("=======================================");
    return reprovados;
}

}  // namespace chegada
