#include "autorizacao_chute_testes.h"

#include <cstdio>

#include "autorizacao_chute.h"

namespace autorizacao {
namespace {

using Tipo = Decisao::Tipo;

struct Cenario {
    Autorizador a;
    uint64_t t = 1000;
    Entrada e{true, true, true, true, 1000, false};
    bool tof_congelado = false;
    int chutes = 0;       // decisões Autorizado (= pulsos que o firmware daria)
    int expirados = 0;
    int descartados = 0;
    Motivo ultimo_motivo = Motivo::Nenhum;
    int falhas = 0;
    const char *nome = "";
    Escrita escrever = nullptr;

    void passo(uint32_t ms = 5)
    {
        t += ms;
        if (!tof_congelado) e.instante_ms = t;
        const Decisao d = a.atualizar(t, e);
        switch (d.tipo) {
            case Tipo::Autorizado: ++chutes; break;
            case Tipo::Expirado: ++expirados; ultimo_motivo = d.motivo; break;
            case Tipo::Descartado: ++descartados; ultimo_motivo = d.motivo; break;
            case Tipo::Bloqueio: ultimo_motivo = d.motivo; break;
            case Tipo::Nada: break;
        }
    }
    void avancar(uint32_t ms)
    {
        for (uint32_t i = 0; i < ms; i += 5) passo(5);
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

void caso_bola_presente(Cenario &c)
{
    c.a.solicitar(c.t, Origem::I2c);
    c.passo();
    VERIFICAR(c, c.chutes == 1);
    VERIFICAR(c, !c.a.pendente());
}

void caso_sem_bola(Cenario &c)
{
    c.e.bola_presente = false;
    c.a.solicitar(c.t, Origem::Serial);
    c.avancar(300);
    VERIFICAR(c, c.chutes == 0);
    VERIFICAR(c, c.expirados == 1);
    VERIFICAR(c, c.ultimo_motivo == Motivo::SemBola);
}

void caso_bola_chega_dentro_do_prazo(Cenario &c)
{
    c.e.bola_presente = false;
    c.a.solicitar(c.t, Origem::I2c);
    c.avancar(150);
    VERIFICAR(c, c.chutes == 0);
    c.e.bola_presente = true;
    c.passo();
    VERIFICAR(c, c.chutes == 1);
}

void caso_bola_chega_depois_do_prazo(Cenario &c)
{
    c.e.bola_presente = false;
    c.a.solicitar(c.t, Origem::I2c);
    c.avancar(250);
    c.e.bola_presente = true;
    c.avancar(500);
    VERIFICAR(c, c.chutes == 0);
    VERIFICAR(c, c.expirados == 1);
}

void caso_leitura_invalida_e_desatualizada(Cenario &c)
{
    c.e.leitura_valida = false;  // bola_presente true: isola a validade
    c.a.solicitar(c.t, Origem::Serial);
    c.passo();
    VERIFICAR(c, c.ultimo_motivo == Motivo::LeituraTofInvalida);
    c.avancar(300);
    VERIFICAR(c, c.chutes == 0);

    c.e.leitura_valida = true;
    c.tof_congelado = true;
    c.avancar(150);
    c.a.solicitar(c.t, Origem::Serial);
    c.passo();
    VERIFICAR(c, c.ultimo_motivo == Motivo::LeituraTofDesatualizada);
    c.avancar(300);
    VERIFICAR(c, c.chutes == 0);
}

void caso_tof_indisponivel(Cenario &c)
{
    c.e.sensor_ok = false;
    c.a.solicitar(c.t, Origem::I2c);
    c.passo();
    VERIFICAR(c, c.ultimo_motivo == Motivo::TofIndisponivel);
    c.avancar(300);
    VERIFICAR(c, c.chutes == 0);
    VERIFICAR(c, c.expirados == 1);
}

void caso_fault_descarta(Cenario &c)
{
    c.e.bola_presente = false;
    c.a.solicitar(c.t, Origem::I2c);
    c.passo();
    c.e.fault = true;
    c.passo();
    VERIFICAR(c, c.descartados == 1);
    VERIFICAR(c, c.ultimo_motivo == Motivo::FaultLt3751);
    VERIFICAR(c, !c.a.pendente());
    // FAULT some e a bola chega ainda dentro dos 200 ms originais: não chuta.
    c.e.fault = false;
    c.e.bola_presente = true;
    c.avancar(100);
    VERIFICAR(c, c.chutes == 0);
}

void caso_pedido_processado_atrasado(Cenario &c)
{
    c.a.solicitar(c.t, Origem::Serial);
    c.passo(300);  // tudo OK, mas o primeiro processamento chega tarde
    VERIFICAR(c, c.chutes == 0);
    VERIFICAR(c, c.expirados == 1);
}

void caso_multiplos_pedidos(Cenario &c)
{
    c.e.bola_presente = false;
    VERIFICAR(c, !c.a.solicitar(c.t, Origem::I2c));
    c.avancar(50);
    VERIFICAR(c, c.a.solicitar(c.t, Origem::I2c));  // substituiu
    c.avancar(50);
    VERIFICAR(c, c.a.solicitar(c.t, Origem::Serial));
    c.e.bola_presente = true;
    c.avancar(500);
    VERIFICAR(c, c.chutes == 1);  // exatamente um chute
}

void caso_um_chute_por_pedido(Cenario &c)
{
    c.a.solicitar(c.t, Origem::AutoKick);
    c.avancar(500);  // bola continua presente o tempo todo
    VERIFICAR(c, c.chutes == 1);
}

void caso_cancelamento(Cenario &c)
{
    c.e.bola_presente = false;
    c.a.solicitar(c.t, Origem::Serial);
    c.passo();
    VERIFICAR(c, c.a.cancelar());
    c.e.bola_presente = true;
    c.avancar(100);
    VERIFICAR(c, c.chutes == 0);
    VERIFICAR(c, !c.a.cancelar());  // nada pendente
}

void caso_sem_pedido_nunca_chuta(Cenario &c)
{
    c.avancar(1000);  // bola presente o tempo todo, nenhum pedido
    VERIFICAR(c, c.chutes == 0);
}

void caso_chegada_prevista(Cenario &c)
{
    // Bola em movimento: o detector ainda não confirmou presença, mas o preditor
    // indica que ela chega dentro da latência do êmbolo => autoriza.
    c.e.bola_presente = false;
    c.a.solicitar(c.t, Origem::I2c);
    c.passo();
    VERIFICAR(c, c.chutes == 0);
    c.e.chegada_prevista = true;
    c.passo();
    VERIFICAR(c, c.chutes == 1);
}

void caso_chegada_prevista_sem_pedido(Cenario &c)
{
    c.e.bola_presente = false;
    c.e.chegada_prevista = true;
    c.avancar(500);
    VERIFICAR(c, c.chutes == 0);
}

void caso_chegada_prevista_com_fault(Cenario &c)
{
    c.e.bola_presente = false;
    c.e.chegada_prevista = true;
    c.e.fault = true;
    c.a.solicitar(c.t, Origem::I2c);
    c.passo();
    VERIFICAR(c, c.chutes == 0);
}

void caso_chegada_prevista_leitura_velha(Cenario &c)
{
    c.e.bola_presente = false;
    c.e.chegada_prevista = true;
    c.tof_congelado = true;  // leitura parou de chegar: mesmo com previsão, não chuta
    c.avancar(150);
    c.a.solicitar(c.t, Origem::I2c);
    c.passo();
    VERIFICAR(c, c.chutes == 0);
}

struct Caso {
    const char *nome;
    void (*funcao)(Cenario &);
};

const Caso CASOS[] = {
    {"A1. Bola presente: autoriza", caso_bola_presente},
    {"A2. Sem bola: expira em 200 ms", caso_sem_bola},
    {"A3. Bola chega dentro do prazo", caso_bola_chega_dentro_do_prazo},
    {"A4. Bola chega depois do prazo", caso_bola_chega_depois_do_prazo},
    {"A5. Leitura invalida/desatualizada", caso_leitura_invalida_e_desatualizada},
    {"A6. TOF indisponivel bloqueia", caso_tof_indisponivel},
    {"A7. FAULT# descarta o pedido", caso_fault_descarta},
    {"A8. Pedido processado atrasado", caso_pedido_processado_atrasado},
    {"A9. Multiplos pedidos: um chute", caso_multiplos_pedidos},
    {"A10. Um chute por pedido", caso_um_chute_por_pedido},
    {"A11. Cancelamento explicito", caso_cancelamento},
    {"A12. Sem pedido nunca chuta", caso_sem_pedido_nunca_chuta},
    {"A13. Chegada prevista autoriza", caso_chegada_prevista},
    {"A14. Chegada prevista sem pedido: nada", caso_chegada_prevista_sem_pedido},
    {"A15. Chegada prevista com FAULT#: nada", caso_chegada_prevista_com_fault},
    {"A16. Chegada prevista com leitura velha: nada", caso_chegada_prevista_leitura_velha},
};

}  // namespace

int executar_testes(Escrita escrever)
{
    int reprovados = 0;
    char linha[160];
    escrever("======= TESTES autorizacao_chute =======");
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
    escrever("========================================");
    return reprovados;
}

}  // namespace autorizacao
