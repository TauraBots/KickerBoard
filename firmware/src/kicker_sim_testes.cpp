#include "kicker_sim_testes.h"

#include <cstdio>

#include "kicker_sim.h"

namespace kicker_sim {
namespace {

using Tipo = Notificacao::Tipo;

// Registra notificações para conferência nos testes.
class Registro : public Ouvinte {
public:
    void notificar(const Notificacao &n) override
    {
        switch (n.tipo) {
            case Tipo::Autorizado: ++autorizados; break;
            case Tipo::ChuteExpirado: ++expirados; ultimo_motivo_expiracao = n.motivo; break;
            case Tipo::ChuteCancelado: ++cancelados; ultimo_motivo_cancelamento = n.motivo; break;
            case Tipo::ChuteRejeitado: ++rejeitados; break;
            case Tipo::ChuteSubstituido: ++substituidos; break;
            case Tipo::Bloqueio: ++bloqueios; ultimo_bloqueio = n.motivo; break;
            default: break;
        }
    }
    int autorizados = 0, expirados = 0, cancelados = 0, rejeitados = 0, substituidos = 0, bloqueios = 0;
    Motivo ultimo_motivo_expiracao = Motivo::Nenhum;
    Motivo ultimo_motivo_cancelamento = Motivo::Nenhum;
    Motivo ultimo_bloqueio = Motivo::Nenhum;
};

// Ambiente simulado: relógio, TOF e falha externa controlados pelo teste.
struct Cenario {
    Registro reg;
    Maquina m{Config(), &reg};
    uint64_t t = 1000;
    EntradaTof tof{true, true, true, true, 1000, 25};
    bool tof_congelado = false;  // true: timestamp do TOF não avança (leitura envelhece)
    bool falha_externa = false;
    bool saida_fisica_ligada = false;  // vira true se QUALQUER passo ligar uma saída
    int falhas = 0;
    const char *nome_caso = "";
    Escrita escrever = nullptr;

    void passo(uint32_t ms = 10)
    {
        t += ms;
        if (!tof_congelado) tof.instante_ms = t;
        m.atualizar(t, tof, falha_externa);
        const SaidasFisicas s = m.saidas_fisicas();
        if (s.carregamento || s.chute) saida_fisica_ligada = true;
    }
    void avancar(uint32_t ms)
    {
        for (uint32_t i = 0; i < ms; i += 10) passo(10);
    }
    void bola(bool presente) { tof.bola_presente = presente; }

    // Habilita, completa a pré-carga e carrega até o setpoint (AGUARDANDO_BOLA).
    void preparar(float setpoint = 80.0f)
    {
        m.habilitar(t);
        avancar(1000);
        m.solicitar_carga(t, setpoint);
        avancar(2000);
    }

    void verificar(bool condicao, const char *expressao, int linha)
    {
        if (condicao) return;
        ++falhas;
        char msg[200];
        snprintf(msg, sizeof(msg), "    FALHOU [%s] linha %d: %s", nome_caso, linha, expressao);
        escrever(msg);
    }
};

#define VERIFICAR(c, cond) (c).verificar((cond), #cond, __LINE__)

// Toda execução termina conferindo a separação lógico/físico.
#define VERIFICAR_SAIDAS_DESLIGADAS(c) VERIFICAR(c, !(c).saida_fisica_ligada)

void caso_bola_presente_tensao_correta(Cenario &c)
{
    c.preparar(80.0f);
    VERIFICAR(c, c.m.estado() == Estado::AguardandoBola);
    VERIFICAR(c, c.m.tensao_simulada() >= 78.0f);
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.passo();
    VERIFICAR(c, c.m.estado() == Estado::ChuteAutorizado);
    VERIFICAR(c, c.reg.autorizados == 1);
    VERIFICAR(c, !c.m.chute_pendente());
    c.passo();  // simulação do disparo: descarrega e volta à pré-carga
    VERIFICAR(c, c.m.estado() == Estado::PreCarga);
    VERIFICAR(c, c.m.tensao_simulada() < 1.0f);
    VERIFICAR_SAIDAS_DESLIGADAS(c);
}

void caso_bola_ausente(Cenario &c)
{
    c.preparar(80.0f);
    c.bola(false);
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.avancar(300);
    VERIFICAR(c, c.reg.autorizados == 0);
    VERIFICAR(c, c.reg.expirados == 1);
    VERIFICAR(c, c.reg.ultimo_motivo_expiracao == Motivo::SemBola);
    VERIFICAR(c, !c.m.chute_pendente());
    VERIFICAR_SAIDAS_DESLIGADAS(c);
}

void caso_tensao_abaixo_do_setpoint(Cenario &c)
{
    c.m.habilitar(c.t);
    c.avancar(1000);  // pré-carga: 50 V
    VERIFICAR(c, c.m.estado() == Estado::AguardandoComando);
    // 50 -> 120 V a 100 V/s leva 700 ms: o comando (200 ms) expira antes.
    VERIFICAR(c, c.m.solicitar_carga(c.t, 120.0f));
    VERIFICAR(c, c.m.estado() == Estado::CargaSolicitada);
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.avancar(300);
    VERIFICAR(c, c.reg.autorizados == 0);
    VERIFICAR(c, c.reg.expirados == 1);
    VERIFICAR(c, c.reg.ultimo_motivo_expiracao == Motivo::TensaoInsuficiente);
    // Mesmo depois de atingir a tensão, o comando antigo não autoriza.
    c.avancar(1000);
    VERIFICAR(c, c.m.estado() == Estado::AguardandoBola);
    VERIFICAR(c, c.reg.autorizados == 0);
    VERIFICAR_SAIDAS_DESLIGADAS(c);
}

void caso_leitura_tof_invalida(Cenario &c)
{
    c.preparar(80.0f);
    c.tof.leitura_valida = false;  // bola_presente ainda true: isola a validade
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.passo();
    VERIFICAR(c, c.reg.ultimo_bloqueio == Motivo::LeituraTofInvalida);
    c.avancar(300);
    VERIFICAR(c, c.reg.autorizados == 0);

    // Leitura desatualizada (sem amostra nova há > 100 ms) também bloqueia.
    c.tof.leitura_valida = true;
    c.tof_congelado = true;
    c.avancar(150);
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.passo();
    VERIFICAR(c, c.reg.ultimo_bloqueio == Motivo::LeituraTofDesatualizada);
    c.avancar(300);
    VERIFICAR(c, c.reg.autorizados == 0);
    VERIFICAR_SAIDAS_DESLIGADAS(c);
}

void caso_comando_expirado(Cenario &c)
{
    c.preparar(80.0f);
    // Comando recebido, mas o primeiro processamento só ocorre 300 ms depois,
    // com todas as demais condições OK: não pode autorizar.
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.passo(300);
    VERIFICAR(c, c.reg.autorizados == 0);
    VERIFICAR(c, c.reg.expirados == 1);
    VERIFICAR(c, c.m.estado() == Estado::ChuteBloqueado);
    c.avancar(500);
    VERIFICAR(c, c.reg.autorizados == 0);
    VERIFICAR_SAIDAS_DESLIGADAS(c);
}

void caso_emergencia_na_pre_carga(Cenario &c)
{
    VERIFICAR(c, c.m.habilitar(c.t));
    c.avancar(200);
    VERIFICAR(c, c.m.estado() == Estado::PreCarga);
    c.m.sinalizar_emergencia(c.t, true);
    VERIFICAR(c, c.m.estado() == Estado::Emergencia);
    VERIFICAR(c, c.m.tensao_simulada() < 1.0f);
    c.avancar(1000);
    VERIFICAR(c, c.m.tensao_simulada() < 1.0f);  // não recarrega em emergência
    VERIFICAR(c, !c.m.solicitar_chute(c.t));
    VERIFICAR(c, !c.m.solicitar_carga(c.t, 80.0f));
    VERIFICAR(c, !c.m.habilitar(c.t));
    VERIFICAR(c, !c.m.rearmar(c.t));  // botão ainda pressionado
    c.m.sinalizar_emergencia(c.t, false);
    c.avancar(500);
    VERIFICAR(c, c.m.estado() == Estado::Emergencia);  // liberar não basta
    VERIFICAR(c, c.m.rearmar(c.t));
    VERIFICAR(c, c.m.estado() == Estado::Desabilitado);  // rearme exige habilitar
    VERIFICAR(c, c.reg.autorizados == 0);
    VERIFICAR_SAIDAS_DESLIGADAS(c);
}

void caso_emergencia_com_chute_pendente(Cenario &c)
{
    c.preparar(80.0f);
    c.bola(false);
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.passo();
    VERIFICAR(c, c.m.chute_pendente());
    c.m.sinalizar_emergencia(c.t, true);
    VERIFICAR(c, !c.m.chute_pendente());
    VERIFICAR(c, c.reg.ultimo_motivo_cancelamento == Motivo::Emergencia);
    c.bola(true);
    c.avancar(100);  // ainda dentro dos 200 ms do comando original
    VERIFICAR(c, c.reg.autorizados == 0);

    // Rearme completo: o comando antigo não volta a valer.
    c.m.sinalizar_emergencia(c.t, false);
    VERIFICAR(c, c.m.rearmar(c.t));
    c.preparar(80.0f);
    VERIFICAR(c, c.m.estado() == Estado::AguardandoBola);
    VERIFICAR(c, c.reg.autorizados == 0);
    // Só um NOVO comando, após o rearme, autoriza.
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.passo();
    VERIFICAR(c, c.reg.autorizados == 1);
    VERIFICAR_SAIDAS_DESLIGADAS(c);
}

void caso_falha_tof_na_espera(Cenario &c)
{
    c.preparar(80.0f);
    c.bola(false);
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.passo();
    c.tof.sensor_ok = false;
    c.passo();
    VERIFICAR(c, c.m.estado() == Estado::Falha);
    VERIFICAR(c, !c.m.chute_pendente());
    VERIFICAR(c, c.reg.ultimo_motivo_cancelamento == Motivo::TofIndisponivel);
    // Sensor volta, bola aparece: continua em FALHA, sem autorização.
    c.tof.sensor_ok = true;
    c.bola(true);
    c.avancar(500);
    VERIFICAR(c, c.m.estado() == Estado::Falha);
    VERIFICAR(c, !c.m.solicitar_chute(c.t));
    VERIFICAR(c, c.reg.autorizados == 0);
    VERIFICAR(c, c.m.rearmar(c.t));
    VERIFICAR(c, c.m.estado() == Estado::Desabilitado);
    VERIFICAR_SAIDAS_DESLIGADAS(c);
}

void caso_bola_volta_apos_comando_expirado(Cenario &c)
{
    c.preparar(80.0f);
    c.bola(false);
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.avancar(250);
    VERIFICAR(c, c.reg.expirados == 1);
    c.bola(true);
    c.avancar(1000);
    VERIFICAR(c, c.reg.autorizados == 0);  // bola voltou, mas sem comando novo
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.passo();
    VERIFICAR(c, c.reg.autorizados == 1);  // novo evento válido
    VERIFICAR_SAIDAS_DESLIGADAS(c);
}

void caso_multiplos_comandos(Cenario &c)
{
    c.preparar(80.0f);
    c.bola(false);
    // Três comandos seguidos: nenhum se acumula; cada um substitui o anterior.
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.avancar(50);
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.avancar(50);
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    VERIFICAR(c, c.reg.substituidos == 2);
    c.bola(true);
    c.avancar(100);
    VERIFICAR(c, c.reg.autorizados == 1);  // exatamente uma autorização

    // Comandos logo após a autorização: banco simulado descarregado => rejeitados.
    VERIFICAR(c, !c.m.solicitar_chute(c.t));
    c.passo();
    VERIFICAR(c, !c.m.solicitar_chute(c.t));
    c.avancar(500);
    VERIFICAR(c, c.reg.autorizados == 1);
    VERIFICAR_SAIDAS_DESLIGADAS(c);
}

void caso_limites_e_cancelamentos(Cenario &c)
{
    VERIFICAR(c, !c.m.solicitar_chute(c.t));  // desabilitado
    c.m.habilitar(c.t);
    c.avancar(1000);
    VERIFICAR(c, !c.m.solicitar_carga(c.t, 250.0f));  // acima do máximo
    VERIFICAR(c, !c.m.solicitar_carga(c.t, 10.0f));   // abaixo do mínimo
    VERIFICAR(c, !c.m.solicitar_carga(c.t, 30.0f));   // abaixo da pré-carga (50 V)
    VERIFICAR(c, c.m.solicitar_carga(c.t, 50.0f));    // igual à pré-carga: pronto
    VERIFICAR(c, c.m.estado() == Estado::AguardandoBola);

    c.bola(false);
    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.m.cancelar_chute(c.t);
    VERIFICAR(c, c.reg.ultimo_motivo_cancelamento == Motivo::CancelamentoExplicito);
    c.bola(true);
    c.avancar(100);
    VERIFICAR(c, c.reg.autorizados == 0);

    VERIFICAR(c, c.m.solicitar_chute(c.t));
    c.m.desabilitar(c.t);
    VERIFICAR(c, c.reg.ultimo_motivo_cancelamento == Motivo::Desabilitado);
    c.avancar(100);
    VERIFICAR(c, c.m.estado() == Estado::Desabilitado);
    VERIFICAR(c, c.reg.autorizados == 0);

    // Falha externa (FAULT#) bloqueia e só sai com rearme após a falha cessar.
    c.m.habilitar(c.t);
    c.falha_externa = true;
    c.passo();
    VERIFICAR(c, c.m.estado() == Estado::Falha);
    VERIFICAR(c, !c.m.rearmar(c.t));
    c.falha_externa = false;
    c.passo();
    VERIFICAR(c, c.m.rearmar(c.t));
    VERIFICAR_SAIDAS_DESLIGADAS(c);
}

struct Caso {
    const char *nome;
    void (*funcao)(Cenario &);
};

const Caso CASOS[] = {
    {"1. Bola presente e tensao correta", caso_bola_presente_tensao_correta},
    {"2. Bola ausente", caso_bola_ausente},
    {"3. Tensao abaixo do setpoint", caso_tensao_abaixo_do_setpoint},
    {"4. Leitura do TOF invalida/desatualizada", caso_leitura_tof_invalida},
    {"5. Comando de chute expirado", caso_comando_expirado},
    {"6. Emergencia durante a pre-carga", caso_emergencia_na_pre_carga},
    {"7. Emergencia com chute pendente", caso_emergencia_com_chute_pendente},
    {"8. Falha do TOF durante a espera", caso_falha_tof_na_espera},
    {"9. Retorno da bola apos comando expirado", caso_bola_volta_apos_comando_expirado},
    {"10. Multiplos comandos de chute consecutivos", caso_multiplos_comandos},
    {"11. Limites de setpoint, cancelamento, desabilitar, FAULT#", caso_limites_e_cancelamentos},
};

}  // namespace

int executar_testes(Escrita escrever)
{
    int reprovados = 0;
    char linha[160];
    escrever("========== TESTES kicker_sim ==========");
    for (const Caso &caso : CASOS) {
        Cenario c;
        c.nome_caso = caso.nome;
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

}  // namespace kicker_sim
