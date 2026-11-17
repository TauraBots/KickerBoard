#include "kicker_sim_firmware.h"

// Compilado apenas no ambiente de simulação; no firmware normal este arquivo fica vazio.
#if defined(KICKER_SIMULACAO) && KICKER_SIMULACAO

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include "autorizacao_chute_testes.h"
#include "bola_detector.h"
#include "esp_timer.h"
#include "kicker_sim.h"
#include "kicker_sim_testes.h"
#include "tof050c.h"

namespace kicker_sim_fw {
namespace {

using kicker_sim::Estado;
using kicker_sim::Motivo;
using Tipo = kicker_sim::Notificacao::Tipo;

// FAULT# real do LT3751 entra como falha externa. Com o LT3751 desabilitado
// (CHARGE em 0) o pino deve ficar inativo; se atrapalhar o teste, mude para false.
constexpr bool USAR_FAULT_REAL = true;
constexpr float SETPOINT_INICIAL_V = 80.0f;
constexpr uint32_t INTERVALO_LOG_TENSAO_MS = 250;  // durante a rampa simulada

float tensao_adc = 0.0f;
float setpoint_selecionado = SETPOINT_INICIAL_V;
bool falha_simulada = false;
uint64_t ultimo_log_tensao = 0;

uint64_t agora_ms() { return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL; }

kicker_sim::EntradaTof ler_tof()
{
    // Cópia protegida por mutex da última leitura: nenhuma transação I2C aqui.
    tof050c::Leitura l = {};
    kicker_sim::EntradaTof e = {};
    if (!tof050c::obter_leitura(&l)) return e;  // módulo não iniciado: tudo falso
    e.sensor_ok = l.sensor_ok;
    e.tem_amostra = l.tem_amostra;
    e.leitura_valida = l.valida;
    e.bola_presente = l.bola_presente;
    e.instante_ms = l.instante_ms;
    e.distancia_mm = l.distancia_mm;
    return e;
}

void imprimir_saidas_fisicas(const kicker_sim::Maquina &m);

class Impressor : public kicker_sim::Ouvinte {
public:
    const kicker_sim::Maquina *maquina = nullptr;

    void notificar(const kicker_sim::Notificacao &n) override
    {
        const float tensao = maquina ? maquina->tensao_simulada() : 0.0f;
        switch (n.tipo) {
            case Tipo::Transicao:
                if (n.motivo == Motivo::Nenhum)
                    printf("[KICKER] Estado: %s -> %s\n", kicker_sim::nome_estado(n.de), kicker_sim::nome_estado(n.para));
                else
                    printf("[KICKER] Estado: %s -> %s (%s)\n", kicker_sim::nome_estado(n.de),
                           kicker_sim::nome_estado(n.para), kicker_sim::descrever_motivo(n.motivo));
                printf("[KICKER] Tensao simulada: %.0f V | ADC real: %.1f V\n", tensao, tensao_adc);
                break;
            case Tipo::TensaoAtingida:
                printf("[KICKER] Tensao simulada: %.0f V (atingida)\n", n.valor);
                break;
            case Tipo::SetpointAceito:
                printf("[KICKER] Setpoint solicitado: %.0f V\n", n.valor);
                break;
            case Tipo::SetpointRejeitado:
                printf("[KICKER] Setpoint %.1f V REJEITADO: %s\n", n.valor, kicker_sim::descrever_motivo(n.motivo));
                break;
            case Tipo::ChuteRecebido:
                printf("[KICKER] Comando de chute recebido (valido por %lu ms)\n",
                       static_cast<unsigned long>(maquina ? maquina->config().validade_comando_ms : 0));
                break;
            case Tipo::ChuteSubstituido:
                printf("[KICKER] Comando de chute anterior substituido pelo novo\n");
                break;
            case Tipo::ChuteRejeitado:
                printf("[KICKER] Comando de chute REJEITADO: %s\n", kicker_sim::descrever_motivo(n.motivo));
                break;
            case Tipo::ChuteExpirado:
                printf("[KICKER] Comando de chute EXPIRADO | ultimo bloqueio: %s\n",
                       kicker_sim::descrever_motivo(n.motivo));
                break;
            case Tipo::ChuteCancelado:
                printf("[KICKER] Comando de chute CANCELADO: %s\n", kicker_sim::descrever_motivo(n.motivo));
                break;
            case Tipo::Bloqueio: {
                const kicker_sim::EntradaTof tof = ler_tof();
                if (n.motivo == Motivo::TensaoInsuficiente || n.motivo == Motivo::TensaoForaDaFaixa)
                    printf("[KICKER] Chute bloqueado: %s (%.0f V, setpoint %.0f V)\n",
                           kicker_sim::descrever_motivo(n.motivo), tensao, maquina ? maquina->setpoint() : 0.0f);
                else if (tof.leitura_valida)
                    printf("[KICKER] Chute bloqueado: %s (TOF %u mm)\n", kicker_sim::descrever_motivo(n.motivo),
                           tof.distancia_mm);
                else
                    printf("[KICKER] Chute bloqueado: %s\n", kicker_sim::descrever_motivo(n.motivo));
                break;
            }
            case Tipo::Autorizado: {
                const kicker_sim::EntradaTof tof = ler_tof();
                printf("[KICKER] CHUTE_AUTORIZADO (SIMULACAO) | %.0f V | TOF %u mm | BOLA_PRESENTE\n", tensao,
                       tof.distancia_mm);
                if (maquina) imprimir_saidas_fisicas(*maquina);
                break;
            }
            case Tipo::RearmeNegado:
                printf("[KICKER] Rearme NEGADO: %s\n", kicker_sim::descrever_motivo(n.motivo));
                break;
        }
    }
};

Impressor impressor;
kicker_sim::Maquina maquina(kicker_sim::Config(), &impressor);

void imprimir_saidas_fisicas(const kicker_sim::Maquina &m)
{
    const kicker_sim::SaidasFisicas s = m.saidas_fisicas();
    printf("[KICKER] GPIO1 (CHUTE) permanece %s | GPIO4 (CARREGAMENTO) %s\n",
           s.chute ? "HABILITADO" : "DESABILITADO", s.carregamento ? "HABILITADO" : "DESABILITADO");
}

void imprimir_status()
{
    const kicker_sim::EntradaTof tof = ler_tof();
    const uint64_t agora = agora_ms();
    char distancia[16];
    if (tof.leitura_valida) snprintf(distancia, sizeof(distancia), "%u mm", tof.distancia_mm);
    else snprintf(distancia, sizeof(distancia), "--");
    printf("\n========== KICKER (SIMULACAO) ==========\n"
           "Estado: %s\n"
           "Tensao simulada: %.0f V | Setpoint: %.0f V | Pre-carga: %.0f V\n"
           "Tensao ADC real: %.1f V (somente monitorada)\n"
           "Setpoint selecionado para '1': %.0f V\n"
           "TOF: %s | %s | %s | idade %llu ms\n"
           "Chute pendente: %s | Autorizacoes: %lu\n"
           "Falha simulada: %s | Avaliacao agora: %s\n",
           kicker_sim::nome_estado(maquina.estado()), maquina.tensao_simulada(), maquina.setpoint(),
           maquina.config().tensao_pre_carga_v, tensao_adc, setpoint_selecionado,
           tof.sensor_ok ? "OK" : "INDISPONIVEL", distancia, tof.bola_presente ? "BOLA_PRESENTE" : "BOLA_AUSENTE",
           static_cast<unsigned long long>(tof.instante_ms <= agora ? agora - tof.instante_ms : 0),
           maquina.chute_pendente() ? "SIM" : "NAO", static_cast<unsigned long>(maquina.autorizacoes()),
           falha_simulada ? "ATIVA" : "INATIVA", kicker_sim::descrever_motivo(maquina.avaliar(agora, tof)));
    imprimir_saidas_fisicas(maquina);
    printf("========================================\n");
}

void imprimir_ajuda()
{
    printf("[KICKER] MODO SIMULACAO: GPIO1 (CHUTE) e GPIO4 (CARREGAMENTO) nunca sao acionados.\n"
           "[KICKER] Comandos: sim hab | sim des | 1 = carga (setpoint selecionado) | 2 = parar | 3 = chute\n"
           "[KICKER]           20-200 = seleciona setpoint | N%% = carga em N%% de 200 V (sem auto-kick)\n"
           "[KICKER]           sim emerg | sim liberar | sim rearmar | sim falha | sim falha off\n"
           "[KICKER]           sim cancelar | sim status | sim teste | sim ajuda\n");
}

void escrever_linha(const char *linha) { printf("%s\n", linha); }

bool processar_sim(const char *arg)
{
    const uint64_t agora = agora_ms();
    if (!strcasecmp(arg, "hab")) {
        if (!maquina.habilitar(agora))
            printf("[KICKER] Habilitar NEGADO no estado %s%s\n", kicker_sim::nome_estado(maquina.estado()),
                   maquina.estado() == Estado::Desabilitado ? " (falha/emergencia ativa)" : " (use 'sim rearmar' ou 'sim des')");
    } else if (!strcasecmp(arg, "des")) {
        maquina.desabilitar(agora);
    } else if (!strcasecmp(arg, "emerg")) {
        printf("[KICKER] Emergencia SIMULADA acionada (o botao fisico e independente do ESP32)\n");
        maquina.sinalizar_emergencia(agora, true);
    } else if (!strcasecmp(arg, "liberar")) {
        printf("[KICKER] Emergencia simulada liberada; use 'sim rearmar'\n");
        maquina.sinalizar_emergencia(agora, false);
    } else if (!strcasecmp(arg, "rearmar")) {
        if (maquina.rearmar(agora)) printf("[KICKER] Rearmado; use 'sim hab' para habilitar\n");
    } else if (!strcasecmp(arg, "falha")) {
        falha_simulada = true;
        maquina.sinalizar_falha(agora);
    } else if (!strcasecmp(arg, "falha off")) {
        falha_simulada = false;
        printf("[KICKER] Falha simulada removida; use 'sim rearmar'\n");
    } else if (!strcasecmp(arg, "cancelar")) {
        maquina.cancelar_chute(agora);
    } else if (!strcasecmp(arg, "status") || !*arg) {
        imprimir_status();
    } else if (!strcasecmp(arg, "teste")) {
        const int reprovados = kicker_sim::executar_testes(escrever_linha) + autorizacao::executar_testes(escrever_linha);
        printf("[KICKER] Testes: %s\n", reprovados ? "HA REPROVACOES" : "TODOS APROVADOS");
    } else if (!strcasecmp(arg, "ajuda")) {
        imprimir_ajuda();
    } else {
        printf("[KICKER] Comando de simulacao invalido: '%s' (use 'sim ajuda')\n", arg);
    }
    return true;
}

}  // namespace

void iniciar()
{
    impressor.maquina = &maquina;
    printf("\n################################################\n"
           "#  KICKER EM MODO SIMULACAO (KICKER_SIMULACAO)  #\n"
           "#  Carga e chute fisicos BLOQUEADOS no firmware #\n"
           "################################################\n");
    printf("[KICKER] Estado: %s | pre-carga %.0f V | comando valido por %lu ms | TOF max %lu ms\n",
           kicker_sim::nome_estado(maquina.estado()), maquina.config().tensao_pre_carga_v,
           static_cast<unsigned long>(maquina.config().validade_comando_ms),
           static_cast<unsigned long>(maquina.config().max_idade_tof_ms));
    imprimir_ajuda();
}

void atualizar(float tensao_adc_real, bool fault_real_ativo)
{
    tensao_adc = tensao_adc_real;
    const uint64_t agora = agora_ms();
    const bool falha = falha_simulada || (USAR_FAULT_REAL && fault_real_ativo);
    maquina.atualizar(agora, ler_tof(), falha);

    // Durante as rampas simuladas, mostra a evolução da tensão.
    const Estado e = maquina.estado();
    if ((e == Estado::PreCarga || e == Estado::CargaSolicitada) && agora - ultimo_log_tensao >= INTERVALO_LOG_TENSAO_MS) {
        ultimo_log_tensao = agora;
        printf("[KICKER] Tensao simulada: %.0f V | ADC real: %.1f V\n", maquina.tensao_simulada(), tensao_adc);
    }
}

bool processar_comando_serial(const char *comando)
{
    const uint64_t agora = agora_ms();
    if (!strncasecmp(comando, "sim", 3) && (comando[3] == '\0' || comando[3] == ' ')) {
        const char *arg = comando + 3;
        while (*arg == ' ') ++arg;
        return processar_sim(arg);
    }
    if (!strcmp(comando, "1")) {
        maquina.solicitar_carga(agora, setpoint_selecionado);
        return true;
    }
    if (!strcmp(comando, "2")) {
        maquina.parar(agora);
        printf("[KICKER] Parar: comando de chute/carga simulados cancelados\n");
        return true;
    }
    if (!strcmp(comando, "3")) {
        maquina.solicitar_chute(agora);
        return true;
    }

    char *fim = nullptr;
    const float valor = strtof(comando, &fim);
    if (fim == comando) return false;  // não numérico: status, tof etc. seguem para o main
    if (*fim == '%' && fim[1] == '\0') {
        if (valor > 0.0f && valor <= 100.0f) {
            printf("[KICKER] %.0f%%: carga simulada para %.0f V. Auto-kick NAO e simulado; envie '3' para o chute.\n",
                   valor, valor / 100.0f * 200.0f);
            maquina.solicitar_carga(agora, valor / 100.0f * 200.0f);
        } else {
            printf("[KICKER] Porcentagem invalida. Envie de 1%% a 100%%.\n");
        }
        return true;
    }
    if (*fim == '\0') {
        if (valor >= maquina.config().setpoint_min_v && valor <= maquina.config().setpoint_max_v) {
            setpoint_selecionado = valor;
            printf("[KICKER] Setpoint selecionado: %.0f V (envie '1' para solicitar a carga simulada)\n", valor);
        } else {
            printf("[KICKER] Setpoint invalido. Use de %.0f a %.0f V.\n", maquina.config().setpoint_min_v,
                   maquina.config().setpoint_max_v);
        }
        return true;
    }
    return false;
}

void processar_comando_remoto(ComandoRemoto comando, float valor)
{
    const uint64_t agora = agora_ms();
    switch (comando) {
        case ComandoRemoto::Carga:
            printf("[KICKER] I2C 0x01: carga simulada para %.0f V\n", setpoint_selecionado);
            maquina.solicitar_carga(agora, setpoint_selecionado);
            break;
        case ComandoRemoto::Parar:
            printf("[KICKER] I2C 0x02: parar (simulado)\n");
            maquina.parar(agora);
            break;
        case ComandoRemoto::Chute:
            printf("[KICKER] I2C 0x03: chute (simulado)\n");
            maquina.solicitar_chute(agora);
            break;
        case ComandoRemoto::Setpoint:
            setpoint_selecionado = valor;
            printf("[KICKER] I2C 0x04: setpoint selecionado %.1f V\n", valor);
            break;
        case ComandoRemoto::Percentual:
            printf("[KICKER] I2C 0x05: carga simulada para %.0f%% (%.0f V); auto-kick NAO e simulado\n", valor,
                   valor / 100.0f * 200.0f);
            maquina.solicitar_carga(agora, valor / 100.0f * 200.0f);
            break;
    }
}

}  // namespace kicker_sim_fw

#endif  // KICKER_SIMULACAO
