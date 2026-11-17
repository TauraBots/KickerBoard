# Etapa 3 — Kicker em modo SIMULAÇÃO

Máquina de estados que integra a tensão **simulada**, a presença da bola (TOF050C) e o comando de
chute, e decide se um chute **seria** autorizado. **Nenhum atuador é acionado.**

## Como compilar e usar

O modo simulação só é ativado por um ambiente próprio do PlatformIO. O ambiente padrão continua sendo
o firmware normal.

```sh
cd firmware
pio run -e esp32-c3-supermini-sim -t upload    # grava o firmware de SIMULAÇÃO
pio device monitor -b 115200
```

No boot aparece o banner `KICKER EM MODO SIMULACAO`. Para voltar ao firmware normal:
`pio run -e esp32-c3-supermini -t upload`.

### Comandos no monitor

| Comando | Efeito na simulação |
|---|---|
| `sim hab` / `sim des` | habilita (inicia a pré-carga simulada de 50 V) / desabilita |
| `20`–`200` | seleciona o setpoint (padrão 80 V) |
| `1` | solicita carga simulada até o setpoint selecionado |
| `2` | para: cancela chute e carga pendentes |
| `3` | comando de chute (válido por 200 ms) |
| `N%` | carga simulada em N% de 200 V (**o auto-kick não é simulado**) |
| `sim emerg` / `sim liberar` / `sim rearmar` | emergência simulada / soltar o botão / rearmar |
| `sim falha` / `sim falha off` | falha externa simulada |
| `sim cancelar` | cancela o comando de chute pendente |
| `sim status` | estado, tensões, TOF, avaliação atual e saídas físicas |
| `sim teste` | **executa os testes automatizados no ESP32** |
| `status`, `tof`, `tof r` | inalterados |

Sequência típica:

```text
sim hab        -> PRE_CARGA -> (50 V) -> AGUARDANDO_COMANDO
80             -> setpoint selecionado
1              -> CARGA_SOLICITADA -> (80 V) -> AGUARDANDO_BOLA
(bola no kicker: [BOLA] PRESENTE)
3              -> CHUTE_AUTORIZADO (SIMULACAO) -> GPIO1 permanece DESABILITADO -> PRE_CARGA
```

## Máquina de estados

| Estado | Significado |
|---|---|
| `DESABILITADO` | inicial; também após `sim des` ou rearme |
| `PRE_CARGA` | rampa simulada até 50 V (100 V/s) |
| `AGUARDANDO_COMANDO` | pré-carregado; aguardando pedido de carga |
| `CARGA_SOLICITADA` | rampa simulada até o setpoint |
| `AGUARDANDO_BOLA` | no setpoint; aguardando bola + comando de chute |
| `CHUTE_AUTORIZADO` | todas as condições satisfeitas (transitório: simula a descarga e volta à `PRE_CARGA`) |
| `CHUTE_BLOQUEADO` | comando expirou sem autorização (transitório) |
| `FALHA` | FAULT# real, falha simulada ou perda do TOF durante carga/espera — exige rearme |
| `EMERGENCIA` | emergência simulada — exige soltar e rearmar |

Autorização exige **todos**: comando de chute com ≤ 200 ms; carga solicitada; tensão simulada em
setpoint ± 2 V (e ≤ 205 V); TOF operante; última leitura válida, com ≤ 100 ms e timestamp coerente;
`BOLA_PRESENTE`; nenhuma falha ou emergência. Cada comando gera **no máximo uma** autorização. Não há
fila: um novo comando substitui o pendente. Emergência, falha, desabilitar, `2` e `sim cancelar`
descartam o comando. Após emergência ou falha, é preciso `sim rearmar` + `sim hab` + novo comando.

Os parâmetros ficam em `kicker_sim::Config` (`src/kicker_sim.h`).

## Garantias de segurança (firmware)

- **Somente duas funções** do firmware levam os atuadores a nível alto: `iniciar_carregamento()`
  (GPIO4) e `solicitar_chute()` (GPIO1). Todos os caminhos passam por elas (Serial, I²C, auto-kick).
  No build de simulação, ambas começam com uma barreira `#if KICKER_SIMULACAO` que força o pino a 0 e
  retorna.
- **Verificado no binário**: no firmware de simulação, as mensagens "KICK disparado!" e
  "Carregamento ON" não existem, porque o compilador eliminou o código de acionamento. No firmware
  normal, elas continuam presentes.
- Comandos Serial e I²C de carga e chute são desviados para a máquina de estados antes de chegar
  àquelas funções.
- A cada ciclo do loop principal, o GPIO1 e o GPIO4 são reescritos em 0.
- `kicker_sim::saidas_fisicas()` é separada do estado lógico e retorna sempre desligado
  (`PERMITE_ACIONAMENTO_FISICO = false`, verificado por `static_assert`). Os testes conferem isso a
  cada passo.
- A tensão simulada nunca é usada fora da máquina de estados.
- A autorização lê o TOF só por cópia protegida por mutex (`tof050c::obter_leitura`), sem transação I²C.

## Limitações

- **O firmware não controla os pinos entre o reset e o `gpio_config()`** (boot ROM e bootloader).
  O nível do GPIO1/GPIO4 nesse intervalo depende do hardware da placa (pull-downs). Isso já valia para
  o firmware normal. **Os testes devem ser feitos com o solenoide e o capacitor fisicamente
  desconectados.**
- **O botão de emergência físico não está ligado a nenhum GPIO**: o ESP32 não sabe quando ele é
  acionado. Ele continua atuando de forma independente no hardware, e a emergência da máquina de
  estados é só a simulada (`sim emerg`).
- **Status I²C de 14 bytes inalterado**: ele continua reportando o estado **físico real**
  (`carregando = 0`, tensão do ADC real), não a tensão simulada. A placa principal não vê os estados
  simulados pelo I²C.
- **Comando `05 PP` (I²C) e `N%`**: na simulação só pedem a carga; o auto-kick não é simulado (é
  preciso o comando de chute). Isso é informado no monitor.
- **Setpoint abaixo da tensão atual é rejeitado**: não existe descarga controlada sem chute.
- **FAULT# real entra como falha** (`USAR_FAULT_REAL` em `src/kicker_sim_firmware.cpp`). Se o pino
  ficar ativo com o LT3751 desligado, a máquina fica em `FALHA`. Mude para `false` só para o teste.

## Testes automatizados

`src/kicker_sim_testes.cpp`, 11 casos: os 10 pedidos + limites, cancelamento, desabilitar e FAULT#.
Em todos é verificado, a cada passo, que as saídas físicas permanecem desligadas.

- **No ESP32**: grave o firmware de simulação e digite `sim teste`.
- **No PC** (precisa de g++/clang++ com C++17): veja `test/host/main.cpp`.
