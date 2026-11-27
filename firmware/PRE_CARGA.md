# Pré-carga mantida (hardware real)

Mantém o banco de capacitores em ~50 V e recarrega sozinho quando a tensão cai. Existe **somente no
firmware normal** (`esp32-c3-supermini`). No firmware de simulação, o comando responde
"indisponível" e o código nem é compilado.

## Comportamento

| Item | Valor (`src/main.cpp`) |
|---|---|
| Alvo | `PRE_CARGA_ALVO_V` = 50 V |
| Recarrega abaixo de | `PRE_CARGA_RECARGA_V` = 45 V |
| Intervalo mínimo entre o fim de uma carga e a próxima recarga | 1 s |
| Máximo de recargas | 6 por janela de 60 s (acima disso, desarma) |
| Duração máxima armada | sem limite (`PRE_CARGA_DURACAO_MAX_MS = 0`) |
| Arma no boot | **sim**, 2 s após ligar (`PRE_CARGA_ARMAR_NO_BOOT`) |

> ⚠️ **O capacitor é carregado até 50 V automaticamente ~2 s depois que a placa recebe energia**,
> inclusive na bancada. Considere alta tensão presente sempre que a placa estiver alimentada
> (firmware normal). O firmware de simulação nunca arma a pré-carga.

- Se o FAULT# estiver ativo no boot, a pré-carga **não** é armada (fica desarmada até `pre on` ou `1`).
- Cada recarga usa a rotina de carga existente: limite de 205 V, timeout de 15 s, FAULT#, DONE#.
- **Desarma** com: `pre off`, `2`, I²C `02`, FAULT#, timeout de carga, limite de segurança ou
  excesso de recargas.
- **Rearma** com: `pre on`, `1` ou I²C `01` (que também carregam até o setpoint), ou reiniciando a placa.
- **Setpoint (`20`–`200` ou I²C `04`) durante uma recarga** é aceito: a recarga termina em 50 V e o
  novo setpoint vale para o próximo `1`. Durante uma carga normal continua sendo recusado.
- **`1` durante uma recarga** continua a carga até o setpoint normal. **`1` com a pré-carga
  mantida** carrega de ~50 V até o setpoint, e a pré-carga continua armada.
- **Depois de um chute** (`3`, auto-kick ou I²C `03`), com a pré-carga armada, o capacitor é
  recarregado até 50 V após 1 s.
- Desarmar **não descarrega** o capacitor: ele continua com a tensão que tinha.
- O pacote de status I²C não mudou: durante uma recarga, o bit "carregando" fica em 1 e a tensão é
  a do ADC.

### Alvo temporário (`pre alvo`)

`pre alvo <V> [histerese]` muda o alvo da pré-carga (20–200 V) e, opcionalmente, a histerese da
recarga (2–30 V, padrão 5 V). **Vale só até o próximo boot**: a placa sempre liga com 50 V, para
nunca energizar já carregando até uma tensão alta.

> ⚠️ Com `pre alvo 150`, o capacitor fica em ~150 V o tempo todo e cada chute é muito mais forte.

A 150 V o capacitor perde ~5 V em 2–6 s. Com a histerese padrão de 5 V, a pré-carga recarrega a
cada poucos segundos e estoura o limite de 6 recargas por minuto. Para calibração a ~150 V use
`pre alvo 150 15`: o capacitor fica entre 135 e 150 V e recarrega bem menos (medido: ~43 recargas
em 10 min com um chute a cada ~14 s, sem desarmar).

## Procedimento de teste

**Antes:**
1. **Solenoide desconectado** (XT30 SOLENOID). No firmware normal, `3` e o auto-kick disparam de verdade.
2. Meio de **descarga** preparado (por exemplo, um resistor de alguns kΩ e 5 W com pontas isoladas).
   Sem o solenoide, o chute não descarrega o capacitor.
3. **Multímetro** nos terminais do capacitor.
4. Botão de emergência ao alcance.

**Gravar e abrir o monitor:**

```sh
pio run -e esp32-c3-supermini -t upload
pio device monitor -e esp32-c3-supermini
```

**Sequência:**

| Passo | Comando | Esperado |
|---|---|---|
| 1 | (boot) | `[PRE] ATENCAO: ... armada automaticamente em 2000 ms` e, após 2 s, `[PRE] ARMADA` |
| 2 | (automático) | `[PRE] Recarga #1: 0.0 V -> 50 V`, rampa e `Carga encerrada: PRE-CARGA ATINGIDA` |
| 3 | (multímetro) | **Comparar com o ADC.** Se a diferença passar de ~2 V, parar e calibrar o ADC |
| 4 | aguardar | A tensão cai pela fuga; abaixo de 45 V: `[PRE] Recarga #n ... X s desde a anterior` |
| 5 | `pre` | Estado, número de recargas e tempo restante |
| 6 | `pre off` | `[PRE] DESARMADA` (o capacitor continua carregado) |
| 7 | descarga | Descarregar com o resistor e conferir 0 V no multímetro |

**Anote:** a tensão do ADC contra a do multímetro, o tempo de cada recarga (`Tempo de carga`) e o
intervalo entre recargas (indica a fuga do banco).

**Situações de parada imediata** (cortar a alimentação, usar a emergência):
- tensão no multímetro **acima de ~55 V**;
- carga que não para;
- aquecimento anormal do LT3751, do MOSFET ou do transformador.
