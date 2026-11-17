# TOF050C (VL6180X) — monitor de distância para diagnóstico

Esta etapa serve **somente para diagnóstico**. O sensor não comanda carga, pré-carga nem disparo.
O I²C escravo do kicker (`0x42`, GPIO6/GPIO7, status de 14 bytes) continua igual.

## Arquivos

| Arquivo | Função |
|---|---|
| `src/i2c_soft.h/.cpp` | Mestre I²C por software (open-drain, ~100 kHz, timeouts, recuperação do barramento) |
| `src/vl6180x.h/.cpp` | Driver do VL6180X: ID, configuração obrigatória AN4545, calibração VHV, medição única periódica, códigos de erro |
| `src/tof050c.h/.cpp` | Tarefa FreeRTOS, dados protegidos por mutex e logs na USB |
| `src/bola_detector.h/.cpp` | Detecção de presença da bola (estados, confirmações, histerese, idade das leituras) |
| `src/main.cpp` | +3 linhas: `#include`, `tof050c::iniciar()` e o comando serial `tof` |

## Ligação

| TOF050C | ESP32-C3 SuperMini |
|---|---|
| VIN | **3V3** (recomendado; veja abaixo) |
| GND | GND |
| SDA | **GPIO5** |
| SCL | **GPIO10** |
| GPIO1 / INT | não ligar |
| XSHUT / SHUT | não ligar (o módulo tem pull-up) ou ligar ao 3V3 |

- Alimente com **3,3 V**. Os pull-ups de SDA/SCL de muitos módulos vão para o VIN; com 5 V, as
  linhas do ESP32-C3 (que não tolera 5 V) ficariam em 5 V.
- Os pull-ups internos do ESP32-C3 estão ativos apenas como reforço. Se o módulo não tiver pull-ups,
  coloque 4,7 kΩ de SDA e de SCL para 3V3.
- Use fios curtos e mantenha os cabos longe do capacitor de alta tensão, do transformador e do solenoide.

### Por que GPIO5 e GPIO10

- GPIO6/GPIO7 já são do I²C escravo. O ESP32-C3 tem um único controlador I²C, por isso o sensor usa um
  barramento por software.
- Não são pinos de *strapping* (no ESP32-C3 são GPIO2, GPIO8 e GPIO9). O GPIO8 também é o LED da SuperMini.
- Não são o UART0 (GPIO20/21), em que a ROM escreve durante o boot.
- Não aparecem em `Schematic1_3` nem na documentação de firmware.
- **Pendente:** confirmar na PCB/EasyEDA da Board1_3 que GPIO5 e GPIO10 não estão roteados para
  nenhuma outra função. Para trocar os pinos, altere `PIN_SDA`/`PIN_SCL` em `src/tof050c.h`.

## Compilar e gravar

```sh
cd firmware
pio run                          # compilar
pio run -t upload                # gravar
pio device monitor -b 115200     # monitor serial
```

## Saída esperada no monitor

```text
[TOF] VL6180X OK em 0x29 | modelo 0xB4 rev 1.1 modulo 1.2 | configuracao obrigatoria carregada
[TOF] Offset fabrica: 3 mm | crosstalk: 0.00 Mcps | convergencia max 30 ms

========== TOF050C ==========
Sensor: VL6180X
I2C: 0x29 (SDA=GPIO5, SCL=GPIO10, software)
Distancia: 82 mm
Status: VALIDA
...
=============================
[TOF]  65 mm | VALIDA | sinal 4.21 Mcps
[TOF]  -- mm | INVALIDA | Codigo: 15 (ranging overflow (fora do alcance)) | bruto 255 | sinal 0.05 Mcps
```

(Os valores acima são ilustrativos, não medidos.)

- As medições são disparadas uma após a outra. A taxa real (Hz) aparece no bloco de resumo, a cada 10 s.
- Linhas `[TOF]`: até 5 por segundo por padrão. O comando `tof r` alterna para **todas as amostras**.
- O comando `tof` liga/desliga as linhas. Erros e timeouts continuam aparecendo.
- `bruto` é o valor cru do registrador, **sem significado** quando a leitura é inválida.
- Também são mostrados erros I²C, timeouts (nenhuma amostra em 500 ms) e falhas de inicialização.
  Sem sensor, uma nova tentativa é feita a cada 3 s.

## Procedimento do teste da zona cega

1. **Desabilite o estágio de potência fisicamente**: desconecte o solenoide (XT30 SOLENOID) e o capacitor
   (XT30 CAPACITOR), ou alimente somente a SuperMini pela USB, sem a fonte de POWER. Não dependa apenas
   do software. Mantenha o botão de emergência operacional.
2. Fixe o sensor e uma régua alinhada ao eixo óptico. Meça a distância da face do sensor até a
   superfície mais próxima da bola (Ø 42,67 mm).
3. Para cada distância, aguarde ~2 s parado e anote várias linhas `[TOF]`: distância, status, código e sinal.
4. Repita com a bola levemente fora do eixo e com outra iluminação ambiente.

| Distância real (mm) | Leituras (mm) | Status / código | Sinal (Mcps) | Observações |
|---:|---|---|---|---|
| 200 | | | | |
| 100 | | | | |
| 80 | | | | |
| 50 | | | | |
| 40 | | | | |
| 30 | | | | |
| 20 | | | | |
| 10 | | | | |
| 5 | | | | |
| 0 (contato) | | | | |

## Detecção de presença da bola

Lógica em `src/bola_detector.h/.cpp`, sem dependências do ESP-IDF. Ela **não aciona o kicker**: só registra
o estado e expõe `tof050c::bola_presente_autorizada()` para uma integração futura.

| Parâmetro (`src/bola_detector.h`) | Valor inicial | Função |
|---|---:|---|
| `LIMITE_ENTRADA_MM` | 49 mm | Leitura < 50 mm = posse da bola (definido pela equipe; encostada centralizada ~21–32 mm, cantos ~40–58 mm) |
| `CONFIRMACOES_NECESSARIAS` | 3 | Leituras válidas **consecutivas** ≤ limite de entrada para confirmar `BOLA_PRESENTE` |
| `LIMITE_SAIDA_MM` | 55 mm | Histerese: em `BOLA_PRESENTE`, só sai com leitura válida > 55 mm |
| `MAX_IDADE_LEITURA_MS` | 100 ms | Idade máxima da última leitura (timestamp real, `esp_timer`) |

Transições para `BOLA_AUSENTE`, todas imediatas (*fail-safe*):

- leitura válida acima de 55 mm;
- **leitura inválida** (código ≠ 0 ou erro de sistema), mesmo dentro da histerese;
- leitura desatualizada (> 100 ms sem amostra nova);
- timeout de medição, erro I²C ou reinicialização do sensor.

A contagem de confirmações é zerada por: leitura inválida, leitura ≥ 50 mm, intervalo > 100 ms entre
amostras e qualquer falha.

Saída no monitor:

```text
[TOF]  82 mm | BOLA_AUSENTE | sinal 15.20 Mcps
[TOF]  34 mm | Confirmacao 1/3 | sinal 120.40 Mcps
[TOF]  29 mm | Confirmacao 2/3 | sinal 160.10 Mcps
[TOF]  22 mm | Confirmacao 3/3 | sinal 180.70 Mcps
[BOLA] PRESENTE | 22 mm | 3 leituras validas consecutivas <= 49 mm
[TOF]  41 mm | BOLA_PRESENTE | sinal 90.00 Mcps
[TOF]  51 mm | BOLA_AUSENTE | sinal 60.00 Mcps
[BOLA] AUSENTE | 61 mm > 55 mm (limite de saida)
```

(Os valores de sinal são ilustrativos.)

- Linhas `Confirmacao k/3`, `(confirmacao interrompida)` e as transições `[BOLA]` são **sempre** impressas.
  As linhas `[BOLA]` aparecem mesmo com o log desligado (`tof`).
- As demais linhas `[TOF]` seguem o limite de 5 por segundo (ou todas, com `tof r`).
- O resumo a cada 10 s mostra o estado, o número de detecções e a taxa de medição.

### Roteiro de testes da detecção

Carregamento e disparo **fisicamente desabilitados** (solenoide e capacitor desconectados). Use `tof r`
para registrar todas as amostras.

| Cenário | Detectou? | Tempo até `PRESENTE` | Saídas indevidas | Inválidas | Taxa (Hz) | Observações |
|---|---|---|---|---|---|---|
| Centralizada, encostada | | | | | | |
| Encostada na extremidade esquerda | | | | | | |
| Encostada na extremidade direita | | | | | | |
| Aproximando-se lentamente | | | | | | |
| Aproximando-se rapidamente | | | | | | |
| Retirada repentinamente | | | | | | |
| **Sem bola** (falso positivo), 1 min | | | | | | |
| Objeto/mão a ~30 mm (falso positivo) | | | | | | |

## Como interpretar leituras inválidas

- **Leitura inválida não é "bola presente".** O firmware nunca transforma uma leitura inválida em 0 mm.
- **Códigos 6, 7, 13 e 15**: pouco sinal ou alvo fora do alcance. Aparecem sem alvo, mas também podem
  aparecer **com a bola encostada**. Nessa situação, o feixe do emissor e o campo de visão do receptor
  podem não se sobrepor na superfície da bola, e quase nenhuma luz retorna.
- **Códigos 12 e 14 (underflow)**: a distância calculada ficou abaixo de zero depois do offset. Indicam
  alvo muito perto, mas só se comprovam com o experimento.
- **Código 11**: luz ambiente excessiva (por exemplo, sol ou lâmpadas fortes).
- **Códigos 1–5 ou erro de sistema (laser safety/PLL)**: falha do sensor, não da medição.
- A taxa de sinal (Mcps) ajuda a distinguir os casos: muito alta perto do contato indica bola próxima;
  muito baixa indica ausência de retorno.

## Limitações conhecidas

- **Alcance**: o VL6180X é especificado para 0–100 mm. Com o fator de escala padrão (×1), a leitura é
  limitada a 255 mm. Os "500 mm" anunciados para o módulo não correspondem ao VL6180X.
  Acima de ~100–150 mm (com a bola pequena e curva), espere erros de convergência.
- **Zona cega de 0–20 mm**: é a especificação do fabricante do módulo, não do chip. O teste existe para medi-la.
- **Calibração**: a calibração de temperatura (VHV) é feita na inicialização e repetida a cada 255
  medições. O offset e o crosstalk de fábrica são lidos e exibidos, mas não alterados. Com janela ou
  tampa na frente do sensor, seria necessária uma calibração de crosstalk.
- **Taxa × ruído**: para chegar a ~120 Hz com o alvo próximo, o readout averaging foi reduzido de 48 para
  16 amostras (4,4 → 2,3 ms), e a convergência máxima, de 49 para 30 ms. Isso aumenta um pouco o ruído e
  reduz o alcance no limite. Os valores ficam em `AVERAGING_AMOSTRAS` e `MAX_CONVERGENCIA_MS` (`src/vl6180x.h`).
  Sem alvo, a taxa cai para ~25 Hz, porque cada medição espera a convergência máxima.
- **I²C por software**: SCL de ~200 kHz; cada transação ocupa a CPU por ~0,15 ms, com ~4 transações por amostra
  (prioridade 1, igual ao loop principal). Interrupções apenas esticam o clock, o que o I²C permite.
- **Validação**: compilação verificada com ESP-IDF 6.1.0. **Os testes físicos ainda não foram feitos.**
