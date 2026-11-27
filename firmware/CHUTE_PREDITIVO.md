# Chute preditivo (TOF050C)

O software do robô **pede** o chute; o kicker **decide o instante** com o sensor TOF050C. Para a
bola parada, basta ela estar encostada. Para a bola em movimento, o firmware estima quando ela vai
tocar o kicker e dispara alguns milissegundos antes.

## Por que prever

Medido em bancada com a regra antiga (só "bola presente", 3 leituras a até 49 mm):

- Com a bola à vista o sensor mede a ~50 Hz (uma leitura a cada ~20 ms).
- A faixa entre 49 mm e o contato (~33 mm) tem 16 mm. A 1 m/s a bola a atravessa em ~15 ms, menos
  que uma leitura. As 3 confirmações levam ~36 ms.
- Resultado: o chute saía **~24 ms depois do contato**, com a bola já quicando para trás. Bolas
  acima de ~1,1 m/s muitas vezes nem eram detectadas no primeiro contato.

## Como funciona

Com um pedido de chute ativo, o chute é autorizado quando **uma** destas condições vale (além de
leitura válida e recente e `FAULT#` inativo):

1. **Bola presente**: parada ou lenta. 3 leituras seguidas a até 49 mm (`src/bola_detector.h`).
2. **Chegada prevista** (`src/preditor_chegada.h`):
   - a cada leitura válida acima de 38 mm (contato + 5 mm), estima a velocidade de aproximação por
     regressão linear das últimas leituras (a partir de 2; usa 3 quando há);
   - `t_contato = t_leitura + (d - 33 mm) / v`;
   - dispara quando `agora >= t_contato - antecipação` (padrão **6 ms**);
   - **não** dispara se a leitura mais recente tiver mais de 30 ms (medição perdida), se a bola se
     afastar, se a velocidade sair de 0,15–6 m/s ou depois de `t_contato + 20 ms`.

O pedido continua obrigatório: o preditor sozinho nunca chuta. O mestre deve reenviar `03` a cada
~50 ms enquanto quiser chutar (veja [I2C_PROTOCOL.md](I2C_PROTOCOL.md)).

### Bola parada

Funciona pela regra de presença. Foram 30 chutes com a bola parada e encostada (20 a ~48 V e 10 a
~140–150 V), todos autorizados por presença; a ~150 V a bola sai a ~3,5 m/s. Com a bola parada
**perto mas sem encostar** (40–49 mm), a presença também autoriza, mas o êmbolo pode não alcançar
a bola: se isso acontecer em jogo, reduza `LIMITE_ENTRADA_MM`.

### Robô andando até a bola

Para o sensor, é a mesma coisa que a bola vindo: o preditor age. Não foi testado com o robô em
movimento.

## Parâmetros

| Parâmetro | Padrão | Serial (só até o próximo boot) |
|---|---|---|
| Preditor ligado | sim | `pred on` / `pred off` |
| Antecipação do disparo | 6 ms | `pred lat <ms>` |
| Leitura com a bola encostada | 33 mm | `pred dc <mm>` |
| Leituras na regressão | 3 | `pred n <2-6>` |
| Idade máxima da leitura para disparar | 30 ms | (só no código) |

`pred` sem argumento mostra a configuração. Os padrões estão em `chegada::Config`
(`src/preditor_chegada.h`).

### A antecipação não é a latência da bola parada

Com a bola parada, o êmbolo leva ~12 ms (a ~48 V) ou ~9 ms (a ~145 V) do sinal até a bola começar a
se mover. Com a bola **chegando**, esse não é o melhor valor:

| Antecipação (~140–150 V) | Forte | Mais ou menos / fraca | Morta |
|---|---|---|---|
| 0 ms | | 2 | |
| 3 ms | 1 | | 1¹ |
| **6 ms** | **5** | | |
| 9 ms | 1 | 1 | 2 |
| 12 ms | | 1 | 1 |
| 15 ms | | | 1 |

¹ Previsão feita sem leitura recente (antes da proteção de 30 ms).

Cedo demais, o êmbolo termina o curso e fica parado estendido (pulso de 50 ms); a bola bate nele
parado e "morre". Tarde demais, pega a bola quicando e o chute sai fraco. **A antecipação depende da
tensão**: recalibre se o jogo usar outra tensão.

## Resultados (bancada, nov/2026)

| Rodada | Chutes | Tensão | Momento do impacto × chegada da bola |
|---|---|---|---|
| Regra antiga | 14 | ~48 V | 13 de 14 chutes ~24 ms depois do contato |
| Preditor, antecipação 12 ms | 42 | ~48 V | mediana +1,7 ms, faixa −10 a +5 ms; bolas de 0,55–2,28 m/s |
| Preditor, antecipação 9 ms | 41 | ~140 V | mediana −1,2 ms, faixa −3 a +6 ms; bolas até 3,0 m/s |
| **Configuração final** (6 ms + proteção) | 15 | ~140 V | 9 fortes, 3 mais ou menos, 2 fracas, 1 morta (avaliação visual) |

Com a antecipação de 9 ms, cerca de metade das bolas era "morta".

## Limitações conhecidas

- **Precisão da previsão (±5 ms)** é da ordem da janela boa (~3–6 ms). As leituras de longe
  (150–190 mm) costumam superestimar a velocidade em relação à que a bola tem perto do kicker, e o
  chute sai cedo. Mais leituras perto do kicker melhorariam isso (taxa maior do sensor).
- **Medições perdidas**: com a bola à vista, 3–7% das medições não retornam e só são refeitas após
  50 ms. Em ~9% das chegadas isso deixa a previsão sem leitura recente; a proteção de 30 ms bloqueia
  o disparo pela previsão, e o chute fica por conta da regra de presença (sai atrasado).
- A avaliação "forte / morta" foi visual. A velocidade de saída medida pelo sensor é subestimada
  (a bola rápida sai do alcance antes de 2 leituras).

## Como recalibrar

Pré-requisitos: ESP na USB (`/dev/ttyACM0`), Python 3 com `pyserial`. ⚠️ Alta tensão: mãos fora da
frente do êmbolo.

```bash
cd firmware
# 1. Latência com a bola PARADA (encoste a bola quando o script avisar)
python3 tools/captura_chute.py --alvo 150 --histerese 15 --latencia --pausa 12 --duracao 240
# 2. Varredura da antecipação com jogadas (alterna a cada jogada)
python3 tools/captura_chute.py --alvo 150 --histerese 15 --varrer-lat 0,3,6,9,12 --pausa 12 --duracao 600
# 3. Validação com o valor escolhido (antes: "pred lat <ms>" pela serial, ou mude o padrão)
python3 tools/captura_chute.py --alvo 150 --histerese 15 --pausa 12 --duracao 420
```

O script pede o chute a cada 50 ms, pausa depois de cada chute até a recarga terminar e avisa por
voz ("pode jogar"). As capturas vão para `tools/capturas/` (fora do git).

Para testar outros parâmetros sem jogar a bola de novo, reproduza uma captura no preditor:

```bash
cd test/host
g++ -std=c++17 -O2 -I../../src ../../src/preditor_chegada.cpp replay_chegada.cpp -o replay_chegada
./replay_chegada ../../tools/capturas/<arquivo>.csv 33 6 3   # contato_mm antecipação_ms leituras
```

## Testes

`teste` pela serial (ou `test/host/main.cpp` no PC) roda, além dos anteriores:

- **P1–P12** (preditor): aproximação constante, bola rápida com 2 leituras, bola se afastando,
  leitura inválida, intervalo longo, bola parada longe, bola que para antes, perto do contato,
  janela pós-contato, velocidade absurda, reset, previsão sem leitura recente.
- **A13–A16** (autorização): chegada prevista autoriza; sem pedido, com `FAULT#` ou com leitura
  velha, não.
