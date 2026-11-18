# Protocolo I²C do KickerBoard

O ESP32-C3 funciona como **escravo I²C**.

- Endereço de 7 bits: `0x42`
- SDA: GPIO6
- SCL: GPIO7
- Nível lógico: 3,3 V
- Frequência recomendada: 100 kHz
- Ordem de inteiros de 16 bits: little-endian (byte menos significativo primeiro)

Use GND comum e resistores pull-up externos (por exemplo, 4,7 kΩ para 3,3 V) em SDA e SCL. Os pull-ups internos também estão habilitados, mas não substituem os externos em uma ligação real.

## Comandos de escrita

Cada comando deve ser enviado em uma transação de escrita separada.

| Bytes | Operação |
|---|---|
| `00` | Selecionar status; não altera o kicker |
| `01` | Iniciar carregamento (também rearma a pré-carga mantida, se estiver desarmada) |
| `02` | Parar carregamento (também desarma a pré-carga mantida, se estiver armada) |
| `03` | Disparar KICK imediatamente |
| `04 VV VV` | Alterar setpoint em décimos de volt, de 200 a 2000 |
| `05 PP` | Carregar e disparar automaticamente em `PP`%, de 1 a 100 |

Exemplos:

- Setpoint de 150,0 V: `04 DC 05` (`0x05DC` = 1500)
- Auto-kick em 80%: `05 50`

O comando de disparo deve ser tratado como uma operação perigosa pelo controlador mestre. Não o retransmita automaticamente em caso de timeout sem antes ler o status.

## Leitura do status

Uma leitura de 14 bytes retorna:

| Offset | Campo | Descrição |
|---:|---|---|
| 0 | Cabeçalho | Sempre `A5` |
| 1 | Versão | Versão do protocolo, atualmente `02` (ver "Mudanças da versão 02") |
| 2 | Flags | Bits descritos abaixo |
| 3 | Sequência | Incrementa a cada resposta |
| 4–5 | Tensão | Tensão medida em décimos de volt |
| 6–7 | Setpoint | Setpoint em décimos de volt |
| 8–9 | Alvo automático | Alvo do auto-kick em décimos de volt |
| 10–11 | ADC | Leitura ADC bruta |
| 12 | Resultado | `00` = último comando aceito; `01` = comando inválido ou fila cheia |
| 13 | CRC-8 | CRC dos bytes 0 a 12 |

Flags do byte 2:

- Bit 0: carregando
- Bit 1: auto-kick armado
- Bit 2: `DONE#` ativo
- Bit 3: `FAULT#` ativo
- Bit 4: bola presente (TOF050C: `BOLA_PRESENTE` confirmado com leitura de até 100 ms)
- Bit 5: o último pedido de chute terminou **sem chute** (bloqueado, expirado ou cancelado);
  volta a 0 no próximo chute autorizado
- Bits 6–7: reservados (0)

Os bits 4 e 5 são uma extensão compatível: antes valiam sempre 0, e o tamanho e os demais campos
do pacote não mudaram.

## Mudanças da versão 02

O formato (14 bytes, campos e CRC) é o mesmo da versão 01. Mudou o **comportamento**:

| Item | Versão 01 | Versão 02 |
|---|---|---|
| `03` | dispara imediatamente | cria um pedido que só chuta com a bola presente em até 200 ms |
| `05 PP` (auto-kick) | dispara ao atingir o alvo | ao atingir o alvo, para a carga e pede o chute (mesma regra) |
| `01` | inicia o carregamento | inicia o carregamento **e rearma a pré-carga mantida** |
| `02` | para o carregamento | para o carregamento, **desarma a pré-carga** e cancela o pedido de chute |
| Flags bit 4 | sempre 0 | bola presente |
| Flags bit 5 | sempre 0 | último pedido de chute terminou sem chute |
| Pré-carga | não existia | mantém o capacitor em ~50 V desde o boot |

O mestre deve conferir o byte 1 e tratar `03` como um **pedido**: depois de enviá-lo, ler o
status e verificar o bit 5 (ou a queda da tensão) para saber se o chute ocorreu.

## Autorização do chute

O comando `03` (assim como `3` pela Serial e o auto-kick) **não dispara imediatamente**: ele cria
um pedido que só gera chute se, **em até 200 ms**, o TOF050C indicar bola presente com leitura
válida e recente e o `FAULT#` estiver inativo. Caso contrário, o pedido expira e o bit 5 do status
vai a 1. Cada pedido gera no máximo um chute. Um novo `03` substitui o pendente (não há fila), e
`02` cancela o pedido pendente. No auto-kick (`05`), a carga para no alvo e o chute fica pendente
pela mesma regra.

O CRC usa polinômio `0x07`, valor inicial `0x00`, sem reflexão e sem XOR final.

O mestre pode fazer uma leitura direta de 14 bytes. Caso a biblioteca exija uma seleção de registrador, escreva `00` e depois faça a leitura.

## Exemplo de mestre com Arduino `Wire`

```cpp
constexpr uint8_t KICKER = 0x42;

// Iniciar carregamento
Wire.beginTransmission(KICKER);
Wire.write(0x01);
Wire.endTransmission();

// Armar auto-kick em 80%
Wire.beginTransmission(KICKER);
Wire.write(0x05);
Wire.write(80);
Wire.endTransmission();

// Ler o pacote de status
uint8_t status[14];
if (Wire.requestFrom(KICKER, sizeof(status)) == sizeof(status)) {
    for (size_t i = 0; i < sizeof(status); ++i) {
        status[i] = Wire.read();
    }
}
```

## Diagnóstico pela Serial

O comando `i2c` na Serial liga e desliga um log do escravo I2C (desligado por padrão):

- `[I2C] RX n byte(s): XX XX -> ACEITO|REJEITADO`: cada escrita do mestre (exceto a seleção `00`);
- `[I2C] 1 s: ...`: a cada segundo, leituras de status, seleções `00`, comandos aceitos e
  rejeitados, e eventos perdidos; `NENHUM TRAFEGO` quando nada chegou.

Uma cadência de leitura bem acima da normal (por exemplo, 200 leituras/s em vez de 50) indica
que o mestre está rejeitando o pacote de status e relendo em modo de erro, como acontecia com
um mestre que só aceitava a versão `01`.
