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
| `01` | Iniciar carregamento |
| `02` | Parar carregamento |
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
| 1 | Versão | Versão do protocolo, atualmente `01` |
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
