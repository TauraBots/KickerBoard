#!/usr/bin/env python3
"""Captura de calibração do chute gated pelo TOF050C.

Fica reenviando o pedido de chute ("3") pela serial, como o software do robô
faria pelo rádio/I2C, e grava cada amostra do sensor (linhas "C,") e cada
disparo (linhas "K,") em CSV, com o relógio da própria ESP (µs).

Depois de cada chute, pausa os pedidos por --pausa segundos (recarga da
pré-carga) e avisa quando dá para jogar a bola de novo.

Uso:
    python3 tools/captura_chute.py --porta /dev/ttyACM0 --duracao 300
"""

import argparse
import csv
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import serial


def abrir(porta: str, baud: int) -> serial.Serial:
    s = serial.Serial()
    s.port = porta
    s.baudrate = baud
    s.timeout = 0.005
    # Não mexe em DTR/RTS: no USB-Serial/JTAG do ESP32-C3 eles podem resetar o chip.
    s.dtr = False
    s.rts = False
    s.open()
    return s


def avisar(texto: str, falar: bool) -> None:
    """Aviso por voz (spd-say) e notificação na tela, sem bloquear a captura."""
    if not falar:
        return
    som = "/usr/share/sounds/freedesktop/stereo/complete.oga"
    partes = []
    if shutil.which("paplay") and Path(som).exists():
        partes.append(f"paplay {som}")
    if shutil.which("spd-say"):
        partes.append(f"spd-say -w -l pt -r 10 '{texto}'")
    if partes:
        # bipe + voz, duas vezes, em segundo plano
        cmd = "; ".join(partes * 2)
        subprocess.Popen(["sh", "-c", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if shutil.which("notify-send"):
        subprocess.Popen(["notify-send", "-t", "3000", "Kicker", texto], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def enviar(s: serial.Serial, comando: str) -> None:
    s.write((comando + "\n").encode())


def ler_linhas(s: serial.Serial, buffer: bytearray):
    dados = s.read(4096)
    if dados:
        buffer.extend(dados)
    while b"\n" in buffer:
        linha, _, resto = buffer.partition(b"\n")
        buffer[:] = resto
        yield linha.decode(errors="replace").strip()


def ligar_captura(s: serial.Serial, buffer: bytearray, ligar: bool) -> bool:
    """Envia 'tof c' até o estado da captura ser o desejado."""
    alvo = "LIGADA" if ligar else "DESLIGADA"
    for _ in range(3):
        enviar(s, "tof c")
        limite = time.monotonic() + 1.0
        while time.monotonic() < limite:
            for linha in ler_linhas(s, buffer):
                if "captura CSV" in linha:
                    if alvo in linha:
                        return True
                    break  # estado oposto: alterna de novo
            else:
                continue
            break
    return False


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--porta", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--duracao", type=float, default=300.0, help="segundos de captura")
    p.add_argument("--periodo", type=float, default=0.05, help="intervalo entre pedidos de chute (s)")
    p.add_argument("--pausa", type=float, default=3.0, help="pausa após cada chute (s)")
    p.add_argument("--saida", default=None, help="arquivo CSV (padrão: capturas/chute_<data>.csv)")
    p.add_argument("--alvo", type=float, default=None,
                   help="alvo da pré-carga em V (pre alvo; volta a 50 V no boot). Espera carregar antes de liberar")
    p.add_argument("--histerese", type=float, default=5.0, help="com --alvo: recarrega abaixo de alvo - histerese (V)")
    p.add_argument("--varrer-lat", default=None,
                   help="lista de latências (ms) alternadas a cada jogada, ex.: 6,9,12,15 (envia 'pred lat')")
    p.add_argument("--sem-voz", action="store_true", help="não avisa por voz/notificação")
    p.add_argument("--latencia", action="store_true",
                   help="teste de latência: só pede o chute com a bola PARADA e encostada (leituras estáveis)")
    a = p.parse_args()

    saida = Path(a.saida) if a.saida else Path(__file__).resolve().parent / "capturas" / time.strftime("chute_%Y%m%d_%H%M%S.csv")
    saida.parent.mkdir(parents=True, exist_ok=True)

    parar = False

    def ao_sinal(*_):
        nonlocal parar
        parar = True

    signal.signal(signal.SIGTERM, ao_sinal)
    signal.signal(signal.SIGINT, ao_sinal)

    s = abrir(a.porta, a.baud)
    buffer = bytearray()
    time.sleep(0.3)
    s.reset_input_buffer()
    if not ligar_captura(s, buffer, True):
        print("ERRO: a ESP nao confirmou 'tof c' (firmware sem a captura?)", flush=True)
        return 1

    if a.alvo is not None:
        enviar(s, f"pre alvo {a.alvo:.0f} {a.histerese:.0f}")
        time.sleep(0.2)
    # Rearma a pré-carga (pode ter sido desarmada pelo limite de recargas por minuto).
    enviar(s, "pre on")
    print(f"Gravando em {saida}", flush=True)
    chutes = 0
    amostras = 0
    with open(saida, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tipo", "t_us", "valida", "bruto_mm", "codigo", "bola", "sinal_mcps", "prev", "t_disparo_us", "v_mm_ms", "tensao_v", "t_host"])
        inicio = time.monotonic()
        proximo_pedido = 0.0
        pausa_ate = 0.0
        avisou_pronto = False
        recentes = []  # (valida, mm) das últimas amostras, para o modo latência
        # Com --alvo, só libera depois da recarga atingir o alvo (ou 25 s, se não houver recarga).
        aguardando_carga = a.alvo is not None
        lats = [float(x) for x in a.varrer_lat.split(",")] if a.varrer_lat else []
        indice_lat = 0
        espera_carga_ate = time.monotonic() + 25.0
        while not parar and time.monotonic() - inicio < a.duracao:
            agora = time.monotonic()
            if aguardando_carga and agora >= espera_carga_ate:
                aguardando_carga = False
            if agora >= pausa_ate and not aguardando_carga:
                if not avisou_pronto:
                    if lats:
                        lat = lats[indice_lat % len(lats)]
                        indice_lat += 1
                        enviar(s, f"pred lat {lat:g}")
                        w.writerow(["L", "", "", "", "", "", "", "", "", "", "", f"{agora:.4f}", lat])
                        print(f"    (latencia desta jogada: {lat:g} ms)", flush=True)
                    if a.latencia:
                        print(f">>> PRONTO: encoste a bola PARADA no kicker ({agora - inicio:.0f} s)", flush=True)
                        avisar("pode encostar a bola", not a.sem_voz)
                    else:
                        print(f">>> PRONTO: pode jogar a bola ({agora - inicio:.0f} s)", flush=True)
                        avisar("pode jogar", not a.sem_voz)
                    avisou_pronto = True
                if a.latencia:
                    # ~0,5 s de leituras válidas, todas em 28..40 mm, variação <= 3 mm.
                    ultimas = recentes[-25:]
                    estavel = (len(ultimas) == 25 and all(v for v, _ in ultimas)
                               and all(28 <= d <= 40 for _, d in ultimas)
                               and max(d for _, d in ultimas) - min(d for _, d in ultimas) <= 3)
                    if estavel and agora >= proximo_pedido:
                        enviar(s, "3")
                        proximo_pedido = agora + 1.0  # um pedido por vez
                elif agora >= proximo_pedido:
                    enviar(s, "3")
                    proximo_pedido = agora + a.periodo
            for linha in ler_linhas(s, buffer):
                campos = linha.split(",")
                if campos[0] == "C" and len(campos) in (7, 10):
                    if len(campos) == 7:
                        campos += ["", "", ""]  # firmware sem preditor
                    w.writerow(campos + ["", f"{agora:.4f}"])
                    amostras += 1
                    recentes.append((campos[2] == "1", int(campos[3])))
                    del recentes[:-50]
                elif campos[0] == "K" and len(campos) == 3:
                    w.writerow(["K", campos[1], "", "", "", "", "", "", "", "", campos[2], f"{agora:.4f}"])
                    chutes += 1
                    print(f"*** CHUTE #{chutes} ({campos[2]} V) - aguarde {a.pausa:.0f} s", flush=True)
                    avisar("chutou, aguarde", not a.sem_voz)
                    pausa_ate = time.monotonic() + a.pausa
                    avisou_pronto = False
                    if a.alvo is not None:
                        aguardando_carga = True
                        espera_carga_ate = time.monotonic() + 25.0
                elif "PRE-CARGA ATINGIDA" in linha or linha.startswith(("Carga encerrada", "Tensão no desligamento")):
                    if "PRE-CARGA ATINGIDA" in linha:
                        aguardando_carga = False
                    print("   " + linha, flush=True)
                elif linha.startswith(("[BOLA]", "[CHUTE] AUTORIZADO", "[TOF] Erro", "[TOF] TIMEOUT", "[PRE]")):
                    print("   " + linha, flush=True)
            f.flush()

    # Só para de pedir chutes (o último pedido expira em 200 ms); a pré-carga segue armada.
    ligar_captura(s, buffer, False)
    s.close()
    print(f"Fim: {chutes} chute(s), {amostras} amostras -> {saida}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
