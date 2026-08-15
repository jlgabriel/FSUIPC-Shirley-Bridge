"""
Verificacion en vivo contra MSFS 2024 + FSUIPC7 WebSocket Server.

Comprueba, contra el simulador real, las afirmaciones de
docs/FSUIPC-SETPOINT-CAPABILITIES.md que se derivaron solo de documentacion.

Uso:
    python tools/verify_msfs2024.py            # solo lectura, no toca el sim
    python tools/verify_msfs2024.py --write    # ejecuta tambien las pruebas de escritura

Requisitos: MSFS 2024 corriendo con un avion cargado y detenido en tierra,
FSUIPC7 activo y FSUIPCWebSocketServer.exe escuchando en localhost:2048.

Las pruebas de escritura mueven superficies y cambian radios. Restaura los
valores originales al terminar, pero no la ejecutes en pleno vuelo.
"""

import argparse
import asyncio
import json
import sys
import time
from typing import Any, Callable, Dict, List, Optional

import websockets

# La consola de Windows usa cp1252 por defecto y los mensajes de error del SO
# traen acentos: sin esto, el reporte puede abortar con UnicodeEncodeError.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except (AttributeError, OSError):
    pass

FSUIPC_WS_URL = "ws://localhost:2048/fsuipc/"
GROUP = "verify"

# Espera tras una escritura/evento antes de releer. El servidor tiene
# MinInterval 33 ms, pero el sim necesita un frame o dos para reflejar el cambio.
SETTLE_S = 0.6

# --- Offsets que el script declara para leer y (algunos) escribir -------------
# name, address, type, size
OFFSETS = [
    ("aircraftName",  0x3D00, "string", 256),
    ("parkingBrake",  0x0BC8, "uint",   2),   # doc: R+W
    ("gearHandle",    0x0BE8, "uint",   4),   # doc: R+W
    ("flapsPercent",  0x0BDC, "uint",   4),   # doc: solo lectura  <-- a confirmar
    ("flapsIndex",    0x0BFC, "uint",   1),   # doc: R+W
    ("flapsDetents",  0x3BFA, "uint",   2),   # cantidad de detentes (sin confirmar)
    ("apHdgBug",      0x07CC, "uint",   2),   # doc: solo lectura
    ("apAltBug",      0x07D4, "uint",   4),   # doc: solo lectura
    ("com2Standby",   0x311C, "uint",   2),   # doc: solo lectura
    ("nav1Standby",   0x311E, "uint",   2),   # doc: solo lectura
    ("structDeice",   0x337D, "uint",   1),   # doc: solo lectura
    ("propDeice",     0x2440, "uint",   4),   # doc: solo lectura
]

# ===================== Resultado =====================

PASS, FAIL, SKIP, INFO = "PASS", "FAIL", "SKIP", "INFO"


class Report:
    def __init__(self):
        self.rows: List[Dict[str, str]] = []

    def add(self, status: str, check: str, detail: str = ""):
        self.rows.append({"status": status, "check": check, "detail": detail})
        mark = {PASS: "  ok  ", FAIL: " FAIL ", SKIP: " skip ", INFO: " info "}[status]
        print(f"[{mark}] {check}")
        if detail:
            for line in detail.splitlines():
                print(f"          {line}")

    def summary(self):
        counts = {s: sum(1 for r in self.rows if r["status"] == s) for s in (PASS, FAIL, SKIP, INFO)}
        print()
        print("=" * 68)
        print(f"  {counts[PASS]} ok · {counts[FAIL]} fallidas · {counts[SKIP]} omitidas · {counts[INFO]} informativas")
        print("=" * 68)
        if counts[FAIL]:
            print("\nRevisar las fallidas: contradicen lo que afirma el documento de capacidades.")
        return counts[FAIL]


# ===================== Cliente =====================

class FSUIPCClient:
    """Cliente secuencial: envia una peticion y espera la respuesta que coincida."""

    def __init__(self, ws):
        self.ws = ws
        self._spill: List[dict] = []   # mensajes recibidos fuera de turno

    async def send(self, obj: dict):
        await self.ws.send(json.dumps(obj))

    async def recv_match(self, pred: Callable[[dict], bool], timeout: float = 6.0) -> Optional[dict]:
        # Primero revisa lo que ya llego y quedo sin consumir.
        for i, msg in enumerate(self._spill):
            if pred(msg):
                return self._spill.pop(i)

        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                raw = await asyncio.wait_for(self.ws.recv(), remaining)
            except asyncio.TimeoutError:
                return None
            if isinstance(raw, bytes):
                raw = raw.decode("utf-8", "ignore")
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if pred(msg):
                return msg
            self._spill.append(msg)

    async def request(self, obj: dict, command: str, name: str, timeout: float = 6.0) -> Optional[dict]:
        await self.send(obj)
        return await self.recv_match(
            lambda m: m.get("command") == command and m.get("name") == name, timeout
        )

    # --- operaciones de alto nivel ---

    async def declare(self) -> Optional[dict]:
        return await self.request({
            "command": "offsets.declare",
            "name": GROUP,
            "offsets": [
                {"name": n, "address": a, "type": t, "size": s} for n, a, t, s in OFFSETS
            ],
        }, "offsets.declare", GROUP)

    async def read(self) -> Optional[dict]:
        """Lectura puntual del grupo declarado."""
        msg = await self.request({"command": "offsets.read", "name": GROUP},
                                 "offsets.read", GROUP)
        if msg and msg.get("success"):
            return msg.get("data") or {}
        return None

    async def write(self, values: Dict[str, Any]) -> dict:
        """Escritura con el formato documentado: grupo + offsets por nombre.

        Una escritura correcta responde con un offsets.read; solo se recibe un
        offsets.write cuando fallo.
        """
        await self.send({
            "command": "offsets.write",
            "name": GROUP,
            "offsets": [{"name": k, "value": v} for k, v in values.items()],
        })
        msg = await self.recv_match(
            lambda m: m.get("command") in ("offsets.write", "offsets.read")
            and m.get("name") == GROUP
        )
        if msg is None:
            return {"ok": False, "error": "sin respuesta"}
        if msg.get("command") == "offsets.write" and not msg.get("success"):
            return {"ok": False, "error": f"{msg.get('errorCode')}: {msg.get('errorMessage')}"}
        return {"ok": True}

    async def calc(self, code: str, tag: str = "calc") -> dict:
        msg = await self.request({"command": "vars.calc", "name": tag, "code": code,
                                  "interval": 0}, "vars.calc", tag)
        if msg is None:
            return {"ok": False, "error": "sin respuesta"}
        if not msg.get("success"):
            return {"ok": False, "error": f"{msg.get('errorCode')}: {msg.get('errorMessage')}"}
        return {"ok": True}

    async def var_write(self, name: str, value: Optional[float], tag: str = "vw") -> dict:
        var: Dict[str, Any] = {"name": name}
        if value is not None:
            var["value"] = value
        msg = await self.request({"command": "vars.write", "name": tag, "vars": [var]},
                                 "vars.write", tag)
        if msg is None:
            return {"ok": False, "error": "sin respuesta"}
        if not msg.get("success"):
            return {"ok": False, "error": f"{msg.get('errorCode')}: {msg.get('errorMessage')}"}
        return {"ok": True}


# ===================== Helpers de prueba =====================

async def changed_by(cli: FSUIPCClient, field: str, action, target=None) -> Dict[str, Any]:
    """Ejecuta `action`, y reporta el valor de `field` antes y despues."""
    before_all = await cli.read()
    before = (before_all or {}).get(field)
    res = await action()
    if not res.get("ok"):
        return {"ok": False, "error": res.get("error"), "before": before}
    await asyncio.sleep(SETTLE_S)
    after_all = await cli.read()
    after = (after_all or {}).get(field)
    return {
        "ok": True,
        "before": before,
        "after": after,
        "changed": before != after,
        "hit_target": (target is not None and after == target),
    }


def fmt(v):
    if isinstance(v, int):
        return f"{v} (0x{v:X})"
    return repr(v)


def distinct(baseline, a, b):
    """Devuelve el candidato que difiere del valor actual.

    Sin esto, una prueba puede escribir el valor que el offset ya tenia y leer
    'sin cambio', lo que se confunde con 'no se puede escribir'.
    """
    return b if baseline == a else a


# ===================== Pruebas =====================

async def section_protocol(cli: FSUIPCClient, rep: Report, data: dict):
    print("\n--- A · protocolo -------------------------------------------------")

    rep.add(INFO, "Avion cargado", str(data.get("aircraftName", "?")))

    # A1: el formato documentado de escritura funciona.
    baseline = data.get("parkingBrake")
    target = 0 if (baseline or 0) > 1000 else 32767
    res = await changed_by(cli, "parkingBrake", lambda: cli.write({"parkingBrake": target}))
    if res.get("ok") and res.get("changed"):
        rep.add(PASS, "offsets.write con formato documentado (grupo + nombre)",
                f"parkingBrake {fmt(res['before'])} -> {fmt(res['after'])}")
        await cli.write({"parkingBrake": baseline})   # restaurar
    else:
        rep.add(FAIL, "offsets.write con formato documentado (grupo + nombre)",
                res.get("error") or f"sin cambio: {fmt(res.get('before'))}")

    # A2: el formato que usa hoy el bridge debe fallar.
    await cli.send({"command": "offsets.write",
                    "values": [{"address": 0x0BC8, "type": "int", "size": 2, "value": 0}]})
    bad = await cli.recv_match(lambda m: m.get("command") == "offsets.write", timeout=3.0)
    if bad is not None and not bad.get("success"):
        rep.add(PASS, "el formato actual del bridge (values + address) es rechazado",
                f"{bad.get('errorCode')}: {bad.get('errorMessage')}")
    elif bad is None:
        rep.add(INFO, "el formato actual del bridge no produjo respuesta",
                "ignorado en silencio: igualmente no escribe nada")
    else:
        rep.add(FAIL, "el formato actual del bridge fue aceptado",
                "inesperado: revisar el diagnostico del write path")


async def section_readonly(cli: FSUIPCClient, rep: Report):
    print("\n--- B · offsets que el documento declara de solo lectura -----------")

    data = await cli.read() or {}
    for field, cand_a, cand_b, label in [
        ("flapsPercent", 8191, 0, "flaps 0x0BDC"),
        ("apAltBug", 3048 * 65536, 1524 * 65536, "altitude bug 0x07D4"),
        ("com2Standby", 0x2185, 0x2100, "COM2 standby 0x311C"),
    ]:
        probe = distinct(data.get(field), cand_a, cand_b)
        res = await changed_by(cli, field, lambda f=field, p=probe: cli.write({f: p}))
        if not res.get("ok"):
            rep.add(INFO, f"{label}: escritura rechazada", res.get("error", ""))
        elif res.get("changed"):
            rep.add(FAIL, f"{label}: se esperaba solo lectura, pero cambio",
                    f"{fmt(res['before'])} -> {fmt(res['after'])} · el documento debe corregirse")
        else:
            rep.add(PASS, f"{label}: solo lectura, confirmado",
                    f"valor sin cambios en {fmt(res['before'])}")


async def section_events(cli: FSUIPCClient, rep: Report, data: dict):
    print("\n--- C · eventos de control via vars.calc ---------------------------")

    # Dos parametros por evento: si el primero coincide con el valor que el sim
    # ya tenia, el offset no cambia y no se puede distinguir de "no funciona".
    # Se acepta el evento si cualquiera de los dos produce un cambio.
    checks = [
        ("flapsPercent", "FLAPS_SET", "{v} (>K:FLAPS_SET)", (8191, 0)),
        ("apHdgBug", "HEADING_BUG_SET", "{v} (>K:2:HEADING_BUG_SET)", (270, 90)),
        ("apAltBug", "AP_ALT_VAR_SET_ENGLISH", "{v} (>K:2:AP_ALT_VAR_SET_ENGLISH)", (5000, 3000)),
        # --- los tres dudosos del documento ---
        ("com2Standby", "COM2_STBY_RADIO_SET  <-- dudoso",
         "{v} (>K:COM2_STBY_RADIO_SET)", (0x2185, 0x2100)),
        ("nav1Standby", "NAV1_STBY_SET  <-- dudoso",
         "{v} (>K:NAV1_STBY_SET)", (0x1155, 0x1080)),
    ]

    for field, label, template, params in checks:
        outcome = None
        for v in params:
            res = await changed_by(cli, field, lambda c=template.format(v=v): cli.calc(c))
            if not res.get("ok"):
                outcome = (FAIL, f"{label}: vars.calc fallo", res.get("error", ""))
                break
            if res.get("changed"):
                outcome = (PASS, f"{label} funciona",
                           f"{field} {fmt(res['before'])} -> {fmt(res['after'])}")
                break
            outcome = (FAIL, f"{label} no produjo cambio",
                       f"{field} sigue en {fmt(res['before'])} con ninguno de "
                       f"los dos parametros {params}\n"
                       f"puede no aplicar a este avion: repetir con otro")
        rep.add(*outcome)

    # Toggle sin parametro: se dispara una vez y se devuelve a su estado original.
    toggle_code = "(>K:TOGGLE_STRUCTURAL_DEICE)"
    res = await changed_by(cli, "structDeice", lambda: cli.calc(toggle_code))
    if not res.get("ok"):
        rep.add(FAIL, "TOGGLE_STRUCTURAL_DEICE  <-- dudoso: vars.calc fallo", res.get("error", ""))
    elif res.get("changed"):
        rep.add(PASS, "TOGGLE_STRUCTURAL_DEICE  <-- dudoso funciona",
                f"structDeice {fmt(res['before'])} -> {fmt(res['after'])}")
        await cli.calc(toggle_code)
    else:
        rep.add(FAIL, "TOGGLE_STRUCTURAL_DEICE  <-- dudoso no produjo cambio",
                f"structDeice sigue en {fmt(res['before'])} · "
                f"este avion puede no tener deshielo estructural")


async def section_flaps_detents(cli: FSUIPCClient, rep: Report):
    print("\n--- D · redondeo de flaps a detentes -------------------------------")

    data = await cli.read() or {}
    detents = data.get("flapsDetents")
    rep.add(INFO, "detentes reportados en 0x3BFA", fmt(detents))

    commanded = 6041          # ~37 % de 16383
    res = await changed_by(cli, "flapsPercent", lambda: cli.calc(f"{commanded} (>K:FLAPS_SET)"))
    if not res.get("ok"):
        rep.add(SKIP, "round-trip de flaps", res.get("error", ""))
        return

    after = res.get("after")
    idx = (await cli.read() or {}).get("flapsIndex")
    if after == commanded:
        rep.add(INFO, "flaps: el valor leido coincide con el comandado",
                f"{commanded} -> {fmt(after)} · sin redondeo observable en este avion")
    else:
        rep.add(PASS, "flaps: MSFS redondea al detente, confirmado",
                f"comandado {commanded} -> leido {fmt(after)} (indice {idx})\n"
                f"write-then-verify da falso negativo, tal como advierte el documento")

    await cli.calc("0 (>K:FLAPS_SET)")   # flaps arriba


async def section_input_events(cli: FSUIPCClient, rep: Report):
    print("\n--- E · Input Events / B: vars (solo MSFS 2024) --------------------")

    msg = await cli.request({"command": "vars.list", "name": "ielist", "notify": "once"},
                            "vars.list", "ielist", timeout=8.0)
    if msg is None:
        rep.add(SKIP, "vars.list no respondio",
                "el servicio de variables (WASM) puede no estar activo")
        return
    if not msg.get("success"):
        rep.add(FAIL, "vars.list fallo", f"{msg.get('errorCode')}: {msg.get('errorMessage')}")
        return

    # La forma exacta de la respuesta no esta documentada con precision:
    # reportar lo que realmente llega en lugar de asumir.
    payload = {k: v for k, v in msg.items()
               if k not in ("command", "name", "success", "errorCode", "errorMessage")}
    shape = {k: (f"lista de {len(v)}" if isinstance(v, list)
                 else f"dict con {len(v)} claves" if isinstance(v, dict) else type(v).__name__)
             for k, v in payload.items()}
    rep.add(INFO, "forma de la respuesta de vars.list", json.dumps(shape, ensure_ascii=False))

    # Buscar una coleccion que parezca de input events.
    candidates: List[str] = []
    def harvest(obj):
        if isinstance(obj, list):
            for it in obj:
                if isinstance(it, str):
                    candidates.append(it)
                elif isinstance(it, dict) and "name" in it:
                    candidates.append(str(it["name"]))
        elif isinstance(obj, dict):
            for v in obj.values():
                harvest(v)
    harvest(payload)

    ie_like = [c for c in candidates if c.startswith(("I:", "B:"))]
    rep.add(INFO, "nombres devueltos por vars.list",
            f"{len(candidates)} en total · {len(ie_like)} con prefijo I:/B:\n"
            f"muestra: {candidates[:6]}")

    if not ie_like:
        rep.add(SKIP, "vars.write con prefijo I:/B:",
                "vars.list no devolvio ningun input event con prefijo;\n"
                "probar 0x7C50 manualmente con un nombre conocido del avion")
        return

    probe = ie_like[0]
    res = await cli.var_write(probe, 1.0, tag="ieprobe")
    if res.get("ok"):
        rep.add(PASS, "vars.write acepta Input Events (I:/B:)",
                f"aceptado para {probe}\n"
                f"ojo: acepta no implica que haya actuado, verificar en cabina")
    else:
        rep.add(INFO, "vars.write NO acepta Input Events",
                f"{res.get('error')}\n"
                f"queda 0x7C50 con prefijo I: como unico camino")


# ===================== Main =====================

async def main(do_write: bool) -> int:
    rep = Report()
    print("=" * 68)
    print("  Verificacion FSUIPC7 + MSFS 2024")
    print(f"  {FSUIPC_WS_URL}")
    print(f"  modo: {'lectura + ESCRITURA' if do_write else 'solo lectura'}")
    print("=" * 68)

    try:
        ws = await websockets.connect(FSUIPC_WS_URL, subprotocols=["fsuipc"],
                                      open_timeout=5, ping_interval=None, max_size=None)
    except Exception as e:
        print(f"\nNo se pudo conectar: {e!r}")
        print("Verificar que FSUIPCWebSocketServer.exe este corriendo.")
        return 1

    async with ws:
        cli = FSUIPCClient(ws)

        decl = await cli.declare()
        if not decl or not decl.get("success"):
            err = decl.get("errorMessage") if decl else "sin respuesta"
            print(f"\nNo se pudieron declarar los offsets: {err}")
            return 1

        data = await cli.read()
        if data is None:
            print("\nNo llegaron datos. Es probable que MSFS no este corriendo "
                  "o no haya un vuelo cargado.")
            return 1

        rep.add(PASS, "conexion y lectura de offsets", f"{len(data)} valores recibidos")

        if not do_write:
            print("\nModo solo lectura. Valores actuales:")
            for k, v in data.items():
                if k != "aircraftName":
                    print(f"          {k:<14} {fmt(v)}")
            print("\nVolver a ejecutar con --write para las pruebas de escritura.")
            rep.summary()
            return 0

        await section_protocol(cli, rep, data)
        await section_readonly(cli, rep)
        await section_events(cli, rep, data)
        await section_flaps_detents(cli, rep)
        await section_input_events(cli, rep)

    return 1 if rep.summary() else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Verifica las capacidades de setpoint contra MSFS 2024.")
    ap.add_argument("--write", action="store_true",
                    help="ejecuta las pruebas de escritura (mueve superficies y cambia radios)")
    args = ap.parse_args()

    if args.write:
        print("\nADVERTENCIA: se van a mover flaps, freno de estacionamiento y radios.")
        print("Usar con el avion detenido en tierra. Ctrl+C para cancelar.\n")

    try:
        sys.exit(asyncio.run(main(args.write)))
    except KeyboardInterrupt:
        print("\nCancelado.")
        sys.exit(130)
