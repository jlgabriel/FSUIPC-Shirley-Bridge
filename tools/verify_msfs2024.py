"""
Verificacion en vivo contra MSFS 2024 + FSUIPC7 WebSocket Server.

Barre campo por campo las afirmaciones de docs/FSUIPC-SETPOINT-CAPABILITIES.md,
que se escribieron a partir de documentacion. Para cada campo prueba las dos
vias posibles -- escritura directa de offset y evento de control -- y reporta
cual funciona de verdad.

Uso:
    python tools/verify_msfs2024.py                # solo lectura, no toca el sim
    python tools/verify_msfs2024.py --write        # barrido completo
    python tools/verify_msfs2024.py --write --position   # incluye teletransporte

Requisitos: MSFS 2024 con un avion cargado y detenido en tierra, motor y
bateria encendidos (varias comprobaciones necesitan avionica alimentada),
FSUIPC7 activo y C:\\FSUIPC7\\Utils\\FSUIPCWebSocketServer.exe corriendo.

NOTAS DE PROTOCOLO (verificadas en vivo contra el servidor v1.1.4):

1. `offsets.read` responde por cambio, no por peticion. Si nada cambio desde
   la ultima respuesta, el servidor NO contesta -- ni siquiera un payload
   vacio. Por eso aca se suscribe una vez con `interval` y se mantiene un
   estado acumulado, en lugar de pedir lecturas puntuales.

2. Un `offsets.write` mal formado (sin nombre de grupo) se descarta en
   silencio: no llega ni una respuesta de error.

3. "El offset cambio" no equivale a "la escritura funciono". Algunos offsets
   se mueven solos y otros interpretan la escritura como un incremento --
   0x07D4 suma 1000 ft por escritura, sin importar el valor. Por eso cada
   comprobacion espera un valor concreto, no un cambio cualquiera.
"""

import argparse
import asyncio
import json
import sys
from typing import Any, Callable, Dict, List, Optional, Tuple

import websockets

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace", line_buffering=True)
except (AttributeError, OSError):
    pass

FSUIPC_WS_URL = "ws://localhost:2048/fsuipc/"
GROUP = "verify"
READ_INTERVAL_MS = 150
CHANGE_TIMEOUT_S = 2.5

# ===================== Offsets declarados =====================
# name, address, type, size
OFFSETS: List[Tuple[str, int, str, int]] = [
    ("aircraftName", 0x3D00, "string", 256),
    # levers
    ("flapsPercent", 0x0BDC, "uint", 4),
    ("flapsIndex",   0x0BFC, "uint", 1),
    ("flapsIncr",    0x3BFA, "uint", 2),
    ("spoilers",     0x0BD0, "uint", 4),
    ("gearHandle",   0x0BE8, "uint", 4),
    ("carbHeat",     0x08B2, "uint", 2),
    # autopilot
    ("apMaster",     0x07BC, "uint", 4),
    ("apHdgLock",    0x07C8, "uint", 4),
    ("apHdgBug",     0x07CC, "uint", 2),
    ("apAltLock",    0x07D0, "uint", 4),
    ("apAltBug",     0x07D4, "uint", 4),
    ("apVsHold",     0x07EC, "uint", 4),
    ("apVsTarget",   0x07F2, "int",  2),
    # radios
    ("com1Standby",  0x311A, "uint", 2),
    ("com2Standby",  0x311C, "uint", 2),
    ("nav1Standby",  0x311E, "uint", 2),
    ("transponder",  0x0354, "uint", 2),
    # systems
    ("battery",      0x281C, "uint", 4),
    ("pitotHeat",    0x029C, "uint", 1),
    ("parkingBrake", 0x0BC8, "uint", 2),
    ("propDeice",    0x2440, "uint", 4),
    ("structDeice",  0x337D, "uint", 1),
    # indicators / environment
    ("kohlsman",     0x0330, "uint", 2),
    ("oat",          0x0E8C, "int",  2),
    ("windSpeed",    0x0E90, "uint", 2),
    ("seaLevelPress",0x0EC6, "uint", 2),
    ("lightsBits",   0x0D0C, "uint", 2),
    # freezes / posicion
    ("llFreeze",     0x3540, "uint", 1),
    ("slewActive",   0x05DC, "uint", 2),

    # --- espejos: la MISMA SimVar expuesta en otro offset, o el indicador de
    # posicion real de la superficie. Solo el simulador los mueve, asi que si
    # el offset escrito cambia y su espejo no, la escritura se quedo en el
    # buffer de FSUIPC y nunca llego al sim.
    ("m_flapsLeft",   0x0BE0, "uint",  4),   # espejo de flapsPercent
    ("m_spoilerLeft", 0x0BD4, "uint",  4),   # espejo de spoilers
    ("m_gearCenter",  0x0BEC, "uint",  4),   # espejo de gearHandle
    ("m_oatDbl",      0x34A8, "float", 8),   # espejo de oat
    ("m_windDbl",     0x3488, "float", 8),   # espejo de windSpeed
    ("m_slpDbl",      0x34A0, "float", 8),   # espejo de seaLevelPress
    ("m_propDeice",   0x337C, "uint",  1),   # espejo de propDeice
    ("m_com1Hz",      0x05CC, "uint",  4),   # espejo de com1Standby (Hz, no BCD)
    ("m_com2Hz",      0x05D0, "uint",  4),   # espejo de com2Standby
]

# ===================== Tabla de comprobaciones =====================
# Cada entrada: campo de Shirley, offset (o None), dos valores candidatos para
# la escritura directa, plantilla de evento (o None) y sus dos parametros.
# Se prueban ambas vias por separado.

Check = Dict[str, Any]

CHECKS: List[Check] = [
    # --- levers ---
    dict(group="levers", field="flapsHandlePercentDown", offset="flapsPercent", mirror="m_flapsLeft",
         values=(10922, 5461), event="{v} (>K:FLAPS_SET)", params=(10922, 5461)),
    dict(group="levers", field="(flaps por indice)", offset="flapsIndex",
         values=(2, 1), event=None),
    dict(group="levers", field="speedBrakesHandlePercentDeployed", offset="spoilers", mirror="m_spoilerLeft",
         values=(16383, 0), event=None, note="el C172 no tiene spoilers"),
    dict(group="levers", field="landingGearHandlePercentDown", offset="gearHandle", mirror="m_gearCenter",
         values=(0, 16383), event="{v} (>K:GEAR_SET)", params=(0, 1),
         note="el C172 es de tren fijo"),
    dict(group="levers", field="carburetorHeatLeverPercentHot", offset="carbHeat",
         values=(1, 0), event="{v} (>K:ANTI_ICE_SET_ENG1)", params=(1, 0)),
    # --- autopilot ---
    dict(group="autopilot", field="isAutopilotEngaged", offset="apMaster",
         values=(1, 0), event="{v} (>K:AP_MASTER)", params=(1, 1),
         note="AP_MASTER alterna, no asigna"),
    dict(group="autopilot", field="isHeadingSelectEnabled", offset="apHdgLock",
         values=(1, 0), event="{v} (>K:AP_PANEL_HEADING_HOLD)", params=(1, 1)),
    dict(group="autopilot", field="magneticHeadingBugDeg", offset="apHdgBug",
         values=(int(270 * 65536 / 360), int(90 * 65536 / 360)),
         event="{v} (>K:2:HEADING_BUG_SET)", params=(270, 90)),
    dict(group="autopilot", field="altitudeBugFt", offset="apAltBug",
         values=(3048 * 65536, 1524 * 65536),
         event="{v} (>K:2:AP_ALT_VAR_SET_ENGLISH)", params=(5000, 3000)),
    dict(group="autopilot", field="altitudeMode=altitudeHold", offset="apAltLock",
         values=(1, 0), event="{v} (>K:AP_PANEL_ALTITUDE_HOLD)", params=(1, 1)),
    dict(group="autopilot", field="altitudeMode=verticalSpeed", offset="apVsHold",
         values=(1, 0), event="{v} (>K:AP_PANEL_VS_HOLD)", params=(1, 1)),
    dict(group="autopilot", field="targetVerticalSpeedUpFpm", offset="apVsTarget",
         values=(500, -500), event="{v} (>K:2:AP_VS_VAR_SET_ENGLISH)", params=(500, -500)),
    # --- radios ---
    dict(group="radiosNavigation", field="standbyFrequencyHz.com1", offset="com1Standby", mirror="m_com1Hz",
         values=(0x2185, 0x2100), event="{v} (>K:COM_STBY_RADIO_SET)", params=(0x2185, 0x2100)),
    dict(group="radiosNavigation", field="standbyFrequencyHz.com2", offset="com2Standby", mirror="m_com2Hz",
         values=(0x2185, 0x2100), event="{v} (>K:COM2_STBY_RADIO_SET)", params=(0x2185, 0x2100)),
    dict(group="radiosNavigation", field="standbyFrequencyHz.nav1", offset="nav1Standby",
         values=(0x1155, 0x1080), event="{v} (>K:NAV1_STBY_SET)", params=(0x1155, 0x1080)),
    dict(group="radiosNavigation", field="transponderCode", offset="transponder",
         values=(0x1200, 0x7000), event="{v} (>K:XPNDR_SET)", params=(0x1200, 0x7000)),
    # --- systems ---
    dict(group="systems", field="parkingBrakeOn", offset="parkingBrake",
         values=(32767, 0), event="{v} (>K:PARKING_BRAKE_SET)", params=(1, 0)),
    dict(group="systems", field="pitotHeatSwitchOn", offset="pitotHeat",
         values=(1, 0), event="{v} (>K:PITOT_HEAT_SET)", params=(1, 0)),
    dict(group="systems", field="propHeatSwitchOn", offset="propDeice", mirror="m_propDeice",
         values=(1, 0), event="{v} (>K:TOGGLE_STRUCTURAL_DEICE)", params=(1, 1),
         note="el C172 no tiene deshielo"),
    # --- indicators / environment ---
    dict(group="indicators", field="altimeterSettingInchesMercury", offset="kohlsman",
         values=(16212, 16000), event="{v} (>K:KOHLSMAN_SET)", params=(16212, 16000)),
    dict(group="environment", field="groundTemperatureDegC", offset="oat", mirror="m_oatDbl",
         values=(20 * 256, 5 * 256), event=None),
    dict(group="environment", field="(viento, lectura de clima)", offset="windSpeed", mirror="m_windDbl",
         values=(15, 5), event=None),
    dict(group="environment", field="seaLevelPressureInchesMercury", offset="seaLevelPress", mirror="m_slpDbl",
         values=(16212, 16000), event=None),
    # --- lights (bitmask: nav=bit0, landing=bit2, taxi=bit3, strobe=bit4) ---
    dict(group="lights", field="navigationLightsSwitchOn", offset=None,
         event="{v} (>K:NAV_LIGHTS_SET)", params=(1, 0), watch="lightsBits"),
    dict(group="lights", field="landingLightsSwitchOn", offset=None,
         event="{v} (>K:LANDING_LIGHTS_SET)", params=(1, 0), watch="lightsBits"),
    dict(group="lights", field="taxiLightsSwitchOn", offset=None,
         event="{v} (>K:TAXI_LIGHTS_SET)", params=(1, 0), watch="lightsBits"),
    dict(group="lights", field="strobeLightsSwitchOn", offset=None,
         event="{v} (>K:STROBES_SET)", params=(1, 0), watch="lightsBits"),
    # --- freezes ---
    dict(group="freezes", field="positionFreezeEnabled", offset=None,
         event="(>K:FREEZE_LATITUDE_LONGITUDE_TOGGLE)", params=(None,), watch="llFreeze"),
]

# La bateria se prueba al final: apagarla corta la avionica y contamina el resto.
LAST_CHECKS: List[Check] = [
    dict(group="systems", field="batteryOn.main", offset="battery",
         values=(0, 1), event="{v} (>K:TOGGLE_MASTER_BATTERY)", params=(1, 1),
         note="TOGGLE alterna, no asigna"),
]

POSITION_CHECKS: List[Check] = [
    dict(group="position", field="(slew mode)", offset="slewActive", values=(1, 0), event=None),
]

# ===================== Reporte =====================

PASS, FAIL, SKIP, INFO = "PASS", "FAIL", "SKIP", "INFO"
OK_MARK = {PASS: "  ok  ", FAIL: " FAIL ", SKIP: " skip ", INFO: " info "}


class Report:
    def __init__(self):
        self.rows: List[dict] = []

    def line(self, status: str, text: str, detail: str = ""):
        self.rows.append({"status": status, "text": text})
        print(f"[{OK_MARK[status]}] {text}")
        if detail:
            for l in detail.splitlines():
                print(f"          {l}")

    def matrix_row(self, check: Check, offset_res: str, event_res: str, detail: str):
        self.rows.append({"status": INFO, "text": check["field"],
                          "offset": offset_res, "event": event_res})
        print(f"  {check['field']:<38} offset:{offset_res:<12} evento:{event_res}")
        if detail:
            for l in detail.splitlines():
                print(f"      {l}")

    def summary(self) -> int:
        fails = sum(1 for r in self.rows if r["status"] == FAIL)
        print("\n" + "=" * 74)
        print(f"  filas evaluadas: {len(self.rows)} · fallidas: {fails}")
        print("=" * 74)
        return fails


# ===================== Cliente =====================

class FSUIPCClient:
    def __init__(self, ws):
        self.ws = ws
        self.state: Dict[str, Any] = {}
        self._pending: List[dict] = []
        self._pump_task: Optional[asyncio.Task] = None

    async def start(self):
        self._pump_task = asyncio.create_task(self._pump())

    async def close(self):
        if self._pump_task:
            self._pump_task.cancel()
            try:
                await self._pump_task
            except asyncio.CancelledError:
                pass

    async def _pump(self):
        try:
            async for raw in self.ws:
                if isinstance(raw, bytes):
                    raw = raw.decode("utf-8", "ignore")
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if (msg.get("command") == "offsets.read" and msg.get("success")
                        and isinstance(msg.get("data"), dict)):
                    self.state.update(msg["data"])   # payload parcial: solo cambios
                else:
                    self._pending.append(msg)
        except asyncio.CancelledError:
            raise
        except Exception:
            pass

    async def send(self, obj: dict):
        await self.ws.send(json.dumps(obj))

    async def response(self, command: str, name: Optional[str],
                       timeout: float = 5.0) -> Optional[dict]:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while loop.time() < deadline:
            for i, m in enumerate(self._pending):
                if m.get("command") == command and (name is None or m.get("name") == name):
                    return self._pending.pop(i)
            await asyncio.sleep(0.02)
        return None

    async def declare(self) -> Optional[dict]:
        await self.send({"command": "offsets.declare", "name": GROUP,
                         "offsets": [{"name": n, "address": a, "type": t, "size": s}
                                     for n, a, t, s in OFFSETS]})
        return await self.response("offsets.declare", GROUP)

    async def subscribe(self, timeout: float = 8.0) -> bool:
        await self.send({"command": "offsets.read", "name": GROUP,
                         "interval": READ_INTERVAL_MS})
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while loop.time() < deadline:
            if len(self.state) >= len(OFFSETS) - 1:
                return True
            await asyncio.sleep(0.05)
        return bool(self.state)

    async def unsubscribe(self):
        try:
            await self.send({"command": "offsets.stop", "name": GROUP})
        except Exception:
            pass

    async def write(self, field: str, value: Any) -> dict:
        await self.send({"command": "offsets.write", "name": GROUP,
                         "offsets": [{"name": field, "value": value}]})
        err = await self.response("offsets.write", GROUP, timeout=0.8)
        if err is not None and not err.get("success"):
            return {"ok": False, "error": f"{err.get('errorCode')}: {err.get('errorMessage')}"}
        return {"ok": True}

    async def calc(self, code: str, tag: str = "calc") -> dict:
        await self.send({"command": "vars.calc", "name": tag, "code": code, "interval": 0})
        msg = await self.response("vars.calc", tag, timeout=5.0)
        if msg is None:
            return {"ok": False, "error": "sin respuesta"}
        if not msg.get("success"):
            return {"ok": False, "error": f"{msg.get('errorCode')}: {msg.get('errorMessage')}"}
        return {"ok": True}


# ===================== Motor de prueba =====================

async def observe(cli: FSUIPCClient, watch: str, action: Callable,
                  expect: Optional[int] = None) -> Dict[str, Any]:
    before = cli.state.get(watch)
    res = await action()
    if not res.get("ok"):
        return {"ok": False, "error": res.get("error"), "before": before}
    loop = asyncio.get_running_loop()
    deadline = loop.time() + CHANGE_TIMEOUT_S
    while loop.time() < deadline:
        now = cli.state.get(watch)
        if now != before and (expect is None or now == expect):
            break
        await asyncio.sleep(0.05)
    after = cli.state.get(watch)
    return {"ok": True, "before": before, "after": after,
            "changed": after != before, "hit": expect is not None and after == expect}


def verdict(res: dict, expect_exact: bool) -> Tuple[str, str]:
    """Traduce el resultado a una etiqueta corta y un detalle."""
    if not res.get("ok"):
        return "error", res.get("error", "")
    if expect_exact and res.get("hit"):
        return "SI", ""
    if res.get("changed"):
        return "parcial", (f"escrito/enviado esperando {fmt(res.get('expect'))}, "
                           f"quedo en {fmt(res['after'])} (venia de {fmt(res['before'])})")
    return "no", f"sin cambio, sigue en {fmt(res['before'])}"


def fmt(v):
    if isinstance(v, int):
        return f"{v} (0x{v:X})"
    return repr(v)


def distinct(baseline, a, b):
    return b if baseline == a else a


async def run_check(cli: FSUIPCClient, rep: Report, chk: Check):
    watch = chk.get("watch") or chk.get("offset")
    detail_parts: List[str] = []

    # --- via 1: escritura directa de offset ---
    off_verdict = "-"
    if chk.get("offset"):
        val = distinct(cli.state.get(chk["offset"]), *chk["values"])
        mirror = chk.get("mirror")
        mirror_before = cli.state.get(mirror) if mirror else None

        res = await observe(cli, chk["offset"],
                            lambda f=chk["offset"], v=val: cli.write(f, v), expect=val)
        res["expect"] = val
        off_verdict, d = verdict(res, expect_exact=True)
        if d:
            detail_parts.append(f"offset: {d}")

        # Guardia contra eco: que el offset devuelva lo escrito no prueba que el
        # simulador lo haya aplicado. FSUIPC mantiene su propio buffer y lo
        # refleja aunque SimConnect ignore la escritura. Solo el espejo -- otra
        # exposicion de la misma SimVar, o el indicador de la superficie real --
        # distingue una escritura efectiva de un eco.
        if off_verdict == "SI":
            if not mirror:
                off_verdict = "SI?"
                detail_parts.append("sin espejo: no se pudo descartar que sea eco")
            else:
                loop = asyncio.get_running_loop()
                deadline = loop.time() + 3.5
                moved = False
                while loop.time() < deadline:
                    if cli.state.get(mirror) != mirror_before:
                        moved = True
                        break
                    await asyncio.sleep(0.05)
                if moved:
                    detail_parts.append(
                        f"espejo {mirror}: {fmt(mirror_before)} -> "
                        f"{fmt(cli.state.get(mirror))} · escritura efectiva")
                else:
                    off_verdict = "eco"
                    detail_parts.append(
                        f"espejo {mirror} sigue en {fmt(mirror_before)}: "
                        f"la escritura no llego al simulador")

    # --- via 2: evento de control ---
    evt_verdict = "-"
    if chk.get("event"):
        got = None
        for p in chk["params"]:
            code = chk["event"] if p is None else chk["event"].format(v=p)
            res = await observe(cli, watch, lambda c=code: cli.calc(c))
            if not res.get("ok"):
                got = ("error", res.get("error", ""))
                break
            if res.get("changed"):
                got = ("SI", "")
                break
            got = ("no", f"sin cambio con parametros {chk['params']}")
        evt_verdict, d = got
        if d:
            detail_parts.append(f"evento: {d}")

    if chk.get("note"):
        detail_parts.append(f"nota: {chk['note']}")

    rep.matrix_row(chk, off_verdict, evt_verdict, "\n".join(detail_parts))


# ===================== Main =====================

async def main(do_write: bool, do_position: bool) -> int:
    rep = Report()
    print("=" * 74)
    print("  Verificacion FSUIPC7 + MSFS 2024 · barrido completo")
    print(f"  {FSUIPC_WS_URL}")
    print(f"  modo: {'lectura + ESCRITURA' if do_write else 'solo lectura'}")
    print("=" * 74)

    try:
        ws = await websockets.connect(FSUIPC_WS_URL, subprotocols=["fsuipc"],
                                      open_timeout=5, ping_interval=None, max_size=None)
    except Exception as e:
        print(f"\nNo se pudo conectar: {e!r}")
        print("Verificar que C:\\FSUIPC7\\Utils\\FSUIPCWebSocketServer.exe este corriendo.")
        return 1

    async with ws:
        cli = FSUIPCClient(ws)
        await cli.start()
        try:
            decl = await cli.declare()
            if not decl or not decl.get("success"):
                print(f"\nNo se pudieron declarar los offsets: "
                      f"{decl.get('errorMessage') if decl else 'sin respuesta'}")
                return 1
            if not await cli.subscribe():
                print("\nLa suscripcion no entrego datos: probablemente no hay vuelo cargado.")
                return 1

            rep.line(PASS, f"conexion y suscripcion · {len(cli.state)} offsets")
            rep.line(INFO, f"avion: {cli.state.get('aircraftName', '?')}")
            incr = cli.state.get("flapsIncr")
            if isinstance(incr, int) and incr:
                rep.line(INFO, f"0x3BFA = {incr} · incremento por detente "
                               f"-> {16383 // incr + 1} posiciones de flaps")

            if not do_write:
                print("\nValores actuales:")
                for k, _, _, _ in OFFSETS:
                    print(f"          {k:<15} {fmt(cli.state.get(k))}")
                print("\nVolver a ejecutar con --write para el barrido.")
                return rep.summary()

            baseline = dict(cli.state)

            checks = list(CHECKS)
            if do_position:
                checks += POSITION_CHECKS
            checks += LAST_CHECKS

            current = None
            for chk in checks:
                if chk["group"] != current:
                    current = chk["group"]
                    print(f"\n--- {current} " + "-" * (66 - len(current)))
                await run_check(cli, rep, chk)

            # --- restauracion ---
            print("\n--- restaurando estado inicial " + "-" * 42)
            restored = 0
            for name, _, _, _ in OFFSETS:
                if name in ("aircraftName", "flapsIncr", "llFreeze"):
                    continue
                if cli.state.get(name) != baseline.get(name) and baseline.get(name) is not None:
                    await cli.write(name, baseline[name])
                    restored += 1
            await asyncio.sleep(0.8)
            rep.line(INFO, f"se reescribieron {restored} offsets a su valor inicial",
                     "los que no aceptan escritura directa pueden haber quedado cambiados")

            return rep.summary()
        finally:
            await cli.unsubscribe()
            await cli.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Verifica las capacidades de setpoint contra MSFS 2024.")
    ap.add_argument("--write", action="store_true",
                    help="ejecuta el barrido de escritura (mueve superficies, radios y luces)")
    ap.add_argument("--position", action="store_true",
                    help="incluye las pruebas de posicion/slew (teletransporta el avion)")
    args = ap.parse_args()

    if args.write:
        print("\nADVERTENCIA: se van a mover superficies, radios, luces y el piloto automatico.")
        print("Usar con el avion detenido en tierra. Ctrl+C para cancelar.\n")

    try:
        sys.exit(1 if asyncio.run(main(args.write, args.position)) else 0)
    except KeyboardInterrupt:
        print("\nCancelado.")
        sys.exit(130)
