"""
Verificación en vivo del puente contra MSFS 2024.

Se conecta al endpoint que el puente le ofrece a Shirley
(ws://localhost:2992/api/v1), o sea que ejercita el puente entero: los offsets
declarados, las transformaciones, el armado del snapshot y el write path.

Dos fases:

  Lectura   Junta snapshots durante unos segundos y muestra cada campo con su
            valor y su tipo, marcando los que faltan o salen fuera de rango.
            Es la fase que atrapa un offset que apunta al SimVar equivocado.

  Escritura (--write) Manda un SetSimData y espera a que el campo cambie en el
            snapshot. El snapshot se alimenta de lecturas de FSUIPC, así que
            sirve de espejo: si el valor se mueve, el simulador aplicó el
            comando de verdad.

            La trampa conocida es que FSUIPC devuelve en el eco lo que uno
            escribió aunque SimConnect lo haya ignorado. Acá casi todo va por
            evento de control, que no toca el buffer de offsets y por lo tanto
            no puede producir ese eco. El único caso que escribe un offset son
            los flaps, y ahí se escribe 0x0BFC y se observa 0x0BDC — otro
            offset, así que tampoco hay eco posible.

Uso:
    python fsuipc_shirley_bridge.py          # en otra consola, con MSFS 2024 abierto
    python tools/verify_bridge_live.py
    python tools/verify_bridge_live.py --write
"""

import argparse
import asyncio
import json
import sys
from typing import Any, Dict, List, Optional, Tuple

import websockets

BRIDGE_URL = "ws://localhost:2992/api/v1"
FSUIPC_URL = "ws://localhost:2048/fsuipc/"
COLLECT_S = 4.0
CHANGE_TIMEOUT_S = 8.0

# Espejos: offsets que respaldan el mismo sistema que el campo escrito, pero
# que el puente no escribe nunca. Hacen falta cuando el campo se lee del mismo
# offset donde se escribe, porque ahí la lectura devuelve lo que uno escribió
# aunque el simulador lo haya ignorado, y el resultado parece un éxito.
#
# El deshielo de hélice es el caso: 0x2440 se escribe y se lee. La primera
# corrida de este arnés lo dio por bueno sobre un C172, que no tiene deshielo
# de hélice — leyó su propia escritura.
MIRRORS = [
    ("propDeiceMirror", 0x337C, "uint", 4),   # el que se mueve cuando el deshielo entra de verdad
    ("gearPosMirror",   0x0BEC, "uint", 4),   # recorrido real del tren, no la palanca
]

OK, WARN, BAD = "  ok  ", " ojo  ", " mal  "


# ===================== expectativas de lectura =====================
# (ruta, mínimo, máximo, obligatorio). Los rangos son de plausibilidad, no de
# validez: buscan el campo que quedó apuntando a otro SimVar, no el que está
# unos decimales corrido.
READ_CHECKS: List[Tuple[str, Optional[float], Optional[float], bool]] = [
    ("position.latitudeDeg",                     -90, 90, True),
    ("position.longitudeDeg",                   -180, 180, True),
    ("position.mslAltitudeFt",                 -1500, 60000, True),
    ("position.aglAltitudeFt",                     0, 60000, False),
    ("position.indicatedAirspeedKts",              0, 600, True),
    ("position.gpsGroundSpeedKts",                 0, 700, True),
    ("position.verticalSpeedUpFpm",           -10000, 10000, False),
    ("attitude.trueHeadingDeg",                    0, 360, True),
    ("attitude.magneticHeadingDeg",                0, 360, False),
    ("attitude.pitchAngleDegUp",                 -90, 90, True),
    ("attitude.rollAngleDegRight",              -180, 180, True),
    ("indicators.altimeterSettingInchesMercury",  27, 32, True),
    ("indicators.engineN1Percent.engine1",         0, 110, False),
    ("indicators.manifoldPressureInchesMercury.engine1", 0, 60, False),
    ("indicators.exhaustGasDegC.engine1",          0, 1200, False),
    ("indicators.stallWarningOn",               None, None, False),
    ("levers.throttlePercentOpen.engine1",      -100, 100, False),
    ("levers.mixtureLeverPercentRich.engine1",     0, 100, False),
    ("levers.propellerLeverPercentCoarse.prop1",-100, 100, False),
    ("levers.flapsHandlePercentDown",              0, 100, True),
    ("levers.landingGearHandlePercentDown",        0, 100, True),
    ("levers.speedBrakesHandlePercentDeployed",    0, 100, False),
    ("levers.carburetorHeatLeverPercentHot.engine1", 0, 100, False),
    ("autopilot.magneticHeadingBugDeg",            0, 360, True),
    ("autopilot.altitudeBugFt",                    0, 60000, True),
    ("autopilot.targetVerticalSpeedUpFpm",     -10000, 10000, False),
    ("autopilot.isAutopilotEngaged",            None, None, True),
    ("autopilot.isFlightDirectorEngaged",       None, None, False),
    ("autopilot.altitudeMode",                  None, None, False),
    ("systems.batteryOn.main",                  None, None, True),
    ("systems.parkingBrakeOn",                  None, None, True),
    ("systems.pitotHeatSwitchOn",               None, None, False),
    ("systems.propHeatSwitchOn",                None, None, False),
    ("lights.navigationLightsSwitchOn",         None, None, True),
    ("lights.landingLightsSwitchOn",            None, None, True),
    ("radiosNavigation.frequencyHz.com1",     108000, 136975, True),
    ("radiosNavigation.standbyFrequencyHz.com1", 108000, 136975, True),
    ("radiosNavigation.frequencyHz.nav1",     108000, 118000, False),
    ("radiosNavigation.transponderCode",           0, 7777, True),
    ("environment.groundTemperatureDegC",        -60, 60, False),
    ("environment.aircraftWindSpeedKts",           0, 200, False),
    ("simulation.aircraftName",                 None, None, True),
]

# Los tipos que exige el schema de Shirley. Un campo con el tipo equivocado
# invalida el grupo entero, porque cada grupo es .strict().
EXPECTED_TYPES = {
    "bool": ["isAutopilotEngaged", "isFlightDirectorEngaged", "stallWarningOn",
             "parkingBrakeOn", "pitotHeatSwitchOn", "propHeatSwitchOn", "main",
             "navigationLightsSwitchOn", "landingLightsSwitchOn",
             "taxiLightsSwitchOn", "strobeLightsSwitchOn", "shouldLevelWings"],
    "str": ["aircraftName", "altitudeMode"],
}


# ===================== pruebas de escritura =====================
# (etiqueta, ruta que se fija, ruta que se observa, objetivo, tolerancia, equipo)
#
# El objetivo puede ser:
#   un número      se manda tal cual y se espera ese valor
#   "flip"         booleano: se manda lo contrario de lo que está ahora, así la
#                  prueba nunca pasa por no haber pedido ningún cambio
#   "distinto"     numérico: el arnés elige un valor distinto del actual
#
# 'equipo' marca los mandos que un avión puede sencillamente no tener. Un
# comando sin efecto ahí no prueba que el puente falle: sobre un C172 los
# aerofrenos y el deshielo de hélice no existen, y esa confusión ya costó tres
# conclusiones equivocadas en la matriz de capacidades.
WRITE_CHECKS = [
    ("selector de rumbo",        "autopilot.magneticHeadingBugDeg",
     "autopilot.magneticHeadingBugDeg", (270, 90), 2, None),

    ("selector de altitud",      "autopilot.altitudeBugFt",
     "autopilot.altitudeBugFt", (5000, 3000), 70, None),

    ("velocidad vertical",       "autopilot.targetVerticalSpeedUpFpm",
     "autopilot.targetVerticalSpeedUpFpm", (500, -500), 10, None),

    ("COM1 en espera",           "radiosNavigation.standbyFrequencyHz.com1",
     "radiosNavigation.standbyFrequencyHz.com1", (121850, 124850), 30, None),

    ("transponder",              "radiosNavigation.transponderCode",
     "radiosNavigation.transponderCode", (4321, 1200), 0, None),

    ("altímetro",                "indicators.altimeterSettingInchesMercury",
     "indicators.altimeterSettingInchesMercury", (29.92, 30.06), 0.06, None),

    ("luces de navegación",      "lights.navigationLightsSwitchOn",
     "lights.navigationLightsSwitchOn", "flip", 0, None),

    ("luces de taxi",            "lights.taxiLightsSwitchOn",
     "lights.taxiLightsSwitchOn", "flip", 0, None),

    ("luces de aterrizaje",      "lights.landingLightsSwitchOn",
     "lights.landingLightsSwitchOn", "flip", 0, None),

    ("freno de estacionamiento", "systems.parkingBrakeOn",
     "systems.parkingBrakeOn", "flip", 0, None),

    ("calefacción de pitot",     "systems.pitotHeatSwitchOn",
     "systems.pitotHeatSwitchOn", "flip", 0, None),

    # El director de vuelo va antes que el piloto automático: engancharlo
    # enciende el director por diseño y el avión no deja apagarlo mientras el
    # piloto automático esté puesto.
    ("director de vuelo",        "autopilot.isFlightDirectorEngaged",
     "autopilot.isFlightDirectorEngaged", "flip", 0, None),

    ("piloto automático",        "autopilot.isAutopilotEngaged",
     "autopilot.isAutopilotEngaged", "flip", 0, None),

    ("aire caliente carburador", "levers.carburetorHeatLeverPercentHot.engine1",
     "levers.carburetorHeatLeverPercentHot.engine1", "distinto", 0,
     "sólo motores de pistón con carburador"),

    # Escribe 0x0BFC (índice de detente) y observa 0x0BDC (porcentaje): son dos
    # offsets distintos, así que un eco de FSUIPC no puede simular éxito.
    ("flaps por detente",        "levers.flapsHandlePercentDown",
     "levers.flapsHandlePercentDown", "distinto", 2, None),

    ("aerofrenos",               "levers.speedBrakesHandlePercentDeployed",
     "levers.speedBrakesHandlePercentDeployed", "distinto", 2,
     "el avión tiene que tener aerofrenos"),

    # Se observa el espejo 0x337C y no el propio 0x2440, que devolvería la
    # escritura tal cual y haría pasar la prueba en cualquier avión.
    ("deshielo de hélice",       "systems.propHeatSwitchOn",
     "mirror:propDeiceMirror", "flip", 0,
     "el avión tiene que tener deshielo de hélice"),

    ("tren de aterrizaje",       "levers.landingGearHandlePercentDown",
     "mirror:gearPosMirror", "distinto", 0,
     "en tierra MSFS 2024 se niega a replegar con peso sobre ruedas"),

    # La batería va última a propósito. Cortarla deja sin barra a todo lo
    # eléctrico, y una prueba anterior que la dejó abierta hizo figurar como
    # fallados el piloto automático y el director de vuelo, que funcionaban
    # perfectamente. Un veredicto sólo vale si la aeronave estaba en condiciones
    # de obedecer.
    ("batería principal",        "systems.batteryOn.main",
     "systems.batteryOn.main", "flip", 0, None),
]

# Se rechazan a propósito: el puente tiene que contestar por qué, no aceptarlos
# en silencio.
REFUSAL_CHECKS = [
    ("temperatura ambiente", {"environment": {"groundTemperatureDegC": 20}}),
    ("presión a nivel del mar", {"environment": {"seaLevelPressureInchesMercury": 29.5}}),
    ("reset del vuelo", {"simulation": {"shouldResetFlight": True}}),
]


# ===================== utilidades =====================

def dig(state: dict, path: str) -> Any:
    node = state
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node:
            return None
        node = node[part]
    return node


def merge(dst: dict, src: dict) -> dict:
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            merge(dst[k], v)
        else:
            dst[k] = v
    return dst


def fmt(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:.2f}"
    return repr(v)


def type_problem(path: str, value: Any) -> Optional[str]:
    leaf = path.split(".")[-1]
    if leaf in EXPECTED_TYPES["bool"] and not isinstance(value, bool):
        return f"debería ser bool, es {type(value).__name__}"
    if leaf in EXPECTED_TYPES["str"] and not isinstance(value, str):
        return f"debería ser str, es {type(value).__name__}"
    if leaf not in EXPECTED_TYPES["bool"] and leaf not in EXPECTED_TYPES["str"]:
        if isinstance(value, bool):
            return "es bool y debería ser número"
    return None


class MirrorReader:
    """Lee offsets espejo por una conexión propia a FSUIPC.

    Deliberadamente aparte del puente: el puente no declara estos offsets ni
    los escribe nunca, que es justamente lo que los hace servir de testigo.
    """

    def __init__(self):
        self.ws = None
        self.state: Dict[str, Any] = {}
        self._task = None

    async def start(self) -> bool:
        try:
            self.ws = await asyncio.wait_for(
                websockets.connect(FSUIPC_URL, subprotocols=["fsuipc"], max_size=None), timeout=5)
        except Exception as e:
            print(f"{WARN} sin espejos: no se pudo conectar a FSUIPC ({e!r})")
            return False
        await self.ws.send(json.dumps({
            "command": "offsets.declare", "name": "mirrors",
            "offsets": [{"name": n, "address": a, "type": t, "size": s} for n, a, t, s in MIRRORS]}))
        await self.ws.send(json.dumps({
            "command": "offsets.read", "name": "mirrors", "interval": 200}))
        self._task = asyncio.create_task(self._pump())
        await asyncio.sleep(1.0)
        return True

    async def _pump(self):
        try:
            async for raw in self.ws:
                msg = json.loads(raw)
                if msg.get("command") == "offsets.read" and isinstance(msg.get("data"), dict):
                    self.state.update(msg["data"])
        except Exception:
            pass

    async def stop(self):
        if self._task:
            self._task.cancel()
        if self.ws:
            try:
                await self.ws.close()
            except Exception:
                pass


class Bridge:
    """Cliente del endpoint que el puente le ofrece a Shirley."""

    def __init__(self, ws):
        self.ws = ws
        self.state: Dict[str, Any] = {}
        self.acks: List[dict] = []
        self.capabilities: Optional[dict] = None
        self._task = None

    async def start(self):
        self._task = asyncio.create_task(self._pump())

    async def stop(self):
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    async def _pump(self):
        async for raw in self.ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if not isinstance(msg, dict):
                continue
            kind = msg.get("type")
            if kind == "Capabilities":
                self.capabilities = msg
            elif kind == "SetSimDataAck":
                self.acks.extend(msg.get("results", []))
            elif kind is None:
                merge(self.state, msg)

    async def send(self, body: dict) -> List[dict]:
        before = len(self.acks)
        await self.ws.send(json.dumps(body))
        loop = asyncio.get_running_loop()
        deadline = loop.time() + 5.0
        while loop.time() < deadline:
            if len(self.acks) > before:
                await asyncio.sleep(0.2)      # dejá llegar el resto del ack
                return self.acks[before:]
            await asyncio.sleep(0.05)
        return []

    async def wait_change(self, path: str, baseline: Any,
                          expect: Any, tol: float) -> Tuple[bool, Any]:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + CHANGE_TIMEOUT_S
        while loop.time() < deadline:
            now = dig(self.state, path)
            if expect is None:
                if now != baseline:
                    return True, now
            elif isinstance(expect, bool):
                if now == expect:
                    return True, now
            elif now is not None and abs(float(now) - float(expect)) <= tol:
                return True, now
            await asyncio.sleep(0.1)
        return False, dig(self.state, path)


# ===================== fases =====================

async def phase_read(bridge: Bridge) -> int:
    print(f"\nJuntando snapshots durante {COLLECT_S:.0f} s...\n")
    await asyncio.sleep(COLLECT_S)

    if not bridge.state:
        print(f"{BAD} El puente no publicó ningún dato.")
        print("      Revisá que MSFS 2024 esté con un vuelo cargado y que FSUIPC7 esté conectado.")
        return 1

    if bridge.capabilities:
        print(f"Capabilities: {len(bridge.capabilities.get('reads', []))} lecturas, "
              f"{len(bridge.capabilities.get('writes', []))} escrituras\n")

    print(f"{'campo':<52} {'valor':>16}   estado")
    print("-" * 92)

    problems = 0
    group_shown = set()
    for path, lo, hi, required in READ_CHECKS:
        group = path.split(".")[0]
        if group not in group_shown:
            group_shown.add(group)
            print()
        value = dig(bridge.state, path)

        if value is None:
            status = f"{BAD} falta" if required else f"{WARN} ausente"
            if required:
                problems += 1
            print(f"{path:<52} {'—':>16}   {status}")
            continue

        note = type_problem(path, value)
        if note is None and lo is not None and isinstance(value, (int, float)) and not isinstance(value, bool):
            if not (lo <= value <= hi):
                note = f"fuera de rango [{lo}, {hi}]"

        if note:
            problems += 1
            print(f"{path:<52} {fmt(value):>16}   {BAD} {note}")
        else:
            print(f"{path:<52} {fmt(value):>16}   {OK}")

    groups = ", ".join(sorted(bridge.state.keys()))
    print(f"\nGrupos publicados: {groups}")
    return problems


def build_body(path: str, value: Any) -> dict:
    """Arma el objeto anidado de SetSimData a partir de una ruta con puntos."""
    body: dict = {}
    node = body
    parts = path.split(".")
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = value
    return body


def choose_target(goal: Any, current: Any, tol: float) -> Any:
    """Elige un valor que de verdad exija un cambio.

    Con un par de valores se alterna al que no sea el actual, así dos corridas
    seguidas prueban lo mismo en vez de saltearse todo porque la corrida
    anterior ya dejó el avión en el valor de destino.
    """
    if goal == "flip":
        return not bool(current)
    if goal == "distinto":
        if isinstance(current, bool) or current is None:
            return 100.0
        return 0.0 if float(current) > 50.0 else 100.0
    if isinstance(goal, tuple):
        first, second = goal
        if current is None:
            return first
        return second if abs(float(current) - float(first)) <= tol else first
    return goal


async def phase_write(bridge: Bridge, mirrors: Optional[MirrorReader]) -> int:
    print("\n" + "=" * 92)
    print("ESCRITURA — se manda el comando y se espera a que el sistema se mueva de verdad")
    print("=" * 92 + "\n")

    failures = 0
    original: Dict[str, Any] = {}
    airborne = (dig(bridge.state, "position.aglAltitudeFt") or 0) > 50

    for label, set_path, watch, goal, tol, equipment in WRITE_CHECKS:
        # Cortar la batería en vuelo no es una prueba, es una emergencia.
        if airborne and "battery" in set_path:
            print(f"{WARN} {label:<26} se omite: la aeronave está en vuelo")
            continue

        via_mirror = watch.startswith("mirror:")
        if via_mirror:
            if mirrors is None:
                print(f"{WARN} {label:<26} se omite: hace falta el espejo y no hay conexión")
                continue
            mirror_name = watch.split(":", 1)[1]
            baseline = mirrors.state.get(mirror_name)
        else:
            baseline = dig(bridge.state, watch)

        # El valor a restaurar sale del propio campo, no del testigo: el espejo
        # está en otra escala y devolverlo como si fuera el campo escribiría
        # cualquier cosa.
        original.setdefault(set_path, dig(bridge.state, set_path))
        target = choose_target(goal, baseline, tol)

        if not via_mirror and baseline is not None and not isinstance(target, bool) and \
                abs(float(baseline) - float(target)) <= tol:
            print(f"{WARN} {label:<26} ya estaba en {fmt(baseline)}, se saltea")
            continue

        acks = await bridge.send(build_body(set_path, target))
        if not acks:
            print(f"{BAD} {label:<26} sin ack del puente")
            failures += 1
            continue

        ack = acks[0]
        if not ack.get("ok"):
            print(f"{BAD} {label:<26} rechazado: {ack.get('error')}")
            failures += 1
            continue

        if via_mirror:
            # Contra un espejo sólo se pide que se mueva: está en otra escala y
            # a veces con otro recorrido, así que exigir un valor exacto sería
            # inventar una expectativa.
            changed, now = await wait_mirror_change(mirrors, mirror_name, baseline)
        else:
            changed, now = await bridge.wait_change(watch, baseline, target, tol)

        if changed:
            via = " (espejo)" if via_mirror else ""
            print(f"{OK} {label:<26} {fmt(baseline)} -> {fmt(now)}{via}")
        elif equipment:
            # Sin efecto, pero puede ser que la aeronave no tenga el sistema. No
            # es lo mismo que una falla del puente y no se cuenta como tal.
            print(f"{WARN} {label:<26} sin efecto — {equipment}")
        else:
            failures += 1
            print(f"{BAD} {label:<26} el puente aceptó el comando pero {watch} "
                  f"sigue en {fmt(baseline)}")

    await restore(bridge, original)
    return failures


async def wait_mirror_change(mirrors: MirrorReader, name: str,
                             baseline: Any) -> Tuple[bool, Any]:
    """Espera a que se mueva un offset que el puente nunca escribe.

    Un espejo quieto no distingue entre 'FSUIPC devolvió el eco' y 'el avión no
    tiene ese sistema'; eso lo resuelve la columna de equipo, no esta función.
    """
    loop = asyncio.get_running_loop()
    deadline = loop.time() + CHANGE_TIMEOUT_S
    while loop.time() < deadline:
        now = mirrors.state.get(name)
        if now != baseline:
            return True, now
        await asyncio.sleep(0.1)
    return False, mirrors.state.get(name)


async def restore(bridge: Bridge, original: Dict[str, Any]) -> None:
    """Devuelve la aeronave a como estaba.

    La prueba mueve interruptores de un vuelo que alguien está usando. Dejar la
    batería cortada y el freno suelto porque el arnés terminó su lista no es
    aceptable — y además arruina cualquier corrida siguiente, que fue justo lo
    que hizo parecer fallados el piloto automático y el director de vuelo.
    """
    print("\nRestaurando el estado original de la aeronave...")
    # La batería primero: sin barra alimentada no entra ningún otro comando.
    order = sorted(original, key=lambda p: 0 if "battery" in p else 1)
    for path in order:
        value = original[path]
        if value is None:
            continue
        await bridge.send(build_body(path, value))
        await asyncio.sleep(0.3)      # que la lectura alcance a reflejarlo
    await asyncio.sleep(2.0)

    pending = [p for p, v in original.items()
               if v is not None and dig(bridge.state, p) != v
               and not isinstance(v, float)]
    if pending:
        print(f"{WARN} quedó sin restaurar: {', '.join(pending)}")
    else:
        print(f"{OK} estado restaurado")


async def phase_refusals(bridge: Bridge) -> int:
    print("\n" + "=" * 92)
    print("RECHAZOS — lo que MSFS 2024 no puede fijar tiene que contestar el motivo")
    print("=" * 92 + "\n")

    failures = 0
    for label, body in REFUSAL_CHECKS:
        acks = await bridge.send(body)
        if not acks:
            print(f"{BAD} {label:<28} sin ack")
            failures += 1
            continue
        ack = acks[0]
        if ack.get("ok"):
            print(f"{BAD} {label:<28} lo aceptó, y no debería")
            failures += 1
        else:
            print(f"{OK} {label:<28} {ack.get('error', '')[:56]}")
    return failures


async def main(do_write: bool) -> int:
    print("=" * 92)
    print("Verificación en vivo del puente FSUIPC -> Shirley, contra MSFS 2024")
    print("=" * 92)

    try:
        ws = await asyncio.wait_for(websockets.connect(BRIDGE_URL, max_size=None), timeout=5)
    except Exception as e:
        print(f"\n{BAD} No se pudo conectar a {BRIDGE_URL}: {e!r}")
        print("      ¿Está corriendo 'python fsuipc_shirley_bridge.py'?")
        return 2

    async with ws:
        bridge = Bridge(ws)
        await bridge.start()
        mirrors = None
        try:
            problems = await phase_read(bridge)
            if do_write:
                mirrors = MirrorReader()
                if not await mirrors.start():
                    mirrors = None
                problems += await phase_write(bridge, mirrors)
                problems += await phase_refusals(bridge)
        finally:
            if mirrors is not None:
                await mirrors.stop()
            await bridge.stop()

    print("\n" + "=" * 92)
    if problems == 0:
        print("Sin observaciones.")
    else:
        print(f"{problems} punto(s) a revisar.")
    print("=" * 92)
    return 0 if problems == 0 else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--write", action="store_true",
                    help="además de leer, manda comandos SetSimData y verifica que se apliquen")
    args = ap.parse_args()
    try:
        sys.exit(asyncio.run(main(args.write)))
    except KeyboardInterrupt:
        sys.exit(130)
