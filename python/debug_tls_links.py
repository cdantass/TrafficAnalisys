import os
import sys

# IMPORTANTE:
# Troque "CC..." pelo nome exato da pasta que começa com CC.
# SUMO_HOME deve apontar para a pasta que contém diretamente:
# bin/, tools/, docs/ e data/.
SUMO_HOME = r"C:\Users\cdbarbosa\Downloads\sumo-win64-1.27.1\sumo-1.27.1"

TOOLS_DIR = os.path.join(SUMO_HOME, "tools")
BIN_DIR = os.path.join(SUMO_HOME, "bin")

if not os.path.isdir(TOOLS_DIR):
    raise FileNotFoundError(
        f"\nPasta tools não encontrada:\n{TOOLS_DIR}\n\n"
        "Corrija SUMO_HOME. Ele deve apontar para a pasta raiz do SUMO, "
        "aquela que contém as pastas bin, tools, docs e data."
    )

if not os.path.isfile(os.path.join(BIN_DIR, "sumo-gui.exe")):
    raise FileNotFoundError(
        f"\nExecutável sumo-gui.exe não encontrado:\n"
        f"{os.path.join(BIN_DIR, 'sumo-gui.exe')}\n\n"
        "Corrija SUMO_HOME."
    )

# Coloca os módulos Python do SUMO (traci e sumolib) antes dos demais paths.
sys.path.insert(0, TOOLS_DIR)

import traci


def main():
    project_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..")
    )

    sumocfg = os.path.join(
        project_root,
        "sumo",
        "config",
        "simulation.sumocfg",
    )

    sumo_gui = os.path.join(BIN_DIR, "sumo-gui.exe")

    if not os.path.isfile(sumocfg):
        raise FileNotFoundError(
            f"\nArquivo simulation.sumocfg não encontrado:\n{sumocfg}"
        )

    print(f"SUMO_HOME: {SUMO_HOME}")
    print(f"SUMO GUI:  {sumo_gui}")
    print(f"SUMOCFG:   {sumocfg}\n")

    traci.start([
        sumo_gui,
        "-c",
        sumocfg,
        "--start",
    ])

    try:
        tls_id = "C"
        links = traci.trafficlight.getControlledLinks(tls_id)

        print(f"TLS: {tls_id}")
        print(f"Total de links controlados: {len(links)}\n")

        for idx, link_list in enumerate(links):
            print(f"INDEX {idx}:")

            if not link_list:
                print("  (sem links neste índice)")
                continue

            for link in link_list:
                from_lane = link[0]
                to_lane = link[1]
                via_lane = link[2]

                from_edge = traci.lane.getEdgeID(from_lane)
                to_edge = traci.lane.getEdgeID(to_lane)
                via_edge = (
                    traci.lane.getEdgeID(via_lane)
                    if via_lane
                    else "(sem via)"
                )

                print(
                    f"  {from_edge} -> {to_edge} via {via_edge} | "
                    f"from_lane={from_lane} | "
                    f"to_lane={to_lane} | "
                    f"via={via_lane or '(vazio)'}"
                )
    finally:
        traci.close()


if __name__ == "__main__":
    main()