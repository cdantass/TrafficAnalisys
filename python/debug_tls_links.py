import os
import sys

# Ajuste este caminho para o SUMO na sua máquina
SUMO_HOME = r"D:\RESTO\sumo-1.27.1"
tools = os.path.join(SUMO_HOME, "tools")
sys.path.append(tools)

import traci
from sumolib import checkBinary


def main():
    sumocfg = os.path.join(
        os.path.dirname(__file__),
        "..",
        "sumo",
        "config",
        "simulation.sumocfg",
    )

    sumo_binary = checkBinary("sumo-gui")  # ou "sumo" se não quiser GUI

    traci.start([sumo_binary, "-c", sumocfg, "--start"])

    tls_id = "C"
    links = traci.trafficlight.getControlledLinks(tls_id)

    for idx, link_list in enumerate(links):
        print(f"INDEX {idx}:")
        if not link_list:
            print("  (sem links neste índice)")
            continue

        for link in link_list:
            # Estrutura típica: (fromLane, toLane, via, linkIndex, tl, controlled)
            from_lane = link[0]
            to_lane = link[1]
            via_lane = link[2]

            from_edge = traci.lane.getEdgeID(from_lane)
            to_edge = traci.lane.getEdgeID(to_lane)
            via_edge = traci.lane.getEdgeID(via_lane) if via_lane != "" else ""

            print(
                f"  {from_edge} -> {to_edge} via {via_edge} | "
                f"from_lane={from_lane} to_lane={to_lane} via={via_lane}"
            )

    traci.close()


if __name__ == "__main__":
    main()