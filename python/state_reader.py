"""
Leitura de estado da simulação a cada passo, via TraCI.

Mantido separado do resto para que a etapa 3 (política adaptativa) e a
comunicação com Unity possam reusar essas funções sem duplicar lógica.
"""
import traci

# Arestas que chegam ao cruzamento central (usadas para medir fila por abordagem)
INCOMING_EDGES = ["N_C", "S_C", "E_C", "W_C"]

# vClass usado pela ambulância no arquivo de rotas
AMBULANCE_VCLASS = "emergency"


def get_queue_length(edge_id: str, speed_threshold: float = 0.3) -> int:
    """Número de veículos parados (velocidade abaixo do limiar) numa aresta."""
    vehicle_ids = traci.edge.getLastStepVehicleIDs(edge_id)
    return sum(
        1 for vid in vehicle_ids
        if traci.vehicle.getSpeed(vid) < speed_threshold
    )


def get_all_queue_lengths() -> dict:
    """Fila (veículos parados) em cada uma das 4 abordagens do cruzamento."""
    return {edge: get_queue_length(edge) for edge in INCOMING_EDGES}


def get_waiting_persons() -> list:
    """
    Lista de (person_id, waiting_time) para pedestres atualmente esperando
    (velocidade ~0) em qualquer lugar da simulação.
    """
    waiting = []
    for pid in traci.person.getIDList():
        if traci.person.getSpeed(pid) < 0.1:
            waiting.append((pid, traci.person.getWaitingTime(pid)))
    return waiting


def get_ambulance_ids() -> list:
    """IDs de veículos atualmente na simulação cujo vClass é 'emergency'."""
    return [
        vid for vid in traci.vehicle.getIDList()
        if traci.vehicle.getVehicleClass(vid) == AMBULANCE_VCLASS
    ]


def get_ambulance_distance_to_junction(vehicle_id: str, junction_id: str) -> float:
    """
    Distância aproximada (em metros) do veículo até o cruzamento,
    usando a posição restante na aresta atual.
    """
    edge_id = traci.vehicle.getRoadID(vehicle_id)
    if edge_id.startswith(":"):
        # já está dentro da área interna do cruzamento
        return 0.0
    lane_id = traci.vehicle.getLaneID(vehicle_id)
    lane_length = traci.lane.getLength(lane_id)
    pos_on_lane = traci.vehicle.getLanePosition(vehicle_id)
    return max(0.0, lane_length - pos_on_lane)


def is_ambulance_approaching(junction_id: str, detection_distance_m: float) -> bool:
    """True se houver alguma ambulância dentro da distância de detecção."""
    for vid in get_ambulance_ids():
        if get_ambulance_distance_to_junction(vid, junction_id) <= detection_distance_m:
            return True
    return False
