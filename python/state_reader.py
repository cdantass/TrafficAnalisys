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

# Mapeamento seguro dos crossings do TLS C para as abordagens veiculares
# que cruzam aquela faixa. Em um cruzamento de 4 vias, cada faixa de pedestre
# pode ter conflito com a via perpendicular ou com o eixo do mesmo sentido.
PED_CROSSINGS = {
    ":C_c0": {"N_C", "S_C"},
    ":C_c1": {"E_C", "W_C"},
    ":C_c2": {"S_C", "N_C"},
    ":C_c3": {"W_C", "E_C"},
}

TURN_DIRECTION_BY_APPROACH = {
    "N_C": {"straight": "C_S", "right": "C_W", "left": "C_E", "uturn": "C_N"},
    "S_C": {"straight": "C_N", "right": "C_E", "left": "C_W", "uturn": "C_S"},
    "E_C": {"straight": "C_W", "right": "C_N", "left": "C_S", "uturn": "C_E"},
    "W_C": {"straight": "C_E", "right": "C_S", "left": "C_N", "uturn": "C_W"},
}


def get_vehicle_turn_type(link_tuple: tuple) -> str:
    """Retorna 'straight', 'right' ou 'left' usando a abordagem e o sentido de saída."""
    if len(link_tuple) < 2:
        return "unknown"
    in_lane, out_lane = link_tuple[0], link_tuple[1]
    in_edge = traci.lane.getEdgeID(in_lane)
    out_edge = traci.lane.getEdgeID(out_lane)
    if in_edge not in TURN_DIRECTION_BY_APPROACH:
        return "unknown"
    for turn_type, expected_edge in TURN_DIRECTION_BY_APPROACH[in_edge].items():
        if expected_edge == out_edge:
            return turn_type
    return "unknown"


def get_tls_link_map(tls_id: str) -> dict:
    """Mapeia índices do estado do TLS para links de veículo/pedestre."""
    controlled_links = traci.trafficlight.getControlledLinks(tls_id)
    vehicle_links = {}
    pedestrian_links = {}
    straight_links = {}
    right_turn_links = {}
    left_turn_links = {}
    uturn_links = {}
    for idx, link_group in enumerate(controlled_links):
        for link in link_group:
            if not link:
                continue
            from_lane = link[0]
            to_lane = link[1]
            edge_id = traci.lane.getEdgeID(from_lane)
            if edge_id.startswith(":C_c"):
                pedestrian_links[idx] = edge_id
            elif edge_id in INCOMING_EDGES:
                vehicle_links[idx] = edge_id
                turn_type = get_vehicle_turn_type((from_lane, to_lane))
                if turn_type == "straight":
                    straight_links[idx] = edge_id
                elif turn_type == "right":
                    right_turn_links[idx] = edge_id
                elif turn_type == "left":
                    left_turn_links[idx] = edge_id
                elif turn_type == "uturn":
                    uturn_links[idx] = edge_id
    return {
        "vehicle_links": vehicle_links,
        "pedestrian_links": pedestrian_links,
        "straight_links": straight_links,
        "right_turn_links": right_turn_links,
        "left_turn_links": left_turn_links,
        "uturn_links": uturn_links,
        "conflicts": PED_CROSSINGS,
    }


def get_pedestrian_conflicting_vehicle_links(crossing_id: str) -> set:
    """Retorna as abordagens veiculares que cruzam esse crossing."""
    return set(PED_CROSSINGS.get(crossing_id, set()))


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