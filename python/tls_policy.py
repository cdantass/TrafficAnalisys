"""
Políticas de controle de semáforo.

Etapa 2: só a política FIXA está implementada de fato. A política
adaptativa é um stub que, por enquanto, se comporta como a fixa — será
implementada na etapa 3 (regras de fila / pedestre / ambulância).
"""
import traci

import state_reader


class FixedPolicy:
    """
    Controle fixo: usa o programa de semáforo padrão gerado automaticamente
    pelo netconvert (tempos pré-definidos, ciclo fixo, sem adaptação).
    O Python aqui não interfere nas fases — só observa e loga.
    """

    name = "fixo"

    def __init__(self, tls_id: str, config: dict):
        self.tls_id = tls_id
        self.config = config

    def on_start(self):
        # Nada a fazer: o programa default do netconvert já é "fixo" por definição.
        pass

    def on_step(self, step: int):
        # Controle fixo não toma nenhuma decisão por passo.
        pass


class AdaptivePolicy:
    """
    Controle adaptativo (etapa 3), com as 3 regras da especificação (2.2):

      1. Pedestre esperando além do gatilho -> estende o verde de pedestre.
      2. Ambulância se aproximando -> encurta a fase atual (se não for a
         dela) para chegar mais rápido na fase que dá verde para ela, e
         estende essa fase enquanto ela ainda não passou.
      3. Fila de veículos acima do limite -> estende o verde da fase atual.

    A ligação entre índice de sinal <-> aresta/pedestre é descoberta em
    tempo real (via TraCI), lendo a topologia real gerada pelo netconvert
    -- não depende de strings de estado escritas à mão.
    """

    name = "adaptativo"

    def __init__(self, tls_id: str, config: dict):
        self.tls_id = tls_id
        self.cfg = config["politica_semaforo"]["adaptativo"]

        self.phase_vehicle_green_approaches = {}  # phase_idx -> {edges}
        self.phase_has_pedestrian_green = {}       # phase_idx -> bool

        self._last_phase = None
        self._extended_pedestrian_this_phase = False
        self._extended_queue_this_phase = False
        self._shortened_for_ambulance_this_phase = False
        self._total_extension_this_phase = 0.0

    # ---------- setup ----------

    def on_start(self):
        self._map_phases()

    def _get_current_logic(self):
        logics = traci.trafficlight.getAllProgramLogics(self.tls_id)
        current_program_id = traci.trafficlight.getProgram(self.tls_id)
        for logic in logics:
            if logic.programID == current_program_id:
                return logic
        return logics[0]

    @staticmethod
    def _is_pedestrian_lane(lane_id: str) -> bool:
        allowed = traci.lane.getAllowed(lane_id)
        return len(allowed) == 1 and allowed[0] == "pedestrian"

    def _map_phases(self):
        """
        Para cada fase do programa atual, descobre:
          - quais arestas de abordagem (N_C/S_C/E_C/W_C) têm verde nela;
          - se essa fase também dá verde para alguma faixa de pedestre.
        """
        logic = self._get_current_logic()
        links = traci.trafficlight.getControlledLinks(self.tls_id)

        index_to_approach = {}
        index_to_is_pedestrian = {}
        for idx, link_list in enumerate(links):
            if not link_list:
                continue
            in_lane = link_list[0][0]
            edge_id = traci.lane.getEdgeID(in_lane)
            if self._is_pedestrian_lane(in_lane):
                index_to_is_pedestrian[idx] = True
            elif edge_id in state_reader.INCOMING_EDGES:
                index_to_approach[idx] = edge_id

        for phase_idx, phase in enumerate(logic.phases):
            state = phase.state
            green_approaches = set()
            has_ped_green = False
            for idx, ch in enumerate(state):
                if ch != "G":
                    continue
                if idx in index_to_approach:
                    green_approaches.add(index_to_approach[idx])
                if index_to_is_pedestrian.get(idx):
                    has_ped_green = True
            self.phase_vehicle_green_approaches[phase_idx] = green_approaches
            self.phase_has_pedestrian_green[phase_idx] = has_ped_green

        print(
            f"[adaptativo] Mapeamento de fases concluído: "
            f"{len(logic.phases)} fases identificadas."
        )

    # ---------- loop principal ----------

    def on_step(self, step: int):
        current_phase = traci.trafficlight.getPhase(self.tls_id)

        if current_phase != self._last_phase:
            self._last_phase = current_phase
            self._extended_pedestrian_this_phase = False
            self._extended_queue_this_phase = False
            self._shortened_for_ambulance_this_phase = False
            self._total_extension_this_phase = 0.0

        green_approaches = self.phase_vehicle_green_approaches.get(current_phase, set())
        has_ped_green = self.phase_has_pedestrian_green.get(current_phase, False)
        remaining = traci.trafficlight.getNextSwitch(self.tls_id) - traci.simulation.getTime()

        self._apply_pedestrian_rule(has_ped_green, remaining)
        self._apply_ambulance_rule(green_approaches, remaining)
        self._apply_queue_rule(green_approaches, remaining)

    def _apply_pedestrian_rule(self, has_ped_green: bool, remaining: float):
        if not has_ped_green or self._extended_pedestrian_this_phase:
            return
        waiting = state_reader.get_waiting_persons()
        threshold = self.cfg["espera_pedestre_extensao_gatilho_s"]
        if any(wait_time >= threshold for _pid, wait_time in waiting):
            extension = self.cfg["extensao_verde_pedestre_s"]
            if self._total_extension_this_phase + extension <= self.cfg["verde_maximo_s"]:
                traci.trafficlight.setPhaseDuration(self.tls_id, remaining + extension)
                self._total_extension_this_phase += extension
                self._extended_pedestrian_this_phase = True

    def _apply_ambulance_rule(self, green_approaches: set, remaining: float):
        detection_distance = self.cfg["distancia_deteccao_ambulancia_m"]
        for amb_id in state_reader.get_ambulance_ids():
            dist = state_reader.get_ambulance_distance_to_junction(amb_id, self.tls_id)
            if dist > detection_distance:
                continue

            amb_edge = traci.vehicle.getRoadID(amb_id)

            if amb_edge in green_approaches:
                # é a vez dela: garante alguns segundos extras pra ela passar
                traci.trafficlight.setPhaseDuration(self.tls_id, remaining + 5)
            elif not self._shortened_for_ambulance_this_phase and remaining > 3:
                # não é a vez dela: encurta a fase atual (respeitando o
                # amarelo normal, que continua acontecendo depois) para
                # chegar mais rápido na fase que abre pra ela
                traci.trafficlight.setPhaseDuration(self.tls_id, 3)
                self._shortened_for_ambulance_this_phase = True

    def _apply_queue_rule(self, green_approaches: set, remaining: float):
        if not green_approaches or self._extended_queue_this_phase:
            return
        queues = state_reader.get_all_queue_lengths()
        limit = self.cfg["limite_fila_veiculos"]
        if any(queues.get(edge, 0) >= limit for edge in green_approaches):
            extension = self.cfg["incremento_verde_fila_s"]
            if self._total_extension_this_phase + extension <= self.cfg["verde_maximo_s"]:
                traci.trafficlight.setPhaseDuration(self.tls_id, remaining + extension)
                self._total_extension_this_phase += extension
                self._extended_queue_this_phase = True


def build_policy(policy_name: str, tls_id: str, config: dict):
    if policy_name == "fixo":
        return FixedPolicy(tls_id, config)
    elif policy_name == "adaptativo":
        return AdaptivePolicy(tls_id, config)
    else:
        raise ValueError(
            f"Política de semáforo desconhecida: '{policy_name}'. "
            f"Use 'fixo' ou 'adaptativo'."
        )