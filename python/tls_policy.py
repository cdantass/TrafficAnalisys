"""
Políticas de controle de semáforo.

O comportamento fixo permanece como fallback. A política adaptativa faz
uma leitura simples do estado do cruzamento via TraCI e aplica regras de
fila, pedestres e ambulância sem quebrar o restante da arquitetura.
"""
import collections

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
        self.phase_has_pedestrian_green = {}      # phase_idx -> bool
        self._phase_by_approach = {}              # edge -> [phase_idx]
        self._queue_history = collections.defaultdict(list)
        self._tls_links = state_reader.get_tls_link_map(self.tls_id)

        # Índices de links para a abordagem Norte (N_C)
        self._right_turn_indices_north = {0}
        self._straight_indices_north   = {1, 2}

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
        """Relaciona cada fase com as abordagens/peões que recebem verde nela."""
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
                if ch.upper() == "G":
                    if idx in index_to_approach:
                        green_approaches.add(index_to_approach[idx])
                    if index_to_is_pedestrian.get(idx):
                        has_ped_green = True
            self.phase_vehicle_green_approaches[phase_idx] = green_approaches
            self.phase_has_pedestrian_green[phase_idx] = has_ped_green
            for edge_id in green_approaches:
                self._phase_by_approach.setdefault(edge_id, []).append(phase_idx)

        print(
            f"[adaptativo] Mapeamento de fases concluído: "
            f"{len(logic.phases)} fases identificadas."
        )

    def _average_queue_for_edge(self, edge_id: str) -> float:
        history = self._queue_history.get(edge_id, [])
        queue_values = history if history else [state_reader.get_all_queue_lengths().get(edge_id, 0)]
        return sum(queue_values) / len(queue_values)

    def _update_queue_history(self):
        queues = state_reader.get_all_queue_lengths()
        for edge_id, queue_size in queues.items():
            history = self._queue_history[edge_id]
            history.append(queue_size)
            if len(history) > 5:
                history.pop(0)

    def _waiting_pedestrians_exceed_threshold(self) -> bool:
        threshold = self.cfg["espera_pedestre_extensao_gatilho_s"]
        return any(wait_time >= threshold for _pid, wait_time in state_reader.get_waiting_persons())

    def _find_phase_for_approach(self, edge_id: str):
        phases = self._phase_by_approach.get(edge_id, [])
        if not phases:
            return None
        return phases[0]

    def _phase_has_safe_pedestrian_green(self, phase_state: str) -> bool:
        """Verifica se a fase atual libera pedestre sem veículo conflitante verde."""
        ped_green = set()
        vehicle_green = set()
        for idx, ch in enumerate(phase_state):
            if ch.lower() != "g":
                continue
            if idx in self._tls_links["pedestrian_links"]:
                ped_green.add(self._tls_links["pedestrian_links"][idx])
            elif idx in self._tls_links["vehicle_links"]:
                vehicle_green.add(self._tls_links["vehicle_links"][idx])

        if not ped_green:
            return False

        for crossing_id in ped_green:
            conflicts = state_reader.get_pedestrian_conflicting_vehicle_links(crossing_id)
            if conflicts.intersection(vehicle_green):
                return False
        return True

    def _phase_is_curve_exclusive(self, phase_state: str) -> bool:
        """Uma curva deve ser tratada como movimento exclusivo e seguro."""
        vehicle_green = set()
        for idx, ch in enumerate(phase_state):
            if ch.lower() == "g" and idx in self._tls_links["vehicle_links"]:
                vehicle_green.add(self._tls_links["vehicle_links"][idx])
        return len(vehicle_green) <= 1

    def _find_safe_pedestrian_phase(self):
        logic = self._get_current_logic()
        for phase_idx, phase in enumerate(logic.phases):
            if self._phase_has_safe_pedestrian_green(phase.state):
                return phase_idx
        return None

    def _phase_has_illegal_north_right_turn(self, phase_state: str) -> bool:
        """
        Retorna True se, na fase atual, há fluxo reto vindo do norte (N_C -> C_S)
        e curva à direita vindo do norte (N_C -> C_W) verdes ao mesmo tempo.
        Quando o principal está verde, a direita deve ficar vermelha.
        """
        has_straight_green = any(
            phase_state[i].lower() == "g" for i in self._straight_indices_north
        )
        has_right_green = any(
            phase_state[i].lower() == "g" for i in self._right_turn_indices_north
        )
        return has_straight_green and has_right_green

    def _find_safe_curve_phase(self):
        logic = self._get_current_logic()
        for phase_idx, phase in enumerate(logic.phases):
            if self._phase_is_curve_exclusive(phase.state) and not self._phase_has_illegal_north_right_turn(phase.state):
                return phase_idx
        return None

    # ---------- loop principal ----------

    def on_step(self, step: int):
        current_phase = traci.trafficlight.getPhase(self.tls_id)
        if current_phase != self._last_phase:
            self._last_phase = current_phase
            self._extended_pedestrian_this_phase = False
            self._extended_queue_this_phase = False
            self._shortened_for_ambulance_this_phase = False
            self._total_extension_this_phase = 0.0

        logic = self._get_current_logic()
        current_phase_obj = logic.phases[current_phase]
        current_state = current_phase_obj.state

        # Regra: não permitir reto + direita verdes ao mesmo tempo vindo do norte
        if self._phase_has_illegal_north_right_turn(current_state):
            safe_curve_phase = self._find_safe_curve_phase()
            if safe_curve_phase is not None and safe_curve_phase != current_phase:
                traci.trafficlight.setPhase(self.tls_id, safe_curve_phase)
                return

        self._update_queue_history()
        green_approaches = self.phase_vehicle_green_approaches.get(current_phase, set())
        remaining = traci.trafficlight.getNextSwitch(self.tls_id) - traci.simulation.getTime()

        # resto das regras que já existem...
        if self._waiting_pedestrians_exceed_threshold():
            safe_phase = self._find_safe_pedestrian_phase()
            if safe_phase is not None and safe_phase != current_phase:
                traci.trafficlight.setPhase(self.tls_id, safe_phase)
                return

        safe_curve_phase = self._find_safe_curve_phase()
        if safe_curve_phase is not None and safe_curve_phase != current_phase:
            if self._phase_is_curve_exclusive(current_state):
                pass
            else:
                traci.trafficlight.setPhase(self.tls_id, safe_curve_phase)
                return

        self._apply_ambulance_rule(current_phase, green_approaches, remaining)
        self._apply_pedestrian_rule(current_phase, remaining)
        self._apply_queue_rule(current_phase, green_approaches, remaining)

    def _apply_pedestrian_rule(self, current_phase: int, remaining: float):
        if self._extended_pedestrian_this_phase or not self._waiting_pedestrians_exceed_threshold():
            return

        if self.phase_has_pedestrian_green.get(current_phase, False):
            extension = self.cfg["extensao_verde_pedestre_s"]
            if self._total_extension_this_phase + extension <= self.cfg["verde_maximo_s"]:
                traci.trafficlight.setPhaseDuration(self.tls_id, remaining + extension)
                self._total_extension_this_phase += extension
                self._extended_pedestrian_this_phase = True
            return

        safe_phase = self._find_safe_pedestrian_phase()
        if safe_phase is not None:
            traci.trafficlight.setPhase(self.tls_id, safe_phase)
            return

    def _apply_ambulance_rule(self, current_phase: int, green_approaches: set, remaining: float):
        detection_distance = self.cfg["distancia_deteccao_ambulancia_m"]
        for amb_id in state_reader.get_ambulance_ids():
            dist = state_reader.get_ambulance_distance_to_junction(amb_id, self.tls_id)
            if dist > detection_distance:
                continue

            amb_edge = traci.vehicle.getRoadID(amb_id)
            if amb_edge in green_approaches:
                traci.trafficlight.setPhaseDuration(self.tls_id, remaining + 5)
                return

            target_phase = self._find_phase_for_approach(amb_edge)
            if target_phase is None:
                continue

            if current_phase != target_phase:
                traci.trafficlight.setPhase(self.tls_id, target_phase)
                return

            if remaining <= 3:
                traci.trafficlight.setPhaseDuration(self.tls_id, 5)
                return

            break

    def _apply_queue_rule(self, current_phase: int, green_approaches: set, remaining: float):
        if not green_approaches or self._extended_queue_this_phase:
            return

        limit = self.cfg["limite_fila_veiculos"]
        for edge_id in green_approaches:
            avg_queue = self._average_queue_for_edge(edge_id)
            if avg_queue >= limit:
                extension = self.cfg["incremento_verde_fila_s"]
                if self._total_extension_this_phase + extension <= self.cfg["verde_maximo_s"]:
                    traci.trafficlight.setPhaseDuration(self.tls_id, remaining + extension)
                    self._total_extension_this_phase += extension
                    self._extended_queue_this_phase = True
                    return

        if current_phase not in self.phase_vehicle_green_approaches:
            return


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