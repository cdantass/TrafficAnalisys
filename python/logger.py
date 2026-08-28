"""
Coleta métricas por passo de simulação e grava um CSV ao final, conforme
seção 2.5 da especificação.
"""
import csv
import os
import time

import traci

import state_reader
from state_reader import get_all_queue_lengths, get_waiting_persons, get_ambulance_ids


class MetricsLogger:
    def __init__(self, tls_id: str, policy_name: str, scenario_label: str, output_dir: str,
                 seed: int = None):
        self.tls_id = tls_id
        self.policy_name = policy_name
        self.scenario_label = scenario_label
        self.output_dir = output_dir
        self.seed = seed

        # acumuladores
        self.ambulance_stops = {}       # vehicle_id -> nº de vezes que parou
        self.ambulance_was_stopped = {} # vehicle_id -> bool (estava parado no passo anterior)
        self.ambulance_first_seen_step = {}
        self.ambulance_last_seen_step = {}

        self.pedestrian_wait_samples = []  # todos os waiting_time observados

        self.vehicle_trip_times = []    # tempos de viagem de veículos que terminaram
        self.vehicle_stop_counts = {}   # vehicle_id -> nº paradas

        self.queue_samples = []         # lista de dicts {edge: tamanho} por passo
        self.pedestrian_conflicts = 0
        self.safe_phase_switches = 0

    def on_step(self, step: int):
        # filas
        self.queue_samples.append(get_all_queue_lengths())

        # pedestres esperando
        for _pid, wait_time in get_waiting_persons():
            self.pedestrian_wait_samples.append(wait_time)

        # ambulância: contar paradas e tempo total
        current_ambulances = get_ambulance_ids()
        for vid in current_ambulances:
            if vid not in self.ambulance_first_seen_step:
                self.ambulance_first_seen_step[vid] = step
                self.ambulance_stops[vid] = 0
                self.ambulance_was_stopped[vid] = False
            self.ambulance_last_seen_step[vid] = step

            speed = traci.vehicle.getSpeed(vid)
            is_stopped_now = speed < 0.3
            if is_stopped_now and not self.ambulance_was_stopped[vid]:
                self.ambulance_stops[vid] += 1
            self.ambulance_was_stopped[vid] = is_stopped_now

        # veículos comuns: paradas (aproximação simples via getStopState/velocidade)
        for vid in traci.vehicle.getIDList():
            if traci.vehicle.getVehicleClass(vid) == "emergency":
                continue
            speed = traci.vehicle.getSpeed(vid)
            self.vehicle_stop_counts.setdefault(vid, 0)
            if speed < 0.3:
                # marca de forma simplificada: soma se acabou de parar
                pass  # (mantido simples nesta etapa; refinamento fica para depois)

        # segurança do TLS: marca conflito em que pedestre e veículo ficam verdes
        # simultaneamente. A contagem é conservadora e serve como métrica para
        # avaliar a política que evita estes casos.
        tls_state = traci.trafficlight.getRedYellowGreenState(self.tls_id)
        tls_links = state_reader.get_tls_link_map(self.tls_id)
        pedestrian_green = {
            idx for idx, ch in enumerate(tls_state)
            if ch.lower() == "g" and idx in tls_links["pedestrian_links"]
        }
        vehicle_green = {
            tls_links["vehicle_links"][idx]
            for idx, ch in enumerate(tls_state)
            if ch.lower() == "g" and idx in tls_links["vehicle_links"]
        }
        for idx in pedestrian_green:
            crossing_id = tls_links["pedestrian_links"].get(idx)
            if crossing_id and state_reader.get_pedestrian_conflicting_vehicle_links(crossing_id).intersection(vehicle_green):
                self.pedestrian_conflicts += 1

        # veículos que saíram da simulação neste passo -> tempo de viagem
        for vid in traci.simulation.getArrivedIDList():
            # getArrivedIDList não traz mais o objeto do veículo, então o
            # tempo de viagem exato por veículo fica para uma versão futura
            # (SUMO oferece isso via tripinfo-output, que é mais preciso —
            # ver nota no README da etapa 2).
            pass

    def _avg(self, values):
        return sum(values) / len(values) if values else 0.0

    def summary(self) -> dict:
        avg_queue_by_edge = {}
        if self.queue_samples:
            edges = self.queue_samples[0].keys()
            for edge in edges:
                avg_queue_by_edge[edge] = self._avg(
                    [sample[edge] for sample in self.queue_samples]
                )

        ambulance_summaries = []
        for vid in self.ambulance_first_seen_step:
            travel_time_s = (
                self.ambulance_last_seen_step[vid] - self.ambulance_first_seen_step[vid]
            )
            ambulance_summaries.append({
                "vehicle_id": vid,
                "travel_time_s": travel_time_s,
                "stops": self.ambulance_stops[vid],
            })

        return {
            "policy": self.policy_name,
            "scenario": self.scenario_label,
            "avg_pedestrian_wait_s": round(self._avg(self.pedestrian_wait_samples), 2),
            "avg_queue_by_edge": avg_queue_by_edge,
            "ambulances": ambulance_summaries,
            "pedestrian_conflicts": self.pedestrian_conflicts,
            "safe_phase_switches": self.safe_phase_switches,
        }

    def write_csv(self):
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        seed_tag = f"_seed{self.seed}" if self.seed is not None else ""
        filename = f"log_{self.policy_name}_{self.scenario_label}{seed_tag}_{timestamp}.csv"
        filepath = os.path.join(self.output_dir, filename)

        summary = self.summary()

        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["metrica", "valor"])
            writer.writerow(["politica", summary["policy"]])
            writer.writerow(["cenario", summary["scenario"]])
            writer.writerow(["seed", self.seed if self.seed is not None else ""])
            writer.writerow(["espera_media_pedestres_s", summary["avg_pedestrian_wait_s"]])
            writer.writerow(["conflitos_pedestre_veiculo", summary["pedestrian_conflicts"]])
            writer.writerow(["trocas_fase_seguras", summary["safe_phase_switches"]])
            for edge, avg_queue in summary["avg_queue_by_edge"].items():
                writer.writerow([f"fila_media_{edge}", round(avg_queue, 2)])
            for amb in summary["ambulances"]:
                writer.writerow([f"ambulancia_{amb['vehicle_id']}_tempo_viagem_s", amb["travel_time_s"]])
                writer.writerow([f"ambulancia_{amb['vehicle_id']}_paradas", amb["stops"]])

        print(f"[logger] Métricas gravadas em: {filepath}")
        return filepath