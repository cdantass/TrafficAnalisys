"""
Script principal da etapa 2.

Inicia a simulação SUMO via TraCI, roda passo a passo, aplica a política
de semáforo escolhida (fixo, por enquanto) e grava métricas em CSV.

Uso (dentro da pasta python/, com SUMO_HOME já definido na sessão):

    python main.py
    python main.py --policy fixo
    python main.py --policy fixo --no-gui
    python main.py --config ..\sumo\config\config.json

A comunicação com Unity e a política adaptativa completa entram nas
próximas etapas.
"""
import argparse
import os
import sys

# --sumo-home precisa ser lido ANTES de importar sumo_env/traci, então
# fazemos um parse leve e manual só desse argumento primeiro.
def _pre_parse_sumo_home():
    if "--sumo-home" in sys.argv:
        idx = sys.argv.index("--sumo-home")
        return sys.argv[idx + 1]
    return None


from sumo_env import setup_traci_import, get_sumo_binary
from config_loader import load_config

setup_traci_import(sumo_home_override=_pre_parse_sumo_home())
import traci  # precisa vir depois de setup_traci_import()

from tls_policy import build_policy
from logger import MetricsLogger


def parse_args():
    parser = argparse.ArgumentParser(description="Controlador de semáforo via TraCI")
    parser.add_argument(
        "--config",
        default=os.path.join("..", "sumo", "config", "config.json"),
        help="Caminho para o config.json (padrão: ../sumo/config/config.json)",
    )
    parser.add_argument(
        "--policy",
        choices=["fixo", "adaptativo"],
        default=None,
        help="Sobrescreve a política definida no config.json",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Roda sem interface gráfica (mais rápido)",
    )
    parser.add_argument(
        "--sumo-home",
        default=None,
        help="Caminho da instalação do SUMO (alternativa a definir SUMO_HOME no terminal)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    config_path = os.path.abspath(args.config)
    config = load_config(config_path)

    policy_name = args.policy or config["politica_semaforo"]["tipo"]
    tls_id = config["juncao"]["id"]
    use_gui = config["sumo"]["gui"] and not args.no_gui

    sumocfg_path = os.path.abspath(os.path.join("..", "sumo", "config", "simulation.sumocfg"))
    if not os.path.isfile(sumocfg_path):
        sumocfg_path = os.path.abspath(os.path.join("..", config["sumo"]["config_file"]))

    sumo_binary = get_sumo_binary(gui=use_gui)
    sumo_cmd = [sumo_binary, "-c", sumocfg_path]

    # rótulo do cenário ativo (usado no nome do arquivo de log)
    cenarios = config["cenarios"]
    scenario_parts = []
    if cenarios.get("pedestres_ativo"):
        scenario_parts.append("pedestres")
    if cenarios.get("ambulancia_ativa"):
        scenario_parts.append("ambulancia")
    scenario_label = "_".join(scenario_parts) if scenario_parts else "base"

    print(f"[main] Iniciando SUMO ({'gui' if use_gui else 'sem gui'})...")
    print(f"[main] Política: {policy_name} | Cenário: {scenario_label}")

    traci.start(sumo_cmd)

    policy = build_policy(policy_name, tls_id, config)
    policy.on_start()

    output_dir = os.path.abspath(os.path.join("..", "sumo", config["logs"]["diretorio_saida"]))
    logger = MetricsLogger(
        tls_id=tls_id,
        policy_name=policy_name,
        scenario_label=scenario_label,
        output_dir=output_dir,
    )

    step = 0
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            policy.on_step(step)
            logger.on_step(step)
            step += 1
    except KeyboardInterrupt:
        print("\n[main] Interrompido pelo usuário.")
    finally:
        traci.close()
        logger.write_csv()
        print(f"[main] Simulação encerrada em {step} passos.")


if __name__ == "__main__":
    main()
