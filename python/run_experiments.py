"""Executa uma matriz de experimentos do controlador SUMO.

Exemplos, a partir de qualquer diretório:

    python python/run_experiments.py --no-gui
    python python/run_experiments.py --policies fixo adaptativo --seeds 1 2 3
    python python/run_experiments.py --runs 10 --seed-start 100 --no-gui

Cada simulação é executada em um processo separado através de ``main.py``.
Os CSVs continuam sendo gravados no diretório configurado pelo projeto.
"""
import argparse
import os
import subprocess
import sys
from itertools import product


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
DEFAULT_CONFIG = os.path.join(PROJECT_ROOT, "sumo", "config", "config.json")
MAIN_SCRIPT = os.path.join(SCRIPT_DIR, "main.py")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Executa várias simulações SUMO com políticas e sementes diferentes."
    )
    parser.add_argument(
        "--policies",
        nargs="+",
        choices=["fixo", "adaptativo"],
        default=["fixo", "adaptativo"],
        help="Políticas a testar (padrão: fixo adaptativo).",
    )
    parser.add_argument(
        "--seeds",
        nargs="+",
        type=int,
        help="Lista explícita de sementes; combina cada semente com cada política.",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=1,
        help="Quantidade de execuções por política quando --seeds não for usado (padrão: 1).",
    )
    parser.add_argument(
        "--seed-start",
        type=int,
        default=1,
        help="Primeira semente gerada por --runs (padrão: 1).",
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG,
        help="Caminho para o config.json.",
    )
    parser.add_argument(
        "--sumo-home",
        default=None,
        help="Caminho da instalação do SUMO; também pode ser definido em SUMO_HOME.",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Executa todas as simulações sem interface gráfica.",
    )
    parser.add_argument(
        "--continue-on-error",
        action="store_true",
        help="Continua com os demais experimentos se um deles falhar.",
    )
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs deve ser maior que zero.")
    return args


def run_experiments(args):
    config_path = os.path.abspath(args.config)
    seeds = args.seeds or [args.seed_start + offset for offset in range(args.runs)]
    experiments = list(product(args.policies, seeds))
    failed = []
    completed = 0

    print(f"[experiments] Total: {len(experiments)} execução(ões).")
    for index, (policy, seed) in enumerate(experiments, start=1):
        command = [
            sys.executable,
            MAIN_SCRIPT,
            "--config",
            config_path,
            "--policy",
            policy,
            "--seed",
            str(seed),
        ]
        if args.no_gui:
            command.append("--no-gui")
        if args.sumo_home:
            command.extend(["--sumo-home", args.sumo_home])

        print(f"\n[experiments] [{index}/{len(experiments)}] política={policy}, seed={seed}")
        result = subprocess.run(command, cwd=SCRIPT_DIR)
        if result.returncode != 0:
            failed.append((policy, seed, result.returncode))
            print(f"[experiments] Falhou (código {result.returncode}).")
            if not args.continue_on_error:
                break
        else:
            completed += 1

    print(f"\n[experiments] Concluídos: {completed}/{len(experiments)}.")
    if failed:
        print("[experiments] Falhas:")
        for policy, seed, returncode in failed:
            print(f"  - política={policy}, seed={seed}, código={returncode}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(run_experiments(parse_args()))