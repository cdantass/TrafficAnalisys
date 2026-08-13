"""
Carregamento do arquivo de configuração (config/config.json).

Todos os parâmetros ajustáveis do sistema (tempos de semáforo, limites de
fila, distância de detecção de ambulância, cenários ativos, etc.) vivem
nesse JSON — nada deve ficar hardcoded no restante do código Python.
"""
import json
import os


def load_config(path: str) -> dict:
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"Arquivo de configuração não encontrado: {path}\n"
            f"Rode o script a partir da pasta 'python/', ou passe o caminho "
            f"correto com --config."
        )
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)
