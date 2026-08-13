"""
Garante que o pacote `traci` (que vem junto com a instalação do SUMO, em
$SUMO_HOME/tools) possa ser importado, mesmo sem ele estar instalado via pip.

Precisa que a variável de ambiente SUMO_HOME esteja definida na sessão do
terminal ANTES de rodar o script Python. No PowerShell (Windows), por
exemplo:

    $env:SUMO_HOME = "C:\caminho\para\sumo-1.27.1"

(o mesmo valor usado para achar o netconvert.exe, um nível acima da pasta bin)
"""
import os
import sys


def setup_traci_import(sumo_home_override: str = None) -> None:
    sumo_home = sumo_home_override or os.environ.get("SUMO_HOME")
    if not sumo_home:
        raise EnvironmentError(
            "Não encontrei o caminho do SUMO.\n\n"
            "Opção 1 (mais simples, não depende de variável de ambiente):\n"
            "  python main.py --sumo-home \"D:\RESTO\sumo-1.27.1\"\n\n"
            "Opção 2: definir SUMO_HOME nesta sessão do PowerShell antes de rodar:\n"
            '  $env:SUMO_HOME = "D:\RESTO\sumo-1.27.1"\n'
            "  python main.py"
        )

    # garante que os.environ também fique com o valor certo, pois outras
    # partes do código (get_sumo_binary) também leem SUMO_HOME
    os.environ["SUMO_HOME"] = sumo_home

    tools_path = os.path.join(sumo_home, "tools")
    if not os.path.isdir(tools_path):
        raise EnvironmentError(
            f"SUMO_HOME está definido como '{sumo_home}', mas não encontrei "
            f"a pasta 'tools' dentro dele ({tools_path}). Confirme se o "
            f"caminho aponta para a pasta raiz da instalação do SUMO "
            f"(a que contém 'bin' e 'tools')."
        )

    if tools_path not in sys.path:
        sys.path.append(tools_path)


def get_sumo_binary(gui: bool) -> str:
    """Retorna o caminho completo para sumo.exe ou sumo-gui.exe."""
    sumo_home = os.environ["SUMO_HOME"]
    exe_name = "sumo-gui" if gui else "sumo"
    if os.name == "nt":
        exe_name += ".exe"
    binary_path = os.path.join(sumo_home, "bin", exe_name)
    if not os.path.isfile(binary_path):
        raise FileNotFoundError(f"Não encontrei o executável do SUMO em: {binary_path}")
    return binary_path
