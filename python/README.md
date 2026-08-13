# Controlador Python (TraCI) — Etapa 2

Esta etapa adiciona o controlador Python que inicia o SUMO, aplica a
política de semáforo **fixa** e gera os primeiros logs de métricas.
A política **adaptativa** ainda é um placeholder (entra na etapa 3).

## Estrutura

```
python/
├── main.py           # orquestra tudo: inicia SUMO, roda o loop, grava logs
├── config_loader.py  # lê o config.json
├── sumo_env.py        # localiza SUMO_HOME e o binário do sumo/sumo-gui
├── state_reader.py    # leitura de filas, pedestres e ambulância via TraCI
├── tls_policy.py       # políticas de semáforo (fixo implementado, adaptativo = stub)
└── logger.py            # coleta métricas por passo e grava CSV em sumo/logs/
```

## Pré-requisitos

- Etapa 1 já rodando (rede `intersection.net.xml` gerada com sucesso).
- Python 3.x instalado.
- `SUMO_HOME` definido **na mesma sessão de terminal** onde for rodar o
  script (o Python usa isso para achar o pacote `traci` e o executável
  do SUMO).

## Como rodar (PowerShell, Windows)

```powershell
# 1. Definir SUMO_HOME nesta sessão (ajuste o caminho para o seu)
$env:SUMO_HOME = "C:\Users\cdbarbosa\Downloads\sumo-win64-1.27.1\sumo-1.27.1"

# 2. Entrar na pasta python/
cd caminho\para\traffic-sim\python

# 3. Rodar com a política fixa (padrão do config.json) e interface gráfica
python main.py

# Variações úteis:
python main.py --policy fixo --no-gui      # mais rápido, sem interface
python main.py --policy adaptativo         # ainda se comporta como fixo (etapa 3 implementa de verdade)
```

## O que esperar

- Uma janela do `sumo-gui` deve abrir e a simulação roda sozinha (o
  Python está pilotando por trás via TraCI).
- No terminal, você vê mensagens de início/fim e, ao encerrar, o caminho
  do arquivo CSV gerado.
- Um arquivo tipo `sumo/logs/log_fixo_pedestres_ambulancia_20250813-...csv`
  vai conter:
  - espera média dos pedestres,
  - fila média por abordagem (N_C, S_C, E_C, W_C),
  - tempo de viagem e nº de paradas da ambulância.

## Limitações conhecidas desta etapa (propositais, para não travar o avanço)

- **Tempo de viagem de veículos comuns** (não-ambulância) ainda não está
  no CSV de forma precisa — o jeito robusto de obter isso é usar a saída
  nativa do SUMO (`--tripinfo-output`), que vamos incorporar numa
  próxima passada, sem precisar recalcular isso manualmente no Python.
- **Contagem de paradas de veículos comuns** está com um placeholder
  (função existe em `logger.py`, mas não incrementa ainda) — mesma
  solução acima resolve isso de forma mais confiável.
- A política **adaptativa** apenas roda igual à fixa por enquanto — as
  regras de fila/pedestre/ambulância entram na etapa 3.

Nenhuma dessas limitações impede testar o fluxo completo end-to-end
(SUMO + Python + logs) — só deixam as métricas de veículos comuns
incompletas por enquanto.

## Próxima etapa

Etapa 3: implementar de fato a **política adaptativa** (extensão de verde
para pedestres, prioridade de ambulância, aumento de verde por fila) e
incorporar `--tripinfo-output` para métricas de veículos mais precisas.
