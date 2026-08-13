# Cenário SUMO — Etapa 1

Esta etapa contém **apenas** o cenário de simulação de tráfego (SUMO puro,
sem Python/TraCI e sem Unity ainda). Ela serve para validar que a rede,
as rotas e o semáforo funcionam antes de conectar o controlador.

## Estrutura

```
sumo/
├── network/
│   ├── intersection.nod.xml   # nós: cruzamento central (C) + 4 pontas (N/S/E/W)
│   ├── intersection.edg.xml   # arestas: 2 faixas por sentido em cada direção
│   ├── intersection.typ.xml   # tipo de via (2 faixas + calçada)
│   ├── build_network.sh       # gera intersection.net.xml via netconvert
│   └── tls_fixed.add.xml      # programa de semáforo fixo (fallback)
├── routes/
│   ├── vehicles.rou.xml       # fluxos de carros nas 4 abordagens
│   ├── pedestrians.rou.xml    # fluxos de pedestres nas faixas
│   └── ambulance.rou.xml      # 1 ambulância com prioridade (vClass=emergency)
├── config/
│   ├── simulation.sumocfg     # junta rede + rotas + semáforo
│   └── config.json            # parâmetros ajustáveis (etapa 2 vai consumir)
└── logs/                      # saída de métricas (vazio por enquanto)
```

## Como rodar

1. Instale o SUMO e configure `SUMO_HOME` (veja instruções que te passei
   antes, ou https://sumo.dlr.de/docs/Downloads.php).

2. Gere a rede:
   ```bash
   cd sumo/network
   chmod +x build_network.sh
   ./build_network.sh
   ```
   Isso cria `intersection.net.xml` na mesma pasta.

3. Rode a simulação com interface gráfica:
   ```bash
   cd ../config
   sumo-gui -c simulation.sumocfg
   ```
   Ou sem interface (mais rápido, útil para testes automatizados depois):
   ```bash
   sumo -c simulation.sumocfg
   ```

## O que conferir quando rodar

- O cruzamento aparece com 4 abordagens, 2 faixas por sentido.
- Carros entram pelas 4 pontas e fazem conversões (frente, esquerda, direita).
- Pedestres aparecem nas calçadas e atravessam nas faixas.
- Por volta de t=60s, a ambulância (vermelha, `ambulance_1`) entra pelo
  Oeste — repare que ela é um veículo `emergency`, mas **ainda não tem
  prioridade de fato**: isso só será implementado na próxima etapa, via
  TraCI (Python), que vai detectar sua aproximação e forçar o verde.
- O semáforo troca de fase sozinho, seguindo `tls_fixed.add.xml` — esse é
  o comportamento "cru" do SUMO, sem nenhuma lógica adaptativa ainda.

## ⚠️ Ponto de atenção

O arquivo `tls_fixed.add.xml` tem uma sequência de estados (`state="GGrrGGrr"`
etc.) que é uma **estimativa plausível** para a topologia gerada, mas o
número exato de conexões/links no cruzamento só é conhecido depois que
`build_network.sh` roda. Se o SUMO reclamar de tamanho de string incompatível
com o número de links, abra `intersection.net.xml` (ou use `netedit`) para
ver o `state` real gerado automaticamente e ajuste `tls_fixed.add.xml` de
acordo. Vou te ajudar a corrigir isso assim que você rodar e me mandar o erro
(se houver).

## Próxima etapa

Com isso validado, o próximo passo é o **script Python com TraCI** que:
- inicia essa mesma simulação programaticamente,
- lê filas/pedestres/ambulância a cada passo,
- aplica a política fixa (lendo os tempos de `config.json`),
- gera os logs em `logs/`.
