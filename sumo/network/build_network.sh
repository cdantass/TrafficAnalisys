#!/bin/bash
# Gera intersection.net.xml a partir dos arquivos .nod.xml / .edg.xml / .typ.xml
# Requer SUMO instalado e SUMO_HOME configurado (netconvert vem com o SUMO).
#
# Uso:
#   cd sumo/network
#   ./build_network.sh

set -e

netconvert \
    --node-files=intersection.nod.xml \
    --edge-files=intersection.edg.xml \
    --type-files=intersection.typ.xml \
    --connection-files=intersection.con.xml \
    --sidewalks.guess=true \
    --crossings.guess=true \
    --walkingareas=true \
    --tls.guess-signals=true \
    --tls.default-type=static \
    --output-file=intersection.net.xml

echo "Rede gerada: intersection.net.xml"