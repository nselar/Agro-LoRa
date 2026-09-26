# Graph Report - Agro-LoRa  (2026-09-27)

## Corpus Check
- 3 files · ~260,890 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 68 nodes · 191 edges · 7 communities detected
- Extraction: 100% EXTRACTED · 0% INFERRED · 0% AMBIGUOUS
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Community 0|Community 0]]
- [[_COMMUNITY_Community 1|Community 1]]
- [[_COMMUNITY_Community 2|Community 2]]
- [[_COMMUNITY_Community 3|Community 3]]
- [[_COMMUNITY_Community 4|Community 4]]
- [[_COMMUNITY_Community 5|Community 5]]
- [[_COMMUNITY_Community 6|Community 6]]

## God Nodes (most connected - your core abstractions)
1. `loop()` - 17 edges
2. `renderDisplay()` - 13 edges
3. `processJoin()` - 11 edges
4. `processButton()` - 10 edges
5. `taskRealTime()` - 10 edges
6. `setup()` - 10 edges
7. `dispLine()` - 9 edges
8. `pushAlert()` - 7 edges
9. `dispBig()` - 7 edges
10. `renderPageInfo()` - 7 edges

## Surprising Connections (you probably didn't know these)
- `loop()` --calls--> `verifyHMACSync()`  [EXTRACTED]
  Nodo_Caseta_Vacon/src/main.cpp → Nodo_Sector/src/main.cpp
- `processJoin()` --calls--> `hmacReg()`  [EXTRACTED]
  Nodo_Caseta_Vacon/src/main.cpp → Nodo_Caseta/src/main.cpp
- `processJoin()` --calls--> `verifyJoin()`  [EXTRACTED]
  Nodo_Caseta_Vacon/src/main.cpp → Nodo_Caseta/src/main.cpp
- `processJoin()` --calls--> `pushAlert()`  [EXTRACTED]
  Nodo_Caseta_Vacon/src/main.cpp → Nodo_Caseta/src/main.cpp
- `taskDisplay()` --calls--> `renderDisplay()`  [EXTRACTED]
  Nodo_Caseta/src/main.cpp → Nodo_Sector/src/main.cpp

## Communities

### Community 0 - "Community 0"
Cohesion: 0.25
Nodes (13): accionarValvula(), accionarValvulaManual(), gatewayConnected(), goToSleep(), performJoin(), readBatteryV(), renderPageInfo(), sendStatus() (+5 more)

### Community 1 - "Community 1"
Cohesion: 0.37
Nodes (13): checkNodeTimeouts(), computeHMAC(), computeHMACJoin(), computeHMACRegister(), loop(), processJoin(), processNodeStatus(), sendLoRaCommand() (+5 more)

### Community 2 - "Community 2"
Cohesion: 0.4
Nodes (10): dispBig(), dispLine(), renderDisplay(), renderMenuManual(), renderPageAlerts(), renderPageManual(), renderPageNodes(), renderPageSectors() (+2 more)

### Community 3 - "Community 3"
Cohesion: 0.36
Nodes (8): BLYNK_WRITE(), gwSetFlag(), handleValveBlynk(), _hmac(), hmacJoin(), hmacPacket(), hmacReg(), verifyJoin()

### Community 4 - "Community 4"
Cohesion: 0.32
Nodes (8): anySectorActive(), btnLongPress(), btnShortPress(), computeVisibleSectors(), displaySleep(), displayWake(), processButton(), taskDisplay()

### Community 5 - "Community 5"
Cohesion: 0.43
Nodes (7): processStatus(), pushAlert(), readOptocouplers(), sendCmd(), taskRealTime(), txAndWaitAck(), vfdReadCb()

### Community 6 - "Community 6"
Cohesion: 0.5
Nodes (5): isNightHour(), pushValveStates(), sendSyncToAll(), taskConnectivity(), valveVpin()

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `loop()` connect `Community 1` to `Community 0`, `Community 2`, `Community 3`, `Community 4`?**
  _High betweenness centrality (0.182) - this node is a cross-community bridge._
- **Why does `processJoin()` connect `Community 1` to `Community 3`, `Community 5`?**
  _High betweenness centrality (0.071) - this node is a cross-community bridge._
- **Why does `setup()` connect `Community 0` to `Community 1`, `Community 2`, `Community 3`?**
  _High betweenness centrality (0.069) - this node is a cross-community bridge._