# Optimización del ALNS para EVRPTW-PR — cambios implementados

Objetivo: obtener la aceleración descrita en *Sugerencias de optimización del
ALNS para EVRPTW-PR* **sin alterar las decisiones del algoritmo** (misma
trayectoria, mismas rutas). Todo lo que sigue se verificó contra el código
original compilado con `g++ -O2`, comparando byte a byte los archivos
`trayectoria_*.csv` y `rutas_*_ALNS.csv` de corridas con la misma semilla.

## Resumen

| Cambio del informe | Qué se hizo | ¿Altera decisiones? |
|---|---|---|
| 1. Caché de inserción con posición | Implementado en `greedy_insertion`, `regret_insertion`, `time_based_insertion` y `zone_insertion` | **No.** Idéntico byte a byte, incluido el greedy con ruido |
| 2. Energía hasta el próximo punto de carga en una pasada hacia atrás | **Descartado** (ver abajo) | — |
| 3. Probar solo las estaciones cercanas (poda top-k, heurística) | Implementado pero **desactivado por defecto**; sustituido por un **prefiltro exacto** | Poda: sí. Prefiltro: **no**, idéntico a la búsqueda exhaustiva |
| 4. Evaluar la inserción sin copiar la ruta | Implementado (`Ruta::calcular_metricas_con_insercion_virtual`) | **No.** 120.000 comparaciones contra insertar de verdad, 0 diferencias |
| 5. Tolerancia unificada de batería | Implementado (`EPS_BATERIA` en `Ruta.h`, usada en los 4 sitios) + corrección del cuelgue de `zone_insertion` | **Sí, a propósito**: evita insertar estaciones redundantes cuando la batería queda en −1e−15 |

Aceleración medida (2.000 iteraciones, semilla 111, un núcleo, `g++ -O2`):

| Instancia | Original | Nuevo | Aceleración | Trayectoria vs. búsqueda exhaustiva |
|---|---|---|---|---|
| rc104_21 | 58,6 s | 7,2 s | 8,2× | idéntica |
| c101_21 | 16,7 s | 2,4 s | 6,9× | idéntica |
| r101_21 | 17,6 s | 3,1 s | 5,7× | idéntica |

Corrida completa del equipo (rc104_21, semilla 111, 25.000 iteraciones):
11 vehículos, distancia 1.196,85 (el informe reporta 1.197,81 para el original
en 27 min). La diferencia se debe únicamente al cambio 5.

## Detalle por cambio

### Cambio 1 — caché con posición (`cache_pos`, `cache_incs`)

* `mejor_insercion_en_ruta(u, r, pos, con_ruido)` centraliza la lógica: si la
  entrada `(u, r)` es válida se devuelve costo y posición cacheados; si no, se
  evalúa, se guarda y se devuelve. Los cuatro operadores de inserción la usan.
* `invalidar_cache_ruta(fuera, r)` borra la columna de la ruta que acaba de
  cambiar; `reset_cache` limpia también `cache_pos` y `cache_incs`.
* **Greedy con ruido.** El informe acepta que con ruido la trayectoria cambia
  porque el ruido de un par queda congelado. Aquí no: `cache_incs[u][r]`
  guarda el incremento *sin ruido* de cada posición, y en cada vuelta se
  aplica ruido fresco posición por posición, en el mismo orden y con las
  mismas extracciones del generador que el código original. Resultado:
  trayectoria idéntica también para este operador.
* `zone_insertion` no usaba la caché y, si `aplicar_insercion` fallaba, volvía
  a elegir el mismo par en cada vuelta sin salir nunca del `while` (defecto
  citado en la conclusión 4 del informe). Ahora usa la caché y marca el par
  como infactible (−2) al fallar.

### Cambio 2 — pasada hacia atrás: descartado

Se implementó y se midió, y se descartó por dos razones:

1. **No es exacto.** Sumar los tramos en orden inverso cambia el último bit
   de la energía necesaria. En r101_21 (semilla 111, iteración 155) eso dio
   batería `0` al llegar al depósito donde el original daba `−7,1e−15`; el
   operador de estaciones del original (que compara con 0) insertaba una
   estación redundante y la versión nueva no, y desde ahí las trayectorias
   divergen. Sin el cambio 5, esa divergencia llevó a la versión nueva al
   cuelgue descrito en el informe.
2. **No ahorra trabajo.** Cada tramo se recorría una sola vez por evaluación
   (una llamada por estación, cada una recorre solo su tramo); el 20 % del
   perfil se debe a que `calcular_metricas` se llama 1.600 millones de veces,
   no a trabajo repetido. El propio informe mide que el cambio 2 *empeoró* el
   tiempo (79 s → 91 s) y que su ganancia es "pequeña".

Se mantiene `bateria_necesaria_hasta_proximo_punto` con el orden de suma
original. La versión virtual (cambio 4) usa exactamente el mismo recorrido.

### Cambio 3 — poda top-k: desactivada; prefiltro exacto en su lugar

La poda del informe está implementada (`estaciones_candidatas_arco`,
`ConfigALNS::usar_topk_en_probar_insercion`, `k_estaciones_arco`,
`topk_fallback_exhaustivo`), pero **desactivada por defecto** porque:

* Con la red de seguridad (repetir la búsqueda exhaustiva cuando la poda no
  encuentra nada) resultó **más lenta** que sin poda (22,5 s vs 20,7 s en
  rc104_21): el 89 % de las búsquedas de estación no encuentran ninguna
  reparación factible, y en esos casos se hace la poda *y* la exhaustiva.
* Sin red de seguridad, k = 3 perdió +1,0 % de distancia en rc104_21 /
  semilla 111 (1.216,4 → 1.228,2).

Instrumentando `probar_insercion` en rc104_21 (500 iteraciones) se vio que de
65,7 millones de evaluaciones (posición, estación) solo el 0,8 % es factible,
y que las fallas se concentran en tres causas:

| Causa | Evaluaciones | % |
|---|---|---|
| No se alcanza la estación desde la parada anterior (batería) | 22,0 M | 33 % |
| Desde la estación no se llega al siguiente punto de carga (batería) | 17,7 M | 27 % |
| El desvío hace llegar tarde al cliente siguiente (ventana) | 12,4 M | 19 % |

Las tres se detectan en O(1) con cotas válidas para **cualquier** política de
carga, sin recorrer la ruta:

* (a) `energía consumida desde el último punto de carga hasta A[p−1]` +
  `d(A[p−1], s)·r` > `CAP_BATT` ⇒ ni con batería llena se llega a s.
* (b) `d(s, A[p])·r` + `energía desde A[p] hasta el próximo punto de carga`
  > `CAP_BATT` ⇒ ni saliendo llena de s se llega al próximo punto de carga.
* (c) con el tiempo mínimo de salida de A[p−1] (el que se tendría sin cargar
  nada en ninguna estación; cargar solo suma), llegar a s o a A[p] viola su
  `due_date`.

Las cotas se aplican con un margen de 1e−6 (varios órdenes de magnitud por
encima del error de redondeo), así que **nunca descartan un candidato que
`calcular_metricas` aceptaría**. Verificación:

* Compilando con `-DVERIFICAR_PREFILTRO` cada candidato descartado se evalúa
  de todos modos y el programa aborta si era factible. En rc104_21, r101_21 y
  c101_21 (400 iteraciones cada una) se verificaron 97 millones de candidatos
  descartados; ninguno era factible.
* Las trayectorias con prefiltro y con búsqueda exhaustiva son idénticas byte
  a byte en las tres instancias.

El prefiltro se controla con `ConfigALNS::usar_prefiltro_exacto` (true).

### Cambio 4 — inserción virtual

`Ruta::calcular_metricas_con_insercion_virtual(…, pos_u, u, pos_s, s, dist,
idx_falla, estado)` recorre la secuencia real tratando a `u` (y opcionalmente
a la estación `s`) como paradas virtuales. No escribe en las paradas ni copia
nada; devuelve estado, distancia y, si falló por batería, el índice de la
parada. Opcionalmente rellena un `EstadoVirtual` con las cotas que usa el
prefiltro. Se eliminó `buffer_insercion`.

### Cambio 5 — tolerancia

`constexpr double EPS_BATERIA = 1e-9` en `Ruta.h`, usada en
`calcular_metricas`, `aplicar_insercion`, `greedy_station_insertion`,
`greedy_station_insertion_comparison` y `best_station_insertion`.

Efecto medido (2.000 iteraciones, 3 instancias × 3 semillas): 6 de 9 corridas
dan exactamente el resultado del original; las 3 que difieren lo hacen en
ambos sentidos (r101/111: 19 vehículos vs 20 del original; r101/222: 20 vs 19;
rc104/222: 1.231,5 vs 1.261,3). Es variación de semilla, no una tendencia.

## Uso

```
make                      # compila ./alns (variante "paper" por defecto, ver main.cpp)
./alns                    # menú interactivo (como antes)
./alns rc104_21 25000 111 # no interactivo: instancia, iteraciones, semilla(s)
make test_equivalencia    # test de Ruta contra la implementación original
make alns_verificar       # binario con verificación del prefiltro
make test_redistribucion  # test de la política de carga del paper
```

Los parámetros de la optimización (`usar_prefiltro_exacto`,
`usar_topk_en_probar_insercion`, `k_estaciones_arco`,
`topk_fallback_exhaustivo`) se fijan en `ConfigALNS`, igual que el resto de la
configuración; en `main.cpp` hay líneas comentadas de ejemplo.

## Qué conviene hacer con el tiempo ahorrado

Como las decisiones no cambian, las 25.000 iteraciones producen la misma
solución que antes, solo 8× más rápido. Para convertir eso en mejores
soluciones hay que gastar el tiempo: más semillas (en los 27 min de la corrida
original caben ~25 corridas) o más iteraciones. Si se aumentan las
iteraciones hay que subir `alpha_enfriamiento` (con 0,9994 la temperatura es
~0 a las 10.000 iteraciones), por ejemplo 0,9998 para 75.000 iteraciones.


# Alineación con el paper (Keskin & Çatay, 2016)

Con el código acelerado se corrieron las 29 instancias tipo 1 a 25.000
iteraciones y se compararon con la Tabla 4 del paper (columna *PR q free*).
Con el algoritmo original, 9 instancias quedaban con un vehículo de más, y en
8 de ellas eso no cambiaba con 6 semillas ni con más frecuencia del segmento
de remoción de rutas: era un límite del algoritmo, no ruido.

Comparando el código con la sección 4 del paper se encontraron cuatro
diferencias, que ahora están implementadas detrás de flags en `ConfigALNS`
(todas apagadas por defecto; `configurar_como_paper()` las activa). Con los
flags apagados el comportamiento es byte a byte el original.

| Pieza | Flag | Qué cambia |
|---|---|---|
| Búsqueda en todas las rutas antes de abrir una nueva | `ampliar_busqueda_antes_de_nueva_ruta` | `rutas_candidatas_para` solo mira las rutas de los 25 vecinos más cercanos; si en ninguna cabía el cliente, el original abría una ruta nueva sin probar el resto. Medido en c107: ~3 rutas nuevas por iteración evitables |
| **Redistribución de carga** | `redistribuir_carga` | Política de carga del paper: si la duración de la carga en una estación hace llegar tarde a un cliente, se traslada carga a la estación anterior, limitado por la capacidad de la batería y las ventanas intermedias. Implementada en el núcleo único de `Ruta.cpp` (real y virtual comparten la misma función). Solo se activa cuando la pasada con carga mínima falla por tiempo tras la segunda estación |
| Time Based Insertion y Zone Insertion por tiempo | `insercion_por_tiempo` | En el código, `time_based_insertion` era una copia del greedy por distancia; el paper lo define como incremento del tiempo total de la ruta, y lo diseña para empacar clientes compatibles (menos vehículos) |
| Regret-3 | `usar_regret3` | Operador adicional (la ruleta pasa a 6 operadores de inserción solo si está activo) |
| Parámetros de la Tabla A.2 | `sigma3 = 12`, `N_s = 5500`, `m_r = 0.3` | El código usaba 9, 2500 y 0,4 |

## Cuál importa: la política de carga

Ablación (semilla 111) en las tres instancias donde la configuración paper
recupera el vehículo:

| | Solo redistribución de carga | Solo inserciones por tiempo + regret-3 | Todo |
|---|---|---|---|
| c103 | **10** / 971,6 | 11 | 10 |
| r104 | 12 | 12 | **11** |
| rc102 | **14** / 1527,1 | 15 | 14 |

El dato que lo explica: en 7 de las 8 instancias rebeldes, el número de
vehículos del código original coincidía exactamente con el que el paper
obtiene **con recarga completa** (columna FR de su Tabla 4), y el vehículo que
faltaba es el que los autores ganan con la recarga parcial. La política de
"cargar lo mínimo para llegar al siguiente punto de carga" es, en rutas con dos
estaciones y ventanas ajustadas, peor que cargar completo: llega casi vacío a
la segunda estación y necesita allí una carga larga donde no hay holgura.

## Resultados: variante paper, semilla 111, 25.000 iteraciones

| Instancia | Paper (PR) | Original | Variante paper | Variante paper vs. paper |
|---|---|---|---|---|
| c101 | 12 / 1051.23 | 12 / 1043.4 | 12 / 1043.4 | -0.74 % |
| c102 | 11 / 1034.24 | 11 / 1038.1 | 11 / 1035.8 | +0.15 % |
| c103 | 10 / 973.39 | 11 / 982.6 | 10 / 971.6 | -0.18 % |
| c104 | 10 / 886.72 | 10 / 899.8 | 10 / 892.5 | +0.65 % |
| c105 | 11 / 1037.78 | 11 / 1081.2 | 11 / 1056.5 | +1.80 % |
| c106 | 11 / 1024.18 | 11 / 1072.6 | 11 / 1044.1 | +1.94 % |
| c107 | 10 / 1058.11 | 11 / 1022.8 | 11 / 1012.2 | +1 veh |
| c108 | 10 / 1033.50 | 11 / 1010.4 | 11 / 1000.6 | +1 veh |
| c109 | 10 / 960.03 | 10 / 971.7 | 10 / 973.4 | +1.39 % |
| r101 | 18 / 1661.33 | 19 / 1651.2 | 18 / 1619.5 | -2.52 % |
| r102 | 16 / 1461.48 | 16 / 1455.6 | 16 / 1451.6 | -0.68 % |
| r103 | 13 / 1262.75 | 14 / 1253.5 | 14 / 1251.0 | +1 veh |
| r104 | 11 / 1078.99 | 12 / 1066.9 | 11 / 1084.5 | +0.51 % |
| r105 | 15 / 1373.94 | 15 / 1394.0 | 15 / 1399.2 | +1.84 % |
| r106 | 13 / 1310.46 | 14 / 1282.7 | 14 / 1272.1 | +1 veh |
| r107 | 12 / 1118.91 | 12 / 1130.7 | 12 / 1115.6 | -0.30 % |
| r108 | 11 / 1031.14 | 11 / 1029.5 | 11 / 1023.9 | -0.70 % |
| r109 | 13 / 1201.04 | 13 / 1229.3 | 13 / 1197.8 | -0.27 % |
| r110 | 11 / 1112.80 | 12 / 1084.6 | 12 / 1106.3 | +1 veh |
| r111 | 12 / 1084.13 | 12 / 1084.8 | 12 / 1093.1 | +0.83 % |
| r112 | 11 / 1017.31 | 11 / 1018.7 | 11 / 1005.3 | -1.18 % |
| rc101 | 16 / 1684.84 | 15 / 1781.3 | 15 / 1705.4 | menos veh |
| rc102 | 14 / 1555.50 | 15 / 1530.1 | 14 / 1522.8 | -2.10 % |
| rc103 | 13 / 1329.58 | 13 / 1382.7 | 13 / 1369.3 | +2.99 % |
| rc104 | 11 / 1202.93 | 11 / 1196.9 | 11 / 1182.8 | -1.67 % |
| rc105 | 14 / 1458.49 | 14 / 1494.3 | 14 / 1471.9 | +0.92 % |
| rc106 | 13 / 1422.96 | 13 / 1481.0 | 14 / 1429.9 | +1 veh |
| rc107 | 12 / 1261.03 | 12 / 1265.7 | 12 / 1273.7 | +1.00 % |
| rc108 | 11 / 1185.68 | 11 / 1197.3 | 11 / 1172.9 | -1.08 % |

Resumen (una semilla):

| | Original | Variante paper |
|---|---|---|
| Menos vehículos que el paper | 1 | 1 |
| Misma flota | 19 | 22 |
| …de las cuales con menor distancia | 4 | 11 |
| Un vehículo de más | 9 | 6 |
| Desviación media de distancia (misma flota) | +1,43 % | +0,12 % |

La variante paper mejora o iguala a la original en 22 de 29 instancias. Las 6
que siguen con un vehículo de más (c107, c108, r103, r106, r110, rc106) son
resultados de una semilla; antes, con la configuración original, r101 y rc102
cayeron con semillas distintas, así que el siguiente paso es correrlas con
3–5 semillas. Nota: rc102 figura en el paper como 1155,50, errata evidente de
1555,50 (con recarga completa reportan 1551,69).

Costo: la redistribución de carga duplica el tiempo en instancias con muchas
estaciones (r104: 66 s → 134 s por corrida de 25.000 iteraciones). Sigue
siendo 10–20× más rápido que el código original.

Verificación: `make test_redistribucion` comprueba sobre 200.000 rutas por
instancia que la redistribución nunca quita factibilidad a una ruta que la
tenía, que toda ruta rescatada es consistente al releer sus paradas (ventanas,
batería no negativa, salida ≤ capacidad, tiempos de carga) y que la
evaluación virtual coincide con la real también con la redistribución activa.

## Las 6 instancias restantes con 5 semillas

| Instancia | Flota por semilla (111, 222, 333, 444, 555) | Mejor de 5 | Paper (PR) | |
|---|---|---|---|---|
| c107 | 11, 11, 11, 11, 11 | 11 / 1012.2 | 10 / 1058.11 | **+1 vehículo en las 5 semillas** |
| c108 | 11, 11, 11, 11, 11 | 11 / 1000.6 | 10 / 1033.50 | **+1 vehículo en las 5 semillas** |
| r103 | 14, 14, 14, 14, 14 | 14 / 1232.5 | 13 / 1262.75 | **+1 vehículo en las 5 semillas** |
| r106 | 14, 14, 14, 13, 14 | 13 / 1283.8 | 13 / 1310.46 | misma flota, -2.0 % distancia |
| r110 | 12, 12, 12, 12, 12 | 12 / 1082.3 | 11 / 1112.80 | **+1 vehículo en las 5 semillas** |
| rc106 | 14, 14, 13, 14, 13 | 13 / 1411.9 | 13 / 1422.96 | misma flota, -0.8 % distancia |

Dos grupos: r106 y rc106 alcanzan la flota del paper con una semilla concreta
y la superan en distancia; c107, c108, r103 y r110 dan la misma flota en las
cinco semillas, lo que ya no es ruido sino un límite residual del algoritmo.
En c107 y c108 la distancia de nuestras soluciones con 11 vehículos (1012,
1001) es bastante menor que la del paper con 10 (1058, 1034): el paper acepta
rutas más largas a cambio del vehículo. Hipótesis pendientes de probar: (a) la
redistribución de carga solo actúa cuando la pasada mínima falla por tiempo;
no cubre el fallo de batería cuando la carga quedó limitada por el tiempo
disponible en la estación; (b) la reparación de batería en `probar_insercion`
y `aplicar_insercion` inserta como máximo una estación, mientras que el GSI
del paper se repite para cada parada con batería negativa.

Contando el mejor de las semillas corridas, la variante paper iguala o supera
al paper en 16 de 29 instancias en flota y distancia, iguala la flota con
distancia dentro de ±3 % en 9, y queda con un vehículo de más en 4.

## Segunda ronda de alineación: operadores restantes

Se añadieron, también detrás de flags y activados por `configurar_como_paper()`:
Zone Removal y Zone Insertion con cuadrícula cartesiana de n_z = 25 zonas
(`zonas_por_cuadricula`), Proximity-, Demand- y Time-Based Removal como
operadores propios (`operadores_shaw_extra`, casos de Shaw con
φ = (1,0,0,0), (0,0,0,1), (0,1,0,0)), evaluación de la inserción en todas las
rutas (`evaluar_todas_las_rutas`) y desactivación del greedy con ruido, que no
está en el paper (`usar_greedy_con_ruido = false`). Con los flags apagados el
comportamiento sigue siendo byte a byte el original.

Efecto medido en las 4 instancias que quedaban con un vehículo de más
(25.000 iteraciones):

| Instancia | Variante paper (ronda 1) | + operadores restantes | Paper (PR) |
|---|---|---|---|
| c107 | 11 / 1012,2 (5 sem.) | 11 / 1012,2 (3 sem.) | 10 / 1058,1 |
| c108 | 11 / 1000,6 (5 sem.) | 11 / 993,6 (3 sem.) | 10 / 1033,5 |
| r103 | 14 / 1232,5 (5 sem.) | 14 / 1232,5 (1 sem.) | 13 / 1262,8 |
| r110 | 12 / 1082,3 (5 sem.) | 12 / 1091,6 (1 sem.) | 11 / 1112,8 |

Ningún cambio de flota; distancias equivalentes. Evaluar todas las rutas
encarece la corrida (c107: 88 s; r110: 208 s). Conclusión: una vez
incorporada la política de carga, los operadores que faltaban no aportan
vehículos; lo que queda es variación entre semillas y la fase de set
covering con CP-SAT (ver nota final). Si el tiempo importa, se puede dejar
`evaluar_todas_las_rutas = false` (la búsqueda ampliada antes de abrir ruta
ya cubre el caso que importaba) sin pérdida medible.

## Tercera ronda: rango de posiciones para la estación de reparación

El código original probaba la estación de reparación solo en las 5 posiciones
anteriores al fallo de batería (`MAX_POS = 5`, fijo), y el GSI que repara
tras remover estaciones solo probaba el arco del fallo. El paper no limita:
GSI "intenta los arcos anteriores" y Best Station Insertion considera todos
los arcos hasta el punto de carga anterior. Ahora ambos son configurables
(`max_posiciones_estacion`, 0 = hasta el punto de carga anterior;
`gsi_retrocede_arcos`) y `configurar_como_paper()` usa el comportamiento del
paper. Medido en las 4 instancias pendientes (semilla 111): c107 11 / 1012,0,
c108 11 / 994,2, r103 14 / 1235,1, r110 12 / 1102,0; y en c107 con
`max_posiciones_estacion` = 3 y 10: 11 vehículos en ambos. El rango no
cambia la flota.

## Conjetura sobre la política de cantidad de carga (probada)

Pista en el paper: tiene operadores *Full Charge Station Removal* ("aims at
promoting the PR option") y *Worst-Charge Usage Station Removal*, que solo
tienen sentido si en sus soluciones PR muchas estaciones cargan al máximo; y
presenta PR como "relajar la restricción de carga completa". Con la política
del código ("lo mínimo para llegar al siguiente punto de carga") eso casi
nunca ocurre. Conjetura: el paper carga **lo máximo que permiten la batería y
las ventanas posteriores** en cada estación intermedia, y reduce solo donde
el tiempo de carga rompe una ventana. Esa política domina a la mínima en
factibilidad (toda ruta factible con carga mínima sigue siéndolo).

Implementado en `Ruta::politica_carga` (`ConfigALNS::politica_carga`):
0 = mínima; 1 = mínima + redistribución tras fallo (defecto hasta ahora);
2 = adelantar siempre a cada estación lo que necesitará la siguiente, dentro
de la holgura de los nodos intermedios; 3 = la 2 más carga adicional hasta
capacidad dentro de la holgura de todo lo posterior. `make test_politica_carga`
verifica sobre 200.000 rutas que 2 y 3 nunca quitan factibilidad, que las
paradas son consistentes y que virtual = real.

Resultado (semilla 111, 25.000 iteraciones, resto de la configuración paper):

| Instancia | Política 1 (10 semillas) | Política 2 | Política 3 | Paper (PR) |
|---|---|---|---|---|
| c107 | 11 ×10 | — | 11 / 1025,1 | 10 / 1058,1 |
| c108 | 11 ×10 | 11 / 993,6 | **10 / 1048,6** (semilla 222: 11 / 995,9) | 10 / 1033,5 |
| r103 | 14 ×10 | — | 14 / 1250,7 | 13 / 1262,8 |
| r110 | 12 ×10 | — | 12 / 1096,9 | 11 / 1112,8 |

c108 baja a 10 vehículos por primera vez en 13 corridas, y solo con la
política 3 (la 2, sin top-up, no lo logra): es el "cargar más de lo necesario
cuando hay holgura" lo que abre la región de 10 vehículos, lo que apoya la
conjetura. Con otra semilla vuelve a 11, así que hace falta medirla con más
semillas (y es ~2× más lenta por las pasadas adicionales: r110 204 s).
Tras medirla con 10 semillas en c107 y 8 en c108 (c108: 10 vehículos en 2 de 8;
ninguna instancia empeoró), queda activada por defecto en `configurar_como_paper()`.

## Cuarta ronda: solución inicial del Algoritmo 1 y re-ajuste del enfriamiento

Se añadió la construcción secuencial de la solución inicial del paper
(`solucion_inicial_secuencial`, activada en `configurar_como_paper()`) y se
probó el enfriamiento dentro del rango de la Tabla A.2, con política de carga 3:

| Instancia | α = 0,9994 | α = 0,9998 | α = 0,9999 | α = 0,9998, 50.000 it. | Paper |
|---|---|---|---|---|---|
| c107 | 11 / 1015,8 | 11 / 1009,3 · 11 / 1015,2 | 11 / 1018,9 | 11 / 1009,3 | 10 / 1058,1 |
| r103 | — | 14 / 1246,5 | 14 / 1259,6 | — | 13 / 1262,8 |
| r110 | — | 12 / 1086,3 | — | — | 11 / 1112,8 |

Sin cambio de flota. Con todo lo descrito en el paper implementado, 10–20
semillas, tres políticas de carga, tres enfriamientos y el doble de
iteraciones, estas tres instancias convergen de forma estable a un vehículo
más que el paper y con distancias claramente menores que las suyas. Lo que
queda sin replicar es la política exacta de cantidad de carga (A2 en el
listado de fidelidad) y detalles de implementación que el paper no documenta.

## Descartado: inserción forzada con penalización adaptativa

Se probó insertar el cliente violando ventanas (penalización adaptativa al
estilo Schneider) en vez de abrir ruta nueva. La solución actual pasaba el
35–40 % del tiempo infactible y no bajó la flota en ninguna instancia; no se
incluye en el entregable.

# Nota sobre el set covering con CP-SAT

Los resultados de la versión anterior del equipo (carpeta `EVRPTW_PR_Analisis`)
que superaban al paper en vehículos (r107 y r111 con 11, c108 y c103 con 10,
r106 y rc106 con 13) **no los obtenía el ALNS sino la fase final de set
covering resuelta con CP-SAT** sobre el pool de rutas: ganó la comparación en
26 de 34 corridas y bajó un vehículo en 11. Todas las mediciones de este
documento se hicieron con el binario compilado sin OR-Tools, donde esa fase
cae a un fallback greedy que nunca mejora. Comparando ALNS contra ALNS, la
variante paper es igual o mejor que la versión anterior en todas las
instancias revisadas.

Para recuperar esa fase: compilar con `USE_ORTOOLS` (como hacía la versión
anterior). Como alternativa, el programa exporta ahora el pool de rutas a
`resultados/<instancia>/pool_<instancia>.csv`, y `tests/setcover_cpsat.py`
resuelve el set covering en dos fases con OR-Tools desde Python
(`pip install ortools`; uso: `python setcover_cpsat.py c108_21 resultados/c108_21`).
Con el ALNS acelerado conviene además unir los pools de varias semillas antes
de resolver: el set covering se beneficia de la diversidad de rutas.
