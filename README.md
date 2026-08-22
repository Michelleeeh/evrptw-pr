# ALNS para el EVRPTW con Recargas Parciales

Implementación en C++ de un **Adaptive Large Neighborhood Search (ALNS)** para el
**Electric Vehicle Routing Problem with Time Windows (EVRPTW)**, permitiendo
recargas parciales en las estaciones (EVRPTW-PR).

> **Base teórica:** el diseño del ALNS (operadores de destrucción/reparación,
> pesos adaptativos, criterio de aceptación por Simulated Annealing) es una
> **réplica no oficial** — con algunas simplificaciones y decisiones de
> implementación propias — del algoritmo propuesto en:
>
> Keskin, M., & Çatay, B. (2016). *Partial Recharge Strategies for the
> Electric Vehicle Routing Problem with Time Windows.* Transportation
> Research Part C.
>
> No es una implementación 1:1 del paper: algunos mecanismos se simplificaron,
> otros se combinaron y algunas decisiones de diseño (estructuras de datos,
> caché de costos, formato de instancias) son propias de este proyecto. Si
> algo del comportamiento no coincide exactamente con lo que describe el
> paper, es intencional o producto de una simplificación práctica, no un bug
> por default. Por lo tanto, es recomendable revisar el paper.

---

## 1. El problema: EVRPTW y EVRPTW-PR

### 1.1. Contexto

En el **VRPTW clásico**, una flota de vehículos con capacidad de carga limitada
debe visitar un conjunto de clientes, cada uno con una demanda, una ventana de
tiempo de atención y un tiempo de servicio, minimizando la distancia total
recorrida (o el número de vehículos, como objetivo jerárquico superior).

El **EVRPTW** añade una restricción más: los vehículos son eléctricos (EVs) y
tienen **autonomía limitada** por la capacidad de su batería. La batería se
consume proporcionalmente a la distancia recorrida, y si un vehículo se queda
sin autonomía suficiente para llegar a su próximo destino, debe desviarse a una
**estación de recarga**.

### 1.2. Recarga total vs. recarga parcial

En el planteamiento original de Schneider et al. (2014), cada vez que un
vehículo visita una estación, **siempre se recarga al 100%** — lo cual es
poco realista porque una recarga completa puede tardar mucho más de lo
necesario.

El paper de Keskin y Çatay relaja esa restricción: el vehículo puede
**recargar solo lo que necesita** para completar el resto de su ruta (o
llegar a la próxima estación), ahorrando tiempo de recarga y potencialmente
permitiendo visitar más clientes dentro de sus ventanas de tiempo. Esta
variante se llama **EVRPTW-PR** (*Partial Recharge*).

**Este código implementa la versión con recarga parcial por defecto.** En
`Ruta::calcular_metricas` (`Ruta.cpp`), cuando la ruta pasa por una estación,
se calcula la energía mínima necesaria para llegar al próximo punto de
recarga o al depósito (`bateria_necesaria_hasta_proximo_punto`) y solo se
recarga esa cantidad — limitada además por el tiempo disponible antes del
cierre de la ventana de tiempo del siguiente nodo y por la capacidad máxima
de la batería. Esto corresponde al caso "**q free**" del paper (la cantidad
recargada es una variable de decisión, no un valor fijo).

### 1.3. Elementos del problema

| Concepto | Descripción |
|---|---|
| **Depósito** | Punto de partida y llegada de todas las rutas. |
| **Clientes** | Tienen demanda, ventana de tiempo `[ready_time, due_date]` y tiempo de servicio. |
| **Estaciones de recarga** | Pueden visitarse múltiples veces, por el mismo o distintos vehículos. No tienen demanda. |
| **Batería** | Se consume a una tasa fija por unidad de distancia (`R_CONSUMO`) y se recarga a una tasa fija por unidad de energía (`R_CARGA`). |
| **Capacidad de carga** | Cada vehículo tiene un límite de carga (`CAP_CARGA`) para la suma de demandas de los clientes que atiende. |
| **Objetivo** | Minimizar, en orden jerárquico: (1) número de vehículos usados, (2) distancia total recorrida. |

---

## 2. Cómo funciona el ALNS

La idea central del ALNS es iterar sobre un ciclo de **destruir y reparar**
la solución actual:

1. **Destruir**: remover un subconjunto de clientes (y/o estaciones) de las
   rutas actuales usando un operador de remoción.
2. **Reparar**: reinsertar esos clientes en la mejor posición posible
   (o crear nuevas rutas si no caben) usando un operador de inserción.
3. **Aceptar o rechazar** la nueva solución según un criterio de Simulated
   Annealing.
4. Repetir miles de veces, ajustando dinámicamente qué operadores se usan
   con más frecuencia según qué tan buenos resultados han dado
   (**pesos adaptativos tipo ruleta**).

### 2.1. Construcción de la solución inicial

`inicializar_solucion()` construye una primera solución factible insertando
todos los clientes uno por uno con `greedy_insertion`, creando una ruta
nueva cuando ningún cliente cabe en las rutas existentes.

### 2.2. Operadores de remoción de clientes

| Operador | Archivo | Idea |
|---|---|---|
| `random_removal` | `ALNS_EVRPTW_RemocionClientes.cpp` | Elige clientes al azar. |
| `worst_distance_removal` | ídem | Remueve clientes cuyo retiro ahorra más distancia (con selección sesgada para evitar quedar atrapado en un óptimo local). |
| `worst_time_removal` | ídem | Prioriza clientes cuya llegada se aleja más de su ventana de tiempo ideal. |
| `shaw_removal` | ídem | Remueve clientes "similares" entre sí (por distancia, ventana de tiempo, ruta y demanda) — típico operador de *relatedness* de Shaw (1998) / Ropke & Pisinger. |

### 2.3. Operadores de remoción de rutas completas y por zona

| Operador | Archivo | Idea |
|---|---|---|
| `random_route_removal` | `ALNS_EVRPTW_RemocionRutas.cpp` | Elimina rutas completas al azar. |
| `greedy_route_removal` | ídem | Elimina las rutas con menos clientes (para intentar reducir el número de vehículos). |
| `zone_removal` | ídem | Divide el mapa en zonas geográficas y remueve clientes concentrados en una zona. |
| `customer_preceding_station` / `customer_succeeding_station` | ídem | Remueven un cliente junto con la estación de recarga inmediatamente anterior/posterior, ya que esa recarga puede volverse innecesaria si el cliente se va. |

### 2.4. Operadores de remoción de estaciones

`ALNS_EVRPTW_RemocionEstaciones.cpp`: remoción aleatoria, por peor distancia,
por nivel de batería con el que se llegó (`worst_charge_station`), y de
estaciones donde la batería quedó completamente cargada
(`full_charge_station_removal`, pensado para promover el uso de recarga
parcial).

### 2.5. Operadores de inserción de clientes

`ALNS_EVRPTW_InsercionClientes.cpp`:

- **`greedy_insertion`**: inserta cada cliente en la posición de menor costo
  marginal, iterando hasta que todos estén insertados.
- **`regret_insertion`**: usa el criterio *regret-2* — prioriza insertar
  primero los clientes cuya diferencia entre su mejor y segunda mejor opción
  de inserción es más grande (evita quedar "atrapado" postergando clientes
  difíciles).
- **`time_based_insertion`**: minimiza el incremento en tiempo total de ruta
  en vez de distancia.
- **`zone_insertion`**: como `greedy_insertion`, pero restringido a las rutas
  de una zona geográfica.

Si una inserción es factible en cuanto a capacidad y ventanas de tiempo pero
no en batería, el código intenta reparar automáticamente insertando una
estación de recarga antes del punto de falla (ver `probar_insercion` y
`aplicar_insercion` en `ALNS_EVRPTW_Auxiliares.cpp`).

### 2.6. Operadores de inserción de estaciones

`ALNS_EVRPTW_InsercionEstaciones.cpp`:

- **`greedy_station_insertion`**: busca solo la posición justo antes del fallo de batería y prueba ahí todas las estaciones, quedándose con la de menor costo.
- **`greedy_station_insertion_comparison`**: prueba todas las posiciones desde el fallo hasta el inicio de la ruta con todas las estaciones — la búsqueda más exhaustiva de las tres.
- **`best_station_insertion`**: retrocede solo hasta el último punto de recarga previo (no hasta el inicio de la ruta), un balance entre exhaustividad y velocidad.

### 2.7. Criterio de aceptación (Simulated Annealing)

En `evaluar_y_aceptar` (`ALNS_EVRPTW_Auxiliares.cpp`):

- Si la nueva solución es mejor que la mejor conocida → se acepta y se
  actualiza `mejor_solucion`.
- Si es peor pero mejor que la solución anterior → se acepta.
- Si es peor que la anterior → se acepta con probabilidad
  `exp(-delta / T)`, donde `T` es la temperatura actual, que decrece
  geométricamente en cada iteración (`T *= alpha`).

### 2.8. Pesos adaptativos

Cada operador tiene un peso que determina su probabilidad de ser elegido
(selección tipo ruleta, `seleccionar_operador`). Cada cierto número de
iteraciones (`N_c` para clientes, `N_s` para estaciones) se recalculan los
pesos según qué tan bien le fue a cada operador (`actualizar_pesos_clientes`,
`actualizar_pesos_estaciones`), premiando más a los operadores que llevaron a
una mejor solución global, una mejora local, o una solución peor pero
aceptada por SA.

### 2.9. Fase final: matheurística (set-covering)

Durante toda la búsqueda, cada vez que se encuentra una ruta factible se
guarda en un *pool* (`registrar_rutas_en_pool`). Al terminar las
iteraciones, se resuelve un problema de **set-covering** sobre ese pool
(`ALNS_EVRPTW_SetCovering.cpp`) para intentar combinar las mejores rutas
encontradas en una solución mejor que la que dejó el ALNS por sí solo. Esta
fase **no está en el paper original** — es un añadido propio de este
proyecto para exprimir más valor del historial de búsqueda.

---

## 3. Estructura del código

| Archivo | Contenido |
|---|---|
| `Nodo.h` | `Nodo` (depósito/cliente/estación) y `Parada` (estado de una parada en una ruta). |
| `Ruta.h` / `Ruta.cpp` | Cálculo de factibilidad de una ruta: distancia, batería, tiempo, capacidad. |
| `ConfigALNS.h` | Parámetros de una corrida (semilla, criterio de aceptación, qué operadores usar). |
| `InstanciaReader.h` / `.cpp` | Lectura de instancias tipo Solomon/Schneider. |
| `ALNS_EVRPTW.h` | Declaración completa de la clase principal. |
| `ALNS_EVRPTW_Core.cpp` | Constructor, matriz de distancias, cálculo de costo/distancia total. |
| `ALNS_EVRPTW_RemocionClientes.cpp` | Operadores de remoción de clientes. |
| `ALNS_EVRPTW_RemocionRutas.cpp` | Remoción de rutas completas, por zona, cliente+estación. |
| `ALNS_EVRPTW_RemocionEstaciones.cpp` | Operadores de remoción de estaciones. |
| `ALNS_EVRPTW_InsercionClientes.cpp` | Operadores de inserción de clientes. |
| `ALNS_EVRPTW_InsercionEstaciones.cpp` | Operadores de inserción de estaciones. |
| `ALNS_EVRPTW_Algoritmo.cpp` | `ejecutar()` — el ciclo principal del ALNS — y exportación de resultados a CSV. |
| `ALNS_EVRPTW_SetCovering.cpp` | Matheurística final (greedy o CP-SAT/OR-Tools, ver sección 5). |
| `ALNS_EVRPTW_Auxiliares.cpp` | Funciones privadas de apoyo (construcción inicial, aceptación, pesos, etc.). |
| `main.cpp` | Punto de entrada: recorre instancias, corre el ALNS, guarda resultados. |

---

## 4. Formato de las instancias

El programa lee archivos `.txt` con formato tipo Solomon/Schneider. Cada línea
de nodo tiene la forma:

```
<id> <tipo> <x> <y> <demanda> <ready_time> <due_date> <service_time>
```

donde `tipo` es `d` (depósito), `c` (cliente) o `f` (estación de recarga).
Además, el archivo debe declarar en alguna parte los parámetros del
vehículo/batería con líneas como:

```
Q / 16.0 /      -> capacidad de batería
C / 200.0 /     -> capacidad de carga
r / 1.0 /       -> tasa de consumo de batería por distancia
g / 3.47 /      -> tasa de recarga (tiempo por unidad de energía)
```

Si alguno de estos parámetros no aparece en el archivo, se usan valores por
defecto razonables (ver `InstanciaReader.cpp`).

**Dónde poner las instancias:** crea una carpeta `instancias2/` al mismo
nivel que el `.vcxproj` del proyecto (ver sección "Directorio de trabajo" más
abajo) y coloca ahí los archivos `.txt`.

---

## 5. Cómo compilar

### 5.1. En Visual Studio (recomendado)

1. Crea un proyecto de tipo **Aplicación de consola (C++)**.
2. Agrega todos los `.h` y `.cpp` de esta carpeta al proyecto
   (*Agregar → Elemento existente...*).
3. Configura el estándar de C++ a **C++17 o superior**
   (*Propiedades → C/C++ → Lenguaje → Estándar de lenguaje C++*).
4. Compila (`Ctrl+Shift+B`).

### 5.2. OR-Tools es OPCIONAL

La fase final de matheurística (`ALNS_EVRPTW_SetCovering.cpp`) puede usar
**CP-SAT de OR-Tools** para resolver el set-covering de forma óptima, pero
esto **no es necesario** para trabajar en el ALNS en sí — el archivo compila
sin problema sin OR-Tools instalado, usando automáticamente un fallback
greedy.

- **Si NO vas a tocar el solver**: no hagas nada especial. El proyecto
  compila y corre de forma completamente autónoma. La fase final usará el
  set-covering greedy en vez de CP-SAT — el resto del ALNS (que es lo que te
  interesa) funciona exactamente igual.
- **Si necesitas activar CP-SAT** (por ejemplo, para comparar resultados):
  agrega el símbolo de preprocesador `USE_ORTOOLS` en
  *Propiedades del proyecto → C/C++ → Preprocesador → Definiciones de
  preprocesador*, e instala/enlaza OR-Tools (recomendado vía
  [vcpkg](https://github.com/microsoft/vcpkg):
  `vcpkg install or-tools:x64-windows` seguido de `vcpkg integrate install`).

### 5.3. Directorio de trabajo

El programa busca las instancias en `./instancias2/`, que en
Visual Studio por defecto se resuelve contra la carpeta del `.vcxproj` (no
contra `Debug/` o `x64/Debug/`). Todas las instancias se encuentran dentro de la carpeta 
`./instancias/`, para analizar una instancia en específico se debe copiar en `./instancias2/`.
Puedes confirmarlo o cambiarlo en *Propiedades del proyecto → Depuración → Directorio de trabajo*.
Los resultados se guardan automáticamente en `./resultados/` y como CSVs sueltos
(`rutas_*.csv`, `trayectoria_*.csv`, `comparacion_*.csv`) en ese mismo
directorio.

---

## 6. Qué produce el programa

Al correr `main.cpp`, por cada instancia en `instancias2/` se generan:

| Archivo | Contenido |
|---|---|
| `resultados/resultados_<instancia>.csv` | Resumen: vehículos, distancia, iteración de última mejora, tiempo. |
| `trayectoria_<instancia>_<variante>.csv` | Evolución de la distancia (mejor y actual) y temperatura cada 10 iteraciones. |
| `rutas_<instancia>_ALNS.csv` | Rutas de la mejor solución encontrada por el ALNS puro. |
| `rutas_<instancia>_CPSAT.csv` | Rutas de la solución tras la fase de set-covering. |
| `rutas_<instancia>.csv` | La solución "oficial" final (la mejor entre ALNS y set-covering). |
| `comparacion_<instancia>.csv` | Comparación lado a lado de ambas soluciones. |

Cada fila de un CSV de rutas indica `RutaID, Posicion, NodoID, X, Y, Tipo`,
lo que permite graficar fácilmente las rutas resultantes (por ejemplo con
Excel o Python/matplotlib).

---

## 7. Parámetros principales (`ConfigALNS.h`)

| Parámetro | Rol |
|---|---|
| `semilla` | Semilla del generador aleatorio, para reproducibilidad. |
| `mu` | Controla la temperatura inicial de SA (qué tan probable es aceptar una solución `mu`% peor al inicio). |
| `alpha_enfriamiento` | Tasa de enfriamiento geométrico de la temperatura. |
| `usar_regret_insertion` / `usar_shaw_removal` | Activan/desactivan esos operadores específicos. |
| `N_SR` / `N_RR` / `n_RR` | Cada cuántas iteraciones se ejecuta el segmento de operadores de estaciones / rutas completas, y cuántas veces seguidas. |
| `N_c` / `N_s` | Cada cuántas iteraciones se actualizan los pesos adaptativos de clientes / estaciones. |

El número total de iteraciones se pasa directamente a `ejecutar()` en
`main.cpp` (actualmente `25000`, en línea con lo reportado en el paper como
suficiente para la convergencia).

---

## 8. Referencia

Keskin, M., & Çatay, B. (2016). *Partial Recharge Strategies for the Electric
Vehicle Routing Problem with Time Windows.* Transportation Research Part C:
Emerging Technologies. [Link](https://www.researchgate.net/publication/297584057_Partial_Recharge_Strategies_for_the_Electric_Routing_Problem_with_Time_Windows)

El problema base (EVRPTW sin recarga parcial) fue introducido originalmente
por:

Schneider, M., Stenger, A., & Goeke, D. (2014). *The Electric Vehicle Routing
Problem with Time Windows and Recharging Stations.* Transportation Science,
48(4), 500-520.
