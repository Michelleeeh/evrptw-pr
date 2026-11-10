#pragma once
// ============================================================================
//  ConfigALNS.h
//  Parametros de configuracion de una corrida del ALNS: criterio de
//  aceptacion, tamano de temperatura inicial, que operadores usar y cada
//  cuantas iteraciones se ejecutan los distintos segmentos del algoritmo.
// ============================================================================

#include <string>

struct ConfigALNS {
    std::string nombre_variante = "baseline";
    unsigned int semilla = 12345;

    enum class Criterio { SIMULATED_ANNEALING, SOLO_MEJORA } criterio = Criterio::SIMULATED_ANNEALING;

    double mu = 0.40;
    double alpha_enfriamiento = 0.9994;

    bool usar_regret_insertion = true;
    bool usar_shaw_removal = true;
    bool usar_segmento_estaciones = true;
    bool usar_segmento_rutas = true;
    bool usar_ruleta_adaptativa = true;

    int N_SR = 60;
    int N_RR = 2000;
    int n_RR = 1250;
    int N_c = 200;
    int N_s = 2500;

    // FRVCP (ver ALNS_EVRPTW_FRVCP.cpp): reoptimiza estaciones/carga sobre
    // una secuencia de clientes ya fija. Se puede activar por separado en
    // cada punto de enganche para medir el efecto de cada uno.
    bool usar_frvcp_en_final = false;  // aplicar sobre mejor_solucion al terminar
    bool usar_frvcp_en_pool = false;   // aplicar sobre el pool antes del set-covering
    int frvcp_k_candidatas = 5;       // estaciones candidatas evaluadas por tramo

    // ------------------------------------------------------------------
    //  Prefiltro EXACTO de estaciones en probar_insercion (ver comentario
    //  en ALNS_EVRPTW_Auxiliares.cpp). Descarta en O(1), con cotas y margen
    //  de seguridad, los candidatos (posicion, estacion) que con certeza
    //  fallan por bateria o ventana de tiempo. No altera ninguna decision:
    //  los resultados son identicos a los de la busqueda exhaustiva.
    // ------------------------------------------------------------------
    bool usar_prefiltro_exacto = true;

    // ------------------------------------------------------------------
    //  Cambio 3 del informe (poda HEURISTICA top-k). Desactivado por
    //  defecto: es el unico cambio que puede alterar las decisiones del
    //  algoritmo, y con el prefiltro exacto ya no hace falta para obtener
    //  la aceleracion. Se deja disponible por si se quiere experimentar.
    //  Al ESTIMAR el costo de insertar un cliente cuya ruta falla por
    //  bateria, prueba solo las k estaciones que menos desvio anaden al
    //  arco donde se insertarian (en vez de las 21). Al APLICAR la
    //  insercion elegida se sigue usando la busqueda exhaustiva.
    // ------------------------------------------------------------------
    bool usar_topk_en_probar_insercion = false; // true = poda heuristica (NO exacta)
    int k_estaciones_arco = 5;                  // 3 = mas rapido; 5 = menor perdida medida
    // Si con las k estaciones no se encuentra ninguna reparacion factible,
    // repetir la busqueda con todas las estaciones antes de declarar la
    // insercion infactible. Garantiza que la poda nunca descarte una
    // insercion que si era factible (solo puede sobreestimar su costo).
    bool topk_fallback_exhaustivo = true;

    // ------------------------------------------------------------------
    //  Piezas del metodo de Keskin & Catay (2016) que faltaban en el codigo.
    //  Todas apagadas por defecto (= comportamiento original). Ver CAMBIOS.md,
    //  seccion "Alineacion con el paper", y configurar_como_paper().
    // ------------------------------------------------------------------
    // Antes de abrir una ruta nueva porque ningun cliente pendiente cabe en
    // sus rutas candidatas (las de sus K_VECINOS vecinos), probar TODAS las
    // rutas. El paper evalua todas las rutas siempre; el codigo original solo
    // las candidatas, y abria una ruta nueva sin mirar el resto.
    bool ampliar_busqueda_antes_de_nueva_ruta = false;
    // Politica de carga del paper (ver Ruta::redistribuir_carga).
    bool redistribuir_carga = false;
    // Politica de cantidad de carga (ver Ruta::politica_carga). 1 = redistribuir_carga.
    int politica_carga = 1;
    // Time Based Insertion / Zone Insertion con criterio de tiempo de ruta
    // (como en el paper) en vez de copia del greedy por distancia.
    bool insercion_por_tiempo = false;
    // Regret-3 como operador adicional (el paper usa regret-2 y regret-3).
    bool usar_regret3 = false;
    // Puntajes de la ruleta adaptativa (paper: 25 / 20 / 12).
    double sigma1 = 25, sigma2 = 20, sigma3 = 9;
    // Cota superior de la proporcion de rutas removidas por RRR/GRR (paper m_r = 0.3).
    double m_r = 0.4;

    /// Activa todas las piezas anteriores con los parametros ajustados del
    /// paper (Tabla A.2). Es la configuracion que mejor replica sus resultados.
    // Zone Removal / Zone Insertion con cuadricula cartesiana de n_z zonas
    // (paper: n_z = 25) en vez de rutas cercanas por centroide.
    bool zonas_por_cuadricula = false;
    int n_zonas = 25;
    // Proximity-, Demand- y Time-Based Removal como operadores propios
    // (casos particulares de Shaw con phi = (1,0,0,0), (0,0,0,1), (0,1,0,0)).
    bool operadores_shaw_extra = false;
    // Evaluar la insercion en TODAS las rutas (paper) en vez de solo las de
    // los K_VECINOS vecinos mas cercanos del cliente.
    bool evaluar_todas_las_rutas = false;
    // Greedy con ruido: no esta en el paper. false lo desactiva (se mapea al greedy sin ruido).
    bool usar_greedy_con_ruido = true;
    // Posiciones hacia atras desde el fallo de bateria en las que se prueba
    // insertar la estacion de reparacion (probar_insercion / aplicar_insercion).
    // 0 = hasta el punto de carga anterior (deposito o estacion), como en el
    // paper. El codigo original usaba 5 fijo.
    int max_posiciones_estacion = 5;
    // GSI del paper: si en el arco donde falla la bateria no cabe ninguna
    // estacion, intentar los arcos anteriores. El original solo probaba ese arco.
    bool gsi_retrocede_arcos = false;
    // Solucion inicial segun el Algoritmo 1 del paper: construccion secuencial
    // (una ruta a la vez, empezando por el cliente mas cercano al deposito, y
    // se abre otra solo cuando ningun cliente cabe). El original usaba el
    // greedy paralelo sobre todos los clientes.
    bool solucion_inicial_secuencial = false;

    void configurar_como_paper() {
        ampliar_busqueda_antes_de_nueva_ruta = true;
        zonas_por_cuadricula = true;
        operadores_shaw_extra = true;
        evaluar_todas_las_rutas = true;
        usar_greedy_con_ruido = false;
        max_posiciones_estacion = 0;
        gsi_retrocede_arcos = true;
        solucion_inicial_secuencial = true;
        politica_carga = 3;
        redistribuir_carga = true;
        insercion_por_tiempo = true;
        usar_regret3 = true;
        sigma3 = 12;
        N_s = 5500;
        m_r = 0.3;
    }
};