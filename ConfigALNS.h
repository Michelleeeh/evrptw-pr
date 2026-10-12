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
};