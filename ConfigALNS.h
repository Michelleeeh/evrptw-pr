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
};
