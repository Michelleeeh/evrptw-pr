#pragma once
// ============================================================================
//  Ruta.h
//  Representa una ruta completa (secuencia de paradas) junto con sus
//  metricas de distancia, carga y violaciones de las restricciones del
//  problema (ventanas de tiempo, autonomia de bateria y capacidad de carga).
// ============================================================================

#include <vector>
#include "Nodo.h"

struct Ruta {
    std::vector<Parada> secuencia;
    double distancia_total = 0;
    double carga_total = 0;

    double violacion_tiempo = 0;
    double violacion_bateria = 0;
    double violacion_capacidad = 0;

    /// Calcula la energia necesaria desde la posicion `idx` de la secuencia
    /// hasta el proximo punto de recarga (estacion o deposito).
    double bateria_necesaria_hasta_proximo_punto(int idx, const std::vector<Nodo>& mapa,
        const std::vector<std::vector<double>>& matriz_dist,
        double r_consumo, int id_deposito) const;

    /// Recorre la secuencia de la ruta y recalcula distancia, carga y
    /// violaciones de tiempo/bateria/capacidad.
    /// Devuelve 0 si la ruta es completamente factible, o un codigo distinto
    /// (1: bateria, 2: tiempo, 3: capacidad) apenas se detecta la primera
    /// violacion, cortando el calculo en ese punto.
    int calcular_metricas(const std::vector<Nodo>& mapa,
        const std::vector<std::vector<double>>& matriz_dist,
        double cap_batt, double cap_carga,
        double r_carga, double r_consumo, int id_deposito);
};
