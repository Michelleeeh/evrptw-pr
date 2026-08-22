#pragma once
// ============================================================================
//  Nodo.h
//  Estructuras basicas de datos: Nodo (deposito/cliente/estacion) y Parada
//  (estado de una parada dentro de la secuencia de una ruta).
// ============================================================================

#include <string>

/// Representa un nodo del problema: puede ser el deposito, un cliente o una
/// estacion de recarga.
struct Nodo {
    int id;
    std::string tipo;
    double x, y;
    double demanda;
    double ready_time;
    double due_date;
    double service_time;
    bool es_estacion;
    bool es_deposito;   // Booleano precalculado para evitar comparar strings repetidamente.
};

/// Representa una parada dentro de la secuencia de una ruta, con el estado
/// de bateria y tiempo al llegar y al salir de ese nodo.
struct Parada {
    int nodo_id;
    double llegada_bateria;
    double salida_bateria;
    double llegada_tiempo;
    double salida_tiempo;
};
