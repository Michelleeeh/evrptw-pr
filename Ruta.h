#pragma once
// ============================================================================
//  Ruta.h
//  Representa una ruta completa (secuencia de paradas) junto con sus
//  metricas de distancia, carga y violaciones de las restricciones del
//  problema (ventanas de tiempo, autonomia de bateria y capacidad de carga).
// ============================================================================

#include <vector>
#include "Nodo.h"

/// Tolerancia de redondeo unica para decidir si una llegada con bateria
/// (ligeramente) negativa cuenta como violacion. Debe usarse en TODOS los
/// sitios que buscan la parada con fallo de bateria, de modo que
/// calcular_metricas y los operadores de insercion de estaciones esten de
/// acuerdo sobre que es "factible". (Cambio 5: evita un cuelgue cuando el
/// redondeo deja la bateria en -1e-15.)
constexpr double EPS_BATERIA = 1e-9;

/// Estado auxiliar por parada que calcular_metricas_con_insercion_virtual
/// puede rellenar (opcional) mientras recorre la ruta. Lo usa probar_insercion
/// para descartar en O(1) candidatos de estacion que con seguridad fallan.
struct EstadoVirtual {
    // energia consumida desde el ultimo punto de recarga (estacion/deposito)
    // hasta llegar a la parada i; 0 en los propios puntos de recarga.
    std::vector<double> energia_desde_ultimo_punto_carga;
    // cota inferior del tiempo de salida de la parada i: tiempo que se
    // tendria si no se cargara nada en ninguna estacion (cargar solo suma).
    std::vector<double> tiempo_salida_minimo;
};

struct Ruta {
    /// Politica de carga del paper (Keskin & Catay): si la duracion de la
    /// carga en una estacion hace llegar tarde a un cliente posterior, se
    /// traslada parte de esa carga a la estacion anterior, aprovechando los
    /// tiempos de espera intermedios y la capacidad de la bateria. Solo se
    /// activa cuando la pasada con carga minima falla por tiempo tras la
    /// segunda estacion; en todo otro caso la evaluacion es identica a la
    /// original. Interruptor global (una sola hebra) fijado desde ConfigALNS.
    static bool redistribuir_carga;
    /// Politica de cantidad de carga (0 = minima; 1 = minima + redistribucion
    /// solo tras fallo por tiempo [= redistribuir_carga]; 2 = adelantar siempre
    /// a cada estacion lo que necesitara la siguiente, dentro de la holgura;
    /// 3 = la 2 mas carga adicional hasta capacidad dentro de la holgura de
    /// todo lo posterior). Ver CAMBIOS.md, "Conjetura sobre la politica de carga".
    static int politica_carga;

    std::vector<Parada> secuencia;
    double distancia_total = 0;
    double carga_total = 0;

    double violacion_tiempo = 0;
    double violacion_bateria = 0;
    double violacion_capacidad = 0;

    /// Calcula la energia necesaria desde la posicion `idx` de la secuencia
    /// hasta el proximo punto de recarga (estacion o deposito), recorriendo
    /// hacia adelante y sumando los tramos en orden.
    ///
    /// NOTA sobre el "cambio 2" del informe de optimizacion (pasada hacia
    /// atras que rellena todas las paradas de una vez): se probo y se
    /// descarto. Cada tramo ya se recorre una sola vez por evaluacion, asi
    /// que no ahorra trabajo (el propio informe mide 0 s de ganancia), y al
    /// sumar los tramos en orden inverso cambia el ultimo bit de la energia
    /// necesaria (p.ej. 0 en vez de -7e-15 al llegar al deposito), lo que
    /// altera decisiones de los operadores de estaciones y hace que la
    /// trayectoria deje de ser identica a la original. Se mantiene el orden
    /// de suma original para que el resultado sea bit a bit el mismo.
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

    /// Cambio 4: evalua la ruta COMO SI el cliente `u` estuviera insertado en
    /// la posicion `pos_u` y, opcionalmente (pos_s >= 0), la estacion `s` en
    /// la posicion `pos_s` de la secuencia resultante tras insertar `u`
    /// (misma convencion que insertar primero u y luego s en el vector).
    /// No modifica la ruta ni copia la secuencia. Devuelve el mismo codigo
    /// de estado que calcular_metricas, la distancia total en `distancia` y,
    /// si fallo por bateria, el indice (en la secuencia virtual) de la parada
    /// donde fallo en `idx_falla` (-1 en otro caso).
    int calcular_metricas_con_insercion_virtual(const std::vector<Nodo>& mapa,
        const std::vector<std::vector<double>>& matriz_dist,
        double cap_batt, double cap_carga,
        double r_carga, double r_consumo, int id_deposito,
        int pos_u, int u, int pos_s, int s,
        double& distancia, int& idx_falla,
        EstadoVirtual* estado = nullptr,
        double* tiempo_fin = nullptr) const;
};
