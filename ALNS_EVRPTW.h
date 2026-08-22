#pragma once
// ============================================================================
//  ALNS_EVRPTW.h
//  Declaracion de la clase principal que implementa la metaheuristica ALNS
//  para el EVRPTW (Electric Vehicle Routing Problem with Time Windows).
//
//  La implementacion de los metodos esta repartida en varios .cpp segun
//  responsabilidad:
//    - ALNS_EVRPTW_Core.cpp             -> constructor, cache, metricas basicas
//    - ALNS_EVRPTW_RemocionClientes.cpp -> operadores de remocion de clientes
//    - ALNS_EVRPTW_RemocionRutas.cpp    -> operadores de remocion de rutas / zonas
//    - ALNS_EVRPTW_RemocionEstaciones.cpp -> operadores de remocion de estaciones
//    - ALNS_EVRPTW_InsercionClientes.cpp -> operadores de insercion de clientes
//    - ALNS_EVRPTW_InsercionEstaciones.cpp -> operadores de insercion de estaciones
//    - ALNS_EVRPTW_Algoritmo.cpp         -> ejecutar() y exportacion de resultados
//    - ALNS_EVRPTW_SetCovering.cpp       -> matheuristica (greedy y CP-SAT)
//    - ALNS_EVRPTW_Auxiliares.cpp        -> funciones auxiliares privadas
// ============================================================================

#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <random>
#include <chrono>

#include "Nodo.h"
#include "Ruta.h"
#include "ConfigALNS.h"

/// Clase principal que implementa la metaheuristica ALNS para el EVRPTW:
/// construye la solucion inicial, ejecuta el ciclo de destruccion/reparacion
/// con seleccion adaptativa de operadores, y aplica una fase final de
/// matheuristica sobre el pool de rutas factibles recolectadas.
class ALNS_EVRPTW {
    std::vector<Nodo> mapa;
    std::vector<std::vector<double>> matriz_dist;
    std::vector<int> estaciones;
    ConfigALNS config;
    double T;
    double CAP_CARGA, CAP_BATT, R_CARGA, R_CONSUMO;
    double alpha_tiempo = 10000.0, beta_bateria = 10000.0, gamma_cap = 10000.0;

    std::chrono::high_resolution_clock::time_point tiempo_inicio = std::chrono::high_resolution_clock::now();
    std::vector<Ruta> solucion_actual;

    // Pesos, puntajes y contadores de uso para la seleccion adaptativa
    // (tipo ruleta) de cada familia de operadores.
    std::vector<double> pesos_rem = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
    std::vector<double> scores_rem = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    std::vector<int> usos_rem = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    std::vector<double> pesos_ins = { 1.0, 1.0, 1.0, 1.0, 1.0 };
    std::vector<double> scores_ins = { 0, 0, 0, 0, 0 };
    std::vector<int> usos_ins = { 0, 0, 0, 0, 0 };

    std::vector<double> pesos_sr = { 1.0, 1.0, 1.0, 1.0 };
    std::vector<double> scores_sr = { 0, 0, 0, 0 };
    std::vector<int> usos_sr = { 0, 0, 0, 0 };

    std::vector<double> pesos_si = { 1.0, 1.0, 1.0 };
    std::vector<double> scores_si = { 0, 0, 0 };
    std::vector<int> usos_si = { 0, 0, 0 };

    // Cache de costos de insercion por (cliente, ruta), para evitar
    // recalcular inserciones que no cambiaron entre iteraciones.
    std::vector<std::vector<double>> cache_costos;
    Ruta buffer_insercion;
    std::mt19937 rng;
    std::uniform_real_distribution<double> dist_01;

    // Lista de vecinos mas cercanos por cliente, usada para restringir la
    // busqueda de rutas candidatas durante la insercion.
    std::vector<std::vector<int>> vecinos_cercanos;
    std::vector<int> ruta_de_nodo; // ruta_de_nodo[nodo_id] = indice de ruta donde esta actualmente, -1 si no asignado
    static const int K_VECINOS = 25;

    /// Ruta factible almacenada en el pool para la fase de matheuristica
    /// (set-covering greedy) al final de la ejecucion.
    struct RutaPool {
        std::vector<int> clientes;
        double distancia;
        std::vector<Parada> secuencia_completa;
    };
    std::vector<RutaPool> pool_rutas;
    std::unordered_map<std::string, std::vector<int>> firmas_pool; // conjunto_clientes -> indices en pool_rutas
    static const int K_VARIANTES_POR_GRUPO = 3; // variantes distintas guardadas por cada conjunto de clientes

    int ID_DEPOSITO;

public:
    std::vector<Ruta> mejor_solucion;
    int ultima_mejora = 0;

    /// Construye el gestor ALNS: precalcula la matriz de distancias, la
    /// lista de vecinos mas cercanos por cliente y reserva la cache de
    /// costos de insercion.
    ALNS_EVRPTW(std::vector<Nodo> m, double cc, double cb, double rc, double rcons, ConfigALNS cfg);

    /// Reinicializa a -1 las entradas de la cache de costos de insercion
    /// correspondientes a los nodos recien removidos, sin reconstruir los
    /// vectores desde cero.
    void reset_cache(const std::vector<int>& fuera, int num_rutas);

    // ------------------------------------------------------------------
    //  4. OPERADORES
    // ------------------------------------------------------------------

    // --- 4.1 Operadores de remocion de clientes ---
    void random_removal(std::vector<int>& fuera, int q);
    void worst_distance_removal(std::vector<int>& fuera, int q);
    void worst_time_removal(std::vector<int>& fuera, int q);
    void shaw_removal(std::vector<int>& fuera, int q);

    // --- 4.2 Operadores de remocion de rutas completas ---
    void random_route_removal(std::vector<int>& fuera);
    void greedy_route_removal(std::vector<int>& fuera);

    // --- 4.3 Operadores relacionados con clientes junto a estaciones ---
    void customer_preceding_station(std::vector<int>& fuera, int q);
    void customer_succeding_station(std::vector<int>& fuera, int q);
    void zone_removal(std::vector<int>& fuera, int q);

    // --- 4.4 Operadores de remocion de estaciones de recarga ---
    void random_station_removal(std::vector<int>& fuera, int q);
    void worst_distance_station(std::vector<int>& fuera, int q);
    void worst_charge_station(std::vector<int>& fuera, int q);
    void full_charge_station_removal(std::vector<int>& fuera, int q);

    // --- 4.5 Operadores de insercion de clientes ---
    void greedy_insertion(std::vector<int>& fuera, bool con_ruido = false);
    void regret_insertion(std::vector<int>& fuera);
    void time_based_insertion(std::vector<int>& fuera);
    void zone_insertion(std::vector<int>& fuera);

    // --- 4.6 Operadores de insercion de estaciones de recarga ---
    void greedy_station_insertion();
    void greedy_station_insertion_comparison();
    void best_station_insertion();

    // ------------------------------------------------------------------
    //  5. ALGORITMO PRINCIPAL
    // ------------------------------------------------------------------

    /// Ejecuta el ciclo completo del ALNS durante `iteraciones` pasos y
    /// exporta la trayectoria y las rutas finales a archivos CSV.
    void ejecutar(int iteraciones, std::string nombre_instancia);

    /// Exporta la mejor solucion encontrada a un archivo CSV.
    void exportar_rutas(std::string filename);

    /// Exporta cualquier vector<Ruta> (util para comparar soluciones).
    void exportar_rutas_de(const std::vector<Ruta>& sol, std::string filename);

    /// Resuelve un set-covering greedy sobre el pool de rutas factibles.
    std::vector<Ruta> resolver_greedy_set_covering();

    /// Resuelve el set-covering sobre pool_rutas con CP-SAT en dos fases.
    std::vector<Ruta> resolver_set_covering_cpsat(double tiempo_limite_fase1 = 20.0,
        double tiempo_limite_fase2 = 30.0);

    /// Devuelve la distancia total recorrida por todas las rutas de `sol`.
    double obtener_distancia(const std::vector<Ruta>& sol) const;

    /// Calcula el costo total usado por el criterio de Simulated Annealing.
    double calcular_costo(const std::vector<Ruta>& sol) const;

private:
    // ------------------------------------------------------------------
    //  6. FUNCIONES AUXILIARES INTERNAS
    // ------------------------------------------------------------------

    void inicializar_solucion();
    void crear_nueva_ruta(int nodo_id);
    double probar_insercion(int r_idx, int pos, int u);
    void reconstruir_indice_rutas();
    void registrar_rutas_en_pool(const std::vector<Ruta>& sol);
    std::vector<int> rutas_candidatas_para(int u);
    bool aplicar_insercion(int r_idx, int pos, int nodo_id);
    void evaluar_y_aceptar(std::vector<Ruta>& backup, int op_rem, int op_ins, int op_sr, int op_si, int iteracion_actual);
    void actualizar_rutas_post_remocion(const std::vector<int>& fuera);
    void limpiar_rutas_vacias();
    int seleccionar_operador(const std::vector<double>& pesos);
    void actualizar_pesos_clientes();
    void actualizar_pesos_estaciones();
    bool mejor(const std::vector<Ruta>& A, const std::vector<Ruta>& B) const;
};
