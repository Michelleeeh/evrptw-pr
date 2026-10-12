// ============================================================================
//  ALNS_EVRPTW_FRVCP.cpp
//  Fixed-Route Vehicle Charging Problem (FRVCP) resuelto con programacion
//  dinamica para el caso de recarga lineal parcial (EVRPTW-PR, Keskin 2016).
//
//  Dado un orden fijo de clientes (sin estaciones), decide optimamente:
//    - entre que par de clientes consecutivos insertar una estacion,
//    - cual estacion elegir,
//    - cuanta energia cargar,
//  minimizando la distancia total y respetando ventanas de tiempo,
//  capacidad de bateria y restriccion de recarga parcial.
//
//  La DP discretiza el nivel de bateria en pasos de PASO_BATERIA unidades.
//  Complejidad por ruta: O(N * B * S * B) con N=clientes, B=niveles,
//  S=estaciones; tipicamente ~10 * 80 * 20 * 80 = 1.3M operaciones.
//
//  Limitacion: inserta a lo sumo UNA estacion por tramo (entre dos clientes
//  consecutivos). Esto es suficiente para la gran mayoria de instancias de
//  Keskin; si un tramo necesita mas de una recarga, se conserva la ruta
//  original.
// ============================================================================

#include "ALNS_EVRPTW.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;

// Paso de discretizacion de bateria (unidades de energia).
static const double PASO_BATERIA = 1.0;

// ============================================================================
//  resolver_frvcp
// ============================================================================
Ruta ALNS_EVRPTW::resolver_frvcp(const vector<int>& clientes) {
    if (clientes.empty()) return Ruta{};

    // ----- 1. Construir secuencia: deposito + clientes + deposito -----
    const int n_cli = (int)clientes.size();
    vector<int> seq;
    seq.reserve(n_cli + 2);
    seq.push_back(ID_DEPOSITO);
    for (int c : clientes) seq.push_back(c);
    seq.push_back(ID_DEPOSITO);
    const int N = (int)seq.size();

    // ----- 2. Discretizar bateria -----
    const int niveles = (int)(CAP_BATT / PASO_BATERIA) + 2;

    // Tablas de la DP: separadas en arrays planos para garantizar
    // inicializacion explicita (evita problemas con MSVC Release).
    const int TOTAL = N * niveles;
    vector<double> dp_dist(TOTAL, 1e18);
    vector<double> dp_tiempo(TOTAL, 0.0);
    vector<int>    dp_prev_b(TOTAL, -1);
    vector<int>    dp_estacion(TOTAL, -1);

    // Macro de acceso lineal: fila i, columna b.
    #define IDX(i, b) ((i) * niveles + (b))

    // Caso base: salir del deposito con bateria llena a tiempo 0.
    int b0 = min((int)(CAP_BATT / PASO_BATERIA), niveles - 1);
    dp_dist[IDX(0, b0)]   = 0.0;
    dp_tiempo[IDX(0, b0)] = 0.0;

    // ----- 3. DP hacia adelante -----
    for (int i = 0; i < N - 1; ++i) {
        const int id_curr = seq[i];
        const int id_next = seq[i + 1];
        const Nodo& nodo_next = mapa[id_next];

        for (int b = 0; b < niveles; ++b) {
            const int idx_src = IDX(i, b);
            if (dp_dist[idx_src] >= 1e17) continue;   // estado no alcanzado

            const double acum  = dp_dist[idx_src];
            const double t_sal = dp_tiempo[idx_src];
            const double bat   = b * PASO_BATERIA;

            // Lambda para registrar transicion si mejora el destino.
            auto intentar = [&](int b_next, double nueva_dist, double t_dep_next, int prev_b, int est) {
                if (b_next < 0 || b_next >= niveles) return;
                const int idx_dst = IDX(i + 1, b_next);
                if (nueva_dist < dp_dist[idx_dst] - 1e-9 ||
                    (nueva_dist < dp_dist[idx_dst] + 1e-9 && t_dep_next < dp_tiempo[idx_dst] - 1e-9)) {
                    dp_dist[idx_dst]     = nueva_dist;
                    dp_tiempo[idx_dst]   = t_dep_next;
                    dp_prev_b[idx_dst]   = prev_b;
                    dp_estacion[idx_dst] = est;
                }
            };

            // ---- Opcion A: viaje DIRECTO ----
            {
                double d = matriz_dist[id_curr][id_next];
                double bat_arr = bat - d * R_CONSUMO;
                if (bat_arr >= -1e-9) {
                    bat_arr = max(0.0, bat_arr);
                    double t_arr = t_sal + d;
                    if (t_arr <= nodo_next.due_date + 1e-9) {
                        double t_ini = max(t_arr, nodo_next.ready_time);
                        double t_dep = t_ini + (nodo_next.es_deposito ? 0.0 : nodo_next.service_time);
                        int b_next = (int)(bat_arr / PASO_BATERIA);
                        intentar(b_next, acum + d, t_dep, b, -1);
                    }
                }
            }

            // ---- Opcion B: desvio por estacion s ----
            for (int s : estaciones) {
                double d1 = matriz_dist[id_curr][s];
                double bat_s = bat - d1 * R_CONSUMO;
                if (bat_s < -1e-9) continue;
                bat_s = max(0.0, bat_s);

                const Nodo& nodo_s = mapa[s];
                double t_arr_s = t_sal + d1;
                if (t_arr_s > nodo_s.due_date + 1e-9) continue;
                double t_ini_s = max(t_arr_s, nodo_s.ready_time);

                double d2 = matriz_dist[s][id_next];
                double e2 = d2 * R_CONSUMO;
                double dist_desvio = d1 + d2;

                double max_t_carga = max(0.0, nodo_s.due_date - t_ini_s);
                double max_carga   = min(max_t_carga / R_CARGA, CAP_BATT - bat_s);
                if (max_carga < -1e-9) continue;
                max_carga = max(0.0, max_carga);

                int bt_min = max((int)ceil(e2 / PASO_BATERIA), (int)ceil(bat_s / PASO_BATERIA));
                int bt_max = min((int)((bat_s + max_carga) / PASO_BATERIA), niveles - 1);
                if (bt_min > bt_max) continue;

                for (int bt = bt_min; bt <= bt_max; ++bt) {
                    double target = bt * PASO_BATERIA;
                    double carga  = target - bat_s;
                    if (carga < -1e-9) continue;
                    carga = max(0.0, carga);

                    double t_dep_s = t_ini_s + carga * R_CARGA;
                    double bat_arr_next = target - e2;
                    if (bat_arr_next < -1e-9) continue;
                    bat_arr_next = max(0.0, bat_arr_next);

                    double t_arr_next = t_dep_s + d2;
                    if (t_arr_next > nodo_next.due_date + 1e-9) continue;

                    double t_ini_next = max(t_arr_next, nodo_next.ready_time);
                    double t_dep_next = t_ini_next + (nodo_next.es_deposito ? 0.0 : nodo_next.service_time);

                    int b_next = (int)(bat_arr_next / PASO_BATERIA);
                    intentar(b_next, acum + dist_desvio, t_dep_next, b, s);
                }
            }
        }
    }

    // ----- 4. Mejor estado final -----
    int best_b = -1;
    double best_dist = 1e17;
    for (int b = 0; b < niveles; ++b) {
        if (dp_dist[IDX(N - 1, b)] < best_dist) {
            best_dist = dp_dist[IDX(N - 1, b)];
            best_b = b;
        }
    }
    if (best_b == -1) return Ruta{};

    // ----- 5. Reconstruir la ruta -----
    // traza_est[k] = estacion usada en el tramo k-1 -> k (-1 = directo)
    vector<int> traza_est(N, -1);
    {
        int cur_b = best_b;
        for (int i = N - 1; i > 0; --i) {
            const int idx = IDX(i, cur_b);
            traza_est[i] = dp_estacion[idx];
            int pb = dp_prev_b[idx];
            if (pb < 0 || pb >= niveles) return Ruta{}; // proteccion
            cur_b = pb;
        }
    }

    Ruta resultado;
    resultado.secuencia.reserve(N + (int)estaciones.size());
    resultado.secuencia.push_back({ ID_DEPOSITO, 0, CAP_BATT, 0, 0 });

    for (int k = 1; k < N; ++k) {
        if (traza_est[k] != -1) {
            resultado.secuencia.push_back({ traza_est[k], 0, 0, 0, 0 });
        }
        resultado.secuencia.push_back({ seq[k], 0, 0, 0, 0 });
    }

    int status = resultado.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
    if (status != 0) return Ruta{};

    #undef IDX
    return resultado;
}

// ============================================================================
//  optimizar_solucion_con_frvcp
// ============================================================================
int ALNS_EVRPTW::optimizar_solucion_con_frvcp(vector<Ruta>& sol) {
    int mejoradas = 0;
    for (auto& ruta : sol) {
        vector<int> clientes;
        for (const auto& p : ruta.secuencia) {
            if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito)
                clientes.push_back(p.nodo_id);
        }
        if (clientes.empty()) continue;

        Ruta optimizada = resolver_frvcp(clientes);
        if (optimizada.secuencia.empty()) continue;

        if (optimizada.distancia_total < ruta.distancia_total - 1e-9) {
            ruta = optimizada;
            mejoradas++;
        }
    }
    return mejoradas;
}

// ============================================================================
//  optimizar_pool_con_frvcp
//  NOTA: rp.clientes esta ordenado por ID (sort en registrar_rutas_en_pool),
//  NO en orden de visita. Para el FRVCP necesitamos el orden de ruta, asi
//  que extraemos los clientes de rp.secuencia_completa.
// ============================================================================
int ALNS_EVRPTW::optimizar_pool_con_frvcp() {
    int mejoradas = 0;
    for (auto& rp : pool_rutas) {
        if (rp.clientes.empty()) continue;

        // Extraer clientes en ORDEN DE VISITA (no el orden por ID de rp.clientes).
        vector<int> clientes_en_orden;
        for (const auto& p : rp.secuencia_completa) {
            if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito)
                clientes_en_orden.push_back(p.nodo_id);
        }
        if (clientes_en_orden.empty()) continue;

        Ruta optimizada = resolver_frvcp(clientes_en_orden);
        if (optimizada.secuencia.empty()) continue;

        if (optimizada.distancia_total < rp.distancia - 1e-9) {
            rp.distancia = optimizada.distancia_total;
            rp.secuencia_completa = optimizada.secuencia;
            mejoradas++;
        }
    }
    return mejoradas;
}
