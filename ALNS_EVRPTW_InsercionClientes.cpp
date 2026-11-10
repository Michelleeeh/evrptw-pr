// ============================================================================
//  ALNS_EVRPTW_InsercionClientes.cpp
//  Operadores de insercion (reparacion) de clientes.
// ============================================================================

#include "ALNS_EVRPTW.h"
#include <limits>
#include <cmath>
#include <algorithm>

using namespace std;

void ALNS_EVRPTW::greedy_insertion(vector<int>& fuera, bool con_ruido) {
    reset_cache(fuera, solucion_actual.size());
    reconstruir_indice_rutas();

    while (!fuera.empty()) {
        double mejor_costo = numeric_limits<double>::max();
        int mejor_c_idx = -1, mejor_r = -1, mejor_p = -1;
        for (int i = 0; i < (int)fuera.size(); ++i) {
            int u = fuera[i];
            vector<int> rutas_cand = rutas_candidatas_para(u);
            for (int r : rutas_cand) {
                int mejor_pos_local = -1;
                double costo_ruta = mejor_insercion_en_ruta(u, r, mejor_pos_local, con_ruido);
                if (costo_ruta < mejor_costo) {
                    mejor_costo = costo_ruta; mejor_c_idx = i; mejor_r = r; mejor_p = mejor_pos_local;
                }
            }
        }
        if (mejor_c_idx != -1) {
            if (aplicar_insercion(mejor_r, mejor_p, fuera[mejor_c_idx])) {
                ruta_de_nodo[fuera[mejor_c_idx]] = mejor_r;
                invalidar_cache_ruta(fuera, mejor_r);
                fuera.erase(fuera.begin() + mejor_c_idx);
            }
            else cache_costos[fuera[mejor_c_idx]][mejor_r] = -2.0;
        }
        else {
            if (config.ampliar_busqueda_antes_de_nueva_ruta && insertar_en_cualquier_ruta(fuera)) continue;
            crear_nueva_ruta(fuera[0]);
            reset_cache(fuera, solucion_actual.size());
            ruta_de_nodo.resize(mapa.size(), -1);
            ruta_de_nodo[fuera[0]] = solucion_actual.size() - 1;
            fuera.erase(fuera.begin());
        }
    }
    limpiar_rutas_vacias();
}

void ALNS_EVRPTW::regret_insertion(vector<int>& fuera, int k) {
    reset_cache(fuera, solucion_actual.size());
    reconstruir_indice_rutas();
    const double INF = numeric_limits<double>::max();
    vector<double> mejores; // k menores costos (INF si no hay)

    while (!fuera.empty()) {
        double max_regret = -1.0;
        int best_c_idx = -1, best_r = -1, best_p = -1;
        for (int i = 0; i < (int)fuera.size(); ++i) {
            int u = fuera[i];
            mejores.assign(k, INF);
            int local_best_r = -1, local_best_p = -1;

            vector<int> rutas_cand = rutas_candidatas_para(u);
            for (int r : rutas_cand) {
                int mejor_pos_local = -1;
                double costo_ruta = mejor_insercion_en_ruta(u, r, mejor_pos_local);
                if (costo_ruta < mejores[0]) { local_best_r = r; local_best_p = mejor_pos_local; }
                // insertar ordenado en los k mejores
                for (int j = 0; j < k; ++j) {
                    if (costo_ruta < mejores[j]) { for (int m = k - 1; m > j; --m) mejores[m] = mejores[m - 1]; mejores[j] = costo_ruta; break; }
                }
            }
            double min_costo = mejores[0];
            double regret;
            if (min_costo == INF) regret = 999999.0;
            else {
                regret = 0.0;
                for (int j = 1; j < k; ++j) regret += (mejores[j] == INF ? 999999.0 : mejores[j] - min_costo);
            }
            if (regret > max_regret) {
                max_regret = regret; best_c_idx = i;
                best_r = local_best_r; best_p = local_best_p;
            }
        }
        if (best_r != -1) {
            if (aplicar_insercion(best_r, best_p, fuera[best_c_idx])) {
                ruta_de_nodo[fuera[best_c_idx]] = best_r;
                invalidar_cache_ruta(fuera, best_r);
                fuera.erase(fuera.begin() + best_c_idx);
            }
            else cache_costos[fuera[best_c_idx]][best_r] = -2.0;
        }
        else {
            if (config.ampliar_busqueda_antes_de_nueva_ruta && insertar_en_cualquier_ruta(fuera)) continue;
            crear_nueva_ruta(fuera[0]);
            reset_cache(fuera, solucion_actual.size());
            ruta_de_nodo.resize(mapa.size(), -1);
            ruta_de_nodo[fuera[0]] = solucion_actual.size() - 1;
            fuera.erase(fuera.begin());
        }
    }
    limpiar_rutas_vacias();
}

void ALNS_EVRPTW::time_based_insertion(vector<int>& fuera) {
    struct Guard { bool& f; Guard(bool& f_, bool v) : f(f_) { f = v; } ~Guard() { f = false; } } guard(modo_costo_tiempo, config.insercion_por_tiempo);
    reset_cache(fuera, solucion_actual.size());
    reconstruir_indice_rutas();
    while (!fuera.empty()) {
        double mejor_costo_global = numeric_limits<double>::max();
        int mejor_c_idx = -1, mejor_r = -1, mejor_p = -1;
        for (int i = 0; i < (int)fuera.size(); ++i) {
            int u = fuera[i];
            vector<int> rutas_cand = rutas_candidatas_para(u);
            for (int r : rutas_cand) {
                int mejor_pos_local = -1;
                double costo_ruta = mejor_insercion_en_ruta(u, r, mejor_pos_local);
                if (costo_ruta < mejor_costo_global) {
                    mejor_costo_global = costo_ruta; mejor_c_idx = i; mejor_r = r; mejor_p = mejor_pos_local;
                }
            }
        }
        if (mejor_c_idx != -1) {
            if (aplicar_insercion(mejor_r, mejor_p, fuera[mejor_c_idx])) {
                ruta_de_nodo[fuera[mejor_c_idx]] = mejor_r;
                invalidar_cache_ruta(fuera, mejor_r);
                fuera.erase(fuera.begin() + mejor_c_idx);
            }
            else cache_costos[fuera[mejor_c_idx]][mejor_r] = -2.0;
        }
        else {
            if (config.ampliar_busqueda_antes_de_nueva_ruta && insertar_en_cualquier_ruta(fuera)) continue;
            crear_nueva_ruta(fuera[0]);
            reset_cache(fuera, solucion_actual.size());
            ruta_de_nodo.resize(mapa.size(), -1);
            ruta_de_nodo[fuera[0]] = solucion_actual.size() - 1;
            fuera.erase(fuera.begin());
        }
    }
    limpiar_rutas_vacias();
}

void ALNS_EVRPTW::zone_insertion(vector<int>& fuera) {
    struct Guard { bool& f; Guard(bool& f_, bool v) : f(f_) { f = v; } ~Guard() { f = false; } } guard(modo_costo_tiempo, config.insercion_por_tiempo);
    if (solucion_actual.empty()) {
        while (!fuera.empty()) { crear_nueva_ruta(fuera[0]); fuera.erase(fuera.begin()); }
        return;
    }
    vector<pair<double, double>> centroides(solucion_actual.size(), { 0,0 });
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        int nodos_ruta = 0;
        for (const auto& p : solucion_actual[r].secuencia) {
            if (!mapa[p.nodo_id].es_deposito) {
                centroides[r].first += mapa[p.nodo_id].x;
                centroides[r].second += mapa[p.nodo_id].y;
                nodos_ruta++;
            }
        }
        if (nodos_ruta > 0) {
            centroides[r].first /= nodos_ruta;
            centroides[r].second /= nodos_ruta;
        }
        else centroides[r] = { mapa[ID_DEPOSITO].x, mapa[ID_DEPOSITO].y };
    }
    int ruta_semilla = rng() % solucion_actual.size();
    vector<pair<double, int>> dist_rutas;
    for (int r = 0; r < (int)solucion_actual.size(); ++r) {
        double dx = centroides[r].first - centroides[ruta_semilla].first;
        double dy = centroides[r].second - centroides[ruta_semilla].second;
        dist_rutas.push_back({ sqrt(dx * dx + dy * dy), r });
    }
    sort(dist_rutas.begin(), dist_rutas.end());
    int num_rutas_zona = max(1, (int)solucion_actual.size() / 2);
    vector<int> rutas_zona;
    if (config.zonas_por_cuadricula) {
        // Paper: solo las rutas que pasan por una zona al azar de la cuadricula.
        int z = zona_aleatoria_no_vacia();
        for (int r = 0; r < (int)solucion_actual.size() && z != -1; ++r)
            for (const auto& p : solucion_actual[r].secuencia)
                if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito && zona_de(p.nodo_id) == z) { rutas_zona.push_back(r); break; }
    }
    if (rutas_zona.empty())
        for (int i = 0; i < num_rutas_zona; ++i) rutas_zona.push_back(dist_rutas[i].second);

    // Cambio 1: zone_insertion tambien usa la cache (cliente, ruta). Ademas,
    // marcar -2 cuando aplicar_insercion falla corrige un cuelgue latente del
    // original, que volvia a elegir el mismo par (cliente, ruta, pos) fallido
    // en cada vuelta sin salir nunca del while.
    reset_cache(fuera, solucion_actual.size());

    while (!fuera.empty()) {
        double mejor_costo_global = numeric_limits<double>::max();
        int mejor_c_idx = -1, mejor_r = -1, mejor_p = -1;
        for (int i = 0; i < (int)fuera.size(); ++i) {
            int u = fuera[i];
            for (int r : rutas_zona) {
                int mejor_pos_local = -1;
                double costo_ruta = mejor_insercion_en_ruta(u, r, mejor_pos_local);
                if (costo_ruta < mejor_costo_global) {
                    mejor_costo_global = costo_ruta; mejor_c_idx = i; mejor_r = r; mejor_p = mejor_pos_local;
                }
            }
        }
        if (mejor_c_idx != -1) {
            if (aplicar_insercion(mejor_r, mejor_p, fuera[mejor_c_idx])) {
                invalidar_cache_ruta(fuera, mejor_r);
                fuera.erase(fuera.begin() + mejor_c_idx);
            }
            else cache_costos[fuera[mejor_c_idx]][mejor_r] = -2.0;
        }
        else {
            if (config.ampliar_busqueda_antes_de_nueva_ruta && insertar_en_cualquier_ruta(fuera)) continue;
            crear_nueva_ruta(fuera[0]); fuera.erase(fuera.begin());
            reset_cache(fuera, solucion_actual.size());
        }
    }
    limpiar_rutas_vacias();
}
