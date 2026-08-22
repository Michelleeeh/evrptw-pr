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
                if (cache_costos[u][r] == -2.0) continue;
                const auto& ruta = solucion_actual[r];
                double costo_ruta = numeric_limits<double>::max();
                int mejor_pos_local = -1;
                if (ruta.carga_total + mapa[u].demanda <= CAP_CARGA) {
                    for (int p = 1; p < (int)ruta.secuencia.size(); ++p) {
                        double inc = probar_insercion(r, p, u);
                        if (con_ruido && inc != numeric_limits<double>::max()) {
                            inc *= (0.8 + 0.4 * dist_01(rng));
                        }
                        if (inc < costo_ruta) { costo_ruta = inc; mejor_pos_local = p; }
                    }
                }
                cache_costos[u][r] = (costo_ruta == numeric_limits<double>::max()) ? -2.0 : costo_ruta;
                if (costo_ruta < mejor_costo) {
                    mejor_costo = costo_ruta; mejor_c_idx = i; mejor_r = r; mejor_p = mejor_pos_local;
                }
            }
        }
        if (mejor_c_idx != -1) {
            if (aplicar_insercion(mejor_r, mejor_p, fuera[mejor_c_idx])) {
                ruta_de_nodo[fuera[mejor_c_idx]] = mejor_r;
                for (int u : fuera) cache_costos[u][mejor_r] = -1.0;
                fuera.erase(fuera.begin() + mejor_c_idx);
            }
            else cache_costos[fuera[mejor_c_idx]][mejor_r] = -2.0;
        }
        else {
            crear_nueva_ruta(fuera[0]);
            reset_cache(fuera, solucion_actual.size());
            ruta_de_nodo.resize(mapa.size(), -1);
            ruta_de_nodo[fuera[0]] = solucion_actual.size() - 1;
            fuera.erase(fuera.begin());
        }
    }
    limpiar_rutas_vacias();
}

void ALNS_EVRPTW::regret_insertion(vector<int>& fuera) {
    reset_cache(fuera, solucion_actual.size());
    reconstruir_indice_rutas();

    while (!fuera.empty()) {
        double max_regret = -1.0;
        int best_c_idx = -1, best_r = -1, best_p = -1;
        for (int i = 0; i < (int)fuera.size(); ++i) {
            int u = fuera[i];
            double min_costo = numeric_limits<double>::max();
            double min_costo_2 = numeric_limits<double>::max();
            int local_best_r = -1, local_best_p = -1;

            vector<int> rutas_cand = rutas_candidatas_para(u);
            for (int r : rutas_cand) {
                if (cache_costos[u][r] == -2.0) continue;
                const auto& ruta = solucion_actual[r];
                double costo_ruta = numeric_limits<double>::max();
                int mejor_pos_local = -1;
                if (ruta.carga_total + mapa[u].demanda <= CAP_CARGA) {
                    for (int p = 1; p < (int)ruta.secuencia.size(); ++p) {
                        double inc = probar_insercion(r, p, u);
                        if (inc < costo_ruta) { costo_ruta = inc; mejor_pos_local = p; }
                    }
                }
                cache_costos[u][r] = (costo_ruta == numeric_limits<double>::max()) ? -2.0 : costo_ruta;
                if (costo_ruta < min_costo) {
                    min_costo_2 = min_costo; min_costo = costo_ruta;
                    local_best_r = r; local_best_p = mejor_pos_local;
                }
                else if (costo_ruta < min_costo_2) min_costo_2 = costo_ruta;
            }
            double regret = (min_costo == numeric_limits<double>::max()) ? 999999.0 : (min_costo_2 - min_costo);
            if (regret > max_regret) {
                max_regret = regret; best_c_idx = i;
                best_r = local_best_r; best_p = local_best_p;
            }
        }
        if (best_r != -1) {
            if (aplicar_insercion(best_r, best_p, fuera[best_c_idx])) {
                ruta_de_nodo[fuera[best_c_idx]] = best_r;
                for (int u : fuera) cache_costos[u][best_r] = -1.0;
                fuera.erase(fuera.begin() + best_c_idx);
            }
            else cache_costos[fuera[best_c_idx]][best_r] = -2.0;
        }
        else {
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
    reset_cache(fuera, solucion_actual.size());
    reconstruir_indice_rutas();
    while (!fuera.empty()) {
        double mejor_costo_global = numeric_limits<double>::max();
        int mejor_c_idx = -1, mejor_r = -1, mejor_p = -1;
        for (int i = 0; i < (int)fuera.size(); ++i) {
            int u = fuera[i];
            vector<int> rutas_cand = rutas_candidatas_para(u);
            for (int r : rutas_cand) {
                if (cache_costos[u][r] == -2.0) continue;
                const auto& ruta = solucion_actual[r];
                double costo_ruta = numeric_limits<double>::max();
                int mejor_pos_local = -1;
                if (ruta.carga_total + mapa[u].demanda <= CAP_CARGA) {
                    for (int p = 1; p < (int)ruta.secuencia.size(); ++p) {
                        double inc = probar_insercion(r, p, u);
                        if (inc < costo_ruta) { costo_ruta = inc; mejor_pos_local = p; }
                    }
                }
                cache_costos[u][r] = (costo_ruta == numeric_limits<double>::max()) ? -2.0 : costo_ruta;
                if (costo_ruta < mejor_costo_global) {
                    mejor_costo_global = costo_ruta; mejor_c_idx = i; mejor_r = r; mejor_p = mejor_pos_local;
                }
            }
        }
        if (mejor_c_idx != -1) {
            if (aplicar_insercion(mejor_r, mejor_p, fuera[mejor_c_idx])) {
                ruta_de_nodo[fuera[mejor_c_idx]] = mejor_r;
                for (int u : fuera) cache_costos[u][mejor_r] = -1.0;
                fuera.erase(fuera.begin() + mejor_c_idx);
            }
            else cache_costos[fuera[mejor_c_idx]][mejor_r] = -2.0;
        }
        else {
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
    for (int i = 0; i < num_rutas_zona; ++i) rutas_zona.push_back(dist_rutas[i].second);

    while (!fuera.empty()) {
        double mejor_costo_global = numeric_limits<double>::max();
        int mejor_c_idx = -1, mejor_r = -1, mejor_p = -1;
        for (int i = 0; i < (int)fuera.size(); ++i) {
            int u = fuera[i];
            for (int r : rutas_zona) {
                const auto& ruta = solucion_actual[r];
                if (ruta.carga_total + mapa[u].demanda <= CAP_CARGA) {
                    for (int p = 1; p < (int)ruta.secuencia.size(); ++p) {
                        double inc = probar_insercion(r, p, u);
                        if (inc < mejor_costo_global) {
                            mejor_costo_global = inc; mejor_c_idx = i; mejor_r = r; mejor_p = p;
                        }
                    }
                }
            }
        }
        if (mejor_c_idx != -1) {
            if (aplicar_insercion(mejor_r, mejor_p, fuera[mejor_c_idx])) fuera.erase(fuera.begin() + mejor_c_idx);
        }
        else {
            crear_nueva_ruta(fuera[0]); fuera.erase(fuera.begin());
        }
    }
    limpiar_rutas_vacias();
}
