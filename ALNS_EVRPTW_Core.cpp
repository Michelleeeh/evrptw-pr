// ============================================================================
//  ALNS_EVRPTW_Core.cpp
//  Constructor del gestor ALNS, manejo de la cache de costos de insercion,
//  y metricas globales (distancia total, costo, comparacion de soluciones).
// ============================================================================

#include "ALNS_EVRPTW.h"
#include <cmath>
#include <algorithm>

using namespace std;

ALNS_EVRPTW::ALNS_EVRPTW(vector<Nodo> m, double cc, double cb, double rc, double rcons, ConfigALNS cfg)
    : mapa(m), CAP_CARGA(cc), CAP_BATT(cb), R_CARGA(rc), R_CONSUMO(rcons),
    rng(cfg.semilla), config(cfg), dist_01(0.0, 1.0) {

    int n = mapa.size();
    matriz_dist.assign(n, vector<double>(n, 0.0));
    cache_costos.assign(n, vector<double>(100, -1.0));
    cache_pos.assign(n, vector<int>(100, -1));
    cache_incs.assign(n, vector<vector<double>>(100));

    for (int i = 0; i < n; ++i) {
        if (mapa[i].es_deposito) ID_DEPOSITO = i;
        if (mapa[i].es_estacion && !mapa[i].es_deposito) estaciones.push_back(i);
        for (int j = 0; j < n; ++j) {
            double dx = mapa[i].x - mapa[j].x;
            double dy = mapa[i].y - mapa[j].y;
            matriz_dist[i][j] = sqrt(dx * dx + dy * dy);
        }
    }

    vecinos_cercanos.assign(n, {});
    for (int i = 0; i < n; ++i) {
        if (mapa[i].es_estacion || mapa[i].es_deposito) continue;
        vector<pair<double, int>> dists;
        dists.reserve(n);
        for (int j = 0; j < n; ++j) {
            if (i == j || mapa[j].es_estacion || mapa[j].es_deposito) continue;
            dists.push_back({ matriz_dist[i][j], j });
        }
        sort(dists.begin(), dists.end());
        int k = min((int)dists.size(), K_VECINOS);
        vecinos_cercanos[i].reserve(k);
        for (int idx = 0; idx < k; ++idx) vecinos_cercanos[i].push_back(dists[idx].second);
    }

    // Precalcular, para cada nodo, las estaciones ordenadas por cercania.
    // Esto evita tener que ordenar en cada llamada a probar_insercion.
    estaciones_cercanas.assign(n, {});
    for (int i = 0; i < n; ++i) {
        vector<pair<double, int>> dists;
        dists.reserve(estaciones.size());
        for (int s : estaciones) {
            dists.push_back({ matriz_dist[i][s], s });
        }
        sort(dists.begin(), dists.end());
        estaciones_cercanas[i].reserve(dists.size());
        for (auto& p : dists) estaciones_cercanas[i].push_back(p.second);
    }

    // Cambio 3: para cada arco (i,j) precalcular las k estaciones que menos
    // desvio anaden al ir de i a j, ordenadas por d(i,s)+d(s,j).
    if (config.usar_topk_en_probar_insercion) {
        int k = max(1, min(config.k_estaciones_arco, (int)estaciones.size()));
        estaciones_candidatas_arco.assign(n, vector<vector<int>>(n));
        vector<pair<double, int>> desvios;
        desvios.reserve(estaciones.size());
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                desvios.clear();
                for (int s : estaciones) desvios.push_back({ matriz_dist[i][s] + matriz_dist[s][j], s });
                sort(desvios.begin(), desvios.end());
                auto& dest = estaciones_candidatas_arco[i][j];
                dest.reserve(k);
                for (int idx = 0; idx < k; ++idx) dest.push_back(desvios[idx].second);
            }
        }
    }

    ruta_de_nodo.assign(n, -1);
    Ruta::redistribuir_carga = config.redistribuir_carga || config.politica_carga >= 1;
    Ruta::politica_carga = config.politica_carga;
    if (!config.usar_regret3) { pesos_ins.resize(5); scores_ins.resize(5); usos_ins.resize(5); }
    if (!config.operadores_shaw_extra) { pesos_rem.resize(9); scores_rem.resize(9); usos_rem.resize(9); }
}

void ALNS_EVRPTW::invalidar_cache_ruta(const vector<int>& fuera, int r) {
    for (int u : fuera) {
        if (r < (int)cache_costos[u].size()) {
            cache_costos[u][r] = -1.0;
            cache_pos[u][r] = -1;
        }
    }
}

void ALNS_EVRPTW::reset_cache(const vector<int>& fuera, int num_rutas) {
    for (int u : fuera) {
        if ((int)cache_costos[u].size() < num_rutas) {
            cache_costos[u].resize(num_rutas, -1.0);
            cache_pos[u].resize(num_rutas, -1);
            cache_incs[u].resize(num_rutas);
        }
        std::fill(cache_costos[u].begin(), cache_costos[u].begin() + num_rutas, -1.0);
        std::fill(cache_pos[u].begin(), cache_pos[u].begin() + num_rutas, -1);
    }
}

double ALNS_EVRPTW::obtener_distancia(const vector<Ruta>& sol) const {
    double d = 0;
    for (const auto& r : sol) d += r.distancia_total;
    return d;
}

double ALNS_EVRPTW::calcular_costo(const vector<Ruta>& sol) const {
    const double M = 1e6;
    double costo = sol.size() * M + obtener_distancia(sol);
    for (const auto& r : sol) {
        costo += (alpha_tiempo * r.violacion_tiempo);
        costo += (beta_bateria * r.violacion_bateria);
        costo += (gamma_cap * r.violacion_capacidad);
    }
    return costo;
}
