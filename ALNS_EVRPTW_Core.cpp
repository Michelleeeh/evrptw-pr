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

    ruta_de_nodo.assign(n, -1);
}

void ALNS_EVRPTW::reset_cache(const vector<int>& fuera, int num_rutas) {
    for (int u : fuera) {
        if (cache_costos[u].size() < num_rutas) {
            cache_costos[u].resize(num_rutas, -1.0);
        }
        std::fill(cache_costos[u].begin(), cache_costos[u].begin() + num_rutas, -1.0);
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
