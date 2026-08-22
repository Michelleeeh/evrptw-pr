// ============================================================================
//  Ruta.cpp
//  Implementacion de la logica de factibilidad de la clase Ruta.
// ============================================================================

#include "Ruta.h"
#include <algorithm>

using namespace std;

double Ruta::bateria_necesaria_hasta_proximo_punto(int idx, const vector<Nodo>& mapa,
    const vector<vector<double>>& matriz_dist,
    double r_consumo, int id_deposito) const {
    double necesaria = 0.0;
    int i = idx;
    while (i < (int)secuencia.size() - 1) {
        double dist = matriz_dist[secuencia[i].nodo_id][secuencia[i + 1].nodo_id];
        necesaria += dist * r_consumo;
        i++;
        int id_sig = secuencia[i].nodo_id;
        if (mapa[id_sig].es_estacion || mapa[id_sig].es_deposito) {
            return necesaria;
        }
    }
    int ultimo = secuencia.back().nodo_id;
    if (!mapa[ultimo].es_deposito) {
        necesaria += matriz_dist[ultimo][id_deposito] * r_consumo;
    }
    return necesaria;
}

int Ruta::calcular_metricas(const vector<Nodo>& mapa,
    const vector<vector<double>>& matriz_dist,
    double cap_batt, double cap_carga,
    double r_carga, double r_consumo, int id_deposito) {
    distancia_total = 0;
    carga_total = 0;
    violacion_tiempo = 0;
    violacion_bateria = 0;
    violacion_capacidad = 0;

    if (secuencia.empty()) return 0;

    secuencia[0].llegada_tiempo = 0;
    secuencia[0].salida_tiempo = 0;
    secuencia[0].llegada_bateria = cap_batt;
    secuencia[0].salida_bateria = cap_batt;

    double tiempo_actual = 0;
    double bateria_actual = cap_batt;
    double carga_actual = 0;

    for (int i = 0; i < (int)secuencia.size() - 1; ++i) {
        int id_actual = secuencia[i].nodo_id;
        int id_siguiente = secuencia[i + 1].nodo_id;
        const Nodo& nodo_sig = mapa[id_siguiente];

        double distancia = matriz_dist[id_actual][id_siguiente];
        distancia_total += distancia;

        double bateria_llegada = bateria_actual - (distancia * r_consumo);
        double tiempo_llegada = tiempo_actual + distancia;

        if (bateria_llegada < -1e-9) {
            violacion_bateria += (-bateria_llegada);
            secuencia[i + 1].llegada_bateria = bateria_llegada;
            secuencia[i + 1].llegada_tiempo = tiempo_llegada;
            secuencia[i + 1].salida_bateria = bateria_llegada;
            secuencia[i + 1].salida_tiempo = tiempo_llegada;
            return 1;
        }

        if (tiempo_llegada > nodo_sig.due_date + 1e-9) {
            violacion_tiempo += (tiempo_llegada - nodo_sig.due_date);
            secuencia[i + 1].llegada_bateria = bateria_llegada;
            secuencia[i + 1].llegada_tiempo = tiempo_llegada;
            secuencia[i + 1].salida_bateria = bateria_llegada;
            secuencia[i + 1].salida_tiempo = tiempo_llegada;
            return 2;
        }

        double tiempo_inicio_servicio = max(tiempo_llegada, nodo_sig.ready_time);

        secuencia[i + 1].llegada_bateria = bateria_llegada;
        secuencia[i + 1].llegada_tiempo = tiempo_inicio_servicio;

        if (nodo_sig.es_estacion && !nodo_sig.es_deposito) {
            double necesaria = bateria_necesaria_hasta_proximo_punto(
                i + 1, mapa, matriz_dist, r_consumo, id_deposito);
            double carga_necesaria = max(0.0, necesaria - bateria_llegada);
            double tiempo_disponible = max(0.0, nodo_sig.due_date - tiempo_inicio_servicio);
            double carga_max_por_tiempo = tiempo_disponible / r_carga;
            double cantidad_carga = min(carga_necesaria, carga_max_por_tiempo);
            cantidad_carga = min(cantidad_carga, cap_batt - bateria_llegada);

            double bateria_salida = bateria_llegada + cantidad_carga;
            double tiempo_carga = cantidad_carga * r_carga;
            double tiempo_salida = tiempo_inicio_servicio + tiempo_carga;

            if (tiempo_salida > nodo_sig.due_date + 1e-9) {
                violacion_tiempo += (tiempo_salida - nodo_sig.due_date);
                secuencia[i + 1].salida_bateria = bateria_salida;
                secuencia[i + 1].salida_tiempo = tiempo_salida;
                return 2;
            }

            secuencia[i + 1].salida_bateria = bateria_salida;
            secuencia[i + 1].salida_tiempo = tiempo_salida;
            bateria_actual = bateria_salida;
            tiempo_actual = tiempo_salida;
        }
        else {
            secuencia[i + 1].salida_bateria = bateria_llegada;
            tiempo_actual = tiempo_inicio_servicio + nodo_sig.service_time;
            secuencia[i + 1].salida_tiempo = tiempo_actual;
            bateria_actual = bateria_llegada;

            if (!nodo_sig.es_estacion && !nodo_sig.es_deposito) {
                carga_actual += nodo_sig.demanda;
                if (carga_actual > cap_carga + 1e-9) {
                    violacion_capacidad += (carga_actual - cap_carga);
                    return 3;
                }
            }
        }
    }

    if (secuencia.size() >= 2 && mapa[secuencia.back().nodo_id].es_deposito) {
        bool hubo_recarga = false;
        for (const auto& p : secuencia) {
            if (mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito) {
                hubo_recarga = true;
                break;
            }
        }
        if (hubo_recarga) {
            secuencia.back().llegada_bateria = 0.0;
            secuencia.back().salida_bateria = 0.0;
        }
    }

    carga_total = carga_actual;
    return 0;
}
