// ============================================================================
//  Ruta.cpp
//  Implementacion de la logica de factibilidad de la clase Ruta.
//  Un unico nucleo de evaluacion (evaluar_nucleo) sirve a calcular_metricas
//  (secuencia real, escribe las paradas) y a la evaluacion con insercion
//  virtual (secuencia mapeada, no escribe nada), de modo que ambas aplican
//  exactamente la misma politica de carga.
// ============================================================================

#include "Ruta.h"
#include <algorithm>
#include <limits>

using namespace std;

bool Ruta::redistribuir_carga = false;
int Ruta::politica_carga = 0;

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

namespace {

// Areas de trabajo por parada (indice en la secuencia evaluada).
struct Scratch {
    vector<double> llegada_t, espera, salida_t, salida_b, carga, extra;
};
thread_local Scratch SC;

struct Resultado {
    double distancia = 0, carga_total = 0, viol_t = 0, viol_b = 0, viol_c = 0, tiempo_fin = 0;
    int idx_falla = -1;
};

// Nucleo. nodo_en(k) devuelve el id del nodo en la posicion k (0..n-1).
// Si `paradas` no es nulo, escribe llegada/salida de bateria y tiempo como
// calcular_metricas original.
template <class NodoEn>
int evaluar_nucleo(int n, NodoEn nodo_en, const vector<Nodo>& mapa,
    const vector<vector<double>>& D, double cap_batt, double cap_carga,
    double r_carga, double r_consumo, int id_deposito, bool redistribuir,
    Resultado& res, EstadoVirtual* estado, Parada* paradas) {

    if (n == 0) return 0;

    auto energia_hasta_prox = [&](int idx) -> double {
        double necesaria = 0.0;
        int i = idx;
        int id_i = nodo_en(i);
        while (i < n - 1) {
            int id_sig = nodo_en(i + 1);
            necesaria += D[id_i][id_sig] * r_consumo;
            i++;
            id_i = id_sig;
            if (mapa[id_i].es_estacion || mapa[id_i].es_deposito) return necesaria;
        }
        if (!mapa[id_i].es_deposito) necesaria += D[id_i][id_deposito] * r_consumo;
        return necesaria;
    };

    if ((int)SC.llegada_t.size() < n) {
        SC.llegada_t.resize(n); SC.espera.resize(n); SC.salida_t.resize(n);
        SC.salida_b.resize(n); SC.carga.resize(n); SC.extra.resize(n);
    }
    if (estado && (int)estado->energia_desde_ultimo_punto_carga.size() < n) {
        estado->energia_desde_ultimo_punto_carga.resize(n);
        estado->tiempo_salida_minimo.resize(n);
    }

    auto escribir = [&](int i, double lb, double sb, double lt, double st) {
        if (paradas) { paradas[i].llegada_bateria = lb; paradas[i].salida_bateria = sb; paradas[i].llegada_tiempo = lt; paradas[i].salida_tiempo = st; }
    };

    // Una pasada hacia adelante con carga minima (+ extra[k] si usar_extra).
    auto pasada = [&](bool usar_extra) -> int {
        res = Resultado();
        escribir(0, cap_batt, cap_batt, 0, 0);
        SC.llegada_t[0] = 0; SC.espera[0] = 0; SC.salida_t[0] = 0; SC.salida_b[0] = cap_batt; SC.carga[0] = 0;
        if (estado) { estado->energia_desde_ultimo_punto_carga[0] = 0.0; estado->tiempo_salida_minimo[0] = 0.0; }

        double tiempo_actual = 0, bateria_actual = cap_batt, carga_actual = 0;
        double energia_desde_pc = 0.0, tiempo_min = 0.0;
        int id_actual = nodo_en(0);

        for (int i = 0; i < n - 1; ++i) {
            int id_sig = nodo_en(i + 1);
            const Nodo& nodo_sig = mapa[id_sig];
            double d = D[id_actual][id_sig];
            res.distancia += d;
            double bateria_llegada = bateria_actual - d * r_consumo;
            double tiempo_llegada = tiempo_actual + d;

            if (estado) {
                bool punto_carga = nodo_sig.es_estacion || nodo_sig.es_deposito;
                energia_desde_pc = punto_carga ? 0.0 : energia_desde_pc + d * r_consumo;
                double llegada_min = tiempo_min + d;
                double inicio_min = max(llegada_min, nodo_sig.ready_time);
                tiempo_min = (nodo_sig.es_estacion && !nodo_sig.es_deposito) ? inicio_min : inicio_min + nodo_sig.service_time;
                estado->energia_desde_ultimo_punto_carga[i + 1] = energia_desde_pc;
                estado->tiempo_salida_minimo[i + 1] = tiempo_min;
            }

            if (bateria_llegada < -EPS_BATERIA) {
                res.viol_b += -bateria_llegada; res.idx_falla = i + 1;
                escribir(i + 1, bateria_llegada, bateria_llegada, tiempo_llegada, tiempo_llegada);
                return 1;
            }
            if (tiempo_llegada > nodo_sig.due_date + 1e-9) {
                res.viol_t += tiempo_llegada - nodo_sig.due_date; res.idx_falla = i + 1;
                escribir(i + 1, bateria_llegada, bateria_llegada, tiempo_llegada, tiempo_llegada);
                return 2;
            }

            double tiempo_inicio = max(tiempo_llegada, nodo_sig.ready_time);
            SC.llegada_t[i + 1] = tiempo_llegada;
            SC.espera[i + 1] = tiempo_inicio - tiempo_llegada;
            SC.carga[i + 1] = 0.0;

            if (nodo_sig.es_estacion && !nodo_sig.es_deposito) {
                double necesaria = energia_hasta_prox(i + 1);
                double carga_necesaria = max(0.0, necesaria - bateria_llegada);
                if (usar_extra) carga_necesaria += SC.extra[i + 1];
                double tiempo_disponible = max(0.0, nodo_sig.due_date - tiempo_inicio);
                double cantidad = min(carga_necesaria, tiempo_disponible / r_carga);
                cantidad = min(cantidad, cap_batt - bateria_llegada);
                double bateria_salida = bateria_llegada + cantidad;
                double tiempo_salida = tiempo_inicio + cantidad * r_carga;
                if (tiempo_salida > nodo_sig.due_date + 1e-9) {
                    res.viol_t += tiempo_salida - nodo_sig.due_date; res.idx_falla = i + 1;
                    if (paradas) { paradas[i + 1].llegada_bateria = bateria_llegada; paradas[i + 1].llegada_tiempo = tiempo_inicio; paradas[i + 1].salida_bateria = bateria_salida; paradas[i + 1].salida_tiempo = tiempo_salida; }
                    return 2;
                }
                SC.carga[i + 1] = cantidad;
                escribir(i + 1, bateria_llegada, bateria_salida, tiempo_inicio, tiempo_salida);
                bateria_actual = bateria_salida; tiempo_actual = tiempo_salida;
            }
            else {
                tiempo_actual = tiempo_inicio + nodo_sig.service_time;
                bateria_actual = bateria_llegada;
                escribir(i + 1, bateria_llegada, bateria_llegada, tiempo_inicio, tiempo_actual);
                if (!nodo_sig.es_estacion && !nodo_sig.es_deposito) {
                    carga_actual += nodo_sig.demanda;
                    if (carga_actual > cap_carga + 1e-9) { res.viol_c += carga_actual - cap_carga; res.idx_falla = i + 1; return 3; }
                }
            }
            SC.salida_t[i + 1] = tiempo_actual; SC.salida_b[i + 1] = bateria_actual;
            id_actual = id_sig;
        }
        res.carga_total = carga_actual;
        res.tiempo_fin = tiempo_actual;
        return 0;
    };

    int st = pasada(false);

    // Politicas 2 y 3: carga adelantada SIEMPRE (no solo tras fallo).
    // Se procesan las estaciones en orden; para cada estacion p (salvo la
    // ultima) se calcula cuanto puede cargar de mas sin violar las ventanas
    // de los nodos siguientes, y se repite la pasada con ese extra fijado.
    if (Ruta::politica_carga >= 2) {
        for (int k = 0; k < n; ++k) SC.extra[k] = 0.0;
        int st_actual = st;
        int ult_pasada = -1;
        // Estaciones de la secuencia (sin la ultima: esa carga justo para llegar al deposito)
        static thread_local vector<int> ests2; ests2.clear();
        for (int k = 1; k < n - 1; ++k) { int id = nodo_en(k); if (mapa[id].es_estacion && !mapa[id].es_deposito) ests2.push_back(k); }
        for (int j = 0; j + 1 < (int)ests2.size(); ++j) {
            int p = ests2[j], k = ests2[j + 1];
            // hasta donde llego la ultima pasada: si fallo antes de p, nada que hacer
            int limite = (st_actual == 0) ? n - 1 : res.idx_falla;
            if (p >= limite) break;
            double cap_libre = cap_batt - (SC.salida_b[p]);
            if (cap_libre <= 1e-9) continue;
            // Holgura de tiempo entre p y k (los nodos posteriores a k se
            // benefician: k cargara menos).
            double x_max = mapa[nodo_en(p)].due_date - SC.salida_t[p];
            double W = 0.0;
            int tope = min(k, limite);
            for (int i = p + 1; i <= tope; ++i) {
                x_max = min(x_max, mapa[nodo_en(i)].due_date - SC.llegada_t[i] + W);
                W += SC.espera[i];
            }
            if (k > limite) {
                // la pasada fallo entre p y k (por bateria): cargar mas en p puede
                // arreglarlo; usamos la holgura hasta el punto de fallo
            }
            double extra_p = 0.0;
            if (x_max > 1e-9) {
                // necesidad de k (si la pasada llego a k) o energia hasta k desde p
                double necesidad_k = (k <= limite && st_actual == 0) ? SC.carga[k] : cap_libre;
                if (k <= limite && st_actual != 0) necesidad_k = cap_libre;
                extra_p = min(cap_libre, min(x_max / r_carga, necesidad_k));
            }
            if (Ruta::politica_carga == 3 && extra_p < cap_libre && st_actual == 0) {
                // top-up: mas alla de lo que k necesita, limitado por la holgura
                // de TODO lo posterior (cota conservadora: no descuenta lo que
                // k y las siguientes dejaran de cargar).
                double x_all = x_max; W = 0.0;
                for (int i = p + 1; i <= n - 1; ++i) { x_all = min(x_all, mapa[nodo_en(i)].due_date - SC.llegada_t[i] + W); W += SC.espera[i]; }
                if (x_all > 1e-9) extra_p = max(extra_p, min(cap_libre, x_all / r_carga));
            }
            if (extra_p > 1e-9) {
                SC.extra[p] += extra_p;
                st_actual = pasada(true);
                ult_pasada = p;
            }
        }
        if (ult_pasada >= 0) st = st_actual; else st = pasada(false);
        if (st != 2 || !redistribuir) return st;
    }

    // Redistribucion (politica del paper): fallo por tiempo tras la segunda
    // estacion -> mover carga de cada estacion a la anterior, limitado por la
    // capacidad de la bateria y por las ventanas de los nodos intermedios.
    if (st == 2 && redistribuir) {
        int f = res.idx_falla;
        static thread_local vector<int> ests;
        ests.clear();
        for (int k = 1; k < f; ++k) { int id = nodo_en(k); if (mapa[id].es_estacion && !mapa[id].es_deposito) ests.push_back(k); }
        if (ests.size() >= 2) {
            for (int k = 0; k < n; ++k) SC.extra[k] = 0.0;
            bool algo = false;
            for (int j = (int)ests.size() - 1; j >= 1; --j) {
                int k = ests[j], p = ests[j - 1];
                double movible = SC.carga[k];
                if (movible <= 1e-9) continue;
                movible = min(movible, cap_batt - (SC.salida_b[p] + SC.extra[p]));
                if (movible <= 1e-9) continue;
                // Maximo tiempo extra de carga en p absorbible antes de k sin
                // violar ventanas: x <= due_i - llegada_i + (esperas acumuladas antes de i).
                double x_max = mapa[nodo_en(p)].due_date - (SC.salida_t[p] + SC.extra[p] * r_carga);
                double W = 0.0;
                for (int i = p + 1; i <= k; ++i) {
                    x_max = min(x_max, mapa[nodo_en(i)].due_date - (SC.llegada_t[i] + SC.extra[p] * r_carga) + W);
                    W += SC.espera[i];
                }
                if (x_max <= 1e-9) continue;
                double e = min(movible, x_max / r_carga);
                if (e > 1e-9) { SC.extra[p] += e; algo = true; }
            }
            if (algo) st = pasada(true);
        }
    }
    return st;
}

} // namespace

int Ruta::calcular_metricas(const vector<Nodo>& mapa,
    const vector<vector<double>>& matriz_dist,
    double cap_batt, double cap_carga,
    double r_carga, double r_consumo, int id_deposito) {
    distancia_total = 0; carga_total = 0;
    violacion_tiempo = 0; violacion_bateria = 0; violacion_capacidad = 0;
    if (secuencia.empty()) return 0;

    Resultado res;
    int n = (int)secuencia.size();
    const Parada* seq = secuencia.data();
    int st = evaluar_nucleo(n, [seq](int k) { return seq[k].nodo_id; }, mapa, matriz_dist,
        cap_batt, cap_carga, r_carga, r_consumo, id_deposito, redistribuir_carga, res, nullptr, secuencia.data());

    distancia_total = res.distancia; carga_total = res.carga_total;
    violacion_tiempo = res.viol_t; violacion_bateria = res.viol_b; violacion_capacidad = res.viol_c;

    if (st == 0 && secuencia.size() >= 2 && mapa[secuencia.back().nodo_id].es_deposito) {
        bool hubo_recarga = false;
        for (const auto& p : secuencia) {
            if (mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito) { hubo_recarga = true; break; }
        }
        if (hubo_recarga) { secuencia.back().llegada_bateria = 0.0; secuencia.back().salida_bateria = 0.0; }
    }
    return st;
}

int Ruta::calcular_metricas_con_insercion_virtual(const vector<Nodo>& mapa,
    const vector<vector<double>>& matriz_dist,
    double cap_batt, double cap_carga,
    double r_carga, double r_consumo, int id_deposito,
    int pos_u, int u, int pos_s, int s,
    double& distancia, int& idx_falla,
    EstadoVirtual* estado, double* tiempo_fin) const {

    const int n_real = (int)secuencia.size();
    const int n_virt = n_real + 1 + (pos_s >= 0 ? 1 : 0);
    const Parada* seq = secuencia.data();
    auto nodo_en = [=](int k) -> int {
        if (pos_s >= 0) { if (k == pos_s) return s; if (k > pos_s) --k; }
        if (k == pos_u) return u;
        if (k > pos_u) --k;
        return seq[k].nodo_id;
    };
    Resultado res;
    int st = evaluar_nucleo(n_virt, nodo_en, mapa, matriz_dist, cap_batt, cap_carga, r_carga, r_consumo,
        id_deposito, redistribuir_carga, res, estado, nullptr);
    distancia = res.distancia;
    idx_falla = (st == 1) ? res.idx_falla : -1;
    if (tiempo_fin) *tiempo_fin = res.tiempo_fin;
    return st;
}
