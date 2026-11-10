// ============================================================================
//  ALNS_EVRPTW_SetCovering.cpp
//  Fase final de matheuristica: resuelve un set-covering (greedy o con
//  CP-SAT via OR-Tools) sobre el pool de rutas factibles recolectadas
//  durante la busqueda del ALNS.
//
//  Este archivo compila SIN OR-Tools instalado. Para activar OR-Tools se debe definir
//  el simbolo de preprocesador USE_ORTOOLS (Propiedades del proyecto -> C/C++
//  -> Preprocesador -> Definiciones de preprocesador). Si NO esta definido,
//  resolver_set_covering_cpsat() simplemente delega en el fallback greedy
//  (resolver_greedy_set_covering), que no requiere ninguna libreria externa.
// ============================================================================

#include "ALNS_EVRPTW.h"
#include <iostream>
#include <limits>
#include <unordered_set>
#include <map>

#ifdef USE_ORTOOLS
#include "ortools/sat/cp_model.h"
#endif

using namespace std;

vector<Ruta> ALNS_EVRPTW::resolver_greedy_set_covering() {
    int n = mapa.size();
    vector<bool> cubierto(n, false);
    int total_clientes = 0;
    for (const auto& nodo : mapa) if (!nodo.es_estacion && !nodo.es_deposito) total_clientes++;

    vector<bool> usada(pool_rutas.size(), false);
    vector<Ruta> resultado;
    int cubiertos_count = 0;

    while (cubiertos_count < total_clientes) {
        int mejor_idx = -1;
        double mejor_ratio = numeric_limits<double>::max();

        for (size_t j = 0; j < pool_rutas.size(); ++j) {
            if (usada[j] || pool_rutas[j].clientes.empty()) continue;
            bool tiene_conflicto = false;
            for (int c : pool_rutas[j].clientes) {
                if (cubierto[c]) { tiene_conflicto = true; break; }
            }
            if (tiene_conflicto) continue;

            int nuevos = (int)pool_rutas[j].clientes.size();
            double ratio = pool_rutas[j].distancia / nuevos;
            if (ratio < mejor_ratio) { mejor_ratio = ratio; mejor_idx = (int)j; }
        }

        if (mejor_idx == -1) break; // No quedan rutas del pool utilizables sin conflicto.
        usada[mejor_idx] = true;
        for (int c : pool_rutas[mejor_idx].clientes) { cubierto[c] = true; cubiertos_count++; }
        Ruta r;
        r.secuencia = pool_rutas[mejor_idx].secuencia_completa;
        r.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
        resultado.push_back(r);
    }

    // Respaldo garantizado: crea una ruta unitaria para cualquier
    // cliente que haya quedado sin cobertura tras el set-covering.
    if (cubiertos_count < total_clientes) {
        cout << "  [fallback] Generando rutas unitarias para " << (total_clientes - cubiertos_count) << " clientes sin cobertura." << endl;
        for (int i = 0; i < (int)mapa.size(); ++i) {
            if (mapa[i].es_estacion || mapa[i].es_deposito) continue;
            if (cubierto[i]) continue;

            Ruta r_unit;
            r_unit.secuencia = { {ID_DEPOSITO, 0, CAP_BATT, 0, 0}, {i, 0, 0, 0, 0}, {ID_DEPOSITO, 0, 0, 0, 0} };
            int status = r_unit.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);

            if (status == 1) {
                // Sin bateria suficiente para el trayecto directo: se
                // inserta la mejor estacion disponible, igual que en
                // crear_nueva_ruta.
                double mejor_costo_est = numeric_limits<double>::max();
                int mejor_est = -1;
                for (int s : estaciones) {
                    r_unit.secuencia.insert(r_unit.secuencia.begin() + 1, { s, 0, 0, 0, 0 });
                    if (r_unit.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO) == 0) {
                        if (r_unit.distancia_total < mejor_costo_est) { mejor_costo_est = r_unit.distancia_total; mejor_est = s; }
                    }
                    r_unit.secuencia.erase(r_unit.secuencia.begin() + 1);
                }
                if (mejor_est != -1) {
                    r_unit.secuencia.insert(r_unit.secuencia.begin() + 1, { mejor_est, 0, 0, 0, 0 });
                    r_unit.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
                }
            }

            resultado.push_back(r_unit);
            cubierto[i] = true;
            cubiertos_count++;
        }
    }

    return resultado;
}

#ifndef USE_ORTOOLS
// ----------------------------------------------------------------------
//  Version SIN OR-Tools: resolver_set_covering_cpsat() delega directo en
//  el set-covering greedy. Se usa cuando el proyecto se compila sin la
//  bandera USE_ORTOOLS (caso tipico de alguien que solo trabaja en el
//  ALNS, sin necesidad de instalar la libreria).
// ----------------------------------------------------------------------
vector<Ruta> ALNS_EVRPTW::resolver_set_covering_cpsat(double /*tiempo_limite_fase1*/,
    double /*tiempo_limite_fase2*/) {
    cout << "  [set-covering] Compilado sin OR-Tools (USE_ORTOOLS no definido)."
        << " Usando fallback greedy." << endl;
    return resolver_greedy_set_covering();
}

#else
// ----------------------------------------------------------------------
//  Version CON OR-Tools: resuelve el set-covering en dos fases con CP-SAT.
// ----------------------------------------------------------------------
vector<Ruta> ALNS_EVRPTW::resolver_set_covering_cpsat(double tiempo_limite_fase1,
    double tiempo_limite_fase2) {
    using namespace operations_research;
    using namespace operations_research::sat;

    // Filtramos las entradas invalidadas por registrar_rutas_en_pool (marcadas
    // con clientes.clear() cuando se descartan por exceder K_VARIANTES_POR_GRUPO).
    // Sin este filtro, pool_rutas.size() incluye miles de entradas "muertas"
    // acumuladas durante toda la corrida del ALNS, y cada una generaba una
    // BoolVar inutil en el modelo de CP-SAT, inflandolo innecesariamente.
    vector<int> rutas_validas;
    rutas_validas.reserve(pool_rutas.size());
    for (int j = 0; j < (int)pool_rutas.size(); ++j) {
        if (!pool_rutas[j].clientes.empty()) rutas_validas.push_back(j);
    }

    int n = (int)rutas_validas.size();
    cout << "  [cpsat] Pool total=" << pool_rutas.size()
        << " | rutas validas=" << n << endl;
    if (n == 0) return resolver_greedy_set_covering();

    auto construir_cobertura = [&](CpModelBuilder& modelo, vector<BoolVar>& x) {
        x.resize(n);
        for (int j = 0; j < n; ++j) x[j] = modelo.NewBoolVar();

        map<int, vector<int>> cobertura;
        for (int j = 0; j < n; ++j) {
            const auto& clientes = pool_rutas[rutas_validas[j]].clientes;
            for (int c : clientes)
                cobertura[c].push_back(j);
        }

        for (auto& [cliente, rutas_j] : cobertura) {
            LinearExpr suma;
            for (int j : rutas_j) suma += x[j];
            modelo.AddGreaterOrEqual(suma, 1);
        }
        };

    // --- Warm-start: mapear mejor_solucion ALNS a rutas del pool ---
    // Para cada ruta de la mejor solucion ALNS, busca la ruta del pool
    // con el mismo conjunto de clientes y menor distancia para usarla
    // como hint en ambas fases del solver.
    vector<bool> hint_alns(n, false);
    int hint_alns_count = 0;
    {
        for (const auto& ruta : mejor_solucion) {
            if (ruta.violacion_tiempo > 1e-5 || ruta.violacion_bateria > 1e-5
                || ruta.violacion_capacidad > 1e-5) continue;
            vector<int> clientes;
            for (const auto& p : ruta.secuencia) {
                if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito)
                    clientes.push_back(p.nodo_id);
            }
            if (clientes.empty()) continue;
            sort(clientes.begin(), clientes.end());

            int best_j = -1;
            double best_dist = numeric_limits<double>::max();
            for (int j = 0; j < n; ++j) {
                if (hint_alns[j]) continue; // ya asignada a otra ruta
                const auto& pr = pool_rutas[rutas_validas[j]];
                if (pr.clientes == clientes && pr.distancia < best_dist) {
                    best_dist = pr.distancia;
                    best_j = j;
                }
            }
            if (best_j != -1) { hint_alns[best_j] = true; hint_alns_count++; }
        }
        cout << "  [cpsat] Warm-start: " << hint_alns_count << "/"
             << mejor_solucion.size() << " rutas ALNS mapeadas al pool" << endl;
    }

    // ---------- FASE 1: minimizar numero de vehiculos ----------
    CpModelBuilder modelo1;
    vector<BoolVar> x1;
    construir_cobertura(modelo1, x1);

    LinearExpr num_rutas1;
    for (int j = 0; j < n; ++j) num_rutas1 += x1[j];
    modelo1.Minimize(num_rutas1);

    // Warm-start fase 1 con la solucion ALNS.
    for (int j = 0; j < n; ++j)
        modelo1.AddHint(x1[j], hint_alns[j] ? int64_t{1} : int64_t{0});

    Model m1;
    SatParameters params1;
    params1.set_max_time_in_seconds(tiempo_limite_fase1);
    params1.set_num_search_workers(8);
    m1.Add(NewSatParameters(params1));

    const CpSolverResponse resp1 = SolveCpModel(modelo1.Build(), &m1);

    if (resp1.status() != CpSolverStatus::OPTIMAL && resp1.status() != CpSolverStatus::FEASIBLE) {
        cout << "  [cpsat] Fase 1 infactible, usando fallback greedy." << endl;
        return resolver_greedy_set_covering();
    }

    int min_vehiculos = (int)llround(resp1.objective_value());
    int cota_vehiculos = min_vehiculos + (resp1.status() == CpSolverStatus::FEASIBLE ? 1 : 0);
    cout << "  [cpsat] Fase 1: minimo de vehiculos = " << min_vehiculos << endl;

    // ---------- FASE 2: minimizar distancia con vehiculos acotados ----------
    CpModelBuilder modelo2;
    vector<BoolVar> x2;
    construir_cobertura(modelo2, x2);

    LinearExpr num_rutas2;
    for (int j = 0; j < n; ++j) num_rutas2 += x2[j];
    modelo2.AddLessOrEqual(num_rutas2, cota_vehiculos);

    // Warm-start fase 2: elegir la mejor solucion de partida.
    // Si la solucion ALNS cabe en la cota de vehiculos, compararla con la
    // de fase 1 y usar la de menor distancia como hint. Esto es clave porque
    // fase 1 optimiza vehiculos (no distancia), asi que su solucion puede
    // tener distancia mucho peor que la del ALNS.
    bool usar_alns_fase2 = false;
    if (hint_alns_count > 0 && (int)mejor_solucion.size() <= cota_vehiculos) {
        // Calcular distancia del hint ALNS (solo las rutas mapeadas al pool).
        double dist_hint_alns = 0;
        for (int j = 0; j < n; ++j) {
            if (hint_alns[j]) dist_hint_alns += pool_rutas[rutas_validas[j]].distancia;
        }
        // Calcular distancia del hint fase 1.
        double dist_hint_f1 = 0;
        for (int j = 0; j < n; ++j) {
            if (SolutionBooleanValue(resp1, x1[j]))
                dist_hint_f1 += pool_rutas[rutas_validas[j]].distancia;
        }
        usar_alns_fase2 = (hint_alns_count == (int)mejor_solucion.size())
                          && (dist_hint_alns < dist_hint_f1);
    }

    if (usar_alns_fase2) {
        for (int j = 0; j < n; ++j)
            modelo2.AddHint(x2[j], hint_alns[j] ? int64_t{1} : int64_t{0});
        cout << "  [cpsat] Warm-start fase 2: solucion ALNS (mejor distancia)" << endl;
    } else {
        for (int j = 0; j < n; ++j)
            modelo2.AddHint(x2[j], SolutionBooleanValue(resp1, x1[j]) ? int64_t{1} : int64_t{0});
        cout << "  [cpsat] Warm-start fase 2: solucion fase 1" << endl;
    }

    const int64_t FACTOR = 1000;
    LinearExpr distancia_total;
    for (int j = 0; j < n; ++j) {
        int64_t d_j = (int64_t)llround(pool_rutas[rutas_validas[j]].distancia * FACTOR);
        distancia_total += x2[j] * d_j;
    }
    modelo2.Minimize(distancia_total);

    Model m2;
    SatParameters params2;
    params2.set_max_time_in_seconds(tiempo_limite_fase2);
    params2.set_num_search_workers(8);
    m2.Add(NewSatParameters(params2));

    const CpSolverResponse resp2 = SolveCpModel(modelo2.Build(), &m2);

    vector<Ruta> resultado;
    if (resp2.status() == CpSolverStatus::OPTIMAL || resp2.status() == CpSolverStatus::FEASIBLE) {
        for (int j = 0; j < n; ++j) {
            if (SolutionBooleanValue(resp2, x2[j])) {
                Ruta r;
                r.secuencia = pool_rutas[rutas_validas[j]].secuencia_completa;
                r.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
                resultado.push_back(r);
            }
        }
        cout << "  [cpsat] Fase 2: vehiculos=" << resultado.size()
            << " | distancia=" << obtener_distancia(resultado) << endl;
    }
    else {
        string motivo = (resp2.status() == CpSolverStatus::INFEASIBLE) ? "INFEASIBLE" : "UNKNOWN/timeout";
        cout << "  [cpsat] Fase 2 no resuelta (status=" << motivo << "), usando fallback greedy." << endl;
        return resolver_greedy_set_covering();
    }

    reparar_solapes_covering(resultado);

    // Fallback: clientes sin cobertura por pool insuficiente.
    unordered_set<int> cubiertos;
    for (const auto& r : resultado)
        for (const auto& p : r.secuencia)
            if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito)
                cubiertos.insert(p.nodo_id);

    for (int i = 0; i < (int)mapa.size(); ++i) {
        if (mapa[i].es_estacion || mapa[i].es_deposito) continue;
        if (cubiertos.count(i)) continue;

        Ruta r_unit;
        r_unit.secuencia = { {ID_DEPOSITO, 0, CAP_BATT, 0, 0}, {i, 0, 0, 0, 0}, {ID_DEPOSITO, 0, 0, 0, 0} };
        int status = r_unit.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
        if (status == 1) {
            double mejor_costo_est = numeric_limits<double>::max();
            int mejor_est = -1;
            for (int s : estaciones) {
                r_unit.secuencia.insert(r_unit.secuencia.begin() + 1, { s, 0, 0, 0, 0 });
                if (r_unit.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO) == 0) {
                    if (r_unit.distancia_total < mejor_costo_est) { mejor_costo_est = r_unit.distancia_total; mejor_est = s; }
                }
                r_unit.secuencia.erase(r_unit.secuencia.begin() + 1);
            }
            if (mejor_est != -1) {
                r_unit.secuencia.insert(r_unit.secuencia.begin() + 1, { mejor_est, 0, 0, 0, 0 });
                r_unit.calcular_metricas(mapa, matriz_dist, CAP_BATT, CAP_CARGA, R_CARGA, R_CONSUMO, ID_DEPOSITO);
            }
        }
        resultado.push_back(r_unit);
    }

    return resultado;
}
#endif // USE_ORTOOLS