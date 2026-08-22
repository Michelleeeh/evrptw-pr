// ============================================================================
//  ALNS_EVRPTW_Algoritmo.cpp
//  Ciclo principal del ALNS (ejecutar) y funciones de exportacion de
//  resultados a archivos CSV.
// ============================================================================

#include "ALNS_EVRPTW.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>

using namespace std;

void ALNS_EVRPTW::ejecutar(int iteraciones, string nombre_instancia) {
    cout << "Generando solucion inicial (Constructor Dinamico)..." << endl;

    inicializar_solucion();
    mejor_solucion = solucion_actual;

    cout << "Solucion inicial: " << mejor_solucion.size()
        << " vehiculos | Distancia: " << obtener_distancia(mejor_solucion) << endl;

    double prob_aceptacion = 0.5;
    double costo_inicial = obtener_distancia(mejor_solucion);
    T = -(config.mu * costo_inicial) / log(0.5);
    double alpha = config.alpha_enfriamiento;

    string sufijo = nombre_instancia + "_" + config.nombre_variante;
    ofstream out_trayectoria("trayectoria_" + sufijo + ".csv");

    cout << "Simulated Annealing -> Temp Inicial: " << T << " | Enfriamiento: " << alpha << endl;
    cout << "Comenzando optimizacion ALNS (" << iteraciones << " iteraciones)...\n" << endl;

    int N_SR = config.N_SR, N_RR = config.N_RR, n_RR = config.n_RR, N_c = config.N_c, N_s = config.N_s;
    int total_clientes = 0;
    for (const auto& nodo : mapa) {
        if (!nodo.es_estacion && !nodo.es_deposito) total_clientes++;
    }

    for (int i = 1; i <= iteraciones; ++i) {
        vector<Ruta> backup = solucion_actual;
        vector<int> fuera;
        int op_rem = -1, op_ins = -1, op_sr = -1, op_si = -1;

        // Segmento de operadores de estaciones de recarga.
        if (config.usar_segmento_estaciones && i % N_SR == 0) {
            int q = 1 + rng() % 5;
            op_sr = seleccionar_operador(pesos_sr); usos_sr[op_sr]++;
            if (op_sr == 0) random_station_removal(fuera, q);
            else if (op_sr == 1) worst_distance_station(fuera, q);
            else if (op_sr == 2) worst_charge_station(fuera, q);
            else full_charge_station_removal(fuera, q);

            op_si = seleccionar_operador(pesos_si); usos_si[op_si]++;
            if (op_si == 0) greedy_station_insertion();
            else if (op_si == 1) greedy_station_insertion_comparison();
            else best_station_insertion();
        }
        // Segmento de operadores de rutas completas.
        else if (config.usar_segmento_rutas && i % N_RR == 0) {
            for (int k = 0; k < n_RR; k++) {
                vector<Ruta> backup_sub = solucion_actual;
                vector<int> fuera_sub;
                op_rem = (rng() % 2 == 0) ? 4 : 5; usos_rem[op_rem]++;
                if (op_rem == 4) random_route_removal(fuera_sub);
                else greedy_route_removal(fuera_sub);
                if (fuera_sub.empty()) continue;

                op_ins = seleccionar_operador(pesos_ins); usos_ins[op_ins]++;
                if (op_ins == 0) greedy_insertion(fuera_sub, false);
                else if (op_ins == 1) greedy_insertion(fuera_sub, true);
                else if (op_ins == 2) regret_insertion(fuera_sub);
                else if (op_ins == 3) time_based_insertion(fuera_sub);
                else zone_insertion(fuera_sub);
                evaluar_y_aceptar(backup_sub, op_rem, op_ins, -1, -1, i);
            }
        }
        // Segmento estandar de operadores de clientes.
        else {
            int min_q = min(30.0, 0.1 * total_clientes);
            int max_q = min(60.0, 0.4 * total_clientes);
            int q = min_q + rng() % (max_q - min_q + 1);
            do {
                op_rem = seleccionar_operador(pesos_rem);
                if (!config.usar_shaw_removal && op_rem == 3) continue;
            } while (op_rem == 4 || op_rem == 5 || (!config.usar_shaw_removal && op_rem == 3));

            usos_rem[op_rem]++;
            if (op_rem == 0) random_removal(fuera, q);
            else if (op_rem == 1) worst_distance_removal(fuera, q);
            else if (op_rem == 2) worst_time_removal(fuera, q);
            else if (op_rem == 3) shaw_removal(fuera, q);
            else if (op_rem == 6) customer_preceding_station(fuera, q);
            else if (op_rem == 7) customer_succeding_station(fuera, q);
            else zone_removal(fuera, q);

            op_ins = seleccionar_operador(pesos_ins);
            if (!config.usar_regret_insertion && op_ins == 2) op_ins = 0;
            usos_ins[op_ins]++;

            bool hay_violacion_bateria = false;
            for (const auto& r : solucion_actual) {
                if (r.violacion_bateria > 1e-5) { hay_violacion_bateria = true; break; }
            }
            if (hay_violacion_bateria) greedy_station_insertion();

            if (op_ins == 0) greedy_insertion(fuera, false);
            else if (op_ins == 1) greedy_insertion(fuera, true);
            else if (op_ins == 2) regret_insertion(fuera);
            else if (op_ins == 3) time_based_insertion(fuera);
            else zone_insertion(fuera);
        }

        evaluar_y_aceptar(backup, op_rem, op_ins, op_sr, op_si, i);
        T *= alpha;

        if (config.usar_ruleta_adaptativa && i % N_c == 0) actualizar_pesos_clientes();
        if (config.usar_ruleta_adaptativa && i % N_s == 0) actualizar_pesos_estaciones();

        if (i % 10 == 0) {
            out_trayectoria << i << "," << obtener_distancia(mejor_solucion) << ","
                << obtener_distancia(solucion_actual) << "," << T << "\n";
        }

        if (i % 1000 == 0) {
            chrono::high_resolution_clock::time_point ahora = chrono::high_resolution_clock::now();
            double segs = chrono::duration<double>(ahora - tiempo_inicio).count();
            cout << "Iter " << i << " | Temp: " << T << " | Mejor Flota: " << mejor_solucion.size()
                << " | Mejor Dist: " << obtener_distancia(mejor_solucion) << " | Tiempo: " << fixed << setprecision(1) << segs << "s" << endl;
        }
    }

    cout << "\n--- Solucion final ---" << endl;
    cout << "Vehiculos: " << mejor_solucion.size() << endl;
    double total_dist = 0, total_viol_t = 0, total_viol_b = 0, total_viol_c = 0;
    for (const auto& r : mejor_solucion) {
        total_dist += r.distancia_total;
        total_viol_t += r.violacion_tiempo;
        total_viol_b += r.violacion_bateria;
        total_viol_c += r.violacion_capacidad;
        cout << "Ruta: dist=" << r.distancia_total << " viol_t=" << r.violacion_tiempo
            << " viol_b=" << r.violacion_bateria << " viol_c=" << r.violacion_capacidad << endl;
    }
    cout << "Total distancia: " << total_dist << endl;
    cout << "Total viol tiempo: " << total_viol_t << endl;
    cout << "Total viol bateria: " << total_viol_b << endl;
    cout << "Total viol capacidad: " << total_viol_c << endl;

    cout << "\nPool de rutas acumuladas: " << pool_rutas.size() << endl;
    vector<Ruta> sol_matheuristica = resolver_set_covering_cpsat();
    int clientes_cubiertos = 0;
    for (const auto& r : sol_matheuristica)
        for (const auto& p : r.secuencia)
            if (!mapa[p.nodo_id].es_estacion && !mapa[p.nodo_id].es_deposito) clientes_cubiertos++;

    cout << "Clientes cubiertos por matheuristica: " << clientes_cubiertos << " / " << total_clientes << endl;
    cout << "Vehiculos matheuristica: " << sol_matheuristica.size() << " | Distancia: " << obtener_distancia(sol_matheuristica) << endl;
    cout << "Vehiculos ALNS: " << mejor_solucion.size() << " | Distancia: " << obtener_distancia(mejor_solucion) << endl;

    // Guardamos la solucion ALNS pura ANTES de decidir si la sobrescribimos,
    // para poder exportar y comparar ambas versiones.
    vector<Ruta> sol_alns_pura = mejor_solucion;

    bool matheuristica_gano = false;
    if (clientes_cubiertos < total_clientes) {
        cout << "  -> La matheuristica NO logro cobertura completa (pool insuficiente)." << endl;
    }
    else if (!mejor(sol_matheuristica, mejor_solucion)) {
        cout << "  -> Cobertura completa, pero la combinacion resultante fue peor que el ALNS." << endl;
    }
    else {
        cout << "La matheuristica (set-covering greedy) mejoro la solucion final." << endl;
        mejor_solucion = sol_matheuristica;
        matheuristica_gano = true;
    }

    out_trayectoria.close();

    // Exportamos las DOS versiones por separado, sin importar cual gano,
    // para poder comparar rutas visualmente.
    exportar_rutas_de(sol_alns_pura, "rutas_" + nombre_instancia + "_ALNS.csv");
    exportar_rutas_de(sol_matheuristica, "rutas_" + nombre_instancia + "_CPSAT.csv");

    // La que quedo como "oficial" (mejor_solucion) tambien se exporta con el
    // nombre original, para no romper el resto del flujo/scripts existentes.
    exportar_rutas("rutas_" + nombre_instancia + ".csv");

    // CSV resumen comparando metricas de ambas soluciones lado a lado.
    ofstream out_comparacion("comparacion_" + nombre_instancia + ".csv");
    out_comparacion << "Metodo,Vehiculos,Distancia,GanoLaComparacion\n";
    out_comparacion << "ALNS," << sol_alns_pura.size() << "," << obtener_distancia(sol_alns_pura)
        << "," << (matheuristica_gano ? "No" : "Si") << "\n";
    out_comparacion << "CPSAT_SetCovering," << sol_matheuristica.size() << "," << obtener_distancia(sol_matheuristica)
        << "," << (matheuristica_gano ? "Si" : "No") << "\n";
    out_comparacion.close();
    cout << "Comparacion guardada en: comparacion_" << nombre_instancia << ".csv" << endl;
}

void ALNS_EVRPTW::exportar_rutas(string filename) {
    ofstream out(filename);
    out << "RutaID,Posicion,NodoID,X,Y,Tipo\n";
    for (size_t r = 0; r < mejor_solucion.size(); ++r) {
        for (size_t p = 0; p < mejor_solucion[r].secuencia.size(); ++p) {
            int n_id = mejor_solucion[r].secuencia[p].nodo_id;
            out << r << "," << p << "," << n_id << "," << mapa[n_id].x << "," << mapa[n_id].y << "," << mapa[n_id].tipo << "\n";
        }
    }
    out.close();
}

void ALNS_EVRPTW::exportar_rutas_de(const vector<Ruta>& sol, string filename) {
    ofstream out(filename);
    out << "RutaID,Posicion,NodoID,X,Y,Tipo\n";
    for (size_t r = 0; r < sol.size(); ++r) {
        for (size_t p = 0; p < sol[r].secuencia.size(); ++p) {
            int n_id = sol[r].secuencia[p].nodo_id;
            out << r << "," << p << "," << n_id << "," << mapa[n_id].x << "," << mapa[n_id].y << "," << mapa[n_id].tipo << "\n";
        }
    }
    out.close();
}
