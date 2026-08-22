#pragma once
// ============================================================================
//  InstanciaReader.h
//  Lectura de instancias en formato tipo Solomon/Schneider (EVRPTW).
// ============================================================================

#include <string>
#include <vector>
#include "Nodo.h"

/// Lee un archivo de instancia EVRPTW (formato tipo Solomon/Schneider):
/// extrae la capacidad de carga (C), la capacidad de bateria (Q), la tasa
/// de consumo (r) y la tasa de recarga (g), ademas de la lista de nodos
/// (deposito, clientes y estaciones). Aplica valores por defecto razonables
/// si algun parametro no se encuentra en el archivo.
bool leerInstancia(std::string filename, std::vector<Nodo>& mapa,
    double& cc, double& cb, double& rc, double& rcons);
